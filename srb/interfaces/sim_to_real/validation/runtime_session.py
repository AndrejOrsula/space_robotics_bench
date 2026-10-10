"""Validation runtime resource lifecycle."""

from __future__ import annotations

import logging
import math
import os
import threading
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np
import yaml

from srb.interfaces._ros_node import RosCleanupError
from srb.interfaces.sim_to_real.validation.artifact_reference import SPEC_YAML
from srb.interfaces.sim_to_real.validation.policy_factory import call_policy_factory
from srb.interfaces.sim_to_real.validation.rollout import ValidationRolloutAdapter
from srb.interfaces.sim_to_real.validation.session_workspace import (
    ValidationSessionIdentity,
)
from srb.interfaces.sim_to_real.validation.telemetry import Telemetry
from srb.utils.failures import attempt_cleanup
from srb.utils.validation_spec import ValidationSpec

ValidationEnvFactory = Callable[[ValidationSpec], Any]
ValidationPolicyFactory = Callable[..., Callable[[Any], np.ndarray]]
ValidationPolicyCallable = Callable[[Any], np.ndarray]

logger = logging.getLogger(__name__)


def _open_or_create_directory_tree(path: Path) -> int:
    """Open a directory path, creating missing components without links."""

    absolute = Path(os.path.abspath(os.fspath(path)))
    components = absolute.parts
    if len(components) <= 1:
        return os.open(absolute.anchor, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)

    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    fd = os.open(absolute.anchor, flags)
    try:
        for component in components[1:]:
            try:
                child_fd = os.open(component, flags, dir_fd=fd)
            except FileNotFoundError:
                try:
                    os.mkdir(component, mode=0o700, dir_fd=fd)
                except FileExistsError:
                    pass
                child_fd = os.open(component, flags, dir_fd=fd)
            os.close(fd)
            fd = child_fd
        return fd
    except BaseException:
        os.close(fd)
        raise


def _create_contained_workspace(path: Path) -> int:
    """Atomically create the explicit session directory under its parent fd."""

    absolute = Path(os.path.abspath(os.fspath(path)))
    parent_fd = _open_or_create_directory_tree(absolute.parent)
    try:
        # mkdirat-style creation never follows an existing final symlink.
        os.mkdir(absolute.name, mode=0o700, dir_fd=parent_fd)
        return os.open(
            absolute.name,
            os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
            dir_fd=parent_fd,
        )
    finally:
        os.close(parent_fd)


def _publish_bytes_at(directory_fd: int, name: str, payload: bytes) -> None:
    """Atomically publish bytes relative to an already anchored directory fd."""

    temporary_name = f".{name}.{uuid.uuid4().hex}.tmp"
    temporary_fd: int | None = None
    try:
        temporary_fd = os.open(
            temporary_name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
            0o600,
            dir_fd=directory_fd,
        )
        with os.fdopen(temporary_fd, "wb") as stream:
            temporary_fd = None
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(
            temporary_name,
            name,
            src_dir_fd=directory_fd,
            dst_dir_fd=directory_fd,
        )
    except Exception:
        if temporary_fd is not None:
            os.close(temporary_fd)
        try:
            os.unlink(temporary_name, dir_fd=directory_fd)
        except OSError:
            pass
        raise


def _write_spec_snapshot_at(spec: ValidationSpec, directory_fd: int) -> None:
    """Write the snapshot through the descriptor for an explicit session dir."""

    try:
        payload = yaml.safe_dump(spec.model_dump(mode="json"), sort_keys=False)
        _publish_bytes_at(directory_fd, os.fspath(SPEC_YAML), payload.encode("utf-8"))
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            f"failed to persist validation spec snapshot ({SPEC_YAML}): {exc}"
        ) from exc


@dataclass
class ValidationRuntimeSession:
    """Owns runtime resources for one validation session."""

    spec: ValidationSpec
    session_dir: Path
    env_factory: ValidationEnvFactory | None = None
    policy_factory: ValidationPolicyFactory | None = None
    wandb_enabled: bool = True
    tensorboard_enabled: bool = True
    rerun_web_enabled: bool = True
    dry_run: bool = False
    identity: ValidationSessionIdentity | None = None
    abort_poll_seconds: float = 0.2
    abort_request_path: Path | None = None
    env: Any = field(default=None, init=False)
    policy: ValidationPolicyCallable | None = field(default=None, init=False)
    rollout: ValidationRolloutAdapter | None = field(default=None, init=False)
    telemetry: Telemetry | None = field(default=None, init=False)
    abort: threading.Event = field(default_factory=threading.Event, init=False)
    _abort_watcher: threading.Thread | None = field(default=None, init=False)
    _closed: bool = field(default=False, init=False)
    _start_started: bool = field(default=False, init=False)
    _closing_started: bool = field(default=False, init=False)
    _cleanup_lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _env_closed: bool = field(default=False, init=False)
    _watcher_joined: bool = field(default=False, init=False)
    _abort_marker_removed: bool = field(default=False, init=False)
    _telemetry_closed: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        self.session_dir = Path(self.session_dir)
        if self.abort_request_path is None:
            self.abort_request_path = self.session_dir.with_name(
                f".{self.session_dir.name}.abort"
            )
        else:
            self.abort_request_path = Path(self.abort_request_path)
        if not math.isfinite(self.abort_poll_seconds) or self.abort_poll_seconds <= 0:
            raise ValueError("abort_poll_seconds must be finite and positive")

    @property
    def can_rollout(self) -> bool:
        return (
            not self.dry_run
            and self.env is not None
            and self.policy is not None
            and self.rollout is not None
        )

    def start(self) -> None:
        if self._closing_started or self._closed:
            raise RuntimeError("validation runtime session cleanup has begun")
        if self._start_started:
            raise RuntimeError("validation runtime session already started")
        self._start_started = True
        self._allocate_telemetry()
        session_fd: int | None = None
        try:
            session_fd = _create_contained_workspace(self.session_dir)
            _write_spec_snapshot_at(self.spec, session_fd)
        except BaseException as primary_error:
            cleanup_error: BaseException | None = None
            if self.telemetry is not None and not self._telemetry_closed:
                _, cleanup_error = attempt_cleanup(self.telemetry.close)
                if cleanup_error is None:
                    self._telemetry_closed = True
            if cleanup_error is not None:
                raise RosCleanupError(
                    "validation workspace rollback incomplete",
                    errors=(primary_error, cleanup_error),
                    cleanup_owner=self,
                ) from primary_error
            raise
        finally:
            if session_fd is not None:
                os.close(session_fd)
        logger.info(
            "External validation abort request path: %s", self.abort_request_path
        )
        self._start_telemetry()
        self._inspect_abort_request()
        self._spawn_abort_watcher()
        self._prepare_env_and_policy()

    def require_telemetry(self) -> Telemetry:
        if self.telemetry is None:
            raise RuntimeError("validation runtime session telemetry is not started")
        return self.telemetry

    def require_policy(self) -> ValidationPolicyCallable:
        if self.policy is None:
            raise RuntimeError("validation runtime session policy is not available")
        return self.policy

    def require_rollout(self) -> ValidationRolloutAdapter:
        if self.rollout is None:
            raise RuntimeError("validation runtime session rollout is not available")
        return self.rollout

    def close(self) -> None:
        if self._closed:
            return
        if not self._cleanup_lock.acquire(blocking=False):
            raise RosCleanupError(
                "validation runtime cleanup already in progress",
                cleanup_owner=self,
            )
        try:
            if self._closed:
                return
            self._closing_started = True
            self.abort.set()
            errors: list[BaseException] = []

            phases: tuple[tuple[bool, Callable[[], object], str], ...] = (
                (self._env_closed, self._close_environment, "environment"),
                (self._watcher_joined, self._join_abort_watcher, "abort watcher"),
                (self._abort_marker_removed, self._remove_abort_marker, "abort marker"),
                (self._telemetry_closed, self._close_telemetry, "telemetry"),
            )
            for complete, operation, name in phases:
                if complete:
                    continue
                _, error = attempt_cleanup(operation)
                if error is not None:
                    errors.append(error)
                    continue
                if name == "environment":
                    self._env_closed = True
                elif name == "abort watcher":
                    self._watcher_joined = True
                elif name == "abort marker":
                    self._abort_marker_removed = True
                else:
                    self._telemetry_closed = True

            self._closed = (
                self._env_closed
                and self._watcher_joined
                and self._abort_marker_removed
                and self._telemetry_closed
            )
            if errors or not self._closed:
                if not errors:
                    errors.append(RuntimeError("validation cleanup is incomplete"))
                raise RosCleanupError(
                    "validation runtime cleanup incomplete",
                    errors=tuple(errors),
                    cleanup_owner=self,
                )
        finally:
            self._cleanup_lock.release()

    def adopt_environment(self, owner: Any) -> None:
        """Retain an environment as soon as construction gives cleanup ownership."""

        if owner is None:
            raise TypeError("validation environment owner cannot be None")
        if not callable(getattr(owner, "close", None)):
            raise TypeError("validation environment owner must provide close()")
        if owner is self:
            raise ValueError("validation runtime session cannot own itself")
        if self._closing_started or self._closed:
            raise RuntimeError("cannot adopt environment after cleanup has begun")
        if self._env_closed:
            raise RuntimeError("cannot replace an environment after it was closed")
        if self.env is owner:
            return
        if self.env is not None:
            raise RuntimeError(
                "validation runtime session adopted multiple environments"
            )
        self.env = owner

    def _replace_with_factory_result(self, owner: Any) -> None:
        """Promote the constructed wrapper after successful factory completion."""

        if owner is None:
            raise TypeError("validation environment factory returned None")
        if not callable(getattr(owner, "close", None)):
            raise TypeError(
                "validation environment factory result must provide close()"
            )
        if self._closing_started or self._closed:
            raise RuntimeError("cannot replace environment after cleanup has begun")
        self.env = owner

    def _close_environment(self) -> None:
        if self.env is not None:
            self.env.close()

    def _remove_abort_marker(self) -> None:
        if self.abort_request_path is None:
            return
        try:
            self.abort_request_path.unlink()
        except FileNotFoundError:
            pass

    def _close_telemetry(self) -> None:
        if self.telemetry is not None:
            self.telemetry.close()

    def _start_telemetry(self) -> None:
        self._allocate_telemetry()
        assert self.telemetry is not None
        self.telemetry.start()

    def _allocate_telemetry(self) -> None:
        if self.telemetry is not None:
            return
        telemetry_kwargs: dict[str, Any] = {
            "wandb_enabled": self.wandb_enabled,
            "tensorboard_enabled": self.tensorboard_enabled,
            "rerun_web_enabled": self.rerun_web_enabled,
        }
        if self.identity is not None:
            telemetry_kwargs["identity"] = self.identity
        self.telemetry = Telemetry(
            self.spec,
            self.session_dir,
            **telemetry_kwargs,
        )

    def _prepare_env_and_policy(self) -> None:
        if self.env_factory is None or self.policy_factory is None:
            if self.dry_run:
                return
            exc = RuntimeError(
                "harness requires env_factory and policy_factory unless dry_run=True"
            )
            self._record_preflight_failure(exc)
            raise exc
        try:
            self._replace_with_factory_result(self.env_factory(self.spec))
            self.rollout = ValidationRolloutAdapter(self.env)
            policy = call_policy_factory(
                self.policy_factory,
                self.spec,
                self.env,
            )
            if not callable(policy):
                raise RuntimeError(
                    "policy_factory returned non-callable policy: "
                    f"{type(policy).__name__}"
                )
            self.policy = policy
        except BaseException as exc:
            partial_owner = getattr(exc, "cleanup_owner", None)
            if partial_owner is not None:
                if self.env is None or self.env is partial_owner:
                    self.adopt_environment(partial_owner)
            if isinstance(exc, Exception):
                _, logging_error = attempt_cleanup(
                    lambda: self._record_preflight_failure(exc)
                )
                if logging_error is not None:
                    logger.warning(
                        "failed to record validation preflight failure: %s",
                        logging_error,
                    )
            raise

    def _spawn_abort_watcher(self) -> None:
        if self._abort_watcher is not None:
            return

        def _watch() -> None:
            while not self.abort.is_set():
                if os.environ.get("SRB_ABORT") == "1":
                    self.abort.set()
                    return
                if self._inspect_abort_request():
                    return
                self.abort.wait(self.abort_poll_seconds)

        self._abort_watcher = threading.Thread(
            target=_watch, daemon=True, name="srb-abort-watch"
        )
        self._abort_watcher.start()

    def _inspect_abort_request(self) -> bool:
        """Set the cancellation event when the request marker exists."""

        assert self.abort_request_path is not None
        try:
            self.abort_request_path.stat()
        except FileNotFoundError:
            return False
        except OSError as exc:
            logger.error(
                "failed to inspect external validation abort request %s: %s",
                self.abort_request_path,
                exc,
            )
            self.abort.set()
            return True
        self.abort.set()
        return True

    def _join_abort_watcher(self) -> None:
        watcher = self._abort_watcher
        if watcher is None or not watcher.is_alive():
            return
        timeout = max(self.abort_poll_seconds * 2.0, 0.05)
        watcher.join(timeout=timeout)
        if watcher.is_alive():
            raise TimeoutError("validation abort watcher did not stop before timeout")

    def _record_preflight_failure(self, exc: Exception) -> None:
        if self.telemetry is not None:
            self.telemetry.log_event(
                "preflight_failure",
                {"exception_type": type(exc).__name__, "message": str(exc)},
            )
