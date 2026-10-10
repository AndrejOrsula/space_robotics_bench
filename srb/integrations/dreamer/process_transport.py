"""Parent-side Dreamer process transport with an all-or-nothing row barrier."""

from __future__ import annotations

import hashlib
import os
import signal
import subprocess
import sys
import tempfile
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from dataclasses import field as _process_transport_field
from multiprocessing import Pipe
from multiprocessing.connection import wait as wait_for_connections
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Callable

import numpy as np

from .process_projection import (
    DreamerProcessProjection,
)
from .process_protocol import (
    ZERO_SCHEMA_HASHES,
    ClosedResponse,
    CloseRequest,
    ErrorResponse,
    HelloRequest,
    HelloResponse,
    ResetRequest,
    StepRequest,
    TransitionResponse,
    recv_message,
    schema_hashes_for,
    send_message,
    validate_landscaping_contract,
    validate_schema_hashes,
)

if TYPE_CHECKING:
    from .worker_bootstrap import DreamerWorkerLaunchSpec


class DreamerProcessTransportError(RuntimeError):
    """One failed process barrier; no partial row result is ever returned."""

    def __init__(
        self,
        reason: str,
        *,
        worker_id: int | None = None,
        pid: int | None = None,
        seed: int | None = None,
        sequence: int | None = None,
        episode_id: int | None = None,
        step_index: int | None = None,
        poll: Any = None,
        returncode: Any = None,
        output_tail: str | None = None,
    ) -> None:
        self.reason = reason
        self.worker_id = worker_id
        self.pid = pid
        self.seed = seed
        self.sequence = sequence
        self.episode_id = episode_id
        self.step_index = step_index
        self.poll = poll
        self.returncode = returncode
        self.output_tail = _bound_diagnostic_text(output_tail)
        context = (
            f"worker_id={worker_id} pid={pid} seed={seed} sequence={sequence} "
            f"episode_id={episode_id} step_index={step_index}"
        )
        if self.output_tail is not None or poll is not None or returncode is not None:
            context += f" poll={poll!r} returncode={returncode!r}"
            signal_name = _signal_name(returncode)
            if signal_name is not None:
                context += f" signal={signal_name}"
            context += f" child_output_tail={self.output_tail!r}"
        super().__init__(f"Dreamer process barrier failed ({context}): {reason}")


class DreamerWorkerLaunchError(DreamerProcessTransportError):
    """A fixed-argv worker process could not be launched or reaped."""


_DREAMER_DIAGNOSTIC_TAIL_BYTES = 12 * 1024


def _bound_diagnostic_text(value: str | None) -> str | None:
    if value is None:
        return None
    encoded = value.encode("utf-8", errors="replace")
    return encoded[-_DREAMER_DIAGNOSTIC_TAIL_BYTES:].decode("utf-8", errors="replace")


def _signal_name(returncode: Any) -> str | None:
    if type(returncode) is not int or returncode >= 0:
        return None
    try:
        return signal.Signals(-returncode).name
    except ValueError:
        return f"SIG{-returncode}"


def _read_diagnostic_tail(diagnostic_file: Any) -> str:
    if diagnostic_file is None:
        return ""
    try:
        descriptor = diagnostic_file.fileno()
        size = os.fstat(descriptor).st_size
        start = max(0, size - _DREAMER_DIAGNOSTIC_TAIL_BYTES)
        if hasattr(os, "pread"):
            data = os.pread(descriptor, _DREAMER_DIAGNOSTIC_TAIL_BYTES, start)
        else:
            diagnostic_file.seek(start)
            data = diagnostic_file.read(_DREAMER_DIAGNOSTIC_TAIL_BYTES)
        if size > _DREAMER_DIAGNOSTIC_TAIL_BYTES:
            marker = b"[... diagnostic output truncated ...]\n"
            data = marker + data[-(_DREAMER_DIAGNOSTIC_TAIL_BYTES - len(marker)) :]
        return data.decode("utf-8", errors="replace")
    except (AttributeError, OSError, TypeError, ValueError):
        return ""


def _process_poll_and_returncode(process: Any) -> tuple[Any, Any, bool]:
    poll_method = getattr(process, "poll", None)
    if not callable(poll_method):
        return None, None, False
    try:
        poll = poll_method()
    except Exception as exc:
        poll = f"<poll failed: {type(exc).__name__}: {exc}>"
    returncode = getattr(process, "returncode", None)
    if returncode is None and type(poll) is int:
        returncode = poll
    return poll, returncode, True


def _worker_failure_diagnostics(
    worker: Any, owner: Any | None
) -> tuple[Any, Any, str | None]:
    for candidate in (owner, worker):
        if candidate is None:
            continue
        diagnostic_method = getattr(candidate, "failure_diagnostics", None)
        if callable(diagnostic_method):
            try:
                poll, returncode, output_tail = diagnostic_method()
            except Exception:
                continue
            return poll, returncode, _bound_diagnostic_text(output_tail or "")

    process = getattr(worker, "process", None)
    if process is None and owner is not None:
        process = getattr(owner, "process", None)
    if process is None:
        return None, None, None
    poll, returncode, has_status = _process_poll_and_returncode(process)
    return poll, returncode, "" if has_status else None


@dataclass(frozen=True, slots=True)
class DreamerProcessSchema:
    """Schema agreed by every worker during the public HELLO barrier."""

    action_projection: DreamerProcessProjection
    observation_projection: DreamerProcessProjection
    schema_hashes: Mapping[str, str]
    demo_projection: Mapping[str, Any] | None = None
    landscaping_contract: Mapping[str, Any] | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "schema_hashes", dict(self.schema_hashes))
        if self.demo_projection is not None:
            from .demo_projection import validate_demo_projection_descriptor

            descriptor = validate_demo_projection_descriptor(
                self.demo_projection,
                action_projection=self.action_projection,
                observation_projection=self.observation_projection,
            )
            object.__setattr__(self, "demo_projection", descriptor)
        if self.landscaping_contract is not None:
            object.__setattr__(
                self,
                "landscaping_contract",
                MappingProxyType(
                    validate_landscaping_contract(self.landscaping_contract)
                ),
            )


@dataclass(frozen=True, slots=True)
class DreamerWorkerDiagnosticIdentity:
    """Bounded process identity; child stdout/stderr are never piped."""

    worker_id: int
    pid: int
    seed: int
    module: str
    argv_sha256: str

    def render(self) -> str:
        return (
            f"worker_id={self.worker_id} pid={self.pid} seed={self.seed} "
            f"module={self.module} argv_sha256={self.argv_sha256}"
        )


class DreamerPopenWorkerOwner:
    """Own one new-session child, its process group, and parent endpoint."""

    def __init__(
        self,
        *,
        process: Any,
        parent_connection: Any,
        spec: "DreamerWorkerLaunchSpec",
        argv_sha256: str,
        killpg: Callable[[int, int], None],
        exit_grace_s: float,
        terminate_grace_s: float,
        kill_grace_s: float,
        diagnostic_file: Any | None = None,
    ) -> None:
        self.process = process
        self.parent_connection = parent_connection
        self._killpg = killpg
        self._exit_grace_s = exit_grace_s
        self._terminate_grace_s = terminate_grace_s
        self._kill_grace_s = kill_grace_s
        self._diagnostic_file = diagnostic_file
        self._stopped = False
        self.diagnostic = DreamerWorkerDiagnosticIdentity(
            worker_id=spec.worker_index,
            pid=process.pid,
            seed=spec.worker_seed,
            module="srb.integrations.dreamer.worker_bootstrap",
            argv_sha256=argv_sha256,
        )
        self.worker_handle = DreamerProcessWorker(
            worker_id=spec.worker_index,
            connection=parent_connection,
            seed=spec.worker_seed,
            pid=process.pid,
            process=process,
            cleanup=self.stop,
        )

    def failure_diagnostics(self) -> tuple[Any, Any, str]:
        # The pipe may close before the exit status is visible. Wait briefly so
        # diagnostics include it on a loaded machine.
        if self.process.poll() is None:
            try:
                self.process.wait(timeout=self._exit_grace_s)
            except subprocess.TimeoutExpired:
                pass
        poll, returncode, _ = _process_poll_and_returncode(self.process)
        return poll, returncode, _read_diagnostic_tail(self._diagnostic_file)

    def stop(self) -> None:
        if self._stopped:
            return
        reap_error: BaseException | None = None
        try:
            if self.process.poll() is None:
                try:
                    self.process.wait(timeout=self._exit_grace_s)
                except subprocess.TimeoutExpired:
                    try:
                        self._killpg(self.process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        self.process.wait(timeout=self._terminate_grace_s)
                    except subprocess.TimeoutExpired:
                        try:
                            self._killpg(self.process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        try:
                            self.process.wait(timeout=self._kill_grace_s)
                        except subprocess.TimeoutExpired:
                            self.process.kill()
                            try:
                                self.process.wait(timeout=self._kill_grace_s)
                            except subprocess.TimeoutExpired as exc:
                                reap_error = exc
        finally:
            try:
                self.parent_connection.close()
            except (AttributeError, OSError):
                pass
            try:
                if self._diagnostic_file is not None:
                    self._diagnostic_file.close()
            except (AttributeError, OSError, ValueError):
                pass
        if reap_error is not None:
            raise DreamerWorkerLaunchError(
                "worker did not reap after TERM and KILL fallbacks",
                worker_id=self.worker_handle.worker_id,
                pid=self.process.pid,
                seed=self.worker_handle.seed,
            ) from reap_error
        self._stopped = True

    close = stop


class DreamerPopenWorkerFactory:
    """Create worker owners using only a fixed argv and one passed endpoint."""

    def __init__(
        self,
        *,
        popen_factory: Callable[..., Any] = subprocess.Popen,
        pipe_factory: Callable[..., tuple[Any, Any]] = Pipe,
        executable: str = sys.executable,
        killpg: Callable[[int, int], None] = os.killpg,
        exit_grace_s: float = 0.25,
        terminate_grace_s: float = 2.0,
        kill_grace_s: float = 1.0,
    ) -> None:
        if exit_grace_s <= 0 or terminate_grace_s <= 0 or kill_grace_s <= 0:
            raise ValueError("worker process cleanup grace periods must be positive")
        self._popen_factory = popen_factory
        self._pipe_factory = pipe_factory
        self._executable = executable
        self._killpg = killpg
        self._exit_grace_s = float(exit_grace_s)
        self._terminate_grace_s = float(terminate_grace_s)
        self._kill_grace_s = float(kill_grace_s)

    def launch(self, spec: "DreamerWorkerLaunchSpec") -> DreamerPopenWorkerOwner:
        if os.name != "posix":
            raise DreamerWorkerLaunchError("passed-FD Dreamer workers require POSIX")
        spec_json = spec.to_json()
        parent_connection, child_connection = self._pipe_factory(duplex=True)
        process: Any | None = None
        diagnostic_file: Any | None = None
        try:
            child_fd = child_connection.fileno()
            argv = (
                self._executable,
                "-m",
                "srb.integrations.dreamer.worker_bootstrap",
                "--control-fd",
                str(child_fd),
                "--spec-json",
                spec_json,
            )
            argv_sha256 = hashlib.sha256("\x00".join(argv).encode("utf-8")).hexdigest()
            launch_env = os.environ.copy()
            # Python reads PYTHONHASHSEED at startup; setting it in the worker
            # would be too late to make hashing deterministic.
            launch_env["PYTHONHASHSEED"] = str(spec.worker_seed)
            launch_env["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
            diagnostic_file = tempfile.TemporaryFile(
                mode="w+b", prefix="srb-dreamer-worker-"
            )
            process = self._popen_factory(
                argv,
                close_fds=True,
                env=launch_env,
                pass_fds=(child_fd,),
                start_new_session=True,
                shell=False,
                stdin=subprocess.DEVNULL,
                stdout=diagnostic_file,
                stderr=diagnostic_file,
            )
            if type(process.pid) is not int or process.pid <= 0:
                raise DreamerWorkerLaunchError("Popen returned an invalid worker PID")
            child_connection.close()
        except BaseException as exc:  # Ctrl-C here must not orphan the worker.
            if process is not None:
                try:
                    _transport_process_stop(process, timeout_s=self._exit_grace_s)
                except Exception:
                    pass
            if diagnostic_file is not None:
                try:
                    diagnostic_file.close()
                except (AttributeError, OSError, ValueError):
                    pass
            for endpoint in (parent_connection, child_connection):
                try:
                    endpoint.close()
                except (AttributeError, OSError):
                    pass
            if isinstance(exc, DreamerWorkerLaunchError) or not isinstance(
                exc, Exception
            ):
                raise
            raise DreamerWorkerLaunchError(f"worker Popen failed: {exc}") from exc
        try:
            return DreamerPopenWorkerOwner(
                process=process,
                parent_connection=parent_connection,
                spec=spec,
                argv_sha256=argv_sha256,
                killpg=self._killpg,
                exit_grace_s=self._exit_grace_s,
                terminate_grace_s=self._terminate_grace_s,
                kill_grace_s=self._kill_grace_s,
                diagnostic_file=diagnostic_file,
            )
        except BaseException as exc:
            try:
                _transport_process_stop(process, timeout_s=self._exit_grace_s)
            except Exception:
                pass
            try:
                parent_connection.close()
            except (AttributeError, OSError):
                pass
            try:
                if diagnostic_file is not None:
                    diagnostic_file.close()
            except (AttributeError, OSError, ValueError):
                pass
            if not isinstance(exc, Exception):
                raise
            raise DreamerWorkerLaunchError(f"worker owner setup failed: {exc}") from exc


def launch_dreamer_worker(
    spec: "DreamerWorkerLaunchSpec", **factory_options: Any
) -> DreamerPopenWorkerOwner:
    return DreamerPopenWorkerFactory(**factory_options).launch(spec)


def _host_array(value: Any, *, name: str) -> np.ndarray:
    if not isinstance(value, np.ndarray) and hasattr(value, "detach"):
        try:
            value = value.detach().cpu().numpy()
        except (AttributeError, RuntimeError, TypeError) as exc:
            raise TypeError(f"{name} tensor is not host-readable") from exc
    try:
        return np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} is not NumPy-compatible") from exc


def _select_batched_row(
    value: Any,
    node: Mapping[str, Any],
    *,
    row: int,
    num_envs: int,
    path: tuple[str | int, ...] = (),
) -> Any:
    kind = node["kind"]
    if kind == "leaf":
        array = _host_array(value, name=f"action at {path!r}")
        if array.ndim == 0 or array.shape[0] != num_envs:
            raise ValueError(
                f"batched action at {path!r} must have first dimension {num_envs}, "
                f"got {array.shape}"
            )
        return np.array(array[row], copy=True)
    if kind == "dict":
        if not isinstance(value, Mapping):
            raise TypeError(f"batched action at {path!r} must be a mapping")
        expected = {child["key"] for child in node["children"]}
        if set(value) != expected:
            raise TypeError(f"batched action keys at {path!r} do not match")
        return {
            child["key"]: _select_batched_row(
                value[child["key"]],
                child["node"],
                row=row,
                num_envs=num_envs,
                path=path + (child["key"],),
            )
            for child in node["children"]
        }
    if type(value) is not tuple or len(value) != len(node["children"]):
        raise TypeError(f"batched action at {path!r} must match the Tuple tree")
    return tuple(
        _select_batched_row(
            value[child["index"]],
            child["node"],
            row=row,
            num_envs=num_envs,
            path=path + (child["index"],),
        )
        for child in node["children"]
    )


__all__ = [
    "DreamerProcessTransport",
    "DreamerProcessTransportError",
    "DreamerProcessSchema",
    "DreamerProcessWorker",
    "DreamerPopenWorkerFactory",
    "DreamerPopenWorkerOwner",
    "DreamerWorkerDiagnosticIdentity",
    "DreamerWorkerLaunchError",
    "ProcessWorkerHandle",
    "WorkerHandle",
    "launch_dreamer_worker",
]


def _transport_snapshot(value: Any) -> Any:
    """Copy a result tree without retaining worker-owned array storage."""

    if isinstance(value, np.ndarray):
        copied = np.array(value, copy=True)
        copied.setflags(write=False)
        return copied
    if isinstance(value, Mapping):
        return {key: _transport_snapshot(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return tuple(_transport_snapshot(item) for item in value)
    if isinstance(value, list):
        return [_transport_snapshot(item) for item in value]
    return value


def _transport_stack(values: Sequence[Any]) -> Any:
    """Stack one decoded observation tree per worker row."""

    if not values:
        return {}
    first = values[0]
    if isinstance(first, Mapping):
        keys = tuple(first)
        if any(tuple(value) != keys for value in values[1:]):
            raise ValueError("worker observations changed their leaf map")
        return {
            key: _transport_stack(tuple(value[key] for value in values)) for key in keys
        }
    if isinstance(first, tuple):
        return tuple(
            _transport_stack(tuple(value[index] for value in values))
            for index in range(len(first))
        )
    if isinstance(first, list):
        return [
            _transport_stack(tuple(value[index] for value in values))
            for index in range(len(first))
        ]
    return np.stack([np.asarray(value) for value in values], axis=0)


@dataclass(frozen=True, slots=True)
class DreamerProcessBatchResult:
    """Immutable, dependency-light batch result consumed by the wrapper seam."""

    observation: Any
    reward: Any
    terminated: Any
    truncated: Any
    info: tuple[Mapping[str, Any] | None, ...]
    is_first: Any
    episode_ids: Any
    step_delta: Any

    def __post_init__(self) -> None:
        object.__setattr__(self, "observation", _transport_snapshot(self.observation))
        object.__setattr__(
            self,
            "reward",
            _transport_snapshot(np.asarray(self.reward, dtype=np.float32)),
        )
        object.__setattr__(
            self,
            "terminated",
            _transport_snapshot(np.asarray(self.terminated, dtype=np.bool_)),
        )
        object.__setattr__(
            self,
            "truncated",
            _transport_snapshot(np.asarray(self.truncated, dtype=np.bool_)),
        )
        object.__setattr__(
            self,
            "is_first",
            _transport_snapshot(np.asarray(self.is_first, dtype=np.bool_)),
        )
        object.__setattr__(
            self,
            "episode_ids",
            _transport_snapshot(np.asarray(self.episode_ids, dtype=np.int64)),
        )
        object.__setattr__(
            self,
            "step_delta",
            _transport_snapshot(np.asarray(self.step_delta, dtype=np.int64)),
        )
        object.__setattr__(
            self,
            "info",
            tuple(
                None if item is None else _transport_snapshot(dict(item))
                for item in self.info
            ),
        )


@dataclass(frozen=True, slots=True)
class DreamerProcessEpisodeState:
    """Snapshot of one transport row's episode identity and progress."""

    env_id: int
    episode_id: int
    step_index: int


DreamerProcessRowEpisodeState = DreamerProcessEpisodeState


def _transport_probe_process(process: Any) -> bool | None:
    """Probe a public process API without assuming its process family."""

    poll = getattr(process, "poll", None)
    if callable(poll):
        try:
            return poll() is None
        except Exception:
            return False
    is_alive = getattr(process, "is_alive", None)
    if callable(is_alive):
        try:
            return bool(is_alive())
        except Exception:
            return False
    return None


def _transport_endpoint_open(connection: Any) -> bool:
    """Check endpoint state without polling or receiving protocol data."""

    try:
        if getattr(connection, "closed", False):
            return False
    except Exception:
        return False
    fileno = getattr(connection, "fileno", None)
    if callable(fileno):
        try:
            return int(fileno()) >= 0
        except Exception:
            return False
    return hasattr(connection, "close")


def _transport_process_stop(process: Any, *, timeout_s: float = 1.0) -> None:
    """Gracefully reap a worker, then terminate it on a bounded fallback."""

    poll = getattr(process, "poll", None)
    if callable(poll):
        try:
            running = poll() is None
        except Exception:
            running = True
        wait = getattr(process, "wait", None)
        if not callable(wait):
            raise DreamerWorkerLaunchError("Popen owner exposes no wait method")
        if running:
            try:
                wait(timeout=timeout_s)
                return
            except subprocess.TimeoutExpired:
                pass
            terminate = getattr(process, "terminate", None)
            if not callable(terminate):
                terminate = getattr(process, "kill", None)
            if not callable(terminate):
                raise DreamerWorkerLaunchError(
                    "Popen owner exposes no terminate or kill method"
                )
            terminate()
        try:
            wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            kill = getattr(process, "kill", None)
            if not callable(kill):
                raise DreamerWorkerLaunchError(
                    "Popen owner did not reap and exposes no kill method"
                )
            kill()
            wait(timeout=timeout_s)
        return

    is_alive = getattr(process, "is_alive", None)
    join = getattr(process, "join", None)
    if not callable(is_alive) or not callable(join):
        raise DreamerWorkerLaunchError(
            "unknown worker process handle cannot be probed or reaped"
        )
    try:
        running = bool(is_alive())
    except Exception as exc:
        raise DreamerWorkerLaunchError("worker process liveness probe failed") from exc
    join(timeout=timeout_s)
    try:
        running = bool(is_alive())
    except Exception as exc:
        raise DreamerWorkerLaunchError(
            "worker process liveness probe failed after join"
        ) from exc
    if running:
        terminate = getattr(process, "terminate", None)
        if not callable(terminate):
            raise DreamerWorkerLaunchError(
                "multiprocessing owner exposes no terminate method"
            )
        terminate()
        join(timeout=timeout_s)
        if bool(is_alive()):
            kill = getattr(process, "kill", None)
            if not callable(kill):
                raise DreamerWorkerLaunchError(
                    "multiprocessing owner cannot kill a live process"
                )
            kill()
            join(timeout=timeout_s)


@dataclass(slots=True)
class DreamerProcessWorker:
    """Injected worker handle with process-family-safe cleanup."""

    worker_id: int
    connection: Any
    seed: int
    pid: int | None = None
    process: Any | None = None
    cleanup: Callable[[], None] | None = None
    schema_hashes: Mapping[str, str] | None = None
    _stopped: bool = _process_transport_field(default=False, init=False)

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        primary: BaseException | None = None
        try:
            if self.cleanup is not None:
                self.cleanup()
            elif self.process is not None:
                _transport_process_stop(self.process)
        except BaseException as exc:
            primary = exc
        finally:
            try:
                self.connection.close()
            except BaseException as exc:
                if primary is None:
                    primary = exc
        if primary is not None:
            raise primary


WorkerHandle = DreamerProcessWorker
ProcessWorkerHandle = DreamerProcessWorker


class DreamerProcessTransport:
    """Parent-side protocol transport with an all-or-nothing row barrier."""

    def __init__(
        self,
        workers: Sequence[Any],
        *,
        timeout_s: float = 5.0,
        startup_timeout_s: float | None = None,
        cleanup_callback: Callable[[Sequence[DreamerProcessWorker]], None]
        | None = None,
    ) -> None:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if startup_timeout_s is not None and startup_timeout_s <= 0:
            raise ValueError("startup_timeout_s must be positive")
        raw_workers = tuple(workers)
        if not raw_workers:
            raise ValueError("at least one worker is required")
        normalized: list[DreamerProcessWorker] = []
        owners: list[Any] = []
        for raw in raw_workers:
            worker = getattr(raw, "worker_handle", raw)
            if not hasattr(worker, "connection"):
                raise DreamerProcessTransportError(
                    "worker owner does not expose a connection"
                )
            normalized.append(worker)
            owners.append(raw)
        self._workers = tuple(normalized)
        self._owners = tuple(owners)
        self._connections = tuple(worker.connection for worker in self._workers)
        self._timeout_s = float(timeout_s)
        self._startup_timeout_s = float(
            timeout_s if startup_timeout_s is None else startup_timeout_s
        )
        self._cleanup_callback = cleanup_callback
        self._schema: DreamerProcessSchema | None = None
        self._sequence = 0
        self._states = tuple(
            DreamerProcessEpisodeState(worker.worker_id, -1, 0)
            for worker in self._workers
        )
        self._cleanup_started = False
        self._closed = False
        self._fatal_error: DreamerProcessTransportError | None = None
        self._cleanup_diagnostics: tuple[str, ...] = ()

    @property
    def schema(self) -> DreamerProcessSchema | None:
        return self._schema

    @property
    def num_envs(self) -> int:
        """Return the stable number of configured worker rows."""

        return len(self._workers)

    @property
    def row_episode_states(self) -> tuple[DreamerProcessEpisodeState, ...]:
        return tuple(
            DreamerProcessEpisodeState(state.env_id, state.episode_id, state.step_index)
            for state in self._states
        )

    @property
    def cleanup_diagnostics(self) -> tuple[str, ...]:
        return self._cleanup_diagnostics

    def is_running(self) -> bool:
        if self._closed:
            return False
        statuses: list[bool] = []
        for worker, owner, connection in zip(
            self._workers, self._owners, self._connections
        ):
            endpoint_alive = _transport_endpoint_open(connection)
            candidates = (
                getattr(owner, "process", None),
                getattr(worker, "process", None),
                owner,
                worker,
            )
            owner_alive: bool | None = None
            for candidate in candidates:
                if candidate is None:
                    continue
                owner_alive = _transport_probe_process(candidate)
                if owner_alive is not None:
                    break
            statuses.append(endpoint_alive and owner_alive is True)
        return bool(statuses) and all(statuses)

    def _error(
        self,
        reason: str,
        *,
        index: int | None = None,
        sequence: int | None = None,
    ) -> DreamerProcessTransportError:
        worker = None if index is None else self._workers[index]
        owner = None
        owners = getattr(self, "_owners", ())
        if index is not None and index < len(owners):
            owner = owners[index]
        poll, returncode, output_tail = _worker_failure_diagnostics(worker, owner)
        return DreamerProcessTransportError(
            reason,
            worker_id=None if worker is None else worker.worker_id,
            pid=None if worker is None else worker.pid,
            seed=None if worker is None else worker.seed,
            sequence=sequence,
            poll=poll,
            returncode=returncode,
            output_tail=output_tail,
        )

    def _annotate_error(self, error: BaseException, diagnostics: Sequence[str]) -> None:
        bounded = tuple(diagnostics[:8])
        self._cleanup_diagnostics = bounded
        try:
            setattr(error, "cleanup_diagnostics", bounded)
            setattr(error, "cleanup_errors", bounded)
        except Exception:
            pass

    def _post_close_exit_error(self) -> DreamerProcessTransportError | None:
        """Report a worker whose runtime cleanup failed after its CLOSED ack."""

        # Workers acknowledge CLOSE before releasing Kit, so only their exit
        # status reveals a failed shutdown.  A worker still running at the
        # deadline is left to the bounded cleanup escalation as before.
        deadline = time.monotonic() + self._timeout_s
        for index, (worker, owner) in enumerate(zip(self._workers, self._owners)):
            process = getattr(worker, "process", None)
            if process is None:
                process = getattr(owner, "process", None)
            wait = getattr(process, "wait", None)
            if not callable(wait):
                continue
            try:
                wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                continue
            _, returncode, _ = _process_poll_and_returncode(process)
            if type(returncode) is int and returncode != 0:
                return self._error(
                    f"worker exited with status {returncode} after acknowledging CLOSE",
                    index=index,
                )
        return None

    def _cleanup_one(self, worker: Any, owner: Any) -> None:
        method = None
        for name in ("stop", "close"):
            candidate = getattr(owner, name, None)
            if callable(candidate):
                method = candidate
                break
        if method is None:
            method = getattr(worker, "stop", None)
        if callable(method):
            method()
        else:
            raise DreamerWorkerLaunchError(
                "worker owner exposes no public stop or close method",
                worker_id=getattr(worker, "worker_id", None),
                pid=getattr(worker, "pid", None),
                seed=getattr(worker, "seed", None),
            )
        process = getattr(worker, "process", None)
        if process is None:
            process = getattr(owner, "process", None)
        if process is not None:
            _transport_process_stop(process)

    def _cleanup(self) -> tuple[BaseException, ...]:
        if self._cleanup_started:
            return ()
        self._cleanup_started = True
        errors: list[BaseException] = []
        for connection in self._connections:
            try:
                connection.close()
            except BaseException as exc:
                errors.append(exc)
        if self._cleanup_callback is not None:
            try:
                self._cleanup_callback(self._workers)
            except BaseException as exc:
                errors.append(exc)
        seen: set[int] = set()
        for worker, owner in zip(self._workers, self._owners):
            if id(owner) in seen:
                continue
            seen.add(id(owner))
            try:
                self._cleanup_one(worker, owner)
            except BaseException as exc:
                errors.append(exc)
                process = getattr(worker, "process", None)
                if process is None:
                    process = getattr(owner, "process", None)
                if process is not None:
                    try:
                        _transport_process_stop(process)
                    except BaseException as fallback_exc:
                        errors.append(fallback_exc)
        diagnostics = tuple(
            f"{type(error).__name__}: {' '.join(str(error).splitlines())[:240]}"
            for error in errors
        )
        self._annotate_error(self._fatal_error or RuntimeError("cleanup"), diagnostics)
        return tuple(errors)

    def _abort(self, error: DreamerProcessTransportError) -> None:
        if self._fatal_error is None:
            self._fatal_error = error
        cleanup_errors = self._cleanup()
        self._closed = True
        self._annotate_error(
            error,
            self._cleanup_diagnostics
            or tuple(
                f"{type(item).__name__}: {' '.join(str(item).splitlines())[:240]}"
                for item in cleanup_errors
            ),
        )

    def _request_common(
        self,
        worker: DreamerProcessWorker,
        hashes: Mapping[str, str],
        *,
        episode_id: int,
        step_index: int,
    ) -> dict[str, Any]:
        # Request PID identifies this controller process, not the worker.
        return {
            "worker_id": worker.worker_id,
            "pid": os.getpid(),
            "seed": worker.seed,
            "sequence": self._sequence,
            "episode_id": episode_id,
            "step_index": step_index,
            "schema_hashes": dict(hashes),
        }

    def _exchange(
        self,
        requests: Sequence[Any],
        *,
        fatal: bool,
        timeout_s: float | None = None,
    ) -> list[Any]:
        responses: list[Any | None] = [None] * len(self._workers)
        pending = {
            id(connection): index for index, connection in enumerate(self._connections)
        }
        primary: DreamerProcessTransportError | None = None
        for index, (connection, request) in enumerate(zip(self._connections, requests)):
            try:
                send_message(connection, request)
            except Exception as exc:
                if primary is None:
                    primary = self._error(
                        f"failed to send {type(request).__name__}: {exc}",
                        index=index,
                        sequence=self._sequence,
                    )
                pending.pop(id(connection), None)
        deadline = time.monotonic() + (
            self._timeout_s if timeout_s is None else timeout_s
        )
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                index = next(iter(pending.values()))
                if primary is None:
                    primary = self._error(
                        "timed out waiting for worker response",
                        index=index,
                        sequence=self._sequence,
                    )
                break
            ready: Sequence[Any]
            try:
                ready = wait_for_connections(
                    [self._connections[index] for index in pending.values()],
                    timeout=remaining,
                )
            except Exception as exc:
                if primary is None:
                    index = next(iter(pending.values()))
                    primary = self._error(
                        f"failed waiting for worker response: {exc}",
                        index=index,
                        sequence=self._sequence,
                    )
                break
            if not ready:
                if primary is None:
                    index = next(iter(pending.values()))
                    primary = self._error(
                        "timed out waiting for worker response",
                        index=index,
                        sequence=self._sequence,
                    )
                break
            for connection in ready:
                index = pending.pop(id(connection), None)
                if index is None:
                    continue
                try:
                    response = recv_message(connection)
                except EOFError:
                    if primary is None:
                        primary = self._error(
                            "EOF while receiving worker response",
                            index=index,
                            sequence=self._sequence,
                        )
                    continue
                except Exception as exc:
                    if primary is None:
                        primary = self._error(
                            f"failed receiving worker response: {exc}",
                            index=index,
                            sequence=self._sequence,
                        )
                    continue
                try:
                    # Every response, including an error response, identifies its child.
                    self._validate_identity(index, response, hashes=None)
                except DreamerProcessTransportError as exc:
                    if primary is None:
                        primary = exc
                    continue
                responses[index] = response
                if isinstance(response, ErrorResponse) and primary is None:
                    primary = self._error(
                        f"{response.code}: {response.message}",
                        index=index,
                        sequence=getattr(response, "sequence", self._sequence),
                    )
        if primary is not None:
            if fatal:
                self._abort(primary)
            raise primary
        if any(response is None for response in responses):
            error = self._error(
                "worker barrier returned an incomplete response set",
                sequence=self._sequence,
            )
            if fatal:
                self._abort(error)
            raise error
        return [response for response in responses if response is not None]

    def _validate_identity(
        self,
        index: int,
        response: Any,
        *,
        hashes: Mapping[str, str] | None,
    ) -> None:
        worker = self._workers[index]
        if getattr(response, "worker_id", None) != worker.worker_id:
            raise self._error(
                "worker response identity mismatch",
                index=index,
                sequence=self._sequence,
            )
        if getattr(response, "seed", None) != worker.seed:
            raise self._error(
                "worker response seed mismatch",
                index=index,
                sequence=self._sequence,
            )
        expected_pid = getattr(worker, "pid", None)
        if type(expected_pid) is not int or expected_pid <= 0:
            raise self._error(
                "worker handle PID is missing or invalid",
                index=index,
                sequence=self._sequence,
            )
        response_pid = getattr(response, "pid", None)
        if (
            type(response_pid) is not int
            or response_pid <= 0
            or response_pid != expected_pid
        ):
            raise self._error(
                "worker response PID does not identify the expected child",
                index=index,
                sequence=self._sequence,
            )
        response_sequence = getattr(response, "sequence", self._sequence)
        if response_sequence != self._sequence:
            raise self._error(
                "worker response sequence mismatch",
                index=index,
                sequence=self._sequence,
            )
        if hashes is not None and dict(getattr(response, "schema_hashes", {})) != dict(
            hashes
        ):
            raise self._error(
                "worker response schema hashes changed",
                index=index,
                sequence=self._sequence,
            )

    def handshake(self) -> DreamerProcessSchema:
        if self._schema is not None:
            return self._schema
        if self._closed:
            raise self._error("transport is closed")
        requests = [
            HelloRequest(
                **self._request_common(
                    worker,
                    ZERO_SCHEMA_HASHES,
                    episode_id=-1,
                    step_index=0,
                )
            )
            for worker in self._workers
        ]
        try:
            responses = self._exchange(
                requests,
                fatal=True,
                timeout_s=self._startup_timeout_s,
            )
            action_metadata = None
            observation_metadata = None
            schema_hashes: Mapping[str, str] | None = None
            action_projection = None
            observation_projection = None
            demo_metadata: Mapping[str, Any] | None = None
            landscaping_contract: Mapping[str, Any] | None = None
            for index, response in enumerate(responses):
                if not isinstance(response, HelloResponse):
                    raise self._error(
                        "worker returned a non-HELLO response during handshake",
                        index=index,
                        sequence=self._sequence,
                    )
                self._validate_identity(index, response, hashes=None)
                hashes = validate_schema_hashes(response.schema_hashes)
                if schema_hashes is None:
                    schema_hashes = hashes
                    action_metadata = response.action_projection
                    observation_metadata = response.observation_projection
                    action_projection = DreamerProcessProjection.from_metadata(
                        action_metadata
                    )
                    observation_projection = DreamerProcessProjection.from_metadata(
                        observation_metadata
                    )
                    if response.demo_projection is not None:
                        from .demo_projection import validate_demo_projection_descriptor

                        demo_metadata = validate_demo_projection_descriptor(
                            response.demo_projection,
                            action_projection=action_projection,
                            observation_projection=observation_projection,
                        )
                    if response.landscaping_contract is not None:
                        landscaping_contract = validate_landscaping_contract(
                            response.landscaping_contract
                        )
                    expected_hashes = schema_hashes_for(
                        action_projection.schema_hash,
                        observation_projection.schema_hash,
                    )
                    if dict(hashes) != dict(expected_hashes):
                        raise self._error(
                            "worker response schema hashes do not match its projections",
                            index=index,
                            sequence=self._sequence,
                        )
                elif (
                    dict(hashes) != dict(schema_hashes)
                    or response.action_projection != action_metadata
                    or response.observation_projection != observation_metadata
                    or response.demo_projection != demo_metadata
                    or response.landscaping_contract != landscaping_contract
                ):
                    raise self._error(
                        "workers negotiated different projections, schema hashes, "
                        "or landscaping contracts",
                        index=index,
                        sequence=self._sequence,
                    )
            if (
                schema_hashes is None
                or action_projection is None
                or observation_projection is None
            ):
                raise self._error("handshake returned no schema")
            self._schema = DreamerProcessSchema(
                action_projection=action_projection,
                observation_projection=observation_projection,
                schema_hashes=schema_hashes,
                demo_projection=demo_metadata,
                landscaping_contract=landscaping_contract,
            )
            self._sequence += 1
            return self._schema
        except DreamerProcessTransportError as exc:
            if not self._closed:
                self._abort(exc)
            raise
        except Exception as exc:
            error = self._error(f"handshake validation failed: {exc}")
            self._abort(error)
            raise error from exc

    def _ensure_ready(self) -> DreamerProcessSchema:
        if self._closed:
            raise self._error("transport is closed")
        return self.handshake()

    def _slice_action(self, value: Any, index: int) -> Any:
        if isinstance(value, Mapping):
            return {key: self._slice_action(item, index) for key, item in value.items()}
        if isinstance(value, tuple):
            return tuple(self._slice_action(item, index) for item in value)
        if isinstance(value, list):
            return [self._slice_action(item, index) for item in value]
        if isinstance(value, np.ndarray):
            if value.ndim == 0:
                return np.array(value, copy=True)
            return np.array(value[index], copy=True)
        return value

    def _requests_for(
        self, actions: Any | None, reset_mask: np.ndarray
    ) -> tuple[list[Any], tuple[str, ...]]:
        schema = self._ensure_ready()
        requests: list[Any] = []
        kinds: list[str] = []
        for index, worker in enumerate(self._workers):
            state = self._states[index]
            common = self._request_common(
                worker,
                schema.schema_hashes,
                episode_id=state.episode_id,
                step_index=state.step_index,
            )
            if bool(reset_mask[index]):
                requests.append(ResetRequest(**common))
                kinds.append("reset")
            else:
                try:
                    action = schema.action_projection.pack(
                        self._slice_action(actions, index)
                    )
                except Exception as exc:
                    raise self._error(
                        f"action does not match the negotiated projection: {exc}",
                        index=index,
                        sequence=self._sequence,
                    ) from exc
                requests.append(StepRequest(action=action, **common))
                kinds.append("step")
        return requests, tuple(kinds)

    def _transition_result(
        self, responses: Sequence[Any], kinds: Sequence[str]
    ) -> DreamerProcessBatchResult:
        schema = self._schema
        if schema is None:
            raise self._error("transition received before handshake")
        row_observations: list[Any] = []
        for index, (response, kind) in enumerate(zip(responses, kinds)):
            if not isinstance(response, TransitionResponse):
                raise self._error(
                    "worker returned a non-transition response",
                    index=index,
                    sequence=self._sequence,
                )
            self._validate_identity(index, response, hashes=schema.schema_hashes)
            if kind == "reset" and (
                not response.is_first
                or response.step_index != 0
                or response.step_delta != 0
            ):
                raise self._error(
                    "reset response has invalid first-step state",
                    index=index,
                    sequence=self._sequence,
                )
            if response.step_index < 0 or response.step_delta < 0:
                raise self._error(
                    "worker returned negative episode progress",
                    index=index,
                    sequence=self._sequence,
                )
            previous = self._states[index]
            if (
                kind == "step"
                and response.episode_id == previous.episode_id
                and response.step_index - previous.step_index != response.step_delta
            ):
                raise self._error(
                    "worker step_delta does not match step_index progress",
                    index=index,
                    sequence=self._sequence,
                )
            try:
                row_observations.append(
                    schema.observation_projection.unpack(response.observation)
                )
            except Exception as exc:
                raise self._error(
                    f"worker observation does not match the negotiated projection: {exc}",
                    index=index,
                    sequence=self._sequence,
                ) from exc
        self._states = tuple(
            DreamerProcessEpisodeState(
                worker.worker_id,
                int(response.episode_id),
                int(response.step_index),
            )
            for worker, response in zip(self._workers, responses)
        )
        return DreamerProcessBatchResult(
            observation=_transport_stack(row_observations),
            reward=np.asarray(
                [response.reward for response in responses], dtype=np.float32
            ),
            terminated=np.asarray(
                [response.terminated for response in responses], dtype=np.bool_
            ),
            truncated=np.asarray(
                [response.truncated for response in responses], dtype=np.bool_
            ),
            info=tuple(dict(response.info) for response in responses),
            is_first=np.asarray(
                [response.is_first for response in responses], dtype=np.bool_
            ),
            episode_ids=np.asarray(
                [response.episode_id for response in responses], dtype=np.int64
            ),
            step_delta=np.asarray(
                [response.step_delta for response in responses], dtype=np.int64
            ),
        )

    def reset_and_step(
        self, actions: Any, reset_mask: Any
    ) -> DreamerProcessBatchResult:
        mask = np.asarray(reset_mask, dtype=np.bool_)
        if mask.shape != (len(self._workers),):
            raise ValueError("reset_mask must have one entry per worker")
        # Invalid caller actions fail before any request is sent, so the
        # workers stay healthy and in sequence; only exchange failures abort.
        requests, kinds = self._requests_for(actions, mask)
        try:
            responses = self._exchange(requests, fatal=True)
            result = self._transition_result(responses, kinds)
            self._sequence += 1
            return result
        except DreamerProcessTransportError as exc:
            if not self._closed:
                self._abort(exc)
            raise
        except Exception as exc:
            error = self._error(f"transition validation failed: {exc}")
            self._abort(error)
            raise error from exc

    def reset(self) -> DreamerProcessBatchResult:
        return self.reset_and_step(
            None,
            np.ones(len(self._workers), dtype=np.bool_),
        )

    def step(self, actions: Any) -> DreamerProcessBatchResult:
        return self.reset_and_step(
            actions,
            np.zeros(len(self._workers), dtype=np.bool_),
        )

    def close(self) -> None:
        if self._closed:
            return
        primary: DreamerProcessTransportError | None = None
        if self._schema is not None and not self._cleanup_started:
            requests = [
                CloseRequest(
                    **self._request_common(
                        worker,
                        self._schema.schema_hashes,
                        episode_id=self._states[index].episode_id,
                        step_index=self._states[index].step_index,
                    )
                )
                for index, worker in enumerate(self._workers)
            ]
            try:
                responses = self._exchange(requests, fatal=False)
                for index, response in enumerate(responses):
                    if not isinstance(response, ClosedResponse):
                        raise self._error(
                            "worker returned a non-CLOSED response",
                            index=index,
                            sequence=self._sequence,
                        )
                    self._validate_identity(
                        index, response, hashes=self._schema.schema_hashes
                    )
                self._sequence += 1
                primary = self._post_close_exit_error()
            except DreamerProcessTransportError as exc:
                primary = exc
            except Exception as exc:
                primary = self._error(f"CLOSE barrier failed: {exc}")
        cleanup_errors = self._cleanup()
        self._closed = True
        if primary is not None:
            self._annotate_error(primary, self._cleanup_diagnostics)
            raise primary
        if cleanup_errors:
            error = cleanup_errors[0]
            self._annotate_error(error, self._cleanup_diagnostics)
            raise error


try:
    __all__
except NameError:
    __all__ = []
for _transport_name in (
    "DreamerProcessBatchResult",
    "DreamerProcessEpisodeState",
    "DreamerProcessRowEpisodeState",
):
    if _transport_name not in __all__:
        __all__.append(_transport_name)
