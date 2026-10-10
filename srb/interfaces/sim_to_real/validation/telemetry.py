"""Multi-sink telemetry fan-out for the validation harness."""

from __future__ import annotations

import json
import logging
import os
import threading
from collections import deque
from collections.abc import Mapping
from pathlib import Path
from typing import IO, Any, cast

import numpy as np

from srb.interfaces.sim_to_real.validation.artifact_reference import (
    ROLLOUT_RRD,
    TENSORBOARD_DIR,
)
from srb.interfaces.sim_to_real.validation.session_workspace import (
    ValidationSessionIdentity,
)
from srb.utils.failures import (
    FALLBACK_ALLOWED_CATEGORIES,
    FailureCategory,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
    attempt_cleanup,
    classify_exception,
    make_degraded_health_event,
)
from srb.utils.publication import ArtifactPublication, PublicationCleanupError
from srb.utils.validation_session_log import (
    build_episode_end_record,
    build_event_record,
    build_step_record,
    validation_log_path,
    write_validation_record,
)
from srb.utils.validation_spec import ValidationSpec

logger = logging.getLogger(__name__)

try:
    import rerun as rr

    _RERUN_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only on bare envs
    rr = None
    _RERUN_AVAILABLE = False

try:
    import wandb

    _WANDB_AVAILABLE = True
except ImportError:  # pragma: no cover
    wandb = None
    _WANDB_AVAILABLE = False

SummaryWriter: Any | None = None
_TENSORBOARD_AVAILABLE: bool | None = None
_MAX_HEALTH_EVENTS = 64
_JSONL_FSYNC_RECORDS = 32


def _write_fd(fd: int, payload: bytes) -> int:
    return os.write(fd, payload)


def _fsync_fd(fd: int) -> None:
    os.fsync(fd)


def _close_fd(fd: int) -> None:
    os.close(fd)


class _PublicationJsonlWriter:
    """Append session records and publish once after close-time fsync."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        self._fd: int | None = os.open(self.path, flags, 0o600)
        self._records_since_fsync = 0
        self._close_synced = False
        self._closing = False
        self._publication_pending = False
        self._publication_cleanup_owner: Any | None = None
        self._publication_committed = False
        self._invalid_partial_record = False

    def write(self, payload: str) -> int:
        if self._fd is None or self._closing:
            raise ValueError("validation JSONL writer is closing or closed")
        encoded = payload.encode("utf-8")
        written = 0
        while written < len(encoded):
            try:
                count = _write_fd(self._fd, encoded[written:])
            except BaseException:
                self._invalid_partial_record = True
                raise
            if count <= 0:
                self._invalid_partial_record = True
                raise OSError("validation JSONL write made no progress")
            written += count
        self._records_since_fsync += 1
        if self._records_since_fsync >= _JSONL_FSYNC_RECORDS:
            _fsync_fd(self._fd)
            self._records_since_fsync = 0
        return len(payload)

    def flush(self) -> None:
        """The unbuffered descriptor makes completed writes immediately visible."""

    def close(self) -> None:
        self._closing = True
        if self._fd is not None:
            if not self._close_synced:
                _fsync_fd(self._fd)
                self._close_synced = True
                self._records_since_fsync = 0
            _close_fd(self._fd)
            self._fd = None
            self._publication_pending = True
        if self._invalid_partial_record:
            raise OSError(
                "cannot publish incomplete validation JSONL record after partial write"
            )
        if self._publication_cleanup_owner is not None:
            self._publication_cleanup_owner.retry_cleanup()
            self._publication_cleanup_owner = None
            if self._publication_committed:
                self._publication_pending = False
            self._publication_committed = False
        if self._publication_pending:
            try:
                ArtifactPublication.file(self.path).append_bytes(b"")
            except PublicationCleanupError as exc:
                self._publication_cleanup_owner = exc.owner
                self._publication_committed = any(
                    Path(path).absolute() == self.path.absolute()
                    for path in exc.committed_paths
                )
                if self._publication_committed:
                    self._publication_pending = False
                raise
            else:
                self._publication_pending = False


def _load_summary_writer() -> Any | None:
    """Load PyTorch TensorBoard support only when the sink is started."""

    global SummaryWriter, _TENSORBOARD_AVAILABLE
    if SummaryWriter is not None:
        _TENSORBOARD_AVAILABLE = True
        return SummaryWriter
    if _TENSORBOARD_AVAILABLE is False:
        return None
    try:
        from torch.utils.tensorboard import (
            SummaryWriter as _SummaryWriter,
        )
    except ImportError:  # pragma: no cover
        _TENSORBOARD_AVAILABLE = False
        return None
    SummaryWriter = _SummaryWriter
    _TENSORBOARD_AVAILABLE = True
    return SummaryWriter


class Telemetry:
    """Fan-out logger: rerun.io + JSONL + optional scalar dashboards."""

    def __init__(
        self,
        spec: ValidationSpec,
        session_dir: Path,
        *,
        wandb_enabled: bool = True,
        tensorboard_enabled: bool = True,
        rerun_web_enabled: bool = True,
        identity: ValidationSessionIdentity | None = None,
    ) -> None:
        self.spec = spec
        self.session_dir = Path(session_dir)
        self.identity = identity
        self._wandb_requested = bool(wandb_enabled)
        self._wandb_enabled = self._wandb_requested and _WANDB_AVAILABLE
        self._tensorboard_enabled = bool(tensorboard_enabled)
        self._rerun_web_enabled = rerun_web_enabled
        self._jsonl_fh: IO[str] | None = None
        self._rerun_started = False
        self._rerun_step_logging_enabled = False
        self._rerun_step_index = 0
        self._n_drops = 0
        self._health_events: deque[HealthEvent] = deque(maxlen=_MAX_HEALTH_EVENTS)
        self._wandb_run: Any | None = None
        self._tb_writer: Any | None = None
        self._rerun_viewer_host: Any | None = None
        self._close_lock = threading.Lock()

    @property
    def n_drops(self) -> int:
        return self._n_drops

    @property
    def health_events(self) -> tuple[HealthEvent, ...]:
        """Return the bounded in-memory view of observed sink health events."""
        return tuple(self._health_events)

    def start(self) -> None:
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self._jsonl_fh = cast(
            IO[str], _PublicationJsonlWriter(validation_log_path(self.session_dir))
        )
        if self.spec.rerun.record_to_rrd:
            if _RERUN_AVAILABLE:
                self._start_rerun()
            else:
                self._record_optional_failure(
                    "rerun",
                    "unavailable",
                    ImportError("rerun is not installed"),
                    category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
                )
        if self._wandb_requested:
            if self._wandb_enabled:
                self._start_wandb()
            else:
                self._record_optional_failure(
                    "wandb",
                    "unavailable",
                    ImportError("wandb is not installed"),
                    category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
                )
        if self._tensorboard_enabled:
            self._start_tensorboard()

    def _start_rerun(self) -> None:
        try:
            rr.init(
                application_id=self.spec.rerun.app_id,
                recording_id=(
                    self.identity.session_id
                    if self.identity is not None
                    else self.session_dir.name
                ),
            )
            # rr.init owns global SDK state. Only its successful return gives
            # this Telemetry instance a cleanup marker; the public API exposes
            # no per-recording handle if init itself raises.
            self._rerun_started = True
            self._rerun_step_logging_enabled = True
            rrd_path = str(self.session_dir / ROLLOUT_RRD)
            served_with_own_sinks = False
            if self._rerun_web_enabled:
                # API varies by rerun version; try each in order.
                try:
                    rr.serve_web(
                        web_port=self.spec.rerun.web_port,
                        grpc_port=self.spec.rerun.grpc_port,
                    )
                except AttributeError:
                    try:
                        rr.serve(
                            web_port=self.spec.rerun.web_port,
                            grpc_port=self.spec.rerun.grpc_port,
                        )
                    except AttributeError:
                        served_with_own_sinks = self._serve_rerun_viewer(rrd_path)
            if not served_with_own_sinks:
                rr.save(rrd_path)
        except Exception as exc:  # noqa: BLE001 - classify at the sink seam
            logger.warning("rerun start failed: %s", exc)
            self._record_optional_failure("rerun", "start_failed", exc)

    def _serve_rerun_viewer(self, rrd_path: str) -> bool:
        """Host the viewer on SDKs with ``serve_grpc`` only.

        A dedicated stream keeps the gRPC server alive while the session stream
        writes both viewer and file sinks. Return True when these sinks are
        installed, so the caller skips ``rr.save``.
        """
        recording_stream = getattr(rr, "RecordingStream", None)
        serve_web_viewer = getattr(rr, "serve_web_viewer", None)
        set_sinks = getattr(rr, "set_sinks", None)
        grpc_sink = getattr(rr, "GrpcSink", None)
        file_sink = getattr(rr, "FileSink", None)
        if recording_stream is None or not hasattr(recording_stream, "serve_grpc"):
            return False
        if serve_web_viewer is None or set_sinks is None:
            return False
        if grpc_sink is None or file_sink is None:
            return False

        host = recording_stream(
            application_id=f"{self.spec.rerun.app_id}-viewer",
            recording_id=(
                self.identity.session_id
                if self.identity is not None
                else self.session_dir.name
            ),
        )
        self._rerun_viewer_host = host
        try:
            uri = host.serve_grpc(grpc_port=self.spec.rerun.grpc_port)
            serve_web_viewer(
                web_port=self.spec.rerun.web_port,
                open_browser=False,
                connect_to=uri,
            )
        except OSError as exc:
            # Preserve the ``.rrd`` file sink when the viewer port is occupied.
            logger.warning("rerun viewer could not be hosted: %s", exc)
            self._record_optional_failure("rerun", "viewer_host_failed", exc)
            return False
        set_sinks(grpc_sink(url=uri), file_sink(rrd_path))
        # Keep the stream alive so its server remains available.
        return True

    def _start_wandb(self) -> None:
        try:
            self._wandb_run = wandb.init(
                project=f"srb-validation-{self.spec.task}",
                name=(
                    self.identity.session_id
                    if self.identity is not None
                    else self.session_dir.name
                ),
                config={
                    "task": self.spec.task,
                    "algo": self.spec.algo,
                    "checkpoint": self.spec.checkpoint,
                    "hardware": self.spec.hardware,
                    "session_identity": (
                        self.identity.to_dict() if self.identity is not None else None
                    ),
                },
                reinit=True,
                mode="online",
            )
        except Exception as exc:  # noqa: BLE001 - classify at the sink seam
            logger.warning("wandb init failed: %s", exc)
            self._wandb_run = None
            self._record_optional_failure("wandb", "init_failed", exc)

    def _start_tensorboard(self) -> None:
        try:
            writer_cls = _load_summary_writer()
            if writer_cls is None:
                self._tensorboard_enabled = False
                self._record_optional_failure(
                    "tensorboard",
                    "unavailable",
                    ImportError("TensorBoard is not installed"),
                    category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
                )
                return
            self._tb_writer = writer_cls(str(self.session_dir / TENSORBOARD_DIR))
            self._tb_writer.add_text(
                "srb/validation_spec",
                json.dumps(
                    {
                        "task": self.spec.task,
                        "algo": self.spec.algo,
                        "checkpoint": self.spec.checkpoint,
                        "hardware": self.spec.hardware,
                        "session_identity": (
                            self.identity.to_dict()
                            if self.identity is not None
                            else None
                        ),
                    },
                    sort_keys=True,
                    allow_nan=False,
                ),
                0,
            )
        except Exception as exc:  # noqa: BLE001 - classify at the sink seam
            logger.warning("tensorboard init failed: %s", exc)
            self._tensorboard_enabled = False
            self._record_optional_failure("tensorboard", "init_failed", exc)

    def log_step(
        self,
        *,
        ep: int,
        step: int,
        obs_summary: dict[str, Any],
        act: np.ndarray,
        reward: float,
        info: dict[str, Any],
        pre_obs_summary: dict[str, Any] | None = None,
        next_obs_summary: dict[str, Any] | None = None,
    ) -> None:
        # JSONL is required (any failure is a crash event).
        rec = build_step_record(
            ep=ep,
            step=step,
            obs_summary=obs_summary,
            act=act,
            reward=reward,
            info=info,
            pre_obs_summary=pre_obs_summary,
            next_obs_summary=next_obs_summary,
        )
        assert self._jsonl_fh is not None, "Telemetry.start() not called"
        self._write_required_record(rec, operation="step")
        # rerun: best-effort.
        if self._rerun_step_logging_enabled:
            try:
                rr.set_time("real", sequence=self._rerun_step_index)
                # rr.Scalar API varies by version; try newer Scalars first.
                act_norm = float(np.linalg.norm(act))
                try:
                    rr.log("real/act/cmd", rr.Scalar(act_norm))
                    rr.log("real/rew", rr.Scalar(float(reward)))
                except (AttributeError, TypeError):
                    rr.log("real/act/cmd", rr.Scalars([act_norm]))
                    rr.log("real/rew", rr.Scalars([float(reward)]))
                self._rerun_step_index += 1
            except Exception as exc:  # noqa: BLE001 - classify at the sink seam
                logger.warning("rerun step logging disabled after API failure: %s", exc)
                self._rerun_step_logging_enabled = False
                self._record_optional_failure("rerun", "step_failed", exc)

    def log_episode_end(
        self,
        *,
        ep: int,
        success: bool,
        term_reason: str,
        ep_metrics: Mapping[str, float],
    ) -> None:
        rec = build_episode_end_record(
            ep=ep,
            success=success,
            term_reason=term_reason,
            ep_metrics=ep_metrics,
        )
        assert self._jsonl_fh is not None
        self._write_required_record(rec, operation="episode_end")
        if self._wandb_run is not None:
            try:
                self._wandb_run.log({f"ep_{ep}/{k}": v for k, v in ep_metrics.items()})
            except Exception as exc:  # noqa: BLE001 - classify at the sink seam
                logger.debug("wandb log dropped: %s", exc)
                self._record_optional_failure("wandb", "log_failed", exc)
        if self._tb_writer is not None:
            try:
                self._tb_writer.add_scalar("episode/success", float(success), ep)
                for key, value in ep_metrics.items():
                    self._tb_writer.add_scalar(f"episode/{key}", float(value), ep)
            except Exception as exc:  # noqa: BLE001 - classify at the sink seam
                logger.debug("tensorboard log dropped: %s", exc)
                self._record_optional_failure("tensorboard", "log_failed", exc)

    def log_event(self, kind: str, payload: dict[str, Any]) -> None:
        rec = build_event_record(kind, payload)
        assert self._jsonl_fh is not None
        self._write_required_record(rec, operation="event")

    def close(self) -> None:
        if not self._close_lock.acquire(blocking=False):
            raise RuntimeError("validation telemetry cleanup is already in progress")
        errors: list[BaseException] = []
        try:
            # The hosted server and rerun's active stream have independent owners.
            host = self._rerun_viewer_host
            if host is not None:
                _, error = attempt_cleanup(host.disconnect)
                if error is None:
                    self._rerun_viewer_host = None
                else:
                    self._collect_optional_close_error(
                        errors, "rerun", "disconnect_failed", error
                    )

            if self._rerun_started:
                _, error = attempt_cleanup(rr.disconnect)
                if error is None:
                    self._rerun_started = False
                else:
                    self._collect_optional_close_error(
                        errors, "rerun", "disconnect_failed", error
                    )

            if self._wandb_run is not None:
                run = self._wandb_run
                _, error = attempt_cleanup(run.finish)
                if error is None:
                    self._wandb_run = None
                else:
                    self._collect_optional_close_error(
                        errors, "wandb", "finish_failed", error
                    )

            if self._tb_writer is not None:
                writer = self._tb_writer
                _, error = attempt_cleanup(writer.close)
                if error is None:
                    self._tb_writer = None
                else:
                    self._collect_optional_close_error(
                        errors, "tensorboard", "close_failed", error
                    )

            if self._jsonl_fh is not None:
                handle = self._jsonl_fh
                _, error = attempt_cleanup(handle.close)
                if error is None:
                    self._jsonl_fh = None
                else:
                    errors.append(error)
        finally:
            self._close_lock.release()

        if not errors:
            return
        if len(errors) == 1:
            raise errors[0]
        # Preserve cancellation types and every independent cleanup failure.
        raise BaseExceptionGroup("validation telemetry cleanup failed", errors)

    def _collect_optional_close_error(
        self,
        errors: list[BaseException],
        sink: str,
        operation: str,
        error: BaseException,
    ) -> None:
        """Record ordinary sink failures and retain every cleanup error."""
        logger.warning("%s %s failed: %s", sink, operation, error)
        if isinstance(error, Exception):
            try:
                self._record_optional_failure(sink, operation, error)
            except BaseException as report_error:  # noqa: BLE001
                errors.append(report_error)
                if not self._contains_exception(report_error, error):
                    errors.append(error)
                return
        errors.append(error)

    @staticmethod
    def _contains_exception(container: BaseException, target: BaseException) -> bool:
        if container is target:
            return True
        if isinstance(container, BaseExceptionGroup):
            return any(
                Telemetry._contains_exception(nested, target)
                for nested in container.exceptions
            )
        return False

    def _record_optional_failure(
        self,
        sink: str,
        operation: str,
        exc: Exception,
        *,
        category: FailureCategory = FailureCategory.TRANSIENT_EXTERNAL_FAILURE,
    ) -> None:
        """Classify, retain, and persist one optional-sink failure.

        A plain exception at an optional sink keeps the historical transient
        drop behavior.  A typed failure is authoritative: only its explicitly
        fallback-allowed category can degrade.  All other failures are written
        as unhealthy evidence and then re-raised.
        """
        self._n_drops += 1
        code = f"telemetry.{sink}.{operation}"
        message = f"optional telemetry sink {sink} {operation}"
        details = {
            "sink": sink,
            "operation": operation,
            "exception_type": type(exc).__name__,
            "error": str(exc),
        }
        failure = classify_exception(
            exc,
            category=category,
            code=code,
            component=f"validation.telemetry.{sink}",
            details=details,
            retryable=category is FailureCategory.TRANSIENT_EXTERNAL_FAILURE,
            fallback_allowed=category in FALLBACK_ALLOWED_CATEGORIES,
            evidence_valid=False,
        )
        fallback_applied = (
            failure.fallback_allowed and failure.category in FALLBACK_ALLOWED_CATEGORIES
        )
        if fallback_applied:
            event = make_degraded_health_event(
                failure,
                component="validation.telemetry",
                # A typed failure's code is part of its caller contract.
                code=failure.code,
                message=f"{message}; JSONL evidence remains required",
                details={"sink": sink, "operation": operation},
            )
        else:
            event = HealthEvent(
                status=HealthStatus.UNHEALTHY,
                severity=(
                    HealthSeverity.CRITICAL
                    if failure.category is FailureCategory.SAFETY_VIOLATION
                    else HealthSeverity.ERROR
                ),
                component="validation.telemetry",
                code=failure.code,
                message=failure.message,
                details={
                    "sink": sink,
                    "operation": operation,
                    "fallback_applied": False,
                },
                failure=failure,
            )
        self._health_events.append(event)
        try:
            self._persist_health_event(event)
        except Exception as persistence_exc:
            if fallback_applied:
                raise
            raise ExceptionGroup(
                "telemetry sink and health evidence persistence failed",
                [exc, persistence_exc],
            ) from persistence_exc
        if not fallback_applied:
            raise exc

    def _write_required_record(
        self, record: Mapping[str, Any], *, operation: str
    ) -> None:
        """Write required evidence and retain one non-recursive failure event."""
        assert self._jsonl_fh is not None
        try:
            write_validation_record(self._jsonl_fh, record)
        except Exception as exc:  # noqa: BLE001 - required evidence boundary
            self._record_required_sink_failure(operation, exc)
            raise

    def _record_required_sink_failure(self, operation: str, exc: Exception) -> None:
        """Retain failed required-sink health without writing to that sink."""
        self._n_drops += 1
        details = {
            "sink": "jsonl",
            "operation": operation,
            "exception_type": type(exc).__name__,
            "error": str(exc),
            "persisted": False,
            "fallback_applied": False,
        }
        failure = classify_exception(
            exc,
            code=f"telemetry.jsonl.{operation}_failed",
            component="validation.telemetry.jsonl",
            details=details,
            evidence_valid=False,
        )
        self._health_events.append(
            HealthEvent(
                status=HealthStatus.UNHEALTHY,
                severity=(
                    HealthSeverity.CRITICAL
                    if failure.category is FailureCategory.SAFETY_VIOLATION
                    else HealthSeverity.ERROR
                ),
                component=failure.component,
                code=failure.code,
                message=failure.message,
                details=details,
                failure=failure,
            )
        )

    def _persist_health_event(self, event: HealthEvent) -> None:
        """Write required sink-health evidence without recursive reporting."""
        if self._jsonl_fh is None:
            exc = RuntimeError(
                f"telemetry health event could not be persisted: {event.code}"
            )
            self._record_required_sink_failure("health_event", exc)
            logger.error("%s", exc)
            raise exc
        try:
            record = build_event_record("health", event.to_mapping())
            self._write_required_record(record, operation="health_event")
        except Exception as exc:  # noqa: BLE001 - avoid recursive health reporting
            logger.error(
                "telemetry health event persistence failed for %s: %s",
                event.code,
                exc,
            )
            raise
