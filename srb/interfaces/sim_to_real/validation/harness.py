"""Validation harness: orchestrates pre-flight, rollout, post-session phases."""

from __future__ import annotations

import logging
from typing import Any, Callable

import numpy as np

from srb.interfaces._ros_node import RosCleanupError
from srb.interfaces.sim_to_real.validation.drift import DriftStore
from srb.interfaces.sim_to_real.validation.episode_runner import (
    ValidationEpisodeRunner,
)
from srb.interfaces.sim_to_real.validation.outcome import EpisodeOutcome
from srb.interfaces.sim_to_real.validation.result import SessionResult
from srb.interfaces.sim_to_real.validation.runtime_session import (
    ValidationRuntimeSession,
)
from srb.interfaces.sim_to_real.validation.session_finalizer import (
    ValidationSessionFinalizer,
)
from srb.interfaces.sim_to_real.validation.session_workspace import (
    build_session_workspace,
)
from srb.interfaces.sim_to_real.validation.telemetry import Telemetry
from srb.utils.validation_spec import ValidationSpec

logger = logging.getLogger(__name__)


class ValidationHarness:
    """Run episodes with telemetry, comparison, drift tracking, and reporting.

    Construction resolves session identity and workspace paths. ``run()``
    performs preflight, rollout, and post-session work, then returns a
    ``SessionResult``.
    """

    def __init__(
        self,
        spec: ValidationSpec,
        *,
        env_factory: Callable[[ValidationSpec], Any] | None = None,
        policy_factory: Callable[..., Callable[[Any], np.ndarray]] | None = None,
        wandb_enabled: bool = True,
        tensorboard_enabled: bool = True,
        rerun_web_enabled: bool = True,
        post_status: bool = False,
        dry_run: bool = False,
    ) -> None:
        self.spec = spec
        workspace = build_session_workspace(spec)
        self.session_dir = workspace.session_dir
        self.abort_request_path = workspace.session_dir / ".abort"
        self.drift_store = DriftStore(workspace.storage_root)
        self._finalizer = ValidationSessionFinalizer(
            spec=spec,
            drift_store=self.drift_store,
            session_dir=self.session_dir,
            git_sha=workspace.git_sha,
            timestamp=workspace.timestamp,
            post_status=post_status,
            dry_run=dry_run,
            staging_dir=workspace.staging_dir,
            identity=workspace.identity,
        )
        self._runtime = ValidationRuntimeSession(
            spec=spec,
            session_dir=workspace.staging_dir,
            abort_request_path=self.abort_request_path,
            env_factory=env_factory,
            policy_factory=policy_factory,
            wandb_enabled=wandb_enabled,
            tensorboard_enabled=tensorboard_enabled,
            rerun_web_enabled=rerun_web_enabled,
            dry_run=dry_run,
            identity=workspace.identity,
        )
        self._runtime_cleanup: Callable[[], Any] = self._runtime.close
        self._runtime_cleanup_owner: Any | None = self._runtime
        self._has_run = False

    def bind_runtime_cleanup(
        self, closer: Callable[[], Any]
    ) -> ValidationRuntimeSession:
        """Bind the process owner as the cleanup entry point.

        Standalone harnesses retain runtime ownership. A CLI owner can bind
        before ``run`` and adopt the runtime to preserve environment/ROS order.
        """
        if self._has_run:
            raise RuntimeError("runtime cleanup must be bound before run")
        if not callable(closer):
            raise TypeError("runtime cleanup closer must be callable")
        self._runtime_cleanup = closer
        self._runtime_cleanup_owner = getattr(closer, "__self__", None)
        return self._runtime

    @property
    def telemetry(self) -> Telemetry | None:
        return self._runtime.telemetry

    def run(self) -> SessionResult:
        self._has_run = True
        try:
            try:
                self._preflight()
            except Exception as exc:  # noqa: BLE001 - preflight failures are results
                try:
                    cleanup_error = self._close_after_primary_error(
                        exc, phase="preflight"
                    )
                except BaseException as cleanup_cancellation:
                    if not isinstance(cleanup_cancellation, Exception):
                        cleanup_cancellation.add_note(
                            "validation preflight also failed: "
                            f"{type(exc).__name__}: {exc}"
                        )
                    raise
                exc = _combine_run_errors(exc, cleanup_error)
                try:
                    result = self._preflight_failure(exc)
                except Exception as finalize_error:
                    # Without spec.yaml, publish failure cannot replace the
                    # preflight root cause.
                    finalize_error.add_note(
                        "validation preflight failed first: "
                        f"{type(exc).__name__}: {exc}"
                    )
                    raise
                self._adopt_published_session_dir()
                return result
            try:
                cleanup_error, evaluated = self._rollout_and_evaluate()
            except Exception as exc:  # noqa: BLE001 - invalid evidence is a result
                # Rollout/evaluation failures are invalid evidence, not
                # threshold verdicts. Publish a non-hardware result (exit 4)
                # so CI gets a report instead of a traceback (exit 1).
                cleanup_error = self._close_after_primary_error(exc, phase="rollout")
                result = self._finalizer.finalize_invalid(
                    _combine_run_errors(exc, cleanup_error)
                )
            else:
                result = (
                    self._finalizer.finalize_invalid(cleanup_error)
                    if cleanup_error is not None or evaluated is None
                    else self._post_session(evaluated)
                )
            self._adopt_published_session_dir()
            return result
        finally:
            self._adopt_published_session_dir()
            self._close_error()

    # ----- Phase 1 -----------------------------------------------------------

    def _preflight(self) -> None:
        self._runtime.start()

    # ----- Phase 2 -----------------------------------------------------------

    def _rollout(self) -> None:
        if not self._runtime.can_rollout:
            return  # dry-run path: no rollout
        for ep in range(self.spec.n_episodes):
            self._rollout_one_episode(ep)
            if self._runtime.abort.is_set():
                break

    def _rollout_one_episode(self, ep: int) -> None:
        runner = ValidationEpisodeRunner(
            spec=self.spec,
            telemetry=self._runtime.require_telemetry(),
            rollout=self._runtime.require_rollout(),
            policy=self._runtime.require_policy(),
            abort=self._runtime.abort,
        )
        self._log_episode_outcome(ep, runner.run_episode(ep))

    def _log_episode_outcome(self, ep: int, outcome: EpisodeOutcome) -> None:
        self._runtime.require_telemetry().log_episode_end(
            ep=ep,
            success=outcome.success,
            term_reason=outcome.term_reason,
            ep_metrics=outcome.ep_metrics,
        )

    # ----- Phase 3 -----------------------------------------------------------

    def _rollout_and_evaluate(self) -> tuple[Exception | None, SessionResult | None]:
        """Roll out, close telemetry, then evaluate the session.

        Returns ``(cleanup_error, evaluated)`` and skips evaluation on cleanup
        failure. Publication stays separate to avoid contradicting partial
        results with a second publication.
        """

        self._rollout()
        cleanup_error = self._close_error()
        if cleanup_error is not None:
            return cleanup_error, None
        return None, self._finalizer.evaluate()

    def _post_session(self, evaluated: SessionResult) -> SessionResult:
        return self._finalizer.publish(evaluated)

    def _preflight_failure(self, exc: Exception) -> SessionResult:
        return self._finalizer.finalize_preflight_failure(exc)

    def _adopt_published_session_dir(self) -> None:
        if self._finalizer.published_session_dir is not None:
            self.session_dir = self._finalizer.published_session_dir

    # ----- always-run cleanup ------------------------------------------------

    def _close_all(self) -> None:
        self._runtime_cleanup()

    def _close_error(self) -> Exception | None:
        try:
            self._close_all()
        except Exception as exc:  # noqa: BLE001 - returned to finalization
            cancellation = _find_cleanup_cancellation(exc)
            if cancellation is not None:
                signal, owner, failures = cancellation
                signal.cleanup_owner = owner
                signal.cleanup_errors = failures
                signal.cleanup_error = exc
                raise signal from None
            return exc
        return None

    def _close_after_primary_error(
        self, primary: Exception, *, phase: str
    ) -> Exception | None:
        cancellation = _find_cleanup_cancellation(primary)
        if cancellation is None:
            return self._close_error()

        cleanup_error: BaseException | None = None
        try:
            cleanup_error = self._close_error()
        except BaseException as exc:
            cleanup_error = exc

        signal, nested_owner, failures = cancellation
        owner = self._runtime_cleanup_owner
        if owner is None:
            owner = nested_owner if nested_owner is not None else self._runtime
        signal.cleanup_owner = owner
        if cleanup_error is signal:
            retry_error = getattr(signal, "cleanup_error", None)
            additional_failures = tuple(getattr(signal, "cleanup_errors", ()))
        else:
            retry_error = cleanup_error
            additional_failures = (
                _flatten_cleanup_failures(cleanup_error, signal)
                if cleanup_error is not None
                else ()
            )
        all_failures = (
            *getattr(signal, "cleanup_errors", ()),
            *failures,
            *additional_failures,
        )
        unique_failures: list[BaseException] = []
        for failure in all_failures:
            if not any(failure is seen for seen in unique_failures):
                unique_failures.append(failure)
        signal.cleanup_errors = tuple(unique_failures)
        signal.cleanup_error = primary
        if retry_error is not None:
            signal.cleanup_retry_error = retry_error
        signal.add_note(
            f"validation {phase} also failed: {type(primary).__name__}: {primary}"
        )
        raise signal from None


def _combine_run_errors(
    primary: Exception,
    cleanup: Exception | None,
) -> Exception:
    if cleanup is None:
        return primary
    return ExceptionGroup("validation run failed during cleanup", [primary, cleanup])


def _find_cleanup_cancellation(
    error: BaseException,
) -> tuple[BaseException, Any | None, tuple[BaseException, ...]] | None:
    """Find cancellation retained by typed cleanup aggregates and their failures."""
    if not isinstance(error, (RosCleanupError, BaseExceptionGroup)):
        return (error, None, ()) if not isinstance(error, Exception) else None

    nested_errors = (
        error.errors if isinstance(error, RosCleanupError) else error.exceptions
    )
    for nested in nested_errors:
        cancellation = _find_cleanup_cancellation(nested)
        if cancellation is None:
            continue
        signal, owner, failures = cancellation
        if isinstance(error, RosCleanupError):
            owner = error.cleanup_owner if error.cleanup_owner is not None else owner
            failures = tuple(
                failure
                for child in error.errors
                for failure in _flatten_cleanup_failures(child, signal)
            )
        return signal, owner, failures
    return None


def _flatten_cleanup_failures(
    error: BaseException, cancellation: BaseException
) -> tuple[BaseException, ...]:
    if error is cancellation:
        return ()
    if isinstance(error, RosCleanupError):
        return tuple(
            failure
            for nested in error.errors
            for failure in _flatten_cleanup_failures(nested, cancellation)
        )
    if isinstance(error, BaseExceptionGroup):
        return tuple(
            failure
            for nested in error.exceptions
            for failure in _flatten_cleanup_failures(nested, cancellation)
        )
    return (error,)
