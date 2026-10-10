"""Validation episode rollout execution."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from srb.interfaces.sim_to_real.core.task_evaluator import RealSessionAbort
from srb.interfaces.sim_to_real.validation.outcome import (
    TASK_TERMINATION_TERM_REASON,
    TIME_LIMIT_TRUNCATION_TERM_REASON,
    EpisodeOutcome,
    ValidationEpisodeOutcome,
)
from srb.interfaces.sim_to_real.validation.rollout import (
    ValidationStep,
    normalize_validation_health_event,
)
from srb.utils.failures import (
    FALLBACK_ALLOWED_CATEGORIES,
    FailureCategory,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
    classify_exception,
)
from srb.utils.validation_spec import ValidationSpec

ValidationPolicy = Callable[[Any], Any]

logger = logging.getLogger(__name__)


class ValidationAbortSignal(Protocol):
    """Abort signal used by the validation episode runner."""

    def is_set(self) -> bool: ...

    def set(self) -> None: ...


class ValidationTelemetrySink(Protocol):
    """Telemetry surface needed while executing one validation episode."""

    def log_step(
        self,
        *,
        ep: int,
        step: int,
        obs_summary: dict[str, Any],
        act: np.ndarray[Any, Any],
        reward: float,
        info: dict[str, Any],
        pre_obs_summary: dict[str, Any] | None = None,
        next_obs_summary: dict[str, Any] | None = None,
    ) -> None: ...

    def log_event(self, kind: str, event: dict[str, Any]) -> None: ...


class ValidationEpisodeRollout(Protocol):
    """Rollout Adapter surface needed by one validation episode."""

    def reset(self) -> Any: ...

    def step(self, action: Any) -> ValidationStep: ...

    def poll_health(self) -> list[dict[str, Any]]: ...

    def summarise_obs(self, obs: Any) -> dict[str, Any]: ...

    def abort_session(self, reason: str) -> None: ...

    def hold_motion(self, reason: str) -> None: ...


@dataclass(frozen=True)
class ValidationEpisodeRunner:
    """Run one validation episode through the rollout interface."""

    spec: ValidationSpec
    telemetry: ValidationTelemetrySink
    rollout: ValidationEpisodeRollout
    policy: ValidationPolicy
    abort: ValidationAbortSignal
    clock: Callable[[], float] = field(default=time.monotonic)
    _stop_attempted: bool = field(default=False, init=False, repr=False, compare=False)

    def run_episode(self, ep: int) -> EpisodeOutcome:
        if self.abort.is_set():
            return self._external_abort_outcome()
        try:
            obs = self.rollout.reset()
        except RealSessionAbort as abort:
            return self._session_abort_outcome(abort, phase="env_reset")
        except Exception as exc:  # noqa: BLE001
            external_abort = self.abort.is_set()
            if external_abort:
                self._latch_session_abort("external_abort")
            self._log_failure_event("hardware_event", phase="env_reset", exc=exc)
            if external_abort or self.abort.is_set():
                self._latch_session_abort("external_abort")
                return ValidationEpisodeOutcome.external_abort()
            return ValidationEpisodeOutcome.hardware_event()

        if self.abort.is_set():
            return self._external_abort_outcome()

        start_episode = getattr(self.policy, "start_episode", None)
        if callable(start_episode):
            try:
                start_episode()
            except Exception as exc:  # noqa: BLE001
                self._log_failure_event("policy_nan", phase="policy_start", exc=exc)
                if self.abort.is_set():
                    return self._external_abort_outcome()
                return ValidationEpisodeOutcome.policy_nan()

        if self.abort.is_set():
            return self._external_abort_outcome()

        ep_start = self.clock()
        step = 0
        outcome = ValidationEpisodeOutcome.timeout()
        while True:
            if self.abort.is_set():
                self._latch_session_abort("external_abort")
                outcome = ValidationEpisodeOutcome.external_abort()
                break
            if (self.clock() - ep_start) > self.spec.max_episode_seconds:
                outcome = ValidationEpisodeOutcome.timeout()
                break

            obs_before_step = obs
            try:
                act = self.policy(obs_before_step)
            except Exception as exc:  # noqa: BLE001
                if self.abort.is_set():
                    self._latch_session_abort("external_abort")
                    outcome = ValidationEpisodeOutcome.external_abort()
                    break
                self._log_failure_event("policy_nan", phase="policy_call", exc=exc)
                outcome = ValidationEpisodeOutcome.policy_nan()
                break
            if self.abort.is_set():
                self._latch_session_abort("external_abort")
                outcome = ValidationEpisodeOutcome.external_abort()
                break
            try:
                act_array = np.asarray(act, dtype=np.float32)
            except (TypeError, ValueError, OverflowError) as exc:
                self._log_failure_event(
                    "policy_nan",
                    phase="policy_action",
                    message=f"policy returned an invalid action: {exc}",
                )
                outcome = ValidationEpisodeOutcome.policy_nan()
                break
            if not np.all(np.isfinite(act_array)):
                self._log_failure_event(
                    "policy_nan",
                    phase="policy_action",
                    message="policy returned non-finite action",
                )
                outcome = ValidationEpisodeOutcome.policy_nan()
                break

            if self.abort.is_set():
                self._latch_session_abort("external_abort")
                outcome = ValidationEpisodeOutcome.external_abort()
                break

            try:
                step_result = self.rollout.step(act_array)
            except RealSessionAbort as abort:
                outcome = self._session_abort_outcome(abort, phase="env_step")
                break
            except Exception as exc:  # noqa: BLE001
                external_abort = self.abort.is_set()
                if external_abort:
                    self._latch_session_abort("external_abort")
                self._log_failure_event("hardware_event", phase="env_step", exc=exc)
                external_abort = external_abort or self.abort.is_set()
                if external_abort:
                    self._latch_session_abort("external_abort")
                outcome = (
                    ValidationEpisodeOutcome.external_abort()
                    if external_abort
                    else ValidationEpisodeOutcome.hardware_event()
                )
                break

            abort_after_step = self.abort.is_set()
            if abort_after_step:
                self._latch_session_abort("external_abort")

            obs = step_result.obs
            try:
                pre_obs_summary = self.rollout.summarise_obs(obs_before_step)
                next_obs_summary = self.rollout.summarise_obs(obs)
                self.telemetry.log_step(
                    ep=ep,
                    step=step,
                    # Keep obs_summary's historical post-step meaning for
                    # existing telemetry readers. Explicit pre/post snapshots
                    # make canonical export lossless, including the final
                    # terminal observation.
                    obs_summary=next_obs_summary,
                    act=act_array,
                    reward=step_result.reward,
                    info=step_result.info,
                    pre_obs_summary=pre_obs_summary,
                    next_obs_summary=next_obs_summary,
                )
            except OSError as exc:
                self._log_failure_event("disk_full", phase="telemetry_step", exc=exc)
                outcome = ValidationEpisodeOutcome.disk_full()
                break
            except Exception as exc:  # noqa: BLE001 - telemetry is an evidence seam
                self._log_failure_event(
                    "hardware_event", phase="telemetry_step", exc=exc
                )
                outcome = ValidationEpisodeOutcome.hardware_event()
                break

            if abort_after_step or self.abort.is_set():
                self._latch_session_abort("external_abort")
                outcome = ValidationEpisodeOutcome.external_abort()
                break

            try:
                health_values = self.rollout.poll_health()
                if not isinstance(health_values, list):
                    raise TypeError(
                        "validation rollout poll_health() must return a list"
                    )
            except Exception as exc:  # noqa: BLE001 - health is an evidence seam
                health_event = self._health_poll_failure_event(exc)
                health_outcome = self._log_health_event(health_event, health_event)
                if health_outcome is not None:
                    outcome = health_outcome
                    break
                outcome = ValidationEpisodeOutcome.hardware_event()
                break

            health_failed = False
            for raw_event in health_values:
                normalized_event = normalize_validation_health_event(
                    raw_event,
                    component="validation",
                )
                if normalized_event is None:
                    continue
                health_outcome = self._log_health_event(raw_event, normalized_event)
                if health_outcome is not None:
                    outcome = health_outcome
                    health_failed = True
                    break
            if health_failed:
                break

            step += 1
            if step_result.done:
                outcome = ValidationEpisodeOutcome.done(
                    step_result.info,
                    ep_start,
                    finished_at=self.clock(),
                    terminated=step_result.terminated,
                    truncated=step_result.truncated,
                )
                break

        return self._hold_after_incomplete_episode(outcome)

    def _external_abort_outcome(self) -> EpisodeOutcome:
        """Latch and safe-stop a request observed at an episode checkpoint."""

        self._latch_session_abort("external_abort")
        return ValidationEpisodeOutcome.external_abort()

    _DONE_TERM_REASONS = frozenset(
        {"success", TASK_TERMINATION_TERM_REASON, TIME_LIMIT_TRUNCATION_TERM_REASON}
    )

    def _hold_after_incomplete_episode(self, outcome: EpisodeOutcome) -> EpisodeOutcome:
        """Zero hardware after an episode ends without environment ``done``.

        Session aborts already stop hardware. Other early exits could leave
        the last command active until ``reset()``; a failed hold is fatal.
        """

        if self.abort.is_set():
            newly_observed_abort = not self._stop_attempted
            if newly_observed_abort:
                self._latch_session_abort("external_abort")
                return ValidationEpisodeOutcome.external_abort()
            if outcome.term_reason in self._DONE_TERM_REASONS:
                return ValidationEpisodeOutcome.external_abort()
            return outcome
        if outcome.term_reason in self._DONE_TERM_REASONS:
            return outcome
        try:
            self.rollout.hold_motion(outcome.term_reason)
        except Exception as exc:  # noqa: BLE001 - escalate to a latched stop
            self._log_failure_event("hardware_event", phase="motion_hold", exc=exc)
            self._latch_session_abort("motion_hold_failed")
            return ValidationEpisodeOutcome.hardware_event()
        return outcome

    def _log_health_event(
        self, raw_event: object, event: HealthEvent
    ) -> EpisodeOutcome | None:
        """Log one canonical event without allowing evidence failures to pass."""
        fatal = self._health_event_is_fatal(event)
        if fatal:
            self._latch_session_abort(event.code)
        # A schema-less dict without a failure is logged as given. The rollout
        # adapter emits canonical mappings; failure-bearing or malformed
        # events use the canonical deterministic mapping below.
        if (
            isinstance(raw_event, dict)
            and "schema" not in raw_event
            and "schema_id" not in raw_event
            and event.failure is None
        ):
            payload = dict(raw_event)
        else:
            payload = event.to_mapping()
        try:
            self.telemetry.log_event(event.code, payload)
        except OSError as exc:
            logger.warning(
                "failed to log validation health event %s: %s", event.code, exc
            )
            return ValidationEpisodeOutcome.disk_full()
        except Exception as exc:  # noqa: BLE001 - logging cannot become success
            logger.warning(
                "failed to log validation health event %s: %s", event.code, exc
            )
            return ValidationEpisodeOutcome.hardware_event()
        if fatal:
            return ValidationEpisodeOutcome.hardware_event()
        return None

    @staticmethod
    def _health_event_is_fatal(event: HealthEvent) -> bool:
        if event.status is HealthStatus.UNHEALTHY:
            return True
        return event.failure is not None and (
            not event.failure.fallback_allowed
            or event.failure.category not in FALLBACK_ALLOWED_CATEGORIES
        )

    @staticmethod
    def _health_poll_failure_event(exc: Exception) -> HealthEvent:
        component = "validation"
        code = "validation.health_poll_exception"
        details = {
            "exception_type": type(exc).__name__,
            "error": str(exc),
        }
        failure = classify_exception(
            exc,
            code=code,
            component=component,
            details=details,
        )
        return HealthEvent(
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

    def _session_abort_outcome(
        self, abort: RealSessionAbort, *, phase: str
    ) -> EpisodeOutcome:
        """Record a typed out-of-MDP session abort with its own outcome."""

        self._latch_session_abort(abort.outcome)
        self._log_failure_event(abort.outcome, phase=phase, exc=abort)
        return ValidationEpisodeOutcome.session_abort(abort)

    def _latch_session_abort(self, reason: str) -> None:
        """Set the shared latch, then request one idempotent environment stop."""

        if not self.abort.is_set():
            self.abort.set()
        if self._stop_attempted:
            return
        object.__setattr__(self, "_stop_attempted", True)
        try:
            self.rollout.abort_session(reason)
        except Exception as exc:  # noqa: BLE001 - preserve the initiating outcome
            logger.error(
                "failed to safe-stop validation rollout after %s: %s: %s",
                reason,
                type(exc).__name__,
                exc,
            )

    def _log_failure_event(
        self,
        kind: str,
        *,
        phase: str,
        exc: Exception | None = None,
        message: str | None = None,
    ) -> None:
        failure_message = str(exc) if exc is not None else str(message or "")
        details = {
            "kind": kind,
            "phase": phase,
            "error": failure_message,
        }
        if exc is not None:
            details["exception_type"] = type(exc).__name__
            cause = exc
        else:
            cause = RuntimeError(failure_message or f"validation failure at {phase}")
        failure = classify_exception(
            cause,
            code=f"validation.episode_runner.{phase}",
            component="validation.episode_runner",
            details=details,
            evidence_valid=False,
        )
        event = HealthEvent(
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
        try:
            # Keep the historical outer event kind/outcome while making the
            # payload canonical and preserving any typed failure verbatim.
            self.telemetry.log_event(kind, event.to_mapping())
        except Exception as log_exc:  # noqa: BLE001
            logger.warning(
                "failed to log validation failure event %s: %s",
                kind,
                log_exc,
            )
