"""Episode outcome policy for validation rollouts."""

from __future__ import annotations

import time
from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite
from typing import Any, NoReturn

import numpy as np

from srb.interfaces.sim_to_real.core.task_evaluator import (
    RealSessionAbort,
)
from srb.interfaces.sim_to_real.validation.numeric import ensure_finite
from srb.utils import validation_contract as _validation_contract

# Keep the historical private spelling available to callers that inspected
# the module while making the utility contract canonical.
CRASH_TERM_REASONS = _validation_contract.CRASH_TERM_REASONS
SESSION_ABORT_TERM_REASONS = _validation_contract.SESSION_ABORT_TERM_REASONS
TASK_TERMINATION_TERM_REASON = _validation_contract.TASK_TERMINATION_TERM_REASON
TIME_LIMIT_TRUNCATION_TERM_REASON = (
    _validation_contract.TIME_LIMIT_TRUNCATION_TERM_REASON
)
_VALID_TERM_REASONS = _validation_contract.VALID_TERM_REASONS

# These names were historically imported from the task-evaluator module here.
# Preserve their exact object identity for direct consumers.
SESSION_ABORT_OUTCOMES = _validation_contract.SESSION_ABORT_OUTCOMES
SESSION_OUTCOME_CALIBRATION_MISMATCH = (
    _validation_contract.SESSION_OUTCOME_CALIBRATION_MISMATCH
)
SESSION_OUTCOME_MANIFEST_MISMATCH = (
    _validation_contract.SESSION_OUTCOME_MANIFEST_MISMATCH
)
SESSION_OUTCOME_OPERATOR_ABORT = _validation_contract.SESSION_OUTCOME_OPERATOR_ABORT
SESSION_OUTCOME_SENSOR_INCOMPLETE = (
    _validation_contract.SESSION_OUTCOME_SENSOR_INCOMPLETE
)
SESSION_OUTCOME_SENSOR_STALE = _validation_contract.SESSION_OUTCOME_SENSOR_STALE
TIME_LIMIT_TRUNCATION = _validation_contract.TIME_LIMIT_TRUNCATION
NON_CRASH_SESSION_ABORT_TERM_REASONS = (
    _validation_contract.NON_CRASH_SESSION_ABORT_TERM_REASONS
)


class _ImmutableMetrics(dict[str, float]):
    """Dict-shaped snapshot that keeps the telemetry seam type-compatible."""

    def __init__(self, values: Mapping[str, float]) -> None:
        dict.__init__(self, values)

    def __setitem__(self, key: str, value: float) -> NoReturn:
        raise TypeError("episode outcome metrics are immutable")

    def __delitem__(self, key: str) -> NoReturn:
        raise TypeError("episode outcome metrics are immutable")

    def clear(self) -> NoReturn:
        raise TypeError("episode outcome metrics are immutable")

    def pop(self, *args: object) -> NoReturn:
        raise TypeError("episode outcome metrics are immutable")

    def popitem(self) -> NoReturn:
        raise TypeError("episode outcome metrics are immutable")

    def setdefault(self, *args: object) -> NoReturn:
        raise TypeError("episode outcome metrics are immutable")

    def update(self, *args: object, **kwargs: object) -> NoReturn:
        raise TypeError("episode outcome metrics are immutable")

    def __ior__(self, other: object) -> NoReturn:  # type: ignore[misc]
        raise TypeError("episode outcome metrics are immutable")


@dataclass(frozen=True, slots=True)
class EpisodeOutcome:
    """Terminal result recorded for one validation episode."""

    success: bool
    term_reason: str
    ep_metrics: Mapping[str, float]

    def __post_init__(self) -> None:
        if type(self.success) is not bool:
            raise TypeError("episode outcome success must be a boolean")
        if type(self.term_reason) is not str:
            raise TypeError("episode outcome term_reason must be a string")
        if self.term_reason not in _VALID_TERM_REASONS:
            raise ValueError(
                f"episode outcome term_reason is unsupported: {self.term_reason!r}"
            )
        object.__setattr__(self, "ep_metrics", _freeze_metrics(self.ep_metrics))


class ValidationEpisodeOutcome:
    """Builds validation episode outcomes from rollout termination causes."""

    @staticmethod
    def hardware_event() -> EpisodeOutcome:
        return _failure("hardware_event")

    @staticmethod
    def policy_nan() -> EpisodeOutcome:
        return _failure("policy_nan")

    @staticmethod
    def disk_full() -> EpisodeOutcome:
        return _failure("disk_full")

    @staticmethod
    def external_abort() -> EpisodeOutcome:
        return _failure("external_abort")

    @staticmethod
    def timeout() -> EpisodeOutcome:
        """Harness wall-clock budget expired — not the env's time limit."""

        return _failure("timeout")

    @staticmethod
    def session_abort(abort: RealSessionAbort) -> EpisodeOutcome:
        """Non-success outcome for a typed out-of-MDP session abort."""

        return _failure(abort.outcome)

    @staticmethod
    def done(
        info: Mapping[str, object] | None,
        started_at: float,
        *,
        finished_at: float | None = None,
        terminated: bool = False,
        truncated: bool = False,
    ) -> EpisodeOutcome:
        """Build an environment-ended outcome; unsuccessful horizon endings are truncations."""

        success = _as_bool((info or {}).get("success", False))
        if success:
            term_reason = "success"
        elif terminated:
            term_reason = TASK_TERMINATION_TERM_REASON
        elif truncated:
            term_reason = TIME_LIMIT_TRUNCATION_TERM_REASON
        else:
            term_reason = "timeout"
        return EpisodeOutcome(
            success=success,
            term_reason=term_reason,
            ep_metrics=episode_metrics(info, started_at, finished_at=finished_at),
        )


def episode_metrics(
    info: Mapping[str, object] | None,
    started_at: float,
    *,
    finished_at: float | None = None,
) -> dict[str, float]:
    """Extract numeric episode metrics from terminal step info."""

    ended_at = time.monotonic() if finished_at is None else finished_at
    out: dict[str, float] = {"wall_seconds": ended_at - started_at}
    ensure_finite(out["wall_seconds"], context="episode metric wall_seconds")
    if info:
        metric_info = dict(info)
        nested_metrics = metric_info.pop("episode_metrics", None)
        nested_metric_keys: set[str] = set()
        if isinstance(nested_metrics, Mapping):
            nested_metric_keys.update(nested_metrics)
            metric_info.update(nested_metrics)
        for key, value in metric_info.items():
            if not isinstance(key, str):
                raise TypeError("episode metric keys must be strings")
            # ``bool`` is an ``int`` subclass, but arbitrary informational
            # flags are not numeric episode metrics.  Keep only the historic
            # success flag for compatibility with existing reports.
            if isinstance(value, (bool, np.bool_)):
                if key == "success":
                    out[key] = float(bool(value))
                    ensure_finite(out[key], context=f"episode metric {key}")
                continue
            if isinstance(value, (int, float)):
                try:
                    out[key] = float(value)
                except (OverflowError, ValueError) as exc:
                    raise ValueError(
                        f"episode metric {key!r} must be a finite number"
                    ) from exc
                ensure_finite(out[key], context=f"episode metric {key}")
            elif key in nested_metric_keys:
                array = np.asarray(_to_numpy_like(value))
                if array.size == 1 and array.dtype.kind in "iuf":
                    out[key] = float(array.reshape(-1)[0])
                    ensure_finite(out[key], context=f"episode metric {key}")
    return _ImmutableMetrics(out)


def is_crash_term_reason(term_reason: str) -> bool:
    return term_reason in CRASH_TERM_REASONS


def is_session_abort_term_reason(term_reason: str) -> bool:
    """True for the five out-of-MDP session outcomes."""

    return term_reason in SESSION_ABORT_TERM_REASONS


def _failure(term_reason: str) -> EpisodeOutcome:
    return EpisodeOutcome(success=False, term_reason=term_reason, ep_metrics={})


def _freeze_metrics(metrics: Mapping[str, object]) -> Mapping[str, float]:
    if not isinstance(metrics, Mapping):
        raise TypeError("episode outcome ep_metrics must be a mapping")
    frozen: dict[str, float] = {}
    for key, value in metrics.items():
        if type(key) is not str:
            raise TypeError("episode outcome metric keys must be strings")
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float)):
            raise TypeError(f"episode outcome metric {key!r} must be a finite number")
        try:
            numeric = float(value)
        except (OverflowError, ValueError) as exc:
            raise ValueError(
                f"episode outcome metric {key!r} must be a finite number"
            ) from exc
        if not isfinite(numeric):
            raise ValueError(f"episode outcome metric {key!r} must be a finite number")
        frozen[key] = numeric
    return _ImmutableMetrics(frozen)


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    normalized = _to_numpy_like(value)
    array = np.asarray(normalized)
    if array.dtype.kind in "fc" and not np.all(np.isfinite(array)):
        return False
    if array.shape == ():
        return bool(array.item())
    if array.size == 1:
        return bool(array.reshape(-1)[0])
    return bool(np.any(array))


def _to_numpy_like(value: object) -> object:
    raw_value: Any = value
    if hasattr(raw_value, "detach"):
        raw_value = raw_value.detach()
    if hasattr(raw_value, "cpu"):
        raw_value = raw_value.cpu()
    if hasattr(raw_value, "numpy"):
        try:
            return raw_value.numpy()
        except TypeError:
            return raw_value
    return raw_value
