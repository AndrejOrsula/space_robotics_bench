"""Validation criterion policies for sim-to-real session results."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from math import isfinite
from numbers import Real
from types import MappingProxyType
from typing import Any, Literal

from srb.interfaces.sim_to_real.validation.drift import z_score
from srb.interfaces.sim_to_real.validation.numeric import (
    NonFiniteValidationError,
    ensure_finite,
)
from srb.utils.failures import IntegrityFailureError
from srb.utils.json_types import ImmutableValueMixin
from srb.utils.validation_contract import (
    EXTERNAL_ABORT_TERM_REASON,
    SESSION_ABORT_OUTCOMES,
)
from srb.utils.validation_spec import ValidationSpec


@dataclass(frozen=True, slots=True)
class CriterionResult(ImmutableValueMixin):
    status: Literal["pass", "fail", "skipped"]
    detail: str
    breakdown: Mapping[str, str | float] = field(default_factory=dict)
    optional: bool = False
    score: float | None = None

    def __post_init__(self) -> None:
        if type(self.status) is not str:
            raise TypeError("criterion result status must be a string")
        if self.status not in {"pass", "fail", "skipped"}:
            raise ValueError(f"criterion result status is unsupported: {self.status!r}")
        if type(self.detail) is not str:
            raise TypeError("criterion result detail must be a string")
        if type(self.optional) is not bool:
            raise TypeError("criterion result optional must be a boolean")
        if self.score is not None:
            if isinstance(self.score, bool) or not isinstance(self.score, Real):
                raise TypeError(
                    "criterion result score must be None or a finite real number"
                )
            try:
                score = float(self.score)
            except (OverflowError, ValueError, TypeError) as exc:
                raise ValueError(
                    "criterion result score must be None or a finite real number"
                ) from exc
            if not isfinite(score):
                raise ValueError(
                    "criterion result score must be None or a finite real number"
                )
            object.__setattr__(self, "score", score)
        if not isinstance(self.breakdown, Mapping):
            raise TypeError("criterion result breakdown must be a mapping")

        breakdown: dict[str, str | float] = {}
        for key, value in self.breakdown.items():
            if type(key) is not str:
                raise TypeError("criterion result breakdown keys must be strings")
            if type(value) is str:
                breakdown[key] = value
                continue
            if isinstance(value, bool) or not isinstance(value, Real):
                raise TypeError(
                    "criterion result breakdown values must be strings "
                    "or finite real numbers"
                )
            try:
                finite = isfinite(float(value))
            except (OverflowError, ValueError, TypeError) as exc:
                raise ValueError(
                    "criterion result breakdown values must be strings "
                    "or finite real numbers"
                ) from exc
            if not finite:
                raise ValueError(
                    "criterion result breakdown values must be strings "
                    "or finite real numbers"
                )
            breakdown[key] = float(value)
        object.__setattr__(self, "breakdown", MappingProxyType(breakdown))


#: Metrics graded only on the real session (absolute floor), never compared
#: with the sim baseline; a baseline is not required to carry them.
REAL_ONLY_METRICS = frozenset({"success_rate"})

# Metric-name tokens that mark a lower-is-better (error/loss) metric. Matched
# against underscore/hyphen-split tokens, so only whole-token hits count (avoids
# e.g. "frames" matching "mae"). Operators override or correct the classification
# explicitly via thresholds.lower_is_better_metrics.
_LOWER_IS_BETTER_TOKENS = frozenset(
    {"error", "err", "loss", "rmse", "mae", "mse", "deviation", "residual"}
)


def _metric_is_lower_is_better(spec: ValidationSpec, metric: str) -> bool:
    """True if smaller real values of ``metric`` are better (error-like)."""
    if metric in spec.thresholds.lower_is_better_metrics:
        return True
    tokens = metric.lower().replace("-", "_").split("_")
    return any(tok in _LOWER_IS_BETTER_TOKENS for tok in tokens)


def _session_completeness(
    spec: ValidationSpec, real: dict[str, float]
) -> tuple[list[str], dict[str, float | str]]:
    """Return failures and breakdown when session completeness is known.

    Fewer completed than requested episodes or any session abort is a
    failure. Without ``n_episodes``, completeness cannot be judged.
    """

    if "n_episodes" not in real:
        return [], {}
    attempted = real["n_episodes"]
    aborts = {
        outcome: real.get(f"{outcome}_count", 0.0) for outcome in SESSION_ABORT_OUTCOMES
    }
    typed_total = max(real.get("session_abort_count", 0.0), sum(aborts.values()))
    external = real.get(f"{EXTERNAL_ABORT_TERM_REASON}_count", 0.0)
    aborts[EXTERNAL_ABORT_TERM_REASON] = external
    abort_total = typed_total + external
    completed = max(attempted - abort_total, 0.0)
    requested = float(spec.n_episodes)
    if completed >= requested and abort_total <= 0.0:
        return [], {}
    reasons = ", ".join(
        f"{outcome}={int(count)}" for outcome, count in aborts.items() if count > 0.0
    )
    line = f"incomplete session: {int(completed)}/{int(requested)} episodes completed"
    if reasons:
        line += f"; session aborts: {reasons}"
    return [line], {
        "completed_episodes": completed,
        "requested_episodes": requested,
        "session_abort_count": typed_total,
        "external_abort_count": external,
    }


def session_is_incomplete(spec: ValidationSpec, real: Mapping[str, float]) -> bool:
    """Whether ``real`` shows too few completed episodes or a session abort."""

    lines, _breakdown = _session_completeness(spec, dict(real))
    return bool(lines)


def _ratio_band_check(
    spec: ValidationSpec, metric: str, real: float, sim: float, ratio: float
) -> tuple[bool, str | None, dict[str, float | str]]:
    """Check a metric against its direction-aware sim/real band.

    Returns ``(failed, detail_line, breakdown_entries)``. Positive baselines
    retain the existing ratio rule; non-positive baselines use value bands.
    """

    ratio_min = spec.thresholds.sim_real_ratio_min
    smooth_max = spec.thresholds.action_smoothness_max_ratio
    if metric == "action_smoothness":
        kind, factor = "max", smooth_max
    elif _metric_is_lower_is_better(spec, metric):
        kind, factor = "max", 1.0 / ratio_min
    else:
        kind, factor = "min", ratio_min
    if sim > 0.0:
        r = ratio
        failed = r > factor if kind == "max" else r < factor
        if not failed:
            return False, None, {}
        if metric == "action_smoothness":
            return True, f"smoothness ratio {r:.3f} > max {smooth_max}", {}
        if kind == "max":
            return (
                True,
                f"{metric} ratio {r:.3f} > max {factor:.3f} (lower-is-better)",
                {},
            )
        return True, f"{metric} ratio {r:.3f} < min {ratio_min}", {}
    # Negative baseline: the ratio real/sim flips direction, so judge the real
    # value against a band of |sim| around the baseline instead.
    bound = sim + (factor - 1.0) * abs(sim)
    failed = real > bound if kind == "max" else real < bound
    entries: dict[str, float | str] = {f"bound_{metric}": bound}
    if not failed:
        return False, None, entries
    comparator = ">" if kind == "max" else "<"
    return (
        True,
        f"{metric} {real:.3f} {comparator} {kind} {bound:.3f} "
        f"(negative sim baseline {sim:.3f}, |sim| band)",
        entries,
    )


def evaluate_threshold(
    spec: ValidationSpec,
    real: dict[str, float],
    sim: dict[str, float] | None,
) -> tuple[CriterionResult, dict[str, float] | None]:
    """Evaluate the success-rate floor and sim/real ratio thresholds.

    The success-rate floor is checked without a baseline; only ratio checks
    are skipped when ``sim`` is ``None``.
    """

    try:
        ensure_finite(real, context="real validation metrics")
        if sim is not None:
            ensure_finite(sim, context="sim baseline metrics")
        ensure_finite(
            {
                "success_rate_min": spec.thresholds.success_rate_min,
                "sim_real_ratio_min": spec.thresholds.sim_real_ratio_min,
                "action_smoothness_max_ratio": spec.thresholds.action_smoothness_max_ratio,
            },
            context="validation thresholds",
        )
    except NonFiniteValidationError as exc:
        return CriterionResult(status="fail", detail=str(exc)), None

    if sim is not None and getattr(sim, "degraded", False):
        health_event = getattr(sim, "health_event", None)
        health_code = getattr(health_event, "code", "unknown")
        return (
            CriterionResult(
                status="fail",
                detail=(
                    "configured sim baseline is degraded; comparison is not "
                    f"publishable ({health_code})"
                ),
            ),
            None,
        )

    incomplete_lines, incomplete_breakdown = _session_completeness(spec, real)

    if sim is not None:
        compared_metrics = {
            metric
            for metric in spec.metrics_to_track
            if metric not in REAL_ONLY_METRICS
        }
        required_real = {*compared_metrics, "success_rate"}
        missing_real = sorted(required_real.difference(real))
        missing_sim = sorted(compared_metrics.difference(sim))
        missing_details: list[str] = []
        if missing_real:
            missing_details.append(
                f"missing required real metrics: {', '.join(missing_real)}"
            )
        if missing_sim:
            missing_details.append(
                f"missing required sim metrics: {', '.join(missing_sim)}"
            )
        if missing_details:
            return (
                CriterionResult(
                    status="fail",
                    detail="; ".join([*incomplete_lines, *missing_details]),
                    breakdown=incomplete_breakdown,
                ),
                None,
            )

    if sim is None and "success_rate" not in real and not incomplete_lines:
        return (
            CriterionResult(status="skipped", detail="no sim baseline available"),
            None,
        )
    detail_lines: list[str] = list(incomplete_lines)
    breakdown: dict[str, float | str] = dict(incomplete_breakdown)
    ratio: dict[str, float] = {}
    failed = bool(incomplete_lines)
    if "success_rate" in real:
        sr = real["success_rate"]
        breakdown["success_rate"] = sr
        if sr < spec.thresholds.success_rate_min:
            failed = True
            detail_lines.append(
                f"success_rate {sr:.3f} < min {spec.thresholds.success_rate_min}"
            )
    if sim is None:
        return (
            CriterionResult(
                status="fail" if failed else "pass",
                detail="; ".join(
                    [
                        *detail_lines,
                        "sim/real ratios skipped: no sim baseline available",
                    ]
                ),
                breakdown=breakdown,
            ),
            None,
        )
    for metric in spec.metrics_to_track:
        if metric in REAL_ONLY_METRICS:
            continue
        if metric not in real or metric not in sim:
            continue
        if sim[metric] == 0:
            # A zero baseline leaves the relative band undefined. Never skip
            # it silently: a perfect 0.0 sim error would wave any real error.
            failed = True
            breakdown[f"ratio_{metric}"] = "undefined: zero sim baseline"
            detail_lines.append(
                f"{metric}: zero sim baseline; relative band undefined "
                f"(real {real[metric]:.3f})"
            )
            continue
        r = real[metric] / sim[metric]
        try:
            ensure_finite(r, context=f"ratio_{metric}")
        except NonFiniteValidationError as exc:
            return CriterionResult(status="fail", detail=str(exc)), None
        ratio[metric] = r
        breakdown[f"ratio_{metric}"] = r
        metric_failed, line, entries = _ratio_band_check(
            spec, metric, real[metric], sim[metric], r
        )
        breakdown.update(entries)
        if metric_failed:
            failed = True
            if line is not None:
                detail_lines.append(line)
    return (
        CriterionResult(
            status="fail" if failed else "pass",
            detail="; ".join(detail_lines) or "all thresholds satisfied",
            breakdown=breakdown,
        ),
        ratio,
    )


def evaluate_crash(spec: ValidationSpec, real: dict[str, float]) -> CriterionResult:
    try:
        ensure_finite(real, context="real validation metrics")
        ensure_finite(
            {"crash_rate_max": spec.thresholds.crash_rate_max},
            context="validation thresholds",
        )
    except NonFiniteValidationError as exc:
        return CriterionResult(status="fail", detail=str(exc))
    if "crash_count" not in real:
        return CriterionResult(
            status="fail",
            detail="missing required real metrics: crash_count",
        )
    n = real.get("n_episodes", 0.0)
    crash = real["crash_count"]
    if n <= 0.0:
        return CriterionResult(
            status="fail",
            detail="no completed validation episodes",
            breakdown={"crash_count": crash, "n_episodes": n},
        )
    if isinstance(crash, bool) or crash < 0 or float(crash) != int(crash):
        return CriterionResult(
            status="fail",
            detail=f"crash_count must be a non-negative integer, got {crash!r}",
            breakdown={"crash_count": crash, "n_episodes": n},
        )
    crash_rate = crash / n if n else 0.0
    try:
        ensure_finite(crash_rate, context="crash rate")
    except NonFiniteValidationError as exc:
        return CriterionResult(status="fail", detail=str(exc))
    if crash_rate > spec.thresholds.crash_rate_max:
        return CriterionResult(
            status="fail",
            detail=f"crash_rate {crash_rate:.3f} > max {spec.thresholds.crash_rate_max}",
            breakdown={"crash_count": crash, "n_episodes": n, "crash_rate": crash_rate},
        )
    return CriterionResult(
        status="pass",
        detail=f"no crashes ({int(crash)}/{int(n)})",
        breakdown={"crash_count": crash, "n_episodes": n},
    )


def evaluate_drift(
    spec: ValidationSpec,
    real: dict[str, float],
    history: list[dict[str, Any]],
) -> tuple[CriterionResult, dict[str, float] | None]:
    _validate_drift_history(history)
    try:
        ensure_finite(real, context="real validation metrics")
        ensure_finite(history, context="validation drift history")
        ensure_finite(
            {"drift_z_max": spec.thresholds.drift_z_max},
            context="validation thresholds",
        )
    except NonFiniteValidationError as exc:
        return CriterionResult(status="fail", detail=str(exc)), None
    if not history:
        return (
            CriterionResult(
                status="skipped",
                detail="optional: no drift history configured",
                optional=True,
            ),
            None,
        )
    if len(history) < 2:
        # One row has no spread, so no z-score exists. This is the same
        # "not enough evidence yet" state as an empty history: optional, never
        # a drift failure (a drift_window below 2 therefore never evaluates).
        return (
            CriterionResult(
                status="skipped",
                detail=(
                    "optional: insufficient drift history "
                    f"({len(history)} row; need at least 2)"
                ),
                optional=True,
            ),
            None,
        )
    z = z_score(real, history, spec.metrics_to_track)
    try:
        ensure_finite(z, context="validation drift z-scores")
    except NonFiniteValidationError as exc:
        return CriterionResult(status="fail", detail=str(exc)), None
    if not z:
        return (
            CriterionResult(
                status="skipped", detail="no comparable metrics", optional=True
            ),
            None,
        )
    failed = [k for k, v in z.items() if abs(v) > spec.thresholds.drift_z_max]
    detail = (
        f"drift z={z}; threshold={spec.thresholds.drift_z_max}"
        if not failed
        else f"metrics over drift threshold: {failed}"
    )
    return (
        CriterionResult(
            status="fail" if failed else "pass",
            detail=detail,
            breakdown={k: v for k, v in z.items()},
        ),
        z,
    )


def _validate_drift_history(history: list[dict[str, Any]]) -> None:
    """Reject malformed history instead of letting z-score skip it."""

    for index, row in enumerate(history):
        if not isinstance(row, Mapping):
            raise IntegrityFailureError(
                code="validation.drift_history.integrity",
                message=f"drift history row {index} must be an object",
                component="validation.criteria",
                details={"row": index},
                evidence_valid=False,
            )
        metrics = row.get("metrics")
        if not isinstance(metrics, Mapping):
            raise IntegrityFailureError(
                code="validation.drift_history.integrity",
                message=f"drift history row {index} has no metrics object",
                component="validation.criteria",
                details={"row": index},
                evidence_valid=False,
            )
        for key, value in metrics.items():
            if type(key) is not str or type(value) not in (int, float):
                raise IntegrityFailureError(
                    code="validation.drift_history.integrity",
                    message=(f"drift history row {index} contains a malformed metric"),
                    component="validation.criteria",
                    details={"row": index, "metric": str(key)},
                    evidence_valid=False,
                )
            if not isfinite(float(value)):
                raise IntegrityFailureError(
                    code="validation.drift_history.integrity",
                    message=(f"drift history row {index} contains a non-finite metric"),
                    component="validation.criteria",
                    details={"row": index, "metric": str(key)},
                    evidence_valid=False,
                )
