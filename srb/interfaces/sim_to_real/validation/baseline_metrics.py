"""Strict simulation-baseline resolution for validation comparisons."""

from __future__ import annotations

import json
import logging
from collections.abc import Mapping
from math import isfinite
from pathlib import Path

from srb.interfaces.sim_to_real.validation.criteria import REAL_ONLY_METRICS
from srb.utils.failures import (
    FailureCategory,
    FailureError,
    HealthEvent,
    IntegrityFailureError,
    OptionalDependencyUnavailableError,
    TransientExternalFailureError,
    UnsupportedCapabilityError,
    make_degraded_health_event,
)
from srb.utils.validation_spec import ValidationSpec

logger = logging.getLogger(__name__)


class BaselineMetrics(dict[str, float]):
    """Metric mapping with baseline resolution state.

    ``None`` means no baseline was configured. ``degraded=True`` means a
    configured source was unavailable and an allowed fallback was used;
    ``health_event`` records that fallback.
    """

    configured: bool
    degraded: bool
    source: str
    health_event: HealthEvent | None

    def __init__(
        self,
        metrics: Mapping[str, float],
        *,
        source: str,
        degraded: bool = False,
        health_event: HealthEvent | None = None,
    ) -> None:
        super().__init__(metrics)
        self.configured = True
        self.degraded = degraded
        self.source = source
        self.health_event = health_event
        if degraded and health_event is None:
            raise ValueError("degraded baseline metrics require a health event")


def load_baseline_metrics(spec: ValidationSpec) -> BaselineMetrics | None:
    """Resolve the configured simulation baseline.

    Only an absent spec reference returns ``None``. Missing configured files
    and allowed optional or network failures return degraded metrics with a
    :class:`HealthEvent`. Malformed data and untyped provider failures raise.
    """

    ref = spec.sim_baseline
    if not (ref.metrics_json or ref.wandb_run_id or ref.local_rrd):
        return None

    fallback_event: HealthEvent | None = None
    if ref.metrics_json:
        try:
            metrics = _read_metrics_json(Path(ref.metrics_json))
            _require_tracked_metrics(
                metrics, spec.metrics_to_track, source="metrics_json"
            )
            return BaselineMetrics(metrics, source="metrics_json")
        except (
            OptionalDependencyUnavailableError,
            TransientExternalFailureError,
        ) as exc:
            fallback_event = _fallback_event(exc, source="metrics_json")

    if ref.wandb_run_id:
        try:
            metrics = _read_wandb_summary(ref.wandb_run_id)
            _require_tracked_metrics(metrics, spec.metrics_to_track, source="wandb")
        except (
            OptionalDependencyUnavailableError,
            TransientExternalFailureError,
        ) as exc:
            fallback_event = _fallback_event(exc, source="wandb")
        else:
            return BaselineMetrics(
                metrics,
                source="wandb",
                degraded=fallback_event is not None,
                health_event=fallback_event,
            )

    if ref.local_rrd:
        try:
            raise UnsupportedCapabilityError(
                code="validation.baseline.local_rrd.unsupported",
                message=(
                    "local_rrd baselines are unsupported; use "
                    "sim_baseline.metrics_json or sim_baseline.wandb_run_id"
                ),
                component="validation.baseline",
                details={"path": ref.local_rrd},
                fallback_allowed=True,
            )
        except UnsupportedCapabilityError as exc:
            fallback_event = _fallback_event(exc, source="local_rrd")

    if fallback_event is None:
        raise IntegrityFailureError(
            code="validation.baseline.unresolved",
            message="configured simulation baseline could not be resolved",
            component="validation.baseline",
            details={"task": spec.task, "algo": spec.algo},
            evidence_valid=False,
        )
    logger.warning(
        "baseline fallback: %s (%s)",
        fallback_event.code,
        fallback_event.message,
    )
    return BaselineMetrics(
        {},
        source="fallback",
        degraded=True,
        health_event=fallback_event,
    )


def _read_metrics_json(path: Path) -> dict[str, float]:
    if not path.is_file():
        if path.exists():
            raise _integrity_failure(
                f"configured baseline path is not a regular file: {path}"
            )
        raise TransientExternalFailureError(
            code="validation.baseline.metrics_json.unavailable",
            message=f"configured baseline file is unavailable: {path}",
            component="validation.baseline",
            details={"path": str(path)},
            retryable=True,
            fallback_allowed=True,
        )
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError) as exc:
        raise _integrity_failure(
            f"could not read configured baseline file: {path}", cause=exc
        ) from exc
    except json.JSONDecodeError as exc:
        raise _integrity_failure(
            f"configured baseline is not valid JSON: {path}", cause=exc
        ) from exc
    return _strict_metrics(payload, source=str(path))


def _read_wandb_summary(run_id: str) -> dict[str, float]:
    try:
        import wandb
    except ModuleNotFoundError as exc:
        raise OptionalDependencyUnavailableError(
            code="validation.baseline.wandb.missing",
            message="wandb is not installed",
            component="validation.baseline",
            details={"run_id": run_id},
            fallback_allowed=True,
        ) from exc

    from wandb.errors import AuthenticationError, CommError, UsageError

    try:
        run = wandb.Api().run(run_id)
    except (AuthenticationError, UsageError) as exc:
        raise _integrity_failure(
            f"W&B baseline request is invalid for {run_id}", cause=exc
        ) from exc
    except CommError as exc:
        raise TransientExternalFailureError(
            code="validation.baseline.wandb.transient",
            message=f"W&B baseline fetch failed for {run_id}: {exc}",
            component="validation.baseline",
            details={"run_id": run_id},
            retryable=True,
            fallback_allowed=True,
        ) from exc
    except (OptionalDependencyUnavailableError, TransientExternalFailureError):
        raise
    except (ConnectionError, TimeoutError) as exc:
        raise TransientExternalFailureError(
            code="validation.baseline.wandb.transient",
            message=f"W&B baseline fetch failed for {run_id}: {exc}",
            component="validation.baseline",
            details={"run_id": run_id},
            retryable=True,
            fallback_allowed=True,
        ) from exc
    summary = getattr(run, "summary", None)
    return _strict_metrics(summary, source=f"wandb:{run_id}")


def _strict_metrics(value: object, *, source: str) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise _integrity_failure(f"baseline metrics must be an object: {source}")
    if not value:
        raise _integrity_failure(f"baseline metrics are empty: {source}")
    metrics: dict[str, float] = {}
    for key, item in sorted(value.items(), key=lambda pair: _metric_sort_key(pair[0])):
        if type(key) is not str or not key:
            raise _integrity_failure(
                f"baseline metric names must be non-empty strings: {source}"
            )
        if type(item) not in (int, float):
            raise _integrity_failure(
                f"baseline metric {key!r} must be an exact int or float: {source}"
            )
        try:
            numeric = float(item)
        except (OverflowError, TypeError, ValueError) as exc:
            raise _integrity_failure(
                f"baseline metric {key!r} is not finite: {source}", cause=exc
            ) from exc
        if not isfinite(numeric):
            raise _integrity_failure(f"baseline metric {key!r} is non-finite: {source}")
        metrics[key] = numeric
    return metrics


def _require_tracked_metrics(
    metrics: Mapping[str, float],
    tracked_names: list[str],
    *,
    source: str,
) -> None:
    """Require each tracked metric after source normalization.

    Real-only metrics such as ``success_rate`` need no baseline. Normalize
    first so malformed present values take precedence over missing names.
    """

    missing = sorted(
        {
            name
            for name in tracked_names
            if name not in metrics and name not in REAL_ONLY_METRICS
        }
    )
    if not missing:
        return
    raise IntegrityFailureError(
        code="validation.baseline.integrity",
        message=(
            "baseline metrics are missing required tracked metrics: "
            + ", ".join(missing)
        ),
        component="validation.baseline",
        details={
            "phase": "metrics_completeness",
            "source": source,
            "missing_metrics": missing,
        },
        evidence_valid=False,
    )


def _metric_sort_key(key: object) -> tuple[int, str, str]:
    if type(key) is str:
        return (0, "", key)
    key_type = type(key)
    return (1, f"{key_type.__module__}.{key_type.__qualname__}", repr(key))


def _fallback_event(exc: FailureError, *, source: str) -> HealthEvent:
    if (
        exc.info.category
        not in {
            FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
            FailureCategory.TRANSIENT_EXTERNAL_FAILURE,
            FailureCategory.UNSUPPORTED_CAPABILITY,
        }
        or not exc.info.fallback_allowed
    ):
        raise exc
    event = make_degraded_health_event(
        exc.info,
        component="validation.baseline",
        code="validation.baseline.fallback",
        message=f"configured baseline source unavailable; fallback applied: {source}",
        details={"source": source},
    )
    return event


def _integrity_failure(
    message: str,
    *,
    cause: BaseException | None = None,
) -> IntegrityFailureError:
    details: dict[str, object] = {}
    if cause is not None:
        details["cause"] = type(cause).__name__
    return IntegrityFailureError(
        code="validation.baseline.integrity",
        message=message,
        component="validation.baseline",
        details=details,
        evidence_valid=False,
    )


__all__ = [
    "BaselineMetrics",
    "load_baseline_metrics",
]
