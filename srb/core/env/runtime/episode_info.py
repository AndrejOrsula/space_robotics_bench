from __future__ import annotations

from collections.abc import Mapping
from typing import Any

EPISODE_SUCCESS_KEY = "success"
EPISODE_METRICS_KEY = "episode_metrics"


def episode_outcome_info(
    *,
    success: Any,
    metrics: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build task episode metadata consumed by validation and recording adapters."""
    info: dict[str, Any] = {EPISODE_SUCCESS_KEY: success}
    if metrics:
        info[EPISODE_METRICS_KEY] = dict(metrics)
    return info


__all__ = [
    "EPISODE_METRICS_KEY",
    "EPISODE_SUCCESS_KEY",
    "episode_outcome_info",
]
