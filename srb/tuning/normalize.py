"""Normalize and aggregate cross-task scores for portfolio trials.

Each task's return is normalized against its random-policy floor and reference
run, then aggregated into Optuna's objective. Pure stdlib; no Optuna, torch, or
Isaac.
"""

from __future__ import annotations

import math
import statistics
from typing import Sequence

_AGGREGATORS = ("median", "mean", "min")


def normalize_score(score: float, *, floor: float, ref: float) -> float:
    """Scale *score* so ``floor`` maps to 0 and ``ref`` to 1.

    Values are not clamped. Raises :class:`ValueError` unless ``ref > floor``;
    otherwise the scale is zero or reverses the maximize direction.
    """
    span = ref - floor
    if span <= 0:
        raise ValueError(
            f"degenerate baseline: ref ({ref!r}) <= floor ({floor!r}); the "
            "reference must beat the random-policy floor"
        )
    return (score - floor) / span


def aggregate(values: Sequence[float], method: str = "median") -> float:
    """Aggregate normalized scores by ``median``, ``mean``, or ``min``.

    Raises :class:`ValueError` for empty input, unknown methods, or non-finite
    values.
    """
    vals = list(values)
    if not vals:
        raise ValueError("cannot aggregate an empty score list")
    if not all(math.isfinite(v) for v in vals):
        # A diverged segment must fail the trial upstream (never reach here); a
        # non-finite value slipping in is a bug, and silent handling is unsafe —
        # ``min([1.0, nan])`` is order-dependent in Python. Fail loudly.
        raise ValueError(f"cannot aggregate non-finite values: {vals!r}")
    if method == "median":
        return statistics.median(vals)
    if method == "mean":
        return sum(vals) / len(vals)
    if method == "min":
        return min(vals)
    raise ValueError(f"unknown aggregate method {method!r}; use one of {_AGGREGATORS}")


def rank_normalize(raw_score: float, pool: Sequence[float]) -> float:
    """Return *raw_score*'s fractional midrank among prior task scores.

    Ties share their midpoint; higher scores rank higher. An empty pool returns
    ``0.5``. Because the pool grows during a study, ranks are non-stationary;
    the frozen manifest prevents changing normalization modes mid-study.
    """
    n = len(pool)
    if n == 0:
        return 0.5
    lt = sum(1 for p in pool if p < raw_score)
    eq = sum(1 for p in pool if p == raw_score)
    return (lt + 0.5 * eq) / n


__all__ = ("normalize_score", "aggregate", "rank_normalize")
