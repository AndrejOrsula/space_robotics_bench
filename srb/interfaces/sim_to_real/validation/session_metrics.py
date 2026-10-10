"""Session metric aggregation for validation telemetry."""

from __future__ import annotations

from pathlib import Path

from srb.interfaces.sim_to_real.core.task_evaluator import SESSION_ABORT_OUTCOMES
from srb.interfaces.sim_to_real.validation.numeric import ensure_finite
from srb.interfaces.sim_to_real.validation.outcome import (
    TASK_TERMINATION_TERM_REASON,
    TIME_LIMIT_TRUNCATION_TERM_REASON,
    is_crash_term_reason,
    is_session_abort_term_reason,
)
from srb.utils.validation_contract import EXTERNAL_ABORT_TERM_REASON
from srb.utils.validation_session_log import iter_episode_end_records

#: Always-reported episode counts explain how the session ended.
#: Include task termination even when current landscaping tasks do not use it.
OUTCOME_COUNT_KEYS: tuple[str, ...] = (
    *(f"{outcome}_count" for outcome in SESSION_ABORT_OUTCOMES),
    f"{TIME_LIMIT_TRUNCATION_TERM_REASON}_count",
    f"{TASK_TERMINATION_TERM_REASON}_count",
    f"{EXTERNAL_ABORT_TERM_REASON}_count",
    "session_abort_count",
)

# Environment metrics cannot shadow these session aggregates. Success rates
# and outcome counts come from `success` and `term_reason`, respectively.
_RESERVED_METRIC_KEYS = frozenset(
    {"success_rate", "n_episodes", "crash_count", *OUTCOME_COUNT_KEYS}
)


def aggregate_session_metrics(session_dir: Path) -> dict[str, float]:
    """Collapse a validation session's episode records into metrics."""

    n_episodes = 0
    n_successes = 0
    metric_sums: dict[str, float] = {}
    metric_counts: dict[str, int] = {}
    crash_count = 0
    outcome_counts: dict[str, int] = {key: 0 for key in OUTCOME_COUNT_KEYS}
    for rec in iter_episode_end_records(session_dir):
        ensure_finite(rec, context="validation episode record")
        success = rec["success"]
        term_reason = rec["term_reason"]
        ep_metrics = rec["ep_metrics"]
        if type(success) is not bool:
            raise TypeError("type(success) is bool is false")
        if type(term_reason) is not str:
            raise TypeError("type(term_reason) is str is false")
        assert isinstance(ep_metrics, dict)
        n_episodes += 1
        n_successes += success
        if is_crash_term_reason(term_reason):
            crash_count += 1
        if is_session_abort_term_reason(term_reason):
            outcome_counts[f"{term_reason}_count"] += 1
            outcome_counts["session_abort_count"] += 1
        elif term_reason == TIME_LIMIT_TRUNCATION_TERM_REASON:
            outcome_counts[f"{TIME_LIMIT_TRUNCATION_TERM_REASON}_count"] += 1
        elif term_reason == TASK_TERMINATION_TERM_REASON:
            outcome_counts[f"{TASK_TERMINATION_TERM_REASON}_count"] += 1
        elif term_reason == EXTERNAL_ABORT_TERM_REASON:
            outcome_counts[f"{EXTERNAL_ABORT_TERM_REASON}_count"] += 1
        for k, v in ep_metrics.items():
            if k in _RESERVED_METRIC_KEYS:
                continue
            value = float(v)
            ensure_finite(value, context=f"ep_metrics.{k}")
            metric_sums[k] = metric_sums.get(k, 0.0) + value
            metric_counts[k] = metric_counts.get(k, 0) + 1
    out: dict[str, float] = {}
    if n_episodes:
        out["success_rate"] = n_successes / n_episodes
    for k, total in metric_sums.items():
        out[k] = total / metric_counts[k]
    ensure_finite(out, context="aggregated validation metrics")
    out["n_episodes"] = float(n_episodes)
    out["crash_count"] = float(crash_count)
    for key, count in outcome_counts.items():
        out[key] = float(count)
    return out
