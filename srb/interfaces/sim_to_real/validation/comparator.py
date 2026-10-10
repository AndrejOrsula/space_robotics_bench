"""Three-criterion pass/fail comparator (threshold + crash + drift)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from srb.interfaces.sim_to_real.validation.baseline_metrics import (
    load_baseline_metrics,
)
from srb.interfaces.sim_to_real.validation.criteria import (
    evaluate_crash,
    evaluate_drift,
    evaluate_threshold,
)
from srb.interfaces.sim_to_real.validation.outcome import CRASH_TERM_REASONS
from srb.interfaces.sim_to_real.validation.result import (
    SessionResult,
    build_session_result,
)
from srb.interfaces.sim_to_real.validation.session_metrics import (
    aggregate_session_metrics,
)
from srb.utils.validation_spec import ValidationSpec

CrashTermReasons = CRASH_TERM_REASONS


def evaluate(
    spec: ValidationSpec,
    session_dir: Path,
    *,
    drift_history: list[dict[str, Any]],
) -> SessionResult:
    real = aggregate_session_metrics(session_dir)
    sim = load_baseline_metrics(spec)
    crit_a, ratio = evaluate_threshold(spec, real, sim)
    crit_b = evaluate_crash(spec, real)
    crit_c, drift = evaluate_drift(spec, real, drift_history)
    return build_session_result(
        criterion_a=crit_a,
        criterion_b=crit_b,
        criterion_c=crit_c,
        metrics=real,
        sim_metrics=sim,
        sim_real_ratio=ratio,
        drift_z=drift,
    )
