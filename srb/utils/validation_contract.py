"""Shared term-reason vocabulary for validation and dataset contracts.

Keep this module free of deployment, hardware, simulator, and episode-execution
code; it is a dependency boundary.
"""

from __future__ import annotations

SESSION_OUTCOME_SENSOR_STALE = "sensor_stale"
SESSION_OUTCOME_SENSOR_INCOMPLETE = "sensor_incomplete"
SESSION_OUTCOME_CALIBRATION_MISMATCH = "calibration_mismatch"
SESSION_OUTCOME_MANIFEST_MISMATCH = "manifest_mismatch"
SESSION_OUTCOME_OPERATOR_ABORT = "operator_abort"
# Term reason for an episode ended by the harness abort latch (``SRB_ABORT``).
EXTERNAL_ABORT_TERM_REASON = "external_abort"

#: Ordered, fixed vocabulary of non-success session outcomes.
SESSION_ABORT_OUTCOMES: tuple[str, ...] = (
    SESSION_OUTCOME_SENSOR_STALE,
    SESSION_OUTCOME_SENSOR_INCOMPLETE,
    SESSION_OUTCOME_CALIBRATION_MISMATCH,
    SESSION_OUTCOME_MANIFEST_MISMATCH,
    SESSION_OUTCOME_OPERATOR_ABORT,
)

#: Episode ended at the configured horizon, with a bootstrap-eligible final
#: observation.  This is distinct from the validation harness wall-clock
#: timeout.
TIME_LIMIT_TRUNCATION = "time_limit_truncation"
TIME_LIMIT_TRUNCATION_TERM_REASON = TIME_LIMIT_TRUNCATION

#: A task ended the episode itself rather than the validation harness.
TASK_TERMINATION_TERM_REASON = "task_terminated"

#: All out-of-MDP session outcomes.
SESSION_ABORT_TERM_REASONS = frozenset(SESSION_ABORT_OUTCOMES)

#: Operator stops are non-success outcomes but are not hardware crashes.
NON_CRASH_SESSION_ABORT_TERM_REASONS = frozenset({SESSION_OUTCOME_OPERATOR_ABORT})

#: Term reasons that count against the crash criterion.
CRASH_TERM_REASONS = frozenset(
    {
        "hardware_event",
        "policy_nan",
        "disk_full",
        SESSION_OUTCOME_SENSOR_STALE,
        SESSION_OUTCOME_SENSOR_INCOMPLETE,
        SESSION_OUTCOME_CALIBRATION_MISMATCH,
        SESSION_OUTCOME_MANIFEST_MISMATCH,
    }
)

#: Complete episode term-reason vocabulary accepted by validation records.
VALID_TERM_REASONS = frozenset(
    {
        "success",
        EXTERNAL_ABORT_TERM_REASON,
        "timeout",
        TIME_LIMIT_TRUNCATION_TERM_REASON,
        TASK_TERMINATION_TERM_REASON,
        *CRASH_TERM_REASONS,
        *SESSION_ABORT_TERM_REASONS,
    }
)

__all__ = [
    "CRASH_TERM_REASONS",
    "EXTERNAL_ABORT_TERM_REASON",
    "NON_CRASH_SESSION_ABORT_TERM_REASONS",
    "SESSION_ABORT_OUTCOMES",
    "SESSION_ABORT_TERM_REASONS",
    "SESSION_OUTCOME_CALIBRATION_MISMATCH",
    "SESSION_OUTCOME_MANIFEST_MISMATCH",
    "SESSION_OUTCOME_OPERATOR_ABORT",
    "SESSION_OUTCOME_SENSOR_INCOMPLETE",
    "SESSION_OUTCOME_SENSOR_STALE",
    "TASK_TERMINATION_TERM_REASON",
    "TIME_LIMIT_TRUNCATION",
    "TIME_LIMIT_TRUNCATION_TERM_REASON",
    "VALID_TERM_REASONS",
]
