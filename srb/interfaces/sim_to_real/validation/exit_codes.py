"""Exit-code reference for structured sim-to-real validation sessions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class ValidationExitCode:
    code: int
    name: str
    criterion: str
    meaning: str


VALIDATION_EXIT_CODES: tuple[ValidationExitCode, ...] = (
    ValidationExitCode(
        code=0,
        name="pass",
        criterion="all criteria pass",
        meaning="Validation met threshold, crash, and drift criteria.",
    ),
    ValidationExitCode(
        code=1,
        name="threshold_fail",
        criterion="criterion_a",
        meaning="Threshold metrics failed, such as success rate or sim/real ratio.",
    ),
    ValidationExitCode(
        code=2,
        name="crash_fail",
        criterion="criterion_b",
        meaning="A crash or safety-relevant termination was observed.",
    ),
    ValidationExitCode(
        code=3,
        name="drift_fail",
        criterion="criterion_c",
        meaning="Tracked metrics drifted beyond the configured z-score limit.",
    ),
    ValidationExitCode(
        code=4,
        name="preflight_fail",
        criterion="preflight",
        meaning=(
            "Validation could not start rollout, for example due to missing "
            "generated environment code, missing hardware wiring, an "
            "incompatible checkpoint, or an explicit non-hardware dry-run; "
            "or evaluation failed after the rollouts. Either way the session "
            "is non-hardware evidence."
        ),
    ),
)


def render_validation_exit_code_table(
    exit_codes: Iterable[ValidationExitCode] = VALIDATION_EXIT_CODES,
) -> str:
    lines = [
        "| Exit code | Name | Criterion | Meaning |",
        "| :--- | :--- | :--- | :--- |",
    ]
    for exit_code in exit_codes:
        lines.append(
            "| "
            f"`{exit_code.code}` | "
            f"`{exit_code.name}` | "
            f"`{exit_code.criterion}` | "
            f"{exit_code.meaning} |"
        )
    return "\n".join(lines)


__all__ = [
    "VALIDATION_EXIT_CODES",
    "ValidationExitCode",
    "render_validation_exit_code_table",
]
