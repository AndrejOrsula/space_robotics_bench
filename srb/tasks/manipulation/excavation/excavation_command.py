"""Pure helpers for commanded single-scoop excavation targets."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

DEFAULT_SCOOP_TARGET_XY_M: tuple[float, float] = (0.55, 0.0)
DEFAULT_SCOOP_TARGET_YAW_RAD: float = 0.0
DEFAULT_SCOOP_TARGET_FRAME: str = "kinova_base_link"
DEFAULT_TARGET_WORKSPACE_BOUNDS_M: tuple[tuple[float, float], tuple[float, float]] = (
    (0.30, 0.85),
    (-0.35, 0.35),
)


@dataclass(frozen=True)
class ScoopTargetValidationReport:
    valid: bool
    errors: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class ScoopTargetCommand:
    """Operator or sim-selected source patch for one scoop episode."""

    xy_m: tuple[float, float] = DEFAULT_SCOOP_TARGET_XY_M
    yaw_rad: float = DEFAULT_SCOOP_TARGET_YAW_RAD
    frame: str = DEFAULT_SCOOP_TARGET_FRAME


def validate_scoop_target_command(
    target: ScoopTargetCommand,
    *,
    workspace_bounds_m: tuple[
        tuple[float, float],
        tuple[float, float],
    ] = DEFAULT_TARGET_WORKSPACE_BOUNDS_M,
    required_frame: str = DEFAULT_SCOOP_TARGET_FRAME,
    yaw_bounds_rad: tuple[float, float] = (-math.pi, math.pi),
) -> ScoopTargetValidationReport:
    """Validate a commanded scoop target without silently clamping it."""
    errors: list[str] = []
    (x_min, x_max), (y_min, y_max) = workspace_bounds_m
    x, y = target.xy_m

    if target.frame != required_frame:
        errors.append(f"target frame {target.frame!r} must be {required_frame!r}")
    if not x_min <= x <= x_max:
        errors.append(
            f"target x {x:.3f} outside workspace bounds [{x_min:.3f}, {x_max:.3f}]"
        )
    if not y_min <= y <= y_max:
        errors.append(
            f"target y {y:.3f} outside workspace bounds [{y_min:.3f}, {y_max:.3f}]"
        )
    if not yaw_bounds_rad[0] <= target.yaw_rad <= yaw_bounds_rad[1]:
        errors.append(
            "target yaw "
            f"{target.yaw_rad:.3f} outside bounds "
            f"[{yaw_bounds_rad[0]:.3f}, {yaw_bounds_rad[1]:.3f}]"
        )

    return ScoopTargetValidationReport(valid=not errors, errors=errors)


@dataclass
class ScoopTargetCommandLock:
    """Scoop target held for an episode, with validated reset-time overrides."""

    current: ScoopTargetCommand
    workspace_bounds_m: tuple[
        tuple[float, float],
        tuple[float, float],
    ] = DEFAULT_TARGET_WORKSPACE_BOUNDS_M
    pending: ScoopTargetCommand | None = None

    def queue(self, command: ScoopTargetCommand) -> None:
        """Validate ``command`` and stash it for the next reset boundary."""
        report = validate_scoop_target_command(
            command, workspace_bounds_m=self.workspace_bounds_m
        )
        if not report.valid:
            raise ValueError("Invalid scoop target: " + "; ".join(report.errors))
        self.pending = command

    def apply_pending(self) -> bool:
        """Promote the queued command, if any. Returns whether it changed."""
        if self.pending is None:
            return False
        self.current = self.pending
        self.pending = None
        return True
