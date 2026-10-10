"""Synthetic sim-to-real fixtures for ground waypoint navigation.

These values are not hardware calibration. Configs opt in by profile name;
historical observation-noise defaults reuse the same constants.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


@dataclass(frozen=True)
class WaypointSyntheticProfile:
    """Versioned, seeded perturbations for an explicitly synthetic run."""

    name: str = "waypoint_navigation_synthetic_v1"
    provenance: str = "synthetic_unmeasured"
    default_seed: int = 20260908

    # The waypoint task runs at 25 Hz, so one agent step is 0.04 s.
    agent_step_s: float = 0.04
    action_delay_steps: tuple[int, int] = (1, 2)
    observation_delay_steps: tuple[int, int] = (1, 2)
    delay_change_freq_s: float = 1.0
    delay_change_prob: float = 0.0

    # Action noise is applied in the normalized policy action domain.
    action_noise_std_normalized: float = 0.02

    # Waypoint-relative observation noise uses physical task units.
    episode_position_noise_std_m: float = 0.01
    episode_yaw_noise_std_rad: float = 0.043633231
    step_position_noise_std_m: float = 0.0025
    step_yaw_noise_std_rad: float = 0.0087266463

    def __post_init__(self) -> None:
        if self.name != "waypoint_navigation_synthetic_v1":
            raise ValueError(
                f"Unknown waypoint-navigation synthetic profile {self.name!r}; "
                "use 'waypoint_navigation_synthetic_v1'"
            )
        if (
            isinstance(self.default_seed, bool)
            or not isinstance(self.default_seed, int)
            or self.default_seed < 0
        ):
            raise ValueError("default_seed must be non-negative")
        if (
            isinstance(self.agent_step_s, bool)
            or not isinstance(self.agent_step_s, Real)
            or not math.isfinite(float(self.agent_step_s))
            or self.agent_step_s <= 0.0
        ):
            raise ValueError("agent_step_s must be finite and positive")
        for field_name in ("action_delay_steps", "observation_delay_steps"):
            delay_range = getattr(self, field_name)
            if (
                not isinstance(delay_range, tuple)
                or len(delay_range) != 2
                or any(
                    isinstance(value, bool) or not isinstance(value, int)
                    for value in delay_range
                )
                or delay_range[0] < 0
                or delay_range[1] < delay_range[0]
            ):
                raise ValueError(f"{field_name} must be an increasing step range")
        if (
            isinstance(self.delay_change_freq_s, bool)
            or not isinstance(self.delay_change_freq_s, Real)
            or not math.isfinite(float(self.delay_change_freq_s))
            or self.delay_change_freq_s <= 0
        ):
            raise ValueError("delay_change_freq_s must be finite and positive")
        if (
            isinstance(self.delay_change_prob, bool)
            or not isinstance(self.delay_change_prob, Real)
            or not 0.0 <= self.delay_change_prob <= 1.0
        ):
            raise ValueError("delay_change_prob must be in [0.0, 1.0]")
        for field_name in (
            "action_noise_std_normalized",
            "episode_position_noise_std_m",
            "episode_yaw_noise_std_rad",
            "step_position_noise_std_m",
            "step_yaw_noise_std_rad",
        ):
            value = getattr(self, field_name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(float(value))
                or value < 0.0
            ):
                raise ValueError(f"{field_name} must be finite and non-negative")


WAYPOINT_SYNTHETIC_PROFILE = WaypointSyntheticProfile()

__all__ = ["WaypointSyntheticProfile", "WAYPOINT_SYNTHETIC_PROFILE"]
