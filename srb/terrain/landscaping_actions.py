"""Filtered wheeled-drive action facts for terrain landscaping.

This module is intentionally free of Isaac and ROS imports so sim task code and
deploy-side adapters share one set of drive values.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class LandscapingDriveActionContract:
    """Filtered wheeled-drive action facts shared by sim and deploy."""

    action_key: str = "robot/cmd_vel"
    hardware_action_key: str = "robot/wheeled_drive"
    action_dim: int = 2
    linear_action_index: int = 0
    wheelbase: tuple[float, float] = (0.2975, 0.3587)
    wheel_radius: float = 0.065
    drive_joint_names: tuple[str, ...] = (
        "wheel_joint_FL",
        "wheel_joint_FR",
        "wheel_joint_RL",
        "wheel_joint_RR",
    )
    scale_linear: float = 0.4
    scale_angular: float = math.radians(60)
    v_lin_max: float = 0.4
    v_ang_max: float = 1.0
    accel_lin_max: float = 0.8
    accel_ang_max: float = 2.0
    filter_tau_s: float = 0.15


LANDSCAPING_DRIVE_ACTION = LandscapingDriveActionContract()


__all__ = [
    "LANDSCAPING_DRIVE_ACTION",
    "LandscapingDriveActionContract",
]
