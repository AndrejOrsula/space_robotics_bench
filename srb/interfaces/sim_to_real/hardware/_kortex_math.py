"""Pure-numeric helpers shared by the Kortex hardware drivers.

No ROS or Isaac imports; these helpers can be tested without hardware.
"""

from __future__ import annotations

import math

import numpy as np


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    two_pi = 2.0 * math.pi
    wrapped = (angle_rad + math.pi) % two_pi - math.pi
    # The modulo maps the -pi boundary to -pi, which is valid.
    return wrapped


def normalize_joint_positions(
    positions_rad: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> np.ndarray:
    """Map per-joint positions to ``[-1, 1]`` via the configured limits."""
    span = upper - lower
    # A zero span is degenerate; use 1 to avoid division by zero.
    safe_span = np.where(np.abs(span) < 1e-6, 1.0, span)
    normalized = 2.0 * (positions_rad - lower) / safe_span - 1.0
    return normalized.astype(np.float32)


__all__ = ["normalize_joint_positions", "wrap_to_pi"]
