"""Isaac-free geometry helpers shared by wheeled-drive action terms."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Literal

SteeringAxle = Literal["front", "rear"]


def ackermann_steering_wheel_indices(
    drive_positions: Sequence[Sequence[float]],
    *,
    steering_count: int,
    steering_axle: SteeringAxle,
) -> list[int]:
    """Select drive-wheel indices on the configured Ackermann steering axle."""
    if steering_count < 0:
        raise ValueError("steering_count must be non-negative")
    if steering_axle not in ("front", "rear"):
        raise ValueError(
            f"Unsupported steering_axle: {steering_axle!r}; "
            "valid choices are 'front' and 'rear'"
        )
    reverse = steering_axle == "front"
    by_x = sorted(
        range(len(drive_positions)),
        key=lambda i: float(drive_positions[i][0]),
        reverse=reverse,
    )
    return by_x[:steering_count]


def ackermann_steering_axle_sign(steering_axle: SteeringAxle) -> float:
    """Return steering-angle sign needed for a requested body-yaw direction."""
    if steering_axle == "front":
        return 1.0
    if steering_axle == "rear":
        return -1.0
    raise ValueError(
        f"Unsupported steering_axle: {steering_axle!r}; "
        "valid choices are 'front' and 'rear'"
    )


__all__ = [
    "SteeringAxle",
    "ackermann_steering_axle_sign",
    "ackermann_steering_wheel_indices",
]
