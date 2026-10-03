from __future__ import annotations

from typing import Any


def validate_delay_steps(name: str, value: Any) -> None:
    """Validate SRB delay-step config values."""

    msg = (
        f"{name} must be a non-negative int or a (min, max) pair of "
        f"non-negative ints with min <= max, got: {value!r}"
    )
    if isinstance(value, bool) or not isinstance(value, (int, tuple, list)):
        raise ValueError(msg)
    if isinstance(value, int):
        if value < 0:
            raise ValueError(msg)
        return
    if (
        len(value) != 2
        or isinstance(value[0], bool)
        or isinstance(value[1], bool)
        or not isinstance(value[0], int)
        or not isinstance(value[1], int)
        or value[0] < 0
        or value[1] < 0
        or value[0] > value[1]
    ):
        raise ValueError(msg)


__all__ = ["validate_delay_steps"]
