"""Typed scalar validation shared by environment runtime layouts."""

from __future__ import annotations


def validate_positive_int(value: object, *, name: str) -> int:
    """Return *value* when it is a positive, non-boolean integer."""
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive int, got {value!r}")
    return value


def validate_non_negative_int(value: object, *, name: str) -> int:
    """Return *value* when it is a non-negative, non-boolean integer."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative int, got {value!r}")
    return value


__all__ = ["validate_non_negative_int", "validate_positive_int"]
