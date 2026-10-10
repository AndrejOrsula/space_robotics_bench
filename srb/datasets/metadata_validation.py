"""Shared metadata validation primitives for SRB dataset formats."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from numbers import Integral, Real
from typing import TypeGuard

from srb.utils.dataset_schema import (
    DEMO_IO_CONTRACT_VERSION,
    validate_demo_io_contract,
)


def normalize_int_value(value: object) -> int | None:
    """Return a normalized integral scalar, excluding boolean values."""
    if isinstance(value, bool) or not isinstance(value, Integral):
        return None
    return int(value)


def is_int_value(value: object) -> TypeGuard[int]:
    """Return whether *value* is integral but not a boolean."""
    return normalize_int_value(value) is not None


def is_number_value(value: object) -> TypeGuard[int | float]:
    """Return whether *value* is a real number but not a boolean."""
    return isinstance(value, Real) and not isinstance(value, bool)


def validate_metadata_count(
    *,
    metadata: Mapping[str, object],
    key: str,
    actual: int,
    errors: list[str],
) -> None:
    """Append a consistent validation error for one declared count."""
    if key not in metadata:
        return
    expected = normalize_int_value(metadata[key])
    if expected is None:
        errors.append(f"metadata {key} must be an integer")
        return
    if expected != actual:
        errors.append(f"metadata {key}={expected} does not match actual {actual}")


def validate_metadata_observation_keys(
    *,
    metadata: Mapping[str, object],
    actual: set[str],
    errors: list[str],
) -> None:
    """Append consistent errors for a declared observation-key collection."""
    raw_keys = metadata.get("obs_keys")
    if raw_keys is None:
        return
    if isinstance(raw_keys, (str, bytes)) or not isinstance(raw_keys, Iterable):
        errors.append("metadata obs_keys must be an iterable of keys")
        return
    declared = {str(key) for key in raw_keys}
    if declared != actual:
        errors.append(
            "metadata obs_keys does not match dataset observations: "
            f"declared={sorted(declared)} actual={sorted(actual)}"
        )


__all__ = [
    "DEMO_IO_CONTRACT_VERSION",
    "is_int_value",
    "is_number_value",
    "normalize_int_value",
    "validate_metadata_count",
    "validate_metadata_observation_keys",
    "validate_demo_io_contract",
]
