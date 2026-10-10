"""Lifecycle helpers for deployment resources."""

from __future__ import annotations

from collections.abc import Sequence


def raise_cleanup_errors(
    errors: Sequence[Exception],
    *,
    context: str,
) -> None:
    """Raise the sole cleanup error, or group multiple errors."""

    if not errors:
        return
    if len(errors) == 1:
        raise errors[0]
    raise ExceptionGroup(context, list(errors))


__all__ = ["raise_cleanup_errors"]
