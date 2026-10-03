"""Isaac-free helpers for unpacking ``ActionManager.find_term_by_type`` results."""

from __future__ import annotations

from typing import TypeVar

T = TypeVar("T")


def get_term_or_none(found: tuple[str, T] | None) -> T | None:
    """Return the term of a ``find_term_by_type`` result, or ``None`` if missing."""
    return found[1] if found is not None else None


def require_term(
    found: tuple[str, T] | None,
    *,
    cls: type,
    reason: str,
) -> T:
    """Return the found term or raise ``ValueError`` with its class and reason."""
    if found is None:
        raise ValueError(
            f"ActionManager: no term of type {cls.__name__} found. {reason}"
        )
    return found[1]
