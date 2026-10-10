"""Finite-value guards shared by validation evidence seams."""

from __future__ import annotations

import math
from collections.abc import Mapping
from numbers import Number
from typing import Any

import numpy as np


class NonFiniteValidationError(ValueError):
    """Raised when validation evidence contains NaN or an infinity."""


def ensure_finite(value: Any, *, context: str = "value") -> None:
    """Reject non-finite numeric values anywhere in a JSON-shaped value."""

    _ensure_finite(value, context=context)


def _ensure_finite(value: Any, *, context: str) -> None:
    if value is None or isinstance(value, (bool, str, bytes)):
        return
    if isinstance(value, Number):
        try:
            finite = math.isfinite(float(value))
        except (OverflowError, TypeError, ValueError):
            finite = False
        if not finite:
            raise NonFiniteValidationError(f"{context} is non-finite")
        return
    if isinstance(value, np.ndarray):
        if np.issubdtype(value.dtype, np.number):
            try:
                finite = bool(np.all(np.isfinite(value)))
            except TypeError:
                finite = True
            if not finite:
                raise NonFiniteValidationError(f"{context} is non-finite")
            return
        _ensure_finite(value.tolist(), context=context)
        return
    if isinstance(value, Mapping):
        for key, item in value.items():
            _ensure_finite(key, context=f"{context}.{key}.key")
            _ensure_finite(item, context=f"{context}.{key}")
        return
    if isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _ensure_finite(item, context=f"{context}[{index}]")
        return
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        _ensure_finite(tolist(), context=context)


__all__ = ["NonFiniteValidationError", "ensure_finite"]
