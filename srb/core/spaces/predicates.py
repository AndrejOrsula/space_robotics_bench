from __future__ import annotations

from typing import Any, TypeGuard

from gymnasium import spaces


def is_box(value: Any) -> TypeGuard[spaces.Box]:
    return isinstance(value, spaces.Box)


def is_dict(value: Any) -> TypeGuard[spaces.Dict]:
    return isinstance(value, spaces.Dict)


def is_tuple(value: Any) -> TypeGuard[spaces.Tuple]:
    return isinstance(value, spaces.Tuple)


def is_discrete(value: Any) -> TypeGuard[spaces.Discrete]:
    return isinstance(value, spaces.Discrete)
