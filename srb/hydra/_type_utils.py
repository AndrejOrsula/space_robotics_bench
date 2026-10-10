import types
from dataclasses import is_dataclass
from typing import Union, get_args, get_origin

from pydantic import BaseModel

from srb.hydra.type_resolution import (
    TYPE_HINT_FALLBACK_DEBUG_ENV as _TYPE_HINT_FALLBACK_DEBUG_ENV,
)
from srb.hydra.type_resolution import _eval_annotation, _get_srb_annotations
from srb.hydra.type_resolution import get_type_hints_safe as _get_type_hints_safe

__all__ = (
    "_TYPE_HINT_FALLBACK_DEBUG_ENV",
    "_eval_annotation",
    "_get_srb_annotations",
    "_get_type_hints_safe",
    "_is_config_instance",
    "_is_config_type",
    "_unwrap_optional",
    "_unwrap_to_config_type",
    "_unwrap_union_members",
)


def _unwrap_union_members(tp) -> list[type]:
    """Return all non-None members of a Union / ``X | Y`` type.

    For non-Union types, returns ``[tp]``.  For ``None``, returns ``[]``.
    """
    if tp is None:
        return []
    origin = get_origin(tp)
    if origin is Union or isinstance(tp, types.UnionType):
        return [a for a in get_args(tp) if a is not type(None)]
    return [tp]


def _unwrap_optional(tp):
    """Unwrap ``Optional[X]`` / ``X | None`` to ``X``.

    Multi-member unions are returned as-is.
    """
    members = _unwrap_union_members(tp)
    return members[0] if len(members) == 1 else tp


def _unwrap_to_config_type(tp) -> type | None:
    """If *tp* (possibly a Union) contains a config type, return it."""
    for member in _unwrap_union_members(tp):
        if _is_config_type(member):
            return member
    return None


def _is_config_type(tp) -> bool:
    """Check if *tp* is a dataclass or Pydantic ``BaseModel`` **type** (not instance)."""
    if tp is None:
        return False
    try:
        return (is_dataclass(tp) and isinstance(tp, type)) or (
            isinstance(tp, type) and issubclass(tp, BaseModel)
        )
    except TypeError:
        return False


def _is_config_instance(obj) -> bool:
    """Return True if *obj* is a config **instance** (dataclass or Pydantic, not a type)."""
    if isinstance(obj, type):
        return False
    return is_dataclass(obj) or isinstance(obj, BaseModel)
