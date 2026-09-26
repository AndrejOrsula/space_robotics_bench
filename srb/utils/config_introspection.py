"""Lightweight helpers for walking SRB-style config objects."""

from __future__ import annotations

import dataclasses
from collections.abc import Iterator, Mapping
from typing import Any, TypeVar, cast

ConfigValueT = TypeVar("ConfigValueT")


def iter_config_items(config: object) -> Iterator[tuple[str, Any]]:
    """Iterate declared config fields plus runtime-added attributes."""
    yielded: set[str] = set()

    if dataclasses.is_dataclass(config):
        for field in dataclasses.fields(config):
            if hasattr(config, field.name):
                yielded.add(field.name)
                yield field.name, getattr(config, field.name)

    model_fields = getattr(type(config), "model_fields", None)
    if isinstance(model_fields, Mapping):
        for name in model_fields:
            if name in yielded or not hasattr(config, name):
                continue
            yielded.add(name)
            yield name, getattr(config, name)

    try:
        runtime_items = vars(config).items()
    except TypeError:
        runtime_items = ()

    for name, value in runtime_items:
        if name not in yielded:
            yielded.add(name)
            yield name, value

    extra_items = getattr(config, "model_extra", None)
    if isinstance(extra_items, Mapping):
        for name, value in extra_items.items():
            if name not in yielded:
                yielded.add(name)
                yield name, value

    for name in _iter_slot_names(type(config)):
        if name in yielded or name.startswith("_"):
            continue
        try:
            value = getattr(config, name)
        except AttributeError:
            continue
        yielded.add(name)
        yield name, value


def iter_config_values(config: object) -> Iterator[Any]:
    """Iterate values from :func:`iter_config_items`."""
    for _, value in iter_config_items(config):
        yield value


def iter_config_items_by_type(
    config: object,
    value_type: type[ConfigValueT] | tuple[type[Any], ...],
) -> Iterator[tuple[str, ConfigValueT]]:
    """Iterate config items whose value matches ``value_type``."""
    for name, value in iter_config_items(config):
        if isinstance(value, value_type):
            yield name, cast(ConfigValueT, value)


def iter_config_values_by_type(
    config: object,
    value_type: type[ConfigValueT] | tuple[type[Any], ...],
) -> Iterator[ConfigValueT]:
    """Iterate config values whose value matches ``value_type``."""
    for _, value in iter_config_items_by_type(config, value_type):
        yield value


def get_config_item(config: object, key: str) -> Any:
    """Return one item from a config object using the shared traversal rules."""
    for name, value in iter_config_items(config):
        if name == key:
            return value
    raise KeyError(key)


def _is_immutability_rejection(exc: BaseException) -> bool:
    """True if exc is a frozen/validated model deliberately rejecting assignment."""
    from dataclasses import FrozenInstanceError

    if isinstance(exc, FrozenInstanceError):
        return True
    try:
        from pydantic import ValidationError
    except ImportError:
        return False
    return isinstance(exc, ValidationError)


def set_config_item(config: object, key: str, value: Any) -> None:
    """Set a config item, falling back to runtime storage when needed."""
    try:
        setattr(config, key, value)
        return
    except (AttributeError, TypeError, ValueError) as exc:
        # Do not bypass model immutability or validation.
        if _is_immutability_rejection(exc):
            raise

    try:
        vars(config)[key] = value
    except TypeError as exc:
        raise TypeError(f"Cannot set config item '{key}' on {type(config)}") from exc


def _iter_slot_names(cls: type) -> Iterator[str]:
    """Iterate public slot names declared across a class hierarchy."""
    for base in reversed(cls.__mro__):
        slots = getattr(base, "__slots__", ())
        if isinstance(slots, str):
            slots = (slots,)
        for name in slots:
            if name in ("__dict__", "__weakref__"):
                continue
            yield name
