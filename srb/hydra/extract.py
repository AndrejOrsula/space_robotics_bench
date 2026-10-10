"""Extract default values from dataclass / Pydantic config classes.

The main entry point is :func:`extract_defaults_from_class`, which walks
the fields of a config class and returns a plain ``dict`` of defaults
suitable for Hydra structured-config registration — without needing to
instantiate the config object.
"""

from __future__ import annotations

import dataclasses
import enum
from dataclasses import MISSING as DC_MISSING
from typing import Any

from pydantic import BaseModel
from pydantic_core import PydanticUndefined

from srb.hydra._type_utils import _is_config_instance
from srb.hydra._type_utils import (
    _is_config_type as _is_config_class,
)
from srb.hydra.type_resolution import get_type_hints_safe
from srb.utils.config_introspection import iter_config_items


def _is_asset_instance(obj) -> bool:
    """Return whether *obj* is a Pydantic instance exposing ``name()`` and
    ``asset_cfg``.
    """
    if not isinstance(obj, BaseModel) or isinstance(obj, type):
        return False
    cls = type(obj)
    return (
        hasattr(cls, "name")
        and callable(getattr(cls, "name"))
        and hasattr(obj, "asset_cfg")
    )


def _is_asset_class(cls) -> bool:
    """Return True if *cls* is a class with ``name()`` and ``asset_cfg``."""
    if not isinstance(cls, type) or not issubclass(cls, BaseModel):
        return False
    if not hasattr(cls, "name") or not callable(getattr(cls, "name")):
        return False
    return any(
        "asset_cfg" in getattr(base, "__annotations__", {}) for base in cls.__mro__
    )


def _is_non_serializable_config(obj) -> bool:
    """Return whether *obj* has the manager-term ``func``/``params`` shape."""
    if isinstance(obj, type) or not dataclasses.is_dataclass(obj):
        return False
    cls = type(obj)
    fields = {f.name for f in dataclasses.fields(cls)}
    return "func" in fields and "params" in fields


def _target_path(obj: object) -> str:
    cls = type(obj)
    return f"{cls.__module__}:{cls.__qualname__}"


def _is_missing(value) -> bool:
    """Return True if *value* is a MISSING sentinel (dataclass or Pydantic).

    Configclass defaults are deep-copied, which clones the dataclass sentinel.
    """
    return isinstance(value, type(DC_MISSING)) or value is PydanticUndefined


def _is_plain_value(value: Any) -> bool:
    """Return whether *value* survives OmegaConf serialization unchanged.

    Enums are excluded because the downstream pass converts them to strings.
    """
    if isinstance(value, enum.Enum):
        return False
    if isinstance(value, _OMEGACONF_PRIMITIVES):
        return True
    if isinstance(value, (list, tuple)):
        return all(_is_plain_value(item) for item in value)
    if isinstance(value, dict):
        # A ``str`` enum key (``BakeType``) satisfies ``isinstance(key, str)``
        # but does not survive the round trip as the same key.
        return all(
            isinstance(key, str)
            and not isinstance(key, enum.Enum)
            and _is_plain_value(item)
            for key, item in value.items()
        )
    return False


def _plain_asset_fields(instance: Any) -> dict[str, Any]:
    """Return serializable instance fields needed for asset reconstruction.

    Emit every plain field so Hydra accepts overrides for fields set to class
    defaults. The caller handles nested assets, configs, and ``action_mode``.
    """
    fields: dict[str, Any] = {}
    for sub_name, class_default in _iter_field_defaults(type(instance)):
        if sub_name.startswith("_") or sub_name in ("action_mode", "asset_cfg"):
            continue
        if not hasattr(instance, sub_name):
            continue
        value = getattr(instance, sub_name)
        # Required fields lack class defaults but remain necessary to rebuild instances.
        if _is_plain_value(value):
            fields[sub_name] = value
        elif (
            _is_config_instance(value)
            and not _is_asset_instance(value)
            and not _is_non_serializable_config(value)
        ):
            # Preserve instance-supplied configs; reconstruction uses the asset constructor.
            fields[sub_name] = _instance_to_dict(value)
    return fields


def _iter_field_defaults(cls) -> list[tuple[str, Any]]:
    """Return ``(name, default)`` pairs; use ``DC_MISSING`` when absent."""
    results: list[tuple[str, Any]] = []

    if dataclasses.is_dataclass(cls) and isinstance(cls, type):
        for f in dataclasses.fields(cls):
            if f.default is not DC_MISSING:
                results.append((f.name, f.default))
            elif f.default_factory is not DC_MISSING:  # type: ignore[arg-type]
                results.append((f.name, f.default_factory()))  # type: ignore[misc]
            else:
                results.append((f.name, DC_MISSING))

    elif isinstance(cls, type) and issubclass(cls, BaseModel):
        for name, field_info in cls.model_fields.items():
            if field_info.default is not PydanticUndefined:
                results.append((name, field_info.default))
            elif field_info.default_factory is not None:
                results.append((name, field_info.default_factory()))
            else:
                results.append((name, DC_MISSING))

    return results


def _filter_missing(d: dict) -> dict:
    """Remove missing values and empty nested dictionaries."""
    out: dict = {}
    for k, v in d.items():
        if _is_missing(v):
            continue
        if isinstance(v, dict):
            nested = _filter_missing(v)
            if nested:
                out[k] = nested
        else:
            out[k] = v
    return out


def _instance_to_dict(obj) -> dict:
    """Convert a dataclass or Pydantic instance to a filtered dictionary."""
    if isinstance(obj, BaseModel):
        result: dict = {"_target_": _target_path(obj)}
        for key, value in iter_config_items(obj):
            if key.startswith("_"):
                continue
            if _is_missing(value):
                continue
            if _is_non_serializable_config(value):
                continue
            if _is_config_instance(value):
                result[key] = _instance_to_dict(value)
            else:
                result[key] = value
        return result

    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        raw = {"_target_": _target_path(obj)}
        for f in dataclasses.fields(obj):
            if f.name.startswith("_"):
                continue
            value = getattr(obj, f.name)
            if _is_missing(value):
                continue
            if _is_non_serializable_config(value):
                continue
            if _is_config_instance(value):
                raw[f.name] = _instance_to_dict(value)
            else:
                raw[f.name] = value
        return _filter_missing(raw)

    raise TypeError(f"Cannot convert {type(obj)} to dict")


def extract_defaults_from_class(cls) -> dict:
    """Extract defaults from dataclass or Pydantic config class *cls*."""
    result: dict = {}

    for name, default in _iter_field_defaults(cls):
        if name.startswith("_"):
            continue

        if _is_missing(default):
            continue

        if _is_non_serializable_config(default):
            continue

        # Registry names resolve assets; ``action_modes`` contains unserializable callables.
        if _is_asset_instance(default):
            try:
                from srb.core.asset.robot.mobile_manipulation.combined._metaclass import (
                    CombinedMobileManipulator,
                )
            except ImportError:
                CombinedMobileManipulator = None  # type: ignore[assignment,misc]

            if CombinedMobileManipulator is not None and isinstance(
                default, CombinedMobileManipulator
            ):
                concrete_cls = type(default)
                # Preserve the base identity recorded during flattening.
                mobile_base_name = getattr(default, "mobile_base_name", None)
                # Flattening copies these values onto the composite; preserve them for reconstruction.
                base_payload: Any = getattr(default, "payload", None)
                payload_dict: dict[str, Any] = {"name": None}
                if _is_asset_instance(base_payload):
                    payload_cls: Any = type(base_payload)
                    payload_dict = {"name": payload_cls.name()}
                    payload_dict.update(_plain_asset_fields(base_payload))
                sub: dict = {
                    "_target_": f"{concrete_cls.__module__}:{concrete_cls.__qualname__}",
                    # Emit the base name so resolution can rerun flattening and accept dot-path overrides.
                    "mobile_base": {
                        "name": mobile_base_name,
                        "action_mode": getattr(default, "action_mode", None),
                        "payload": payload_dict,
                    },
                    # Null denotes an absent asset; a ``MISSING`` string would trigger registry lookup.
                    "manipulator": {"name": None},
                }
                # Composite mode belongs to the flattened mobile base.
                if hasattr(default, "manipulator") and _is_asset_instance(
                    default.manipulator
                ):
                    manipulator_default = default.manipulator
                    manip_dict: dict[str, Any] = {
                        "name": type(manipulator_default).name(),
                        "action_mode": getattr(
                            manipulator_default, "action_mode", None
                        ),
                    }
                    if hasattr(manipulator_default, "end_effector"):
                        ee = manipulator_default.end_effector
                        if ee is not None and _is_asset_instance(ee):
                            ee_dict: dict[str, Any] = {
                                "name": type(ee).name(),
                            }
                            if hasattr(ee, "action_mode"):
                                ee_dict["action_mode"] = getattr(
                                    ee, "action_mode", None
                                )
                            ee_dict.update(_plain_asset_fields(ee))
                            manip_dict["end_effector"] = ee_dict
                        else:
                            # Keep the key available for Hydra overrides.
                            manip_dict["end_effector"] = {"name": None}
                    sub["manipulator"] = manip_dict
                result[name] = sub
                continue

            asset_dict: dict[str, Any] = {
                "name": type(default).name(),
            }
            if hasattr(default, "action_mode"):
                asset_dict["action_mode"] = getattr(default, "action_mode", None)
            # Carry plain fields through reconstruction so instance values reach the spawner.
            asset_dict.update(_plain_asset_fields(default))
            # Emit nested assets and typed nulls so Hydra accepts nested overrides.
            from srb.hydra._type_utils import (
                _unwrap_union_members as _uw,
            )

            sub_hints = get_type_hints_safe(type(default))
            for sub_name, sub_class_default in _iter_field_defaults(type(default)):
                if sub_name.startswith("_") or sub_name in ("action_mode",):
                    continue
                if _is_missing(sub_class_default):
                    continue
                # Instance values override class defaults supplied as ``None``.
                sub_default = getattr(default, sub_name, sub_class_default)
                if _is_asset_instance(sub_default):
                    sub_asset_dict: dict[str, Any] = {
                        "name": type(sub_default).name(),
                    }
                    if hasattr(sub_default, "action_mode"):
                        sub_asset_dict["action_mode"] = getattr(
                            sub_default, "action_mode", None
                        )
                    sub_asset_dict.update(_plain_asset_fields(sub_default))
                    asset_dict[sub_name] = sub_asset_dict
                elif sub_default is None and sub_name in sub_hints:
                    # A typed null keeps the Hydra key available.
                    sub_type = sub_hints[sub_name]
                    if any(
                        isinstance(m, type) and _is_asset_class(m)
                        for m in _uw(sub_type)
                    ):
                        asset_dict[sub_name] = {"name": None}
            result[name] = asset_dict
            continue

        if _is_config_instance(default):
            result[name] = _filter_missing(_instance_to_dict(default))
            continue

        if _is_config_class(type(default)):
            result[name] = extract_defaults_from_class(type(default))
            continue

        result[name] = default

    # Convert unsupported values before OmegaConf registration.
    return _sanitize_value(result)


_OMEGACONF_PRIMITIVES = (str, int, float, bool, type(None))


def _sanitize_value(value: Any) -> Any:
    """Convert unsupported values to OmegaConf-compatible forms."""
    import enum
    from pathlib import PurePath

    if isinstance(value, enum.Enum):
        return value  # handled by replace_enums_with_strings
    # Keep StrEnum identity while demoting other string subclasses for OmegaConf.
    if isinstance(value, str) and type(value) is not str:
        return str.__new__(str, value)
    if isinstance(value, _OMEGACONF_PRIMITIVES):
        return value
    if isinstance(value, slice):
        return value  # handled by replace_slices_with_strings
    try:
        from gymnasium.spaces import Space

        if isinstance(value, Space):
            return value  # handled by replace_dict_spaces_with_strings
    except ImportError:
        pass
    if isinstance(value, PurePath):
        return str(value)
    if isinstance(value, set):
        return [_sanitize_value(v) for v in value]
    if isinstance(value, (list, tuple)):
        sanitized = [_sanitize_value(v) for v in value]
        return type(value)(sanitized) if isinstance(value, tuple) else sanitized
    if isinstance(value, dict):
        return {k: _sanitize_value(v) for k, v in value.items()}
    # Nested configs may survive inside dictionaries and still need conversion.
    if _is_config_instance(value) and not _is_non_serializable_config(value):
        return _sanitize_value(_filter_missing(_instance_to_dict(value)))
    if (
        callable(value)
        and hasattr(value, "__module__")
        and hasattr(value, "__qualname__")
    ):
        return f"{value.__module__}:{value.__qualname__}"
    return str(value)
