"""Build config instances recursively from classes and dictionaries."""

from __future__ import annotations

import collections.abc
import enum
import importlib
import sys
from dataclasses import MISSING as DC_MISSING
from dataclasses import fields as dc_fields
from dataclasses import is_dataclass
from typing import Any, Callable, get_args, get_origin

from pydantic import BaseModel

from srb.hydra._type_utils import (
    _is_config_type,
    _unwrap_optional,
    _unwrap_to_config_type,
    _unwrap_union_members,
)
from srb.hydra.type_resolution import get_type_hints_safe


def create_instance(cls: type, cfg_dict: dict | None) -> Any:
    """Construct a config object from a dataclass or Pydantic model."""
    if cfg_dict is None:
        return cls()

    hints = get_type_hints_safe(cls)
    kwargs: dict[str, Any] = {}

    for field_name in _iter_fields(cls):
        if field_name not in cfg_dict:
            continue

        value = cfg_dict[field_name]
        field_type = _unwrap_optional(hints.get(field_name, type(None)))

        if value is None:
            kwargs[field_name] = None
        elif isinstance(value, dict) and (
            _is_config_type(field_type)
            or _unwrap_to_config_type(field_type)
            or isinstance(value.get("_target_"), str)
        ):
            # _target_ also supports Pydantic types outside SRB's config check.
            cfg_type = (
                field_type
                if _is_config_type(field_type)
                else _unwrap_to_config_type(field_type)
            )
            kwargs[field_name] = _reconstruct_config_value(cfg_type, value)
        elif isinstance(value, dict) and _is_dict_type(field_type):
            val_type = _get_dict_value_type(field_type)
            if val_type is not None and _is_config_type(val_type):
                kwargs[field_name] = {
                    k: _reconstruct_dict_value(val_type, v) for k, v in value.items()
                }
            else:
                kwargs[field_name] = value
        elif isinstance(value, str) and _is_enum_type(field_type):
            kwargs[field_name] = _resolve_enum(field_type, value)
        elif (
            isinstance(value, str) and ":" in value and _is_callable_or_type(field_type)
        ):
            kwargs[field_name] = _resolve_callable(value)
        elif isinstance(value, (list, tuple)) and _is_set_type(field_type):
            kwargs[field_name] = set(value)
        elif isinstance(value, (list, tuple)):
            item_types = get_args(field_type) if _is_tuple_type(field_type) else ()
            reconstructed = [
                _reconstruct_sequence_value(
                    item_types[index]
                    if index < len(item_types) and item_types[-1] is not Ellipsis
                    else _get_sequence_value_type(field_type),
                    item,
                )
                for index, item in enumerate(value)
            ]
            kwargs[field_name] = (
                tuple(reconstructed) if _is_tuple_type(field_type) else reconstructed
            )
        else:
            kwargs[field_name] = value

    # Preserve runtime-added fields only for Pydantic models with extra='allow'.
    model_config = getattr(cls, "model_config", None)
    if isinstance(model_config, dict) and model_config.get("extra") == "allow":
        declared = set(_iter_fields(cls))
        for key, value in cfg_dict.items():
            if key in kwargs or key in declared or key == "_target_":
                continue
            kwargs[key] = value

    try:
        return cls(**kwargs)
    except TypeError as e:
        if "abstract class" not in str(e):
            raise
        concrete = _find_concrete_subclass(cls)
        if concrete is not None:
            return concrete(**kwargs)
        raise


def _iter_fields(cls: type) -> list[str]:
    if is_dataclass(cls) and isinstance(cls, type):
        return [f.name for f in dc_fields(cls)]
    if isinstance(cls, type) and issubclass(cls, BaseModel):
        return list(cls.model_fields.keys())
    return []


def _is_dict_type(tp) -> bool:
    if tp is dict:
        return True
    return get_origin(tp) is dict


def _get_dict_value_type(tp) -> type | None:
    args = get_args(tp)
    if args and len(args) == 2:
        return args[1]
    return None


def _get_sequence_value_type(tp) -> type | None:
    args = get_args(tp)
    if not args:
        return None
    if len(args) == 2 and args[1] is Ellipsis:
        return args[0]
    return args[0]


def _is_tuple_type(tp) -> bool:
    if tp is tuple:
        return True
    return get_origin(tp) is tuple


def _reconstruct_config_value(cfg_type: type | None, value: dict) -> Any:
    config_value = dict(value)
    target = config_value.pop("_target_", None)
    if target and isinstance(target, str) and ":" in target:
        try:
            resolved_target = _resolve_callable(target)
            cfg_type = (
                resolved_target
                if isinstance(resolved_target, type)
                else type(resolved_target)
            )
        except (ImportError, AttributeError) as exc:
            # Fail fast without a fallback type; otherwise NoneType silently
            # drops sibling fields and causes a later, misleading error.
            if not isinstance(cfg_type, type):
                raise ValueError(
                    f"Cannot resolve config _target_ '{target}' and no fallback "
                    "config type is available; the instance cannot be "
                    "reconstructed (likely a stale or hand-edited config, or a "
                    "class renamed/moved since serialization)."
                ) from exc
    elif isinstance(cfg_type, type):
        cfg_type = _infer_concrete_type(cfg_type, config_value)

    if not isinstance(cfg_type, type):
        cfg_type = type(cfg_type)
    return create_instance(cfg_type, config_value)


def _reconstruct_sequence_value(item_type: type | None, value: Any) -> Any:
    if not isinstance(value, dict):
        return value

    if isinstance(value.get("_target_"), str):
        return _reconstruct_config_value(None, value)

    cfg_type = _config_type_for_mapping(item_type, value)
    if cfg_type is not None:
        return _reconstruct_config_value(cfg_type, value)
    return value


def _config_type_for_mapping(declared_type: type | None, value: dict) -> type | None:
    candidates = [
        member
        for member in _unwrap_union_members(declared_type)
        if _is_config_type(member)
    ]
    if not candidates:
        cfg_type = _unwrap_to_config_type(declared_type)
        if cfg_type is None and _is_config_type(declared_type):
            return declared_type
        return cfg_type
    if len(candidates) == 1:
        return candidates[0]

    keys = {key for key in value if key != "_target_"}
    best = max(
        candidates,
        key=lambda candidate: len(keys.intersection(_iter_fields(candidate))),
    )
    if keys.intersection(_iter_fields(best)):
        return best
    return candidates[0]


def _reconstruct_dict_value(val_type: type, value: Any) -> Any:
    if not isinstance(value, dict):
        return value

    concrete = _infer_concrete_type(val_type, value)
    return _reconstruct_config_value(concrete, value)


def _infer_concrete_type(base: type, data: dict) -> type:
    """Infer a concrete subclass from the serialized ``func`` discriminator."""
    func_val = data.get("func")
    func_ref: str | None
    if isinstance(func_val, str) and ":" in func_val:
        func_ref = func_val
        try:
            resolved_func = _resolve_callable(func_val)
        except (ImportError, AttributeError):
            return base
    elif callable(func_val):
        func_ref = f"{func_val.__module__}:{func_val.__qualname__}"
        resolved_func = func_val
    else:
        return base

    for sub in _iter_subclasses(base):
        if not is_dataclass(sub):
            continue
        for f in dc_fields(sub):
            if f.name != "func":
                continue
            if f.default is resolved_func:
                return sub
            # Isaac Lab may expose callable defaults as lazy strings.
            if isinstance(f.default, str) and _str_func_matches(
                f.default, func_ref, sub
            ):
                return sub
            if f.default_factory is not DC_MISSING:  # type: ignore[arg-type]
                try:
                    factory_val = f.default_factory()  # type: ignore[misc]
                    if factory_val is resolved_func:
                        return sub
                    if isinstance(factory_val, str) and _str_func_matches(
                        factory_val, func_ref, sub
                    ):
                        return sub
                except Exception:
                    pass

    return base


def _str_func_matches(default: str, func_val: str, sub: type) -> bool:
    """Compare callable references, resolving Isaac Lab's ``{DIR}`` placeholder."""
    if default == func_val:
        return True
    if "{DIR}" not in default:
        return False
    module_name = getattr(sub, "__module__", "") or ""
    module_dir = module_name.rsplit(".", 1)[0] if "." in module_name else module_name
    if not module_dir:
        return False
    return default.replace("{DIR}", module_dir) == func_val


def _iter_subclasses(cls: type):
    for sub in cls.__subclasses__():
        yield sub
        yield from _iter_subclasses(sub)


def _find_concrete_subclass(cls: type) -> type | None:
    cls_name = cls.__name__
    generic_name = f"Generic{cls_name}"
    for sub in _iter_subclasses(cls):
        if sub.__name__ == generic_name:
            return sub
    # Fall back to a direct subclass without an asset entrypoint.
    for sub in cls.__subclasses__():
        if not any(
            hasattr(sub, attr)
            for attr in ("mobile_manipulator_entrypoint", "asset_entrypoint")
        ):
            return sub
    return None


def _is_set_type(tp) -> bool:
    if tp is set:
        return True
    return get_origin(tp) is set


def _is_enum_type(tp) -> bool:
    try:
        return isinstance(tp, type) and issubclass(tp, enum.Enum)
    except TypeError:
        return False


def _is_callable_or_type(tp) -> bool:
    if tp is type:
        return True
    origin = get_origin(tp)
    if origin is type:
        return True
    if tp is collections.abc.Callable:
        return True
    if origin is collections.abc.Callable:
        return True
    try:
        if tp is Callable:
            return True
    except TypeError:
        pass
    return False


def _resolve_callable(value: str):
    """Resolve callable references; import fallback uses modules in ``sys.modules``."""
    mod_name, attr_name = value.split(":", 1)
    try:
        mod = importlib.import_module(mod_name)
    except (ImportError, ValueError):
        mod = sys.modules.get(mod_name)
        if mod is None:
            raise
    obj = mod
    for part in attr_name.split("."):
        obj = getattr(obj, part)
    return obj


def _resolve_enum(enum_cls: type, value: str):
    try:
        return enum_cls[value.strip().upper()]
    except KeyError:
        return enum_cls(value)
