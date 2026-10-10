"""Resolve Hydra assets; Isaac Sim imports stay lazy for lightweight helpers."""

from __future__ import annotations

import dataclasses
import enum
import importlib
from collections.abc import Callable as AbcCallable
from collections.abc import Iterable
from importlib.util import find_spec
from typing import TYPE_CHECKING, Any, Callable, Literal, get_args, get_origin

from srb.hydra._type_utils import (
    _unwrap_to_config_type,
    _unwrap_union_members,
)
from srb.hydra.type_resolution import get_type_hints_safe

if TYPE_CHECKING:
    from srb.core.asset import (
        AssetVariant,
        Manipulator,
        MobileManipulator,
        MobileRobot,
    )


def _resolve_module_path(path: str, *, call_callable: bool = True) -> Any:
    """Import a module attribute, optionally calling it when callable."""
    mod_name, attr_name = path.split(":", 1)
    if find_spec(mod_name) is None:
        raise ModuleNotFoundError(f"Module '{mod_name}' not found.")
    mod = importlib.import_module(mod_name)
    if not hasattr(mod, attr_name):
        raise AttributeError(f"Attribute '{attr_name}' not found in '{mod_name}'.")
    attr = getattr(mod, attr_name)
    if call_callable and callable(attr):
        return attr()
    return attr


def _is_str_type(field_type: Any) -> bool:
    members = _unwrap_union_members(field_type)
    return bool(members) and all(
        m is str
        or (get_origin(m) is Literal and all(isinstance(a, str) for a in get_args(m)))
        for m in members
    )


def _is_callable_type(field_type: Any) -> bool:
    return any(
        member is AbcCallable or get_origin(member) is AbcCallable
        for member in _unwrap_union_members(field_type)
    )


def resolve_asset_variant(name: str) -> AssetVariant:
    """Resolve a string to an :class:`AssetVariant` enum member."""
    from srb.core.asset import AssetVariant

    variant = AssetVariant.from_str(name)
    if variant is None:
        raise ValueError(
            f"'{name}' is not valid for AssetVariant. "
            f"Valid values: {[v.name for v in AssetVariant]}"
        )
    return variant


def resolve_manipulator(name: str) -> Manipulator:
    """Resolve a manipulator name, optionally with ``+end_effector``."""
    # Reject trailing ``+`` before importing Isaac Sim so CLI typos fail fast.
    base, _, suffix = name.partition("+")
    if base and "+" in name and not suffix:
        raise ValueError(
            f"Cannot resolve a manipulator from {name!r} — a trailing "
            "'+' must be followed by an end-effector name."
        )

    from srb.core.asset import (
        Manipulator,
        ManipulatorRegistry,
        Tool,
    )

    if "+" in name:
        manipulator_name, end_effector_name = name.split("+", 1)

        end_effector = None
        if end_effector_name:
            end_effector_class = next(
                (
                    cls
                    for cls in Tool.object_registry()
                    if cls.name() == end_effector_name
                ),
                None,
            )
            if end_effector_class is None:
                registered = sorted(cls.name() for cls in Tool.object_registry())
                raise ValueError(
                    f'Tool "{end_effector_name}" is not registered under '
                    f"{Tool.__name__}. Registered tools: {registered}"
                )
            end_effector = end_effector_class()  # type: ignore[misc]

        if not manipulator_name:
            raise ValueError(
                "Cannot resolve a manipulator from '+end_effector' alone — "
                "a base manipulator name is required."
            )

        manipulator_class = ManipulatorRegistry.get_by_name(manipulator_name)
        if manipulator_class is None:
            registered = sorted(m.name() for m in ManipulatorRegistry.values_inner())
            raise ValueError(
                f'Manipulator "{manipulator_name}" is not registered under '
                f"{Manipulator.__name__}. Registered manipulators: {registered}"
            )
        manipulator = manipulator_class()  # type: ignore[misc]
        if end_effector is not None:
            manipulator.end_effector = end_effector
        return manipulator

    manipulator_class = ManipulatorRegistry.get_by_name(name)
    if manipulator_class is None:
        raise ValueError(
            f'Manipulator "{name}" is not registered under {Manipulator.__name__}'
        )
    return manipulator_class()  # type: ignore[return-value]


def resolve_mobile_robot(name: str) -> MobileRobot:
    """Resolve a mobile robot name, optionally with ``+payload``."""
    # Reject trailing ``+`` before importing Isaac Sim so CLI typos fail fast.
    base, _, suffix = name.partition("+")
    if base and "+" in name and not suffix:
        raise ValueError(
            f"Cannot resolve a mobile robot from {name!r} — a trailing "
            "'+' must be followed by a payload or manipulator name."
        )

    from srb.core.asset import (
        MobileRobot,
        MobileRobotRegistry,
        Payload,
    )

    if "+" in name:
        # Try registered ground-manipulator combinations before base+payload.
        combo = _build_ground_manipulator_combo(name)
        if combo is not None:
            return combo  # type: ignore[return-value]

        mobile_robot_name, payload_name = name.split("+", 1)

        payload = None
        if payload_name:
            payload_class = next(
                (
                    cls
                    for cls in Payload.object_registry()
                    if cls.name() == payload_name
                ),
                None,
            )
            if payload_class is None:
                raise ValueError(
                    f'Payload "{payload_name}" is not registered under '
                    f"{Payload.__name__}"
                )
            payload = payload_class()  # type: ignore[misc]

        if not mobile_robot_name:
            raise ValueError(
                "Cannot resolve a mobile robot from '+payload' alone — "
                "a base mobile robot name is required."
            )

        mobile_robot_class = MobileRobotRegistry.get_by_name(mobile_robot_name)
        if mobile_robot_class is None:
            raise ValueError(
                f'MobileRobot "{mobile_robot_name}" is not registered under '
                f"{MobileRobot.__name__}"
            )
        mobile_robot = mobile_robot_class()  # type: ignore[misc]
        if payload is not None:
            mobile_robot.payload = payload
        return mobile_robot

    mobile_robot_class = MobileRobotRegistry.get_by_name(name)
    if mobile_robot_class is None:
        raise ValueError(
            f'MobileRobot "{name}" is not registered under {MobileRobot.__name__}'
        )
    return mobile_robot_class()  # type: ignore[return-value]


def resolve_mobile_manipulator(name: str) -> MobileManipulator:
    """Resolve a mobile manipulator by name from the registry."""
    from srb.core.asset import MobileManipulator, MobileManipulatorRegistry

    mobile_manipulator_class = MobileManipulatorRegistry.get_by_name(name)
    if mobile_manipulator_class is None:
        raise ValueError(
            f'MobileManipulator "{name}" is not registered under '
            f"{MobileManipulator.__name__}"
        )
    return mobile_manipulator_class()  # type: ignore[return-value]


def _build_ground_manipulator_combo(name: str):
    if "+" not in name:
        return None

    from srb.core.asset import ManipulatorRegistry, MobileRobotRegistry
    from srb.core.asset.robot.mobile.ground_robot import GroundRobot

    base_name, manipulator_spec = name.split("+", 1)
    if not base_name or not manipulator_spec:
        return None

    base_cls = MobileRobotRegistry.get_by_name(base_name)
    if base_cls is None or not issubclass(base_cls, GroundRobot):
        return None

    manipulator_name = manipulator_spec.split("+", 1)[0]
    if ManipulatorRegistry.get_by_name(manipulator_name) is None:
        return None

    from srb.assets.robot.mobile_manipulation import GenericGroundManipulator

    manipulator = resolve_manipulator(manipulator_spec)
    return GenericGroundManipulator(
        mobile_base=base_cls(),  # type: ignore[misc]
        manipulator=manipulator,
    )


def _composite_component_keys(name: str, instance: Any) -> set[str]:
    if "+" not in name or not name.split("+", 1)[1]:
        return set()
    try:
        from srb.core.asset import Manipulator, MobileManipulator, MobileRobot
    except ImportError:
        return set()
    if isinstance(instance, MobileManipulator):
        return set()
    if isinstance(instance, Manipulator):
        return {"end_effector"}
    if isinstance(instance, MobileRobot):
        return {"payload"}
    return set()


def _normalize_plain(value: Any) -> Any:
    if isinstance(value, (list, tuple)):
        return [_normalize_plain(item) for item in value]
    if isinstance(value, dict):
        return {key: _normalize_plain(item) for key, item in value.items()}
    return value


def _is_plain_sub_key(sub_key: str, sub_val: Any) -> bool:
    if sub_key == "action_mode":
        return False
    return not (isinstance(sub_val, dict) and "name" in sub_val)


def _split_plain_overrides(
    instance: Any, value: dict[str, Any], skip: set[str]
) -> dict[str, Any]:
    overrides: dict[str, Any] = {}
    for sub_key, sub_val in value.items():
        if sub_key in ("name", "_target_", "action_mode") or sub_key in skip:
            continue
        if isinstance(sub_val, dict) and "name" in sub_val:
            continue
        if not hasattr(instance, sub_key):
            raise ValueError(
                f"Asset {type(instance).__name__!r} has no attribute "
                f"{sub_key!r}; cannot apply Hydra override"
            )
        current = getattr(instance, sub_key)
        sub_val = _typed_config_value(current, sub_val)
        if _normalize_plain(current) == _normalize_plain(sub_val):
            continue
        overrides[sub_key] = sub_val
    return overrides


def _typed_config_value(current: Any, value: Any) -> Any:
    if not isinstance(value, dict) or "name" in value:
        return value
    cls: Any = None
    target = value.get("_target_")
    if isinstance(target, str) and ":" in target:
        mod_name, attr_name = target.split(":", 1)
        try:
            cls = getattr(importlib.import_module(mod_name), attr_name, None)
        except ImportError:
            cls = None
    if not isinstance(cls, type):
        cls = type(current) if _is_config_value(current) else None
    if cls is None or not _is_config_class(cls):
        return value
    from srb.hydra.create import create_instance

    return create_instance(
        cls, {key: item for key, item in value.items() if key != "_target_"}
    )


def _is_config_class(cls: type) -> bool:
    from pydantic import BaseModel

    return dataclasses.is_dataclass(cls) or issubclass(cls, BaseModel)


def _is_config_value(value: Any) -> bool:
    return (
        value is not None
        and not isinstance(value, type)
        and _is_config_class(type(value))
        and not _is_asset_instance(value)
    )


def _is_asset_instance(obj: Any) -> bool:
    cls = type(obj)
    return (
        obj is not None
        and not isinstance(obj, type)
        and callable(getattr(cls, "name", None))
        and hasattr(obj, "asset_cfg")
    )


def _asset_matches_spec(current: Any, spec: Any) -> bool:
    if isinstance(spec, dict) and "name" in spec:
        if spec["name"] is None:
            return current is None
        if not _is_asset_instance(current) or type(current).name() != spec["name"]:
            return False
        for key, value in spec.items():
            if key in ("name", "_target_"):
                continue
            if not hasattr(current, key):
                return False
            if not _asset_matches_spec(getattr(current, key), value):
                return False
        return True
    if isinstance(spec, enum.Enum):
        return False
    return bool(_normalize_plain(current) == _normalize_plain(spec))


def _class_default(cls: type, key: str) -> Any:
    from srb.hydra.extract import _is_missing, _iter_field_defaults

    for name, default in _iter_field_defaults(cls):
        if name == key:
            return None if _is_missing(default) else default
    return None


def _is_scenery_field(field_type: Any) -> bool:
    try:
        from srb.core.asset import Scenery
    except ImportError:
        return False
    return any(
        isinstance(member, type) and issubclass(member, Scenery)
        for member in _unwrap_union_members(field_type)
    )


def _mapping_value_changed(value: dict[str, Any], declared: Any, key: str) -> bool:
    if key not in value:
        return False
    if declared is None or not hasattr(declared, key):
        return value.get(key) is not None
    return _normalize_plain(value[key]) != _normalize_plain(getattr(declared, key))


def _region_selection_requested(value: dict[str, Any], declared: Any = None) -> bool:
    return any(
        value.get(key) is not None and _mapping_value_changed(value, declared, key)
        for key in ("lat_deg", "lon_deg", "size_m")
    )


def _selects_other_asset(selector: Any, declared: Any) -> bool:
    if selector is None or not _is_asset_instance(declared):
        return False
    if isinstance(selector, enum.Enum) or not isinstance(selector, str):
        return True
    return selector.split("+", 1)[0] != type(declared).name()


def _replaces_declared_asset(instance: Any, declared: Any) -> bool:
    return _is_asset_instance(declared) and type(instance) is not type(declared)


def _inherited_from_declared(declared: Any, key: str, item: Any) -> bool:
    if not hasattr(declared, key):
        return False
    current = getattr(declared, key)
    if isinstance(item, dict) and "name" in item:
        return _asset_matches_spec(current, item)
    item = _typed_config_value(current, item)
    return _normalize_plain(current) == _normalize_plain(item)


class _SubAssetFields(dict):
    """Explicit sub-fields to apply to the selected asset's *own* sub-asset."""


def _explicit_sub_asset_fields(
    mapping: dict[str, Any], explicit: frozenset[tuple[str, ...]]
) -> _SubAssetFields:
    fields = _SubAssetFields()
    for key, item in mapping.items():
        under = frozenset(path[1:] for path in explicit if path[:1] == (key,))
        if not under or key in ("name", "_target_"):
            continue
        if _keeps_whole_sub_asset(item, under):
            fields[key] = item
        else:
            fields[key] = _explicit_sub_asset_fields(item, under)
    return fields


def _keeps_whole_sub_asset(item: Any, under: frozenset[tuple[str, ...]]) -> bool:
    if not (isinstance(item, dict) and "name" in item):
        return True
    return () in under or ("name",) in under


def _without_inherited_asset_fields(
    value: dict[str, Any],
    declared: Any,
    explicit: frozenset[tuple[str, ...]] = frozenset(),
) -> dict[str, Any]:
    # Preserve explicit and changed values while dropping fields inherited
    # from the old asset, so the selected asset keeps its own defaults.
    result: dict[str, Any] = {}
    for key, item in value.items():
        under = frozenset(path[1:] for path in explicit if path[:1] == (key,))
        if key in ("name", "_target_") or isinstance(item, _SubAssetFields):
            result[key] = item
        elif under:
            result[key] = (
                item
                if _keeps_whole_sub_asset(item, under)
                else _explicit_sub_asset_fields(item, under)
            )
        elif not _inherited_from_declared(declared, key, item):
            result[key] = item
    return result


def _without_empty_region_placeholders(value: dict[str, Any]) -> dict[str, Any]:
    if any(
        value.get(key) is not None for key in ("body", "lat_deg", "lon_deg", "size_m")
    ):
        return value
    return {
        key: item
        for key, item in value.items()
        if key not in ("body", "lat_deg", "lon_deg", "size_m")
    }


def _asset_variant_from_selector(value: Any) -> Any | None:
    try:
        from srb.core.asset import AssetVariant
    except ImportError:
        return None
    if isinstance(value, AssetVariant):
        return value
    if isinstance(value, str):
        return AssetVariant.from_str(value)
    return None


def _rebuild_with_overrides(instance: Any, overrides: dict[str, Any]) -> Any:
    if not overrides:
        return instance
    kwargs = {key: getattr(instance, key) for key in instance.model_fields_set}
    kwargs.update(overrides)
    try:
        return type(instance)(**kwargs)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"Cannot apply Hydra overrides {sorted(overrides)} to asset "
            f"{type(instance).__name__!r}: {exc}"
        ) from exc


def resolve_robot(name: str) -> Any:
    combo = _build_ground_manipulator_combo(name)
    if combo is not None:
        return combo

    if "+" in name:
        from srb.core.asset import ManipulatorRegistry, MobileRobotRegistry

        base = name.split("+", 1)[0]
        if base:
            if ManipulatorRegistry.get_by_name(base) is not None:
                return resolve_manipulator(name)
            if MobileRobotRegistry.get_by_name(base) is not None:
                return resolve_mobile_robot(name)

    from srb.core.asset import RobotRegistry

    return resolve_from_registry(RobotRegistry)(name)


def resolve_from_registry(registry) -> Callable[[str], Any]:

    def _resolver(name: str) -> Any:
        try:
            from srb.core.asset import AssetVariant

            variant = AssetVariant.from_str(name)
            if variant is not None:
                return variant
        except ImportError:
            pass
        cls = registry.get_by_name(name)
        if cls is None:
            raise ValueError(f'"{name}" is not registered in {registry.__name__}')
        return cls()

    return _resolver


def _build_resolvers() -> list[tuple[type, Callable[[str], Any]]]:
    from simforge import Asset as SimForgeAsset
    from simforge import AssetRegistry as SimForgeAssetRegistry

    from srb.core.action import ActionGroup, ActionGroupRegistry
    from srb.core.asset import (
        Asset,
        AssetRegistry,
        AssetVariant,
        Manipulator,
        MobileManipulator,
        MobileRobot,
        Object,
        ObjectRegistry,
        Robot,
        Scenery,
        SceneryRegistry,
    )

    return [
        (AssetVariant, resolve_asset_variant),
        (MobileManipulator, resolve_mobile_manipulator),
        (Manipulator, resolve_manipulator),
        (MobileRobot, resolve_mobile_robot),
        (Scenery, resolve_from_registry(SceneryRegistry)),
        (Object, resolve_from_registry(ObjectRegistry)),
        (Robot, resolve_robot),
        (Asset, resolve_from_registry(AssetRegistry)),
        (ActionGroup, resolve_from_registry(ActionGroupRegistry)),
        (SimForgeAsset, resolve_from_registry(SimForgeAssetRegistry)),
    ]


class AssetResolver:
    """Walks a config dict and resolves string values to typed objects."""

    RESOLVERS: list[tuple[type, Callable[[str], Any]]] | None = None

    def __init__(
        self, explicit_overrides: Iterable[Iterable[str]] | None = None
    ) -> None:
        self._explicit: frozenset[tuple[str, ...]] = frozenset(
            tuple(path) for path in explicit_overrides or ()
        )
        if AssetResolver.RESOLVERS is None:
            try:
                AssetResolver.RESOLVERS = _build_resolvers()
            except ImportError:
                AssetResolver.RESOLVERS = []

    def _resolve_real_terrain_mapping(
        self,
        field_type: Any,
        value: dict[str, Any],
        declared: Any,
        cfg_dict: dict[str, Any],
    ) -> tuple[bool, Any]:
        # RealTerrain construction can fetch or bake terrain, and region
        # selection needs a body instead of an ordinary registry entry.
        if not _is_scenery_field(field_type):
            return False, None

        region_requested = _region_selection_requested(value, declared)
        declared_is_named_real = (
            declared is not None
            and callable(getattr(type(declared), "name", None))
            and type(declared).name() == "real_terrain"
        )
        named_real = value.get("name") == "real_terrain"
        variant_body = (
            not declared_is_named_real
            and not named_real
            and _asset_variant_from_selector(value.get("name")) is not None
            and _mapping_value_changed(value, declared, "body")
        )
        if not (
            region_requested or declared_is_named_real or named_real or variant_body
        ):
            return False, None

        try:
            from srb.assets.scenery.real_terrain import RealTerrain
        except ImportError:
            return False, None

        declared_is_real = isinstance(declared, RealTerrain)
        real_name = RealTerrain.name()
        named_real = value.get("name") == real_name
        selector_changed = (
            declared_is_real
            and "name" in value
            and value.get("name") is not None
            and not named_real
        )
        if selector_changed and not region_requested:
            # Inherited RealTerrain fields must not override explicit selectors;
            # explicit bodies must reach validation with missing regions.
            return False, None
        if not (region_requested or named_real or declared_is_real or variant_body):
            return False, None

        model_fields = getattr(RealTerrain, "model_fields", {})
        allowed = set(model_fields) if model_fields else set()
        kwargs: dict[str, Any] = {}

        if allowed:
            unknown = sorted(
                name
                for name in value
                if name not in ("name", "_target_", "asset_cfg") and name not in allowed
            )
            if unknown:
                raise ValueError(
                    f"RealTerrain scenery has unknown Hydra fields: {unknown}"
                )

        if declared_is_real:
            for name in allowed:
                if name != "asset_cfg" and hasattr(declared, name):
                    kwargs[name] = getattr(declared, name)

        for name, item in value.items():
            if name in ("name", "_target_", "asset_cfg"):
                continue
            kwargs[name] = item

        body = kwargs.get("body")
        if body is None:
            domain = cfg_dict.get("domain")
            domain_name = getattr(domain, "name", domain)
            if domain_name is not None:
                body = str(domain_name).lower()
        if isinstance(body, enum.Enum):
            body = getattr(body, "name", str(body)).lower()
        elif body is not None:
            body = str(body).lower()
        if body not in {"moon", "mars", "earth"}:
            raise ValueError(
                "RealTerrain scenery needs body=moon|mars|earth for a region "
                f"override; got {body!r} (set env.domain or env.scenery.body)"
            )
        kwargs["body"] = body
        return True, RealTerrain(**kwargs)

    def _resolve_variant_mapping(
        self, field_type: Any, value: dict[str, Any]
    ) -> Any | None:
        if not _is_scenery_field(field_type):
            return None
        try:
            from srb.core.asset import AssetVariant
        except ImportError:
            return None
        if not any(
            member is AssetVariant for member in _unwrap_union_members(field_type)
        ):
            return None
        if _region_selection_requested(value) or value.get("body") is not None:
            return None
        # Collapse only registration-added empty keys; preserve other fields for
        # normal validation instead of hiding changed or invalid overrides.
        unsupported = sorted(
            key
            for key in value
            if key not in {"name", "_target_", "body", "lat_deg", "lon_deg", "size_m"}
        )
        if unsupported:
            raise ValueError(
                "AssetVariant scenery selector has unsupported Hydra fields: "
                f"{unsupported}"
            )
        return _asset_variant_from_selector(value.get("name"))

    def _explicit_under(self, path: tuple[str, ...]) -> frozenset[tuple[str, ...]]:
        depth = len(path)
        return frozenset(
            explicit[depth:]
            for explicit in self._explicit
            if len(explicit) > depth and explicit[:depth] == path
        )

    def resolve(
        self, cls: type, cfg_dict: dict, *, _path: tuple[str, ...] = ()
    ) -> None:
        """Resolve string and dict values in *cfg_dict* in-place."""
        hints = get_type_hints_safe(cls)

        for key, value in list(cfg_dict.items()):
            # ``_target_`` is consumed by create_instance; resolving it here
            # would instantiate the target only to discard it.
            if key == "_target_":
                continue

            field_type = hints.get(key)

            # Preserve typed enums: str enums otherwise reach registry lookup,
            # where AssetVariant's auto() value is not its member name.
            if isinstance(value, enum.Enum):
                continue

            # Dynamic dict keys bypass asset shortcut rewriting, so resolve each
            # entry against the mapping's value type here.
            if field_type is not None and isinstance(value, dict):
                if get_origin(field_type) is dict:
                    type_args = get_args(field_type)
                    entry_type = type_args[1] if len(type_args) == 2 else None
                    if (
                        entry_type is not None
                        and self._find_resolver(entry_type) is not None
                    ):
                        for sub_key, sub_val in list(value.items()):
                            value[sub_key] = self._resolve_entry(entry_type, sub_val)
                        continue

            if isinstance(value, str):
                # Plain string fields keep values such as ``"blosc:zstd"``.
                if ":" in value and not _is_str_type(field_type):
                    try:
                        cfg_dict[key] = _resolve_module_path(
                            value,
                            call_callable=not _is_callable_type(field_type),
                        )
                        continue
                    except (ImportError, AttributeError, TypeError, ValueError):
                        pass

                if field_type is None:
                    continue

                resolver_fn = self._find_resolver(field_type)
                if resolver_fn is not None:
                    # Propagate invalid asset references; downstream code expects
                    # typed objects and would otherwise fail with AttributeError.
                    cfg_dict[key] = resolver_fn(value)

            elif isinstance(value, dict):
                declared = _class_default(cls, key)
                region_requested = _region_selection_requested(value, declared)
                declared_is_real_terrain = (
                    declared is not None
                    and callable(getattr(type(declared), "name", None))
                    and type(declared).name() == "real_terrain"
                )
                if "name" in value and (
                    not region_requested or declared_is_real_terrain
                ):
                    # Keep typed instances with fields OmegaConf cannot represent;
                    # region mappings still need selector handling.
                    if _asset_matches_spec(declared, value):
                        cfg_dict[key] = declared
                        continue

                # When a selector changes, inherited defaults could override the
                # new asset (for example, a Franka hand on Kinova). Keep explicit
                # or changed values for validation; reset nested defaults cautiously.
                inherited_source = (
                    declared if not _path or declared_is_real_terrain else None
                )
                explicit = self._explicit_under((*_path, key))
                if (
                    "name" in value
                    and not region_requested
                    and _selects_other_asset(value.get("name"), inherited_source)
                ):
                    value = _without_inherited_asset_fields(
                        value, inherited_source, explicit
                    )
                    cfg_dict[key] = value

                handled, real_terrain = self._resolve_real_terrain_mapping(
                    field_type, value, declared, cfg_dict
                )
                if handled:
                    cfg_dict[key] = real_terrain
                    continue

                variant = self._resolve_variant_mapping(field_type, value)
                if variant is not None:
                    cfg_dict[key] = variant
                    continue

                    # Remove registration placeholders when selecting named scenery
                    # so they do not become invalid asset overrides.
                if "name" in value and not region_requested:
                    value = _without_empty_region_placeholders(value)

                if "name" in value and value["name"] is None:
                    cfg_dict[key] = None
                    continue
                if "name" in value and isinstance(value["name"], str):
                    resolver_fn = (
                        self._find_resolver(field_type) if field_type else None
                    )
                    if resolver_fn is not None:
                        instance = resolver_fn(value["name"])
                        if _replaces_declared_asset(instance, inherited_source):
                            # A composite selector may share a base name but resolve
                            # to another class.
                            value = _without_inherited_asset_fields(
                                value, inherited_source, explicit
                            )
                        # Preserve attached composite suffixes from overwrite by
                        # the serialized default sub-component.
                        composed_keys = _composite_component_keys(
                            value["name"], instance
                        )
                        sub_asset_fields = {
                            sub_key: sub_val
                            for sub_key, sub_val in value.items()
                            if isinstance(sub_val, _SubAssetFields)
                        }
                        value = {
                            sub_key: sub_val
                            for sub_key, sub_val in value.items()
                            if sub_key not in sub_asset_fields
                        }
                        plain_overrides = _split_plain_overrides(
                            instance, value, composed_keys
                        )
                        instance = _rebuild_with_overrides(instance, plain_overrides)
                        for sub_key, fields in sub_asset_fields.items():
                            self._apply_sub_asset_fields(instance, sub_key, fields)
                        for sub_key, sub_val in value.items():
                            if sub_key in ("name", "_target_"):
                                continue
                            if sub_key in composed_keys:
                                continue
                            if _is_plain_sub_key(sub_key, sub_val):
                                # Constructor rebuild runs model_post_init; setattr
                                # would bypass it and could replace tuples with lists.
                                continue
                            if not hasattr(instance, sub_key):
                                raise ValueError(
                                    f"Asset {type(instance).__name__!r} has "
                                    f"no attribute {sub_key!r}; cannot apply "
                                    f"Hydra override"
                                )
                            if (
                                isinstance(sub_val, dict)
                                and "name" in sub_val
                                and sub_val["name"] is None
                            ):
                                setattr(instance, sub_key, None)
                                continue
                            if (
                                isinstance(sub_val, dict)
                                and "name" in sub_val
                                and isinstance(sub_val["name"], str)
                            ):
                                if _asset_matches_spec(
                                    getattr(instance, sub_key, None), sub_val
                                ):
                                    continue
                                sub_field_type = get_type_hints_safe(
                                    type(instance)
                                ).get(sub_key)
                                sub_resolver = (
                                    self._find_resolver(sub_field_type)
                                    if sub_field_type
                                    else None
                                )
                                if sub_resolver is not None:
                                    sub_instance = sub_resolver(sub_val["name"])
                                    declared_sub = (
                                        getattr(inherited_source, sub_key, None)
                                        if _is_asset_instance(inherited_source)
                                        else None
                                    )
                                    if _replaces_declared_asset(
                                        sub_instance, declared_sub
                                    ):
                                        sub_val = _without_inherited_asset_fields(
                                            sub_val,
                                            declared_sub,
                                            self._explicit_under(
                                                (*_path, key, sub_key)
                                            ),
                                        )
                                    sub_plain = _split_plain_overrides(
                                        sub_instance, sub_val, set()
                                    )
                                    sub_instance = _rebuild_with_overrides(
                                        sub_instance, sub_plain
                                    )
                                    for sk, sv in sub_val.items():
                                        if sk in ("name", "_target_"):
                                            continue
                                        if isinstance(sv, _SubAssetFields):
                                            self._apply_sub_asset_fields(
                                                sub_instance, sk, sv
                                            )
                                            continue
                                        if isinstance(sv, dict) and isinstance(
                                            sv.get("name"), str
                                        ):
                                            nested_type = get_type_hints_safe(
                                                type(sub_instance)
                                            ).get(sk)
                                            nested_resolver = (
                                                self._find_resolver(nested_type)
                                                if nested_type is not None
                                                else None
                                            )
                                            if nested_resolver is not None:
                                                nested_instance = nested_resolver(
                                                    sv["name"]
                                                )
                                                declared_nested = getattr(
                                                    sub_instance, sk, None
                                                )
                                                if _replaces_declared_asset(
                                                    nested_instance, declared_nested
                                                ):
                                                    sv = (
                                                        _without_inherited_asset_fields(
                                                            sv,
                                                            declared_nested,
                                                            self._explicit_under(
                                                                (
                                                                    *_path,
                                                                    key,
                                                                    sub_key,
                                                                    sk,
                                                                )
                                                            ),
                                                        )
                                                    )
                                                nested_instance = (
                                                    _rebuild_with_overrides(
                                                        nested_instance,
                                                        _split_plain_overrides(
                                                            nested_instance, sv, set()
                                                        ),
                                                    )
                                                )
                                                setattr(
                                                    sub_instance, sk, nested_instance
                                                )
                                                continue
                                        if _is_plain_sub_key(sk, sv):
                                            continue
                                        if not hasattr(sub_instance, sk):
                                            raise ValueError(
                                                f"Asset {type(sub_instance).__name__!r} has "
                                                f"no attribute {sk!r}; cannot apply "
                                                f"Hydra override"
                                            )
                                        setattr(sub_instance, sk, sv)
                                    setattr(instance, sub_key, sub_instance)
                                    continue
                            setattr(instance, sub_key, sub_val)
                        cfg_dict[key] = instance
                        continue

                inner_type = _unwrap_to_config_type(field_type) if field_type else None
                target = value.get("_target_")
                if target and isinstance(target, str) and ":" in target:
                    try:
                        inner_type = _resolve_module_path(target)
                    except (ImportError, AttributeError, TypeError, ValueError):
                        pass
                if inner_type is not None:
                    if not isinstance(inner_type, type):
                        inner_type = type(inner_type)
                    inner_hints = get_type_hints_safe(inner_type)
                    str_keys = [
                        k
                        for k, v in value.items()
                        if isinstance(v, str) and k != "_target_"
                    ]
                    if str_keys and not all(k in inner_hints for k in str_keys):
                        from srb.hydra.create import _find_concrete_subclass

                        concrete = _find_concrete_subclass(inner_type)
                        if concrete is not None:
                            inner_type = concrete
                    self.resolve(inner_type, value, _path=(*_path, key))

    def _apply_sub_asset_fields(
        self, owner: Any, sub_key: str, fields: _SubAssetFields
    ) -> None:
        current = getattr(owner, sub_key, None)
        if not _is_asset_instance(current):
            raise ValueError(
                f"Asset {type(owner).__name__!r} has no {sub_key!r} sub-asset; "
                f"cannot apply Hydra overrides {sorted(fields)}"
            )
        nested = {k: v for k, v in fields.items() if isinstance(v, _SubAssetFields)}
        own = {k: v for k, v in fields.items() if k not in nested}
        current = _rebuild_with_overrides(
            current, _split_plain_overrides(current, own, set())
        )
        for field_name, field_value in own.items():
            if _is_plain_sub_key(field_name, field_value):
                continue
            if not hasattr(current, field_name):
                raise ValueError(
                    f"Asset {type(current).__name__!r} has no attribute "
                    f"{field_name!r}; cannot apply Hydra override"
                )
            if isinstance(field_value, dict) and "name" in field_value:
                field_type = get_type_hints_safe(type(current)).get(field_name)
                field_value = self._resolve_entry(field_type, field_value)
            setattr(current, field_name, field_value)
        for field_name, nested_fields in nested.items():
            self._apply_sub_asset_fields(current, field_name, nested_fields)
        setattr(owner, sub_key, current)

    def _resolve_entry(self, entry_type, value):
        if isinstance(value, enum.Enum):
            return value

        resolver_fn = self._find_resolver(entry_type) if entry_type else None

        if isinstance(value, str):
            if ":" in value:
                try:
                    return _resolve_module_path(
                        value,
                        call_callable=not _is_callable_type(entry_type),
                    )
                except (ImportError, AttributeError, TypeError, ValueError):
                    pass
            if resolver_fn is not None:
                return resolver_fn(value)
            return value

        if isinstance(value, dict):
            if "name" in value and value["name"] is None:
                return None
            if isinstance(value.get("name"), str) and resolver_fn is not None:
                instance = resolver_fn(value["name"])
                plain_overrides = _split_plain_overrides(instance, value, set())
                instance = _rebuild_with_overrides(instance, plain_overrides)
                for sub_key, sub_val in value.items():
                    if sub_key in ("name", "_target_"):
                        continue
                    if _is_plain_sub_key(sub_key, sub_val):
                        continue
                    if not hasattr(instance, sub_key):
                        raise ValueError(
                            f"Asset {type(instance).__name__!r} has no attribute "
                            f"{sub_key!r}; cannot apply Hydra override"
                        )
                    setattr(instance, sub_key, sub_val)
                return instance

        return value

    def _find_resolver(self, field_type) -> Callable[[str], Any] | None:
        assert self.RESOLVERS is not None
        seen: set[int] = set()
        fns: list[Callable[[str], Any]] = []
        # RESOLVERS order wins over union annotation order when several match.
        for resolver_type, resolver_fn in self.RESOLVERS:
            if id(resolver_fn) in seen:
                continue
            for candidate in _unwrap_union_members(field_type):
                try:
                    if isinstance(candidate, type) and issubclass(
                        candidate, resolver_type
                    ):
                        fns.append(resolver_fn)
                        seen.add(id(resolver_fn))
                        break
                except TypeError:
                    continue
        if not fns:
            return None
        if len(fns) == 1:
            return fns[0]

        def _chain(name: str) -> Any:
            last_err: Exception | None = None
            for fn in fns:
                try:
                    return fn(name)
                except (ValueError, KeyError) as e:
                    last_err = e
                    continue
            assert last_err is not None
            raise last_err

        return _chain
