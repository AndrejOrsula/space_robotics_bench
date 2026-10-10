"""Build and publish SRB offline caches.

Separate from ``srb.utils.cache`` to keep readers free of registry imports.
Write through ``_cache`` so patches to its writer intercept all writes.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Union

from srb.utils import cache as _cache
from srb.utils.cache import (
    _CACHE_BUILD_ROOT,
    OFFLINE_CACHE_REQUIRED_MEMBERS,
    OfflineCacheArtifact,
    OfflineCacheSnapshot,
    _cache_generation_semantic_problems,
    _offline_cache_artifacts_at,
    _read_offline_cache,
    doctor_offline_srb_cache,
    read_offline_srb_action_modes_cache,
    read_offline_srb_agent_config_keys_cache,
    read_offline_srb_agent_enum_values_cache,
    read_offline_srb_config_keys_cache,
    read_offline_srb_enum_values_cache,
    read_offline_srb_env_cache,
    read_offline_srb_hardware_interface_cache,
    read_offline_srb_task_defaults_cache,
)
from srb.utils.isaacsim import is_isaacsim_initialized
from srb.utils.path import (
    SRB_ACTION_MODES_CACHE_PATH,
    SRB_AGENT_CONFIG_KEYS_CACHE_PATH,
    SRB_AGENT_ENUM_VALUES_CACHE_PATH,
    SRB_CONFIG_KEYS_CACHE_PATH,
    SRB_ENUM_VALUES_CACHE_PATH,
    SRB_ENV_CACHE_PATH,
    SRB_HARDWARE_INTERFACE_CACHE_PATH,
    SRB_OBJECT_CACHE_PATH,
    SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH,
    SRB_ROBOT_CACHE_PATH,
    SRB_SCENERY_CACHE_PATH,
    SRB_TASK_DEFAULTS_CACHE_PATH,
)
from srb.utils.publication import ArtifactPublication

# Spellings accepted by an enum's `_missing_` hook, which is not introspectable.
_ENUM_COMPLETION_ALIASES: dict[str, tuple[str, ...]] = {
    "srb.core.sensor.category.SensorCategory": ("events",),
}
_AGENT_VALUE_PLACEHOLDERS = frozenset({"MISSING", "REQUIRED_AT_RUNTIME"})
_COMPLETABLE_AGENT_VALUE = re.compile(r"[A-Za-z0-9_.:/+-]+")

if TYPE_CHECKING:
    from srb.core.asset import (
        ManipulatorType,
        MobileManipulatorType,
        MobileRobotType,
        ObjectType,
        RobotType,
        SceneryType,
    )


def update_offline_srb_cache(*, generation_root: Path | None = None) -> None:
    """Build and atomically publish a validated offline-cache generation.

    Updaters write to private staging; legacy fixed cache files stay untouched.
    Publication promotes it only after all twelve members validate.
    """
    generation_root = generation_root or SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH

    def build(staging: Path) -> None:
        token = _CACHE_BUILD_ROOT.set(staging)
        try:
            update_offline_srb_env_cache()
            update_offline_srb_robot_cache()
            update_offline_srb_scenery_cache()
            update_offline_srb_object_cache()
            update_offline_srb_hardware_interface_cache()
            update_offline_srb_config_keys_cache()
            update_offline_srb_enum_values_cache()
            update_offline_srb_agent_caches()
            update_offline_srb_action_modes_cache()
            update_offline_srb_task_defaults_cache()
            update_offline_srb_gui_catalog_cache(cache_dir=staging)

            artifacts = _offline_cache_artifacts_at(staging)
            _validate_offline_cache_build(artifacts)
        finally:
            _CACHE_BUILD_ROOT.reset(token)

    ArtifactPublication.generation(generation_root).publish(
        build,
        required_members=OFFLINE_CACHE_REQUIRED_MEMBERS,
    )


def _validate_offline_cache_build(
    artifacts: Sequence[OfflineCacheArtifact],
) -> None:
    doctor = doctor_offline_srb_cache(artifacts=artifacts)
    if not doctor.ok:
        invalid = ", ".join(
            entry.name for entry in doctor.entries if entry.state != "ok"
        )
        raise RuntimeError(
            "Cannot publish an SRB cache generation with invalid artifacts: " + invalid
        )
    semantic_problems = _cache_generation_semantic_problems(artifacts)
    if semantic_problems:
        raise RuntimeError(
            "Cannot publish an incomplete SRB cache generation: "
            + "; ".join(semantic_problems)
        )


def update_offline_srb_env_cache():
    from srb.tasks import ensure_registered
    from srb.utils import logging
    from srb.utils.registry import get_srb_tasks

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the cache of registered environments will likely fail because Isaac Sim is not initialized"
        )

    ensure_registered()
    from srb import tasks as _  # noqa: F401

    registered_envs = sorted(get_srb_tasks().keys())

    if not registered_envs:
        logging.warning(
            "Cannot update the cache of registered environments because no environments are registered"
        )
        return

    _cache._write_cache_if_changed(
        SRB_ENV_CACHE_PATH,
        registered_envs,
        read_offline_srb_env_cache(),
        label="registered environments",
    )


def update_offline_srb_robot_cache():
    from srb.core.asset import (
        ManipulatorRegistry,
        ManipulatorType,
        MobileManipulatorRegistry,
        MobileManipulatorType,
        MobileRobotRegistry,
        MobileRobotType,
        RobotType,
    )
    from srb.utils import logging

    registry_map = {
        RobotType.MOBILE_ROBOT: (MobileRobotRegistry, MobileRobotType),
        RobotType.MANIPULATOR: (ManipulatorRegistry, ManipulatorType),
        RobotType.MOBILE_MANIPULATOR: (
            MobileManipulatorRegistry,
            MobileManipulatorType,
        ),
    }

    robots_cache = {}

    for robot_type, (registry, _subtype_enum_cls) in registry_map.items():
        type_data = {
            subtype.name.lower(): sorted(robot.name() for robot in robots)
            for subtype, robots in registry.items()
            if robots
        }
        if type_data:
            robots_cache[robot_type.name.lower()] = type_data

    if not robots_cache:
        logging.warning(
            "Cannot update the cache of registered robots because no robots are registered"
        )
        return

    _cache._write_cache_if_changed(
        SRB_ROBOT_CACHE_PATH,
        robots_cache,
        read_offline_srb_robot_cache(),
        label="registered robots",
    )


def update_offline_srb_scenery_cache():
    from srb.core.asset import SceneryRegistry
    from srb.utils import logging

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the cache of registered sceneries will likely fail because Isaac Sim is not initialized"
        )

    from srb.assets import scenery as _  # noqa: F401

    sceneries = {}

    for scenery_type, _sceneries in SceneryRegistry.items():
        if _sceneries:
            names = sorted(scenery.name() for scenery in _sceneries)
            sceneries[scenery_type.name.lower()] = names

    if not sceneries:
        logging.warning(
            "Cannot update the cache of registered sceneries because no sceneries are registered"
        )
        return

    _cache._write_cache_if_changed(
        SRB_SCENERY_CACHE_PATH,
        sceneries,
        read_offline_srb_scenery_cache(),
        label="registered sceneries",
    )


def update_offline_srb_object_cache():
    from srb.core.asset import ObjectRegistry
    from srb.utils import logging

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the cache of registered objects will likely fail because Isaac Sim is not initialized"
        )

    from srb.assets import object as _  # noqa: F401

    objects = {}

    for object_type, _objects in ObjectRegistry.items():
        if _objects:
            names = sorted(object.name() for object in _objects)
            objects[object_type.name.lower()] = names

    if not objects:
        logging.warning(
            "Cannot update the cache of registered objects because no objects are registered"
        )
        return

    _cache._write_cache_if_changed(
        SRB_OBJECT_CACHE_PATH,
        objects,
        read_offline_srb_object_cache(),
        label="registered objects",
    )


def update_offline_srb_hardware_interface_cache():
    from srb.interfaces.sim_to_real import hardware as _  # noqa: F401
    from srb.interfaces.sim_to_real.core.hardware import HardwareInterfaceRegistry
    from srb.utils import logging

    hardware_interfaces = sorted(
        hardware_interface.class_name()
        for hardware_interface in HardwareInterfaceRegistry.registry
    )

    if not hardware_interfaces:
        logging.warning(
            "Cannot update the cache of registered hardware interfaces because no hardware interfaces are registered"
        )
        return

    _cache._write_cache_if_changed(
        SRB_HARDWARE_INTERFACE_CACHE_PATH,
        hardware_interfaces,
        read_offline_srb_hardware_interface_cache(),
        label="registered hardware interfaces",
    )


def _parse_robot_cache(raw: object):
    if not isinstance(raw, dict):
        return None

    from srb.core.asset import (
        ManipulatorType,
        MobileManipulatorType,
        MobileRobotType,
        RobotType,
    )

    subtype_cls_for: dict[
        "RobotType",
        type["MobileRobotType | ManipulatorType | MobileManipulatorType"] | None,
    ] = {
        RobotType.MOBILE_ROBOT: MobileRobotType,
        RobotType.MANIPULATOR: ManipulatorType,
        RobotType.MOBILE_MANIPULATOR: MobileManipulatorType,
    }

    converted: dict[
        RobotType,
        dict[
            Union[MobileRobotType, ManipulatorType, MobileManipulatorType],
            Sequence[str],
        ],
    ] = {}
    for rt_str, subtypes_dict in raw.items():
        rt_enum = RobotType[rt_str.upper()]
        inner: dict = {}
        subtype_cls = subtype_cls_for.get(rt_enum)
        if subtype_cls and isinstance(subtypes_dict, dict):
            for subtype_str, names in subtypes_dict.items():
                subtype_enum = subtype_cls[subtype_str.upper()]
                inner[subtype_enum] = names
        converted[rt_enum] = inner

    return converted


def _parse_scenery_cache(raw: object):
    if not isinstance(raw, dict):
        return None

    from srb.core.asset import SceneryType

    converted: dict["SceneryType", Sequence[str]] = {}
    for st_str, names in raw.items():
        if not (
            isinstance(names, list) and all(isinstance(item, str) for item in names)
        ):
            return None
        converted[SceneryType[st_str.upper()]] = names
    return converted


def _parse_object_cache(raw: object):
    if not isinstance(raw, dict):
        return None

    from srb.core.asset import ObjectType

    converted: dict["ObjectType", Sequence[str]] = {}
    for ot_str, names in raw.items():
        if not (
            isinstance(names, list) and all(isinstance(item, str) for item in names)
        ):
            return None
        converted[ObjectType[ot_str.upper()]] = names
    return converted


def read_offline_srb_robot_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> (
    dict[
        "RobotType",
        dict[
            "MobileRobotType | ManipulatorType | MobileManipulatorType",
            Sequence[str],
        ],
    ]
    | None
):
    return _read_offline_cache(
        SRB_ROBOT_CACHE_PATH,
        _parse_robot_cache,
        snapshot=snapshot,
    )


def read_offline_srb_scenery_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict["SceneryType", Sequence[str]] | None:
    return _read_offline_cache(
        SRB_SCENERY_CACHE_PATH,
        _parse_scenery_cache,
        snapshot=snapshot,
    )


def read_offline_srb_object_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict["ObjectType", Sequence[str]] | None:
    return _read_offline_cache(
        SRB_OBJECT_CACHE_PATH,
        _parse_object_cache,
        snapshot=snapshot,
    )


def update_offline_srb_config_keys_cache():
    from srb.tasks import ensure_registered
    from srb.utils import logging
    from srb.utils.registry import get_srb_tasks

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the config keys cache will likely fail because Isaac Sim is not initialized"
        )

    import gymnasium

    from srb.hydra.extract import extract_defaults_from_class

    ensure_registered()
    all_task_keys = {}
    for short_id, task_id in sorted(get_srb_tasks().items()):
        try:
            task_cfg_cls = gymnasium.spec(task_id).kwargs.get("task_cfg")
            if task_cfg_cls is None:
                continue
            defaults_dict = extract_defaults_from_class(task_cfg_cls)
            flat_keys = sorted(_flatten_keys(defaults_dict, "env."))
            all_task_keys[short_id] = flat_keys
        except Exception as e:
            logging.warning(f"Failed to extract config keys for {task_id}: {e}")

    if not all_task_keys:
        logging.warning("Cannot update config keys cache — no tasks processed")
        return

    common_keys = sorted(set.intersection(*(set(v) for v in all_task_keys.values())))
    all_task_keys["__common__"] = common_keys

    _cache._write_cache_if_changed(
        SRB_CONFIG_KEYS_CACHE_PATH,
        all_task_keys,
        read_offline_srb_config_keys_cache(),
        label="config keys",
    )


def _flatten_keys(d: dict, prefix: str = "") -> list[str]:
    keys = []
    for k, v in d.items():
        # Hide internal keys from completion.
        if k in ("name", "_target_"):
            continue
        full_key = f"{prefix}{k}" if prefix else k
        if isinstance(v, dict):
            # Named asset dicts are completable themselves and have sub-keys.
            if "name" in v:
                keys.append(full_key)
            keys.extend(_flatten_keys(v, f"{full_key}."))
        else:
            keys.append(full_key)
    return keys


def _unwrap_container_element(tp) -> tuple[str, list] | None:
    """Return a completion marker and element union members for containers."""
    import collections.abc
    from typing import get_args, get_origin

    from srb.hydra._type_utils import _unwrap_union_members

    origin = get_origin(tp)
    if origin is None:
        return None

    _SET_ORIGINS = (set, frozenset, collections.abc.Set)
    _LIST_ORIGINS = (list, tuple, collections.abc.Sequence)
    _DICT_ORIGINS = (dict, collections.abc.Mapping)

    if any(origin is c for c in _SET_ORIGINS):
        kind = "__set__"
    elif any(origin is c for c in _LIST_ORIGINS):
        kind = "__list__"
    elif any(origin is c for c in _DICT_ORIGINS):
        kind = "__dict__"
    else:
        return None

    args = get_args(tp)
    if not args:
        return None

    if kind == "__dict__":
        # Completion values for dicts are based on their key type.
        return (kind, _unwrap_union_members(args[0]))

    # Heterogeneous tuples do not have one completable element type.
    if origin is tuple and len(args) == 2 and args[1] is Ellipsis:
        return (kind, _unwrap_union_members(args[0]))
    elif origin is tuple and len(args) != 1:
        return None

    return (kind, _unwrap_union_members(args[0]))


def _detect_enum_values_for_class(cls, _prefix: str = "env.") -> dict[str, list[str]]:
    """Collect completable asset, enum, Literal, and bool values recursively."""
    import enum
    from typing import Literal, get_args, get_origin

    from srb.hydra._type_utils import (
        _unwrap_to_config_type,
        _unwrap_union_members,
    )
    from srb.hydra.type_resolution import get_type_hints_safe

    try:
        from srb.core.asset import AssetVariant, Object, Robot, Scenery

        _has_asset_types = True
    except Exception:
        _has_asset_types = False
        AssetVariant = Robot = Scenery = Object = None  # type: ignore[assignment]

    result: dict[str, list[str]] = {}
    type_hints = get_type_hints_safe(cls)

    for field_name, field_type in type_hints.items():
        if field_name.startswith("_"):
            continue
        key = f"{_prefix}{field_name}"
        members = _unwrap_union_members(field_type)

        values: list[str] = []

        # Resolve asset completions from registries.
        if _has_asset_types:
            from srb.core.asset.robot.mobile_manipulation.combined._metaclass import (
                CombinedMobileManipulator,
            )

            for m in members:
                try:
                    if isinstance(m, type) and issubclass(m, Robot):
                        # Composite robots are configured through their parts.
                        if issubclass(m, CombinedMobileManipulator):
                            from srb.core.asset import Manipulator
                            from srb.core.asset.robot.mobile import MobileRobot

                            # Future annotations may not resolve in subclasses.
                            base_type = MobileRobot
                            for parent in m.__mro__:
                                if (
                                    parent is not m
                                    and parent is not CombinedMobileManipulator
                                    and isinstance(parent, type)
                                    and issubclass(parent, MobileRobot)
                                    and parent is not MobileRobot
                                ):
                                    base_type = parent
                                    break
                            result[f"{key}.mobile_base"] = sorted(
                                r.name()
                                for r in base_type.robot_registry()
                                if issubclass(r, base_type)
                            )
                            # The manipulator completion uses the base type.
                            result[f"{key}.manipulator"] = sorted(
                                r.name()
                                for r in Manipulator.robot_registry()
                                if issubclass(r, Manipulator)
                            )
                            # Include object-valued manipulator fields.
                            manip_hints = get_type_hints_safe(Manipulator)
                            for sf, st in manip_hints.items():
                                if sf.startswith("_"):
                                    continue
                                for sm in _unwrap_union_members(st):
                                    try:
                                        if isinstance(sm, type) and issubclass(
                                            sm, Object
                                        ):
                                            result[f"{key}.manipulator.{sf}"] = sorted(
                                                o.name() for o in sm.object_registry()
                                            )
                                            break
                                    except TypeError:
                                        continue
                            # Include object-valued mobile-base fields.
                            mb_hints = get_type_hints_safe(base_type)
                            for sf, st in mb_hints.items():
                                if sf.startswith("_"):
                                    continue
                                for sm in _unwrap_union_members(st):
                                    try:
                                        if isinstance(sm, type) and issubclass(
                                            sm, Object
                                        ):
                                            result[f"{key}.mobile_base.{sf}"] = sorted(
                                                o.name() for o in sm.object_registry()
                                            )
                                            break
                                    except TypeError:
                                        continue
                            # Exclude generic composites configured through their parts.
                            values.extend(
                                sorted(
                                    r.name()
                                    for r in m.robot_registry()
                                    if issubclass(r, m)
                                    and not r.name().startswith("generic_")
                                )
                            )
                        else:
                            values.extend(
                                sorted(
                                    r.name()
                                    for r in m.robot_registry()
                                    if issubclass(r, m)
                                )
                            )
                    # Add completions for robot sub-asset fields.
                    if isinstance(m, type) and issubclass(m, Robot):
                        sub_hints = get_type_hints_safe(m)
                        for sub_field, sub_type in sub_hints.items():
                            if sub_field.startswith("_"):
                                continue
                            sub_members = _unwrap_union_members(sub_type)
                            for sm in sub_members:
                                try:
                                    if isinstance(sm, type) and issubclass(sm, Object):
                                        result[f"{key}.{sub_field}"] = sorted(
                                            o.name() for o in sm.object_registry()
                                        )
                                        break
                                except TypeError:
                                    continue

                    elif isinstance(m, type) and issubclass(m, Scenery):
                        from srb.core.asset import SceneryRegistry

                        values.extend(
                            sorted(
                                s.name()
                                for sceneries in SceneryRegistry.registry.values()
                                for s in sceneries
                            )
                        )
                    elif isinstance(m, type) and issubclass(m, Object):
                        from srb.core.asset import ObjectRegistry

                        values.extend(
                            sorted(
                                o.name()
                                for objects in ObjectRegistry.registry.values()
                                for o in objects
                            )
                        )
                except TypeError:
                    continue

        # Include AssetVariant names alongside asset references.
        if _has_asset_types:
            for m in members:
                try:
                    if m is AssetVariant:
                        values.extend(e.name for e in AssetVariant)
                        break
                except TypeError:
                    continue

        # Mark containers so shell completion can emit bracket/brace syntax.
        _container_kind = None
        if not values:
            _container_result = _unwrap_container_element(field_type)
            if _container_result is not None:
                _container_kind, members = _container_result

        # Add other enum types.
        if not values:
            for m in members:
                try:
                    if (
                        isinstance(m, type)
                        and issubclass(m, enum.Enum)
                        and (not _has_asset_types or m is not AssetVariant)
                    ):
                        values = [e.name for e in m]
                        values.extend(
                            _ENUM_COMPLETION_ALIASES.get(
                                f"{m.__module__}.{m.__qualname__}", ()
                            )
                        )
                        break
                except TypeError:
                    continue

        if not values:
            for m in members:
                origin = get_origin(m)
                if origin is Literal:
                    values = [str(a) for a in get_args(m)]
                    break
            if not values:
                origin = get_origin(field_type)
                if origin is Literal:
                    values = [str(a) for a in get_args(field_type)]

        if not values and (field_type is bool or bool in members):
            values = ["true", "false"]

        if values:
            if _container_kind is not None:
                values = [_container_kind] + values
            result[key] = values
        elif not values:
            nested_cfg_type = _unwrap_to_config_type(field_type)
            if nested_cfg_type is not None:
                result.update(
                    _detect_enum_values_for_class(nested_cfg_type, _prefix=f"{key}.")
                )

    return result


def update_offline_srb_enum_values_cache():
    from srb.tasks import ensure_registered
    from srb.utils import logging
    from srb.utils.registry import get_srb_tasks

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the enum values cache will likely fail because Isaac Sim is not initialized"
        )

    import gymnasium

    from srb.core.env.common.base.env_cfg import BaseEnvCfg

    ensure_registered()
    common_values = _detect_enum_values_for_class(BaseEnvCfg)

    all_enum_values: dict[str, dict[str, list[str]]] = {"__common__": common_values}
    failed: list[str] = []

    for short_id, task_id in sorted(get_srb_tasks().items()):
        try:
            task_cfg_cls = gymnasium.spec(task_id).kwargs.get("task_cfg")
            if task_cfg_cls is None:
                continue
            task_values = _detect_enum_values_for_class(task_cfg_cls)

            task_specific = {}
            for key, values in task_values.items():
                if key not in common_values or values != common_values[key]:
                    task_specific[key] = values

            if task_specific:
                all_enum_values[short_id] = task_specific
        except Exception as e:
            logging.warning(f"Failed to extract enum values for {task_id}: {e}")
            failed.append(task_id)

    # Tasks without task-specific values are omitted from the payload, so the
    # generation validator cannot detect a skipped task; refuse it here.
    if failed:
        raise RuntimeError(
            "Cannot publish an incomplete enum values cache; extraction failed "
            "for: " + ", ".join(failed)
        )

    _cache._write_cache_if_changed(
        SRB_ENUM_VALUES_CACHE_PATH,
        all_enum_values,
        read_offline_srb_enum_values_cache(),
        label="enum values",
    )


def _build_agent_cache_payloads() -> tuple[
    dict[str, list[str]],
    dict[str, list[str]],
]:
    import gymnasium

    from srb.cli._parsing import SupportedILAlgo, SupportedRLAlgo
    from srb.tasks import ensure_registered
    from srb.utils import logging
    from srb.utils.agent_config_catalog import (
        SUPPORTED_FRAMEWORKS,
        agent_config_entry_point_key,
    )
    from srb.utils.cfg import (
        load_cfg_from_registry,
    )
    from srb.utils.registry import get_srb_tasks

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the agent caches will likely fail because Isaac Sim "
            "is not initialized"
        )

    ensure_registered()
    tasks = get_srb_tasks()
    if not tasks:
        logging.warning("Cannot update agent caches — no tasks registered")
        return {}, {}

    all_algo_keys: dict[str, list[str]] = {}
    enum_values: dict[str, list[str]] = {}
    algo_names = [str(algo) for algo in (*SupportedRLAlgo, *SupportedILAlgo)]

    for algo_name in algo_names:
        entry_point_key = agent_config_entry_point_key(
            algo_name,
            SUPPORTED_FRAMEWORKS,
        )
        if entry_point_key is None:
            logging.warning(
                f"Cannot determine entry point key for algorithm: {algo_name}"
            )
            continue

        task_ids = [
            task_id
            for task_id in tasks.values()
            if gymnasium.spec(task_id).kwargs.get(entry_point_key) is not None
        ]
        if not task_ids:
            logging.warning(
                f"No task has entry point key '{entry_point_key}' "
                f"for algorithm: {algo_name}"
            )
            continue

        configs = []
        for task_id in task_ids:
            try:
                cfg = load_cfg_from_registry(task_id, entry_point_key)
            except Exception as e:
                logging.warning(f"Failed to load config for {algo_name}: {e}")
                continue
            if isinstance(cfg, dict):
                configs.append(cfg)
        if not configs:
            logging.warning(f"No valid configs for {algo_name}, skipping")
            continue

        flat_keys = sorted(
            set().union(*(_flatten_keys(cfg, "agent.") for cfg in configs))
        )
        if flat_keys:
            all_algo_keys[algo_name] = flat_keys
        for cfg in configs:
            _collect_enum_values_from_dict(cfg, "agent.", enum_values)

    # `__common__` alone passes generation validation, so a skipped algorithm
    # would silently lose its keys; every supported algorithm ships a config.
    skipped = [name for name in algo_names if name not in all_algo_keys]
    if skipped:
        raise RuntimeError(
            "Cannot publish incomplete agent caches; no config keys for: "
            + ", ".join(skipped)
        )

    if all_algo_keys:
        common_keys = sorted(
            set.intersection(*(set(v) for v in all_algo_keys.values()))
        )
        all_algo_keys["__common__"] = common_keys

    return all_algo_keys, enum_values


def update_offline_srb_agent_caches() -> None:
    from srb.utils import logging

    all_algo_keys, enum_values = _build_agent_cache_payloads()
    if not all_algo_keys:
        logging.warning(
            "Cannot update agent config keys cache — no algorithms processed"
        )
    else:
        _cache._write_cache_if_changed(
            SRB_AGENT_CONFIG_KEYS_CACHE_PATH,
            all_algo_keys,
            read_offline_srb_agent_config_keys_cache(),
            label="agent config keys",
        )

    if not enum_values:
        logging.warning("Cannot update agent enum values cache — no values found")
    else:
        _cache._write_cache_if_changed(
            SRB_AGENT_ENUM_VALUES_CACHE_PATH,
            enum_values,
            read_offline_srb_agent_enum_values_cache(),
            label="agent enum values",
        )


def _collect_enum_values_from_dict(d: dict, prefix: str, result: dict[str, list[str]]):
    """Collect value suggestions from an untyped config dictionary.

    Untyped configs declare no choice sets, so besides booleans the string
    defaults seen across all algorithms (e.g. SB3 ``policy``) are the finite
    candidates. Placeholders and values that are not single shell words are
    skipped.
    """
    for k, v in d.items():
        full_key = f"{prefix}{k}"
        if isinstance(v, dict):
            _collect_enum_values_from_dict(v, f"{full_key}.", result)
            continue
        if isinstance(v, bool):
            values = ["true", "false"]
        elif (
            isinstance(v, str)
            and v not in _AGENT_VALUE_PLACEHOLDERS
            and _COMPLETABLE_AGENT_VALUE.fullmatch(v)
        ):
            values = [v]
        else:
            continue
        merged = result.setdefault(full_key, [])
        merged.extend(value for value in values if value not in merged)


def _build_robot_action_modes_dict() -> dict[str, list[str]]:
    """Map assets to ordered action-mode keys without instantiating them."""
    from srb import assets as _  # noqa: F401  — trigger asset module imports
    from srb.core.asset import Robot
    from srb.core.asset.object.payload import ActivePayload
    from srb.core.asset.object.tool import ActiveTool

    result: dict[str, list[str]] = {}

    for robot_cls in Robot.robot_registry():
        modes = getattr(robot_cls, "action_modes", None)
        if not modes:
            continue
        result[robot_cls.name()] = list(modes.keys())

    for asset_cls in (*ActiveTool.object_registry(), *ActivePayload.object_registry()):
        modes = getattr(asset_cls, "action_modes", None)
        if not modes:
            continue
        result[asset_cls.name()] = list(modes.keys())

    return result


def update_offline_srb_action_modes_cache() -> None:
    from srb.utils import logging

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the action modes cache will likely fail because "
            "Isaac Sim is not initialized"
        )

    modes = _build_robot_action_modes_dict()
    if not modes:
        logging.warning(
            "Cannot update the action modes cache because no assets "
            "declare action_modes"
        )
        return

    _cache._write_cache_if_changed(
        SRB_ACTION_MODES_CACHE_PATH,
        modes,
        read_offline_srb_action_modes_cache(),
        label="action modes",
    )


def _task_default_domain_name(default: object | None) -> str | None:
    if default is None:
        return None

    name = getattr(default, "name", None)
    if isinstance(name, str):
        return name.upper()

    return str(default).upper()


def _task_default_scenery_name(
    default: object | None,
    is_asset_instance: Callable[[object], bool],
) -> str | None:
    if default is None:
        return None
    if not is_asset_instance(default):
        return None
    return type(default).name()


def _build_task_defaults_dict() -> dict[str, dict[str, str | bool | None]]:
    """Extract task defaults without instantiating configs."""
    import gymnasium

    from srb.hydra.extract import _is_asset_instance, _iter_field_defaults
    from srb.tasks import ensure_registered
    from srb.utils import logging
    from srb.utils.registry import get_srb_tasks

    ensure_registered()

    result: dict[str, dict[str, str | bool | None]] = {}

    for short_id, task_id in sorted(get_srb_tasks().items()):
        try:
            task_cfg_cls = gymnasium.spec(task_id).kwargs.get("task_cfg")
            if task_cfg_cls is None:
                continue

            defaults_per_task: dict[str, str | bool | None] = {
                "domain": None,
                "stack": False,
                "scenery": None,
                "robot": None,
                "manipulator": None,
                "mobile_base": None,
                "end_effector": None,
                "payload": None,
            }

            for field_name, default in _iter_field_defaults(task_cfg_cls):
                if field_name == "domain":
                    defaults_per_task["domain"] = _task_default_domain_name(default)
                    continue

                if field_name == "stack":
                    defaults_per_task["stack"] = (
                        default if isinstance(default, bool) else False
                    )
                    continue

                if field_name == "scenery":
                    defaults_per_task["scenery"] = _task_default_scenery_name(
                        default, _is_asset_instance
                    )
                    continue

                if field_name != "robot":
                    continue
                if not _is_asset_instance(default):
                    continue
                defaults_per_task["robot"] = type(default).name()

                # Composite robots flatten their base and keep only its name.
                mobile_base = getattr(default, "mobile_base", None)
                if _is_asset_instance(mobile_base):
                    defaults_per_task["mobile_base"] = type(mobile_base).name()
                elif isinstance(getattr(default, "mobile_base_name", None), str):
                    defaults_per_task["mobile_base"] = default.mobile_base_name

                if hasattr(default, "manipulator") and _is_asset_instance(
                    default.manipulator
                ):
                    defaults_per_task["manipulator"] = type(default.manipulator).name()
                    if (
                        hasattr(default.manipulator, "end_effector")
                        and default.manipulator.end_effector is not None
                        and _is_asset_instance(default.manipulator.end_effector)
                    ):
                        defaults_per_task["end_effector"] = type(
                            default.manipulator.end_effector
                        ).name()
                elif (
                    hasattr(default, "end_effector")
                    and default.end_effector is not None
                    and _is_asset_instance(default.end_effector)
                ):
                    defaults_per_task["end_effector"] = type(
                        default.end_effector
                    ).name()

                if (
                    hasattr(default, "payload")
                    and default.payload is not None
                    and _is_asset_instance(default.payload)
                ):
                    defaults_per_task["payload"] = type(default.payload).name()

            result[short_id] = defaults_per_task
        except Exception as exc:
            logging.warning(f"Failed to extract task defaults for {task_id}: {exc}")
            continue

    return result


def update_offline_srb_task_defaults_cache() -> None:
    from srb.utils import logging

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the task defaults cache will likely fail because "
            "Isaac Sim is not initialized"
        )

    defaults = _build_task_defaults_dict()
    if not defaults:
        logging.warning("Cannot update the task defaults cache — no tasks processed")
        return

    _cache._write_cache_if_changed(
        SRB_TASK_DEFAULTS_CACHE_PATH,
        defaults,
        read_offline_srb_task_defaults_cache(),
        label="task defaults",
    )


def update_offline_srb_gui_catalog_cache(*, cache_dir: Path | None = None) -> None:
    from srb.cli.gui_catalog import update_offline_srb_gui_catalog_cache as _update

    _update(cache_dir=cache_dir)
