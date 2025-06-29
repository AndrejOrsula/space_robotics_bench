from typing import TYPE_CHECKING, Dict, Sequence, Union

import orjson as json

from srb.utils.isaacsim import is_isaacsim_initialized
from srb.utils.path import (
    SRB_CACHE_PATH,
    SRB_ENV_CACHE_PATH,
    SRB_HARDWARE_INTERFACE_CACHE_PATH,
    SRB_OBJECT_CACHE_PATH,
    SRB_ROBOT_CACHE_PATH,
    SRB_SCENERY_CACHE_PATH,
)

if TYPE_CHECKING:
    from srb.core.asset import (
        ManipulatorType,
        MobileManipulatorType,
        MobileRobotType,
        ObjectType,
        RobotType,
        SceneryType,
    )


def update_offline_srb_cache():
    if not SRB_CACHE_PATH.is_dir():
        SRB_CACHE_PATH.mkdir(parents=True)

    update_offline_srb_env_cache()
    update_offline_srb_robot_cache()
    update_offline_srb_scenery_cache()
    update_offline_srb_object_cache()
    update_offline_srb_hardware_interface_cache()


def update_offline_srb_env_cache():
    from srb.utils import logging
    from srb.utils.registry import get_srb_tasks

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the cache of registered environments will likely fail because Isaac Sim is not initialized"
        )

    from srb import tasks as _  # noqa: F401

    registered_envs = sorted(map(lambda env: env.removeprefix("srb/"), get_srb_tasks()))

    if not registered_envs:
        logging.warning(
            "Cannot update the cache of registered environments because no environments are registered"
        )
        return

    current_cache = read_offline_srb_env_cache()
    if registered_envs == current_cache:
        logging.trace("The cache of registered environments is up-to-date")
        return

    with SRB_ENV_CACHE_PATH.open("wb") as f:
        f.write(json.dumps(registered_envs))
    logging.debug(
        f"Updated the cache of registered environments to {SRB_ENV_CACHE_PATH}"
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
    robots_cache_enum_keys = {}

    for robot_type, (registry, subtype_enum_cls) in registry_map.items():
        type_data = {
            subtype.name.lower(): sorted(robot.name() for robot in robots)
            for subtype, robots in registry.items()
            if robots
        }
        if type_data:
            robots_cache[robot_type.name.lower()] = type_data

            inner_enum_dict = {}
            for subtype_str, names in type_data.items():
                try:
                    subtype_enum = subtype_enum_cls[subtype_str.upper()]
                    inner_enum_dict[subtype_enum] = names
                except KeyError:
                    logging.warning(
                        f"Invalid robot subtype key '{subtype_str}' for type '{robot_type.name.lower()}', skipping."
                    )
            robots_cache_enum_keys[robot_type] = inner_enum_dict

    if not robots_cache:
        logging.warning(
            "Cannot update the cache of registered robots because no robots are registered"
        )
        return

    current_cache = read_offline_srb_robot_cache()
    if robots_cache_enum_keys == current_cache:
        logging.trace("The cache of registered robots is up-to-date")
        return

    with SRB_ROBOT_CACHE_PATH.open("wb") as f:
        f.write(json.dumps(robots_cache))
    logging.debug(f"Updated the cache of registered robots to {SRB_ROBOT_CACHE_PATH}")


def update_offline_srb_scenery_cache():
    from srb.core.asset import SceneryRegistry
    from srb.utils import logging

    if not is_isaacsim_initialized():
        logging.critical(
            "Updating the cache of registered sceneries will likely fail because Isaac Sim is not initialized"
        )

    from srb.assets import scenery as _  # noqa: F401

    sceneries = {}
    sceneries_enum_keys = {}

    for scenery_type, _sceneries in SceneryRegistry.items():
        if _sceneries:
            names = sorted(scenery.name() for scenery in _sceneries)
            sceneries[scenery_type.name.lower()] = names
            sceneries_enum_keys[scenery_type] = names

    if not sceneries:
        logging.warning(
            "Cannot update the cache of registered sceneries because no sceneries are registered"
        )
        return

    current_cache = read_offline_srb_scenery_cache()
    if sceneries_enum_keys == current_cache:
        logging.trace("The cache of registered sceneries is up-to-date")
        return

    with SRB_SCENERY_CACHE_PATH.open("wb") as f:
        f.write(json.dumps(sceneries))
    logging.debug(
        f"Updated the cache of registered sceneries to {SRB_SCENERY_CACHE_PATH}"
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
    objects_enum_keys = {}

    for object_type, _objects in ObjectRegistry.items():
        if _objects:
            names = sorted(object.name() for object in _objects)
            objects[object_type.name.lower()] = names
            objects_enum_keys[object_type] = names

    if not objects:
        logging.warning(
            "Cannot update the cache of registered objects because no objects are registered"
        )
        return

    current_cache = read_offline_srb_object_cache()
    if objects_enum_keys == current_cache:
        logging.trace("The cache of registered objects is up-to-date")
        return

    with SRB_OBJECT_CACHE_PATH.open("wb") as f:
        f.write(json.dumps(objects))
    logging.debug(f"Updated the cache of registered objects to {SRB_OBJECT_CACHE_PATH}")


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

    current_cache = read_offline_srb_hardware_interface_cache()
    if hardware_interfaces == current_cache:
        logging.trace("The cache of registered hardware interfaces is up-to-date")
        return

    with SRB_HARDWARE_INTERFACE_CACHE_PATH.open("wb") as f:
        f.write(json.dumps(hardware_interfaces))
    logging.debug(
        f"Updated the cache of registered hardware interfaces to {SRB_HARDWARE_INTERFACE_CACHE_PATH}"
    )


def read_offline_srb_env_cache() -> Sequence[str] | None:
    if not SRB_ENV_CACHE_PATH.exists():
        return None
    try:
        with SRB_ENV_CACHE_PATH.open("rb") as f:
            data = json.loads(f.read())
        if isinstance(data, list) and all(isinstance(item, str) for item in data):
            return data
        else:
            SRB_ENV_CACHE_PATH.unlink()
            return None
    except Exception:
        SRB_ENV_CACHE_PATH.unlink()
        return None


def read_offline_srb_robot_cache() -> (
    Dict[
        "RobotType",
        Dict[
            "MobileRobotType | ManipulatorType | MobileManipulatorType",
            Sequence[str],
        ],
    ]
    | None
):
    if not SRB_ROBOT_CACHE_PATH.exists():
        return None

    from srb.core.asset import (
        ManipulatorType,
        MobileManipulatorType,
        MobileRobotType,
        RobotType,
    )

    try:
        with SRB_ROBOT_CACHE_PATH.open("rb") as f:
            raw_cache = json.loads(f.read())
    except Exception:
        SRB_ROBOT_CACHE_PATH.unlink()
        return None

    if not isinstance(raw_cache, dict):
        SRB_ROBOT_CACHE_PATH.unlink()
        return None

    converted_cache: Dict[
        RobotType,
        Dict[
            Union[MobileRobotType, ManipulatorType, MobileManipulatorType],
            Sequence[str],
        ],
    ] = {}
    for rt_str, subtypes_dict in raw_cache.items():
        try:
            rt_enum = RobotType[rt_str.upper()]
            inner_converted_dict = {}
            subtype_enum_cls = None
            if rt_enum == RobotType.MOBILE_ROBOT:
                subtype_enum_cls = MobileRobotType
            elif rt_enum == RobotType.MANIPULATOR:
                subtype_enum_cls = ManipulatorType
            elif rt_enum == RobotType.MOBILE_MANIPULATOR:
                subtype_enum_cls = MobileManipulatorType

            if subtype_enum_cls and isinstance(subtypes_dict, dict):
                for subtype_str, names in subtypes_dict.items():
                    try:
                        subtype_enum = subtype_enum_cls[subtype_str.upper()]
                        inner_converted_dict[subtype_enum] = names
                    except Exception:
                        SRB_ROBOT_CACHE_PATH.unlink()
                        return None
            converted_cache[rt_enum] = inner_converted_dict
        except Exception:
            SRB_ROBOT_CACHE_PATH.unlink()
            return None

    return converted_cache


def read_offline_srb_scenery_cache() -> Dict["SceneryType", Sequence[str]] | None:
    if not SRB_SCENERY_CACHE_PATH.exists():
        return None

    from srb.core.asset import SceneryType

    try:
        with SRB_SCENERY_CACHE_PATH.open("rb") as f:
            raw_cache = json.loads(f.read())
    except Exception:
        SRB_SCENERY_CACHE_PATH.unlink()
        return None

    if not isinstance(raw_cache, dict):
        SRB_SCENERY_CACHE_PATH.unlink()
        return None

    converted_cache: Dict[SceneryType, Sequence[str]] = {}
    if isinstance(raw_cache, dict):
        for st_str, names in raw_cache.items():
            try:
                st_enum = SceneryType[st_str.upper()]
                if isinstance(names, list) and all(
                    isinstance(item, str) for item in names
                ):
                    converted_cache[st_enum] = names
                else:
                    SRB_SCENERY_CACHE_PATH.unlink()
                    return None
            except Exception:
                SRB_SCENERY_CACHE_PATH.unlink()
                return None
    else:
        SRB_SCENERY_CACHE_PATH.unlink()
        return None

    return converted_cache


def read_offline_srb_object_cache() -> Dict["ObjectType", Sequence[str]] | None:
    if not SRB_OBJECT_CACHE_PATH.exists():
        return None

    from srb.core.asset import ObjectType

    try:
        with SRB_OBJECT_CACHE_PATH.open("rb") as f:
            raw_cache = json.loads(f.read())
    except Exception:
        SRB_OBJECT_CACHE_PATH.unlink()
        return None

    if not isinstance(raw_cache, dict):
        SRB_OBJECT_CACHE_PATH.unlink()
        return None

    converted_cache: Dict[ObjectType, Sequence[str]] = {}
    if isinstance(raw_cache, dict):
        for ot_str, names in raw_cache.items():
            try:
                ot_enum = ObjectType[ot_str.upper()]
                if isinstance(names, list) and all(
                    isinstance(item, str) for item in names
                ):
                    converted_cache[ot_enum] = names
                else:
                    SRB_OBJECT_CACHE_PATH.unlink()
                    return None
            except Exception:
                SRB_OBJECT_CACHE_PATH.unlink()
                return None
    else:
        SRB_OBJECT_CACHE_PATH.unlink()
        return None

    return converted_cache


def read_offline_srb_hardware_interface_cache() -> Sequence[str] | None:
    if not SRB_HARDWARE_INTERFACE_CACHE_PATH.exists():
        return None
    try:
        with SRB_HARDWARE_INTERFACE_CACHE_PATH.open("rb") as f:
            data = json.loads(f.read())
        if isinstance(data, list) and all(isinstance(item, str) for item in data):
            return data
        else:
            SRB_HARDWARE_INTERFACE_CACHE_PATH.unlink()
            return None
    except Exception:
        SRB_HARDWARE_INTERFACE_CACHE_PATH.unlink()
        return None
