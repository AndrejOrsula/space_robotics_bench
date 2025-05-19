import datetime
import enum
import functools
import importlib
import inspect
import os
import string
import sys
from collections.abc import Callable
from dataclasses import is_dataclass
from importlib.util import find_spec
from pathlib import Path
from typing import (
    TYPE_CHECKING,
    Any,
    Dict,
    Iterable,
    Mapping,
    Sequence,
    Set,
    Tuple,
    Type,
    get_type_hints,
)

import gymnasium
import hydra
import yaml
from hydra.core.config_store import ConfigStore
from isaaclab.utils import configclass  # noqa: F401
from omegaconf import DictConfig, OmegaConf
from pydantic import BaseModel
from simforge import Asset as SimForgeAsset
from simforge import AssetRegistry as SimForgeAssetRegistry

from srb.core.action import ActionGroup, ActionGroupRegistry
from srb.core.asset import (
    Asset,
    AssetRegistry,
    AssetVariant,
    Manipulator,
    ManipulatorRegistry,
    MobileManipulator,
    MobileManipulatorRegistry,
    MobileRobot,
    MobileRobotRegistry,
    Object,
    ObjectRegistry,
    Payload,
    Robot,
    RobotRegistry,
    Scenery,
    SceneryRegistry,
    Tool,
)
from srb.utils import logging
from srb.utils.dict import replace_slices_with_strings, replace_strings_with_slices
from srb.utils.path import SRB_LOGS_DIR
from srb.utils.spaces import (
    replace_env_cfg_spaces_with_strings,
    replace_strings_with_env_cfg_spaces,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg

SUPPORTED_FRAMEWORKS = {
    "dreamer": {"multi_algo": False},
    "sb3": {"multi_algo": True},
    "sbx": {"multi_algo": True},
    "skrl": {"multi_algo": True},
    "robomimic": {"multi_algo": True},
}
SUPPORTED_CFG_FILE_EXTENSIONS = (
    "json",
    "toml",
    "yaml",
    "yml",
)
FRAMEWORK_CFG_ENTRYPOINT_KEY = "{FRAMEWORK}_cfg"
FRAMEWORK_MULTI_ALGO_CFG_ENTRYPOINT_KEY = "{FRAMEWORK}_{ALGO}_cfg"


def parse_algo_configs(cfg_dir: str) -> Mapping[str, str]:
    algo_config = {}

    for root, _, files in os.walk(cfg_dir):
        for file in files:
            if not file.endswith(SUPPORTED_CFG_FILE_EXTENSIONS):
                continue
            file = os.path.join(root, file)

            key = _identify_config(root, file)
            if key is not None:
                algo_config[key] = file

    return algo_config


def _identify_config(root: str, file) -> str | None:
    basename = os.path.basename(file).split(".")[0]

    for framework, properties in SUPPORTED_FRAMEWORKS.items():
        algo = basename.replace(f"{framework}_", "")
        if root.endswith(framework):
            assert properties["multi_algo"]
            return FRAMEWORK_MULTI_ALGO_CFG_ENTRYPOINT_KEY.format(
                FRAMEWORK=framework, ALGO=algo
            )
        elif basename.startswith(f"{framework}"):
            if properties["multi_algo"]:
                return FRAMEWORK_MULTI_ALGO_CFG_ENTRYPOINT_KEY.format(
                    FRAMEWORK=framework, ALGO=algo
                )
            else:
                return FRAMEWORK_CFG_ENTRYPOINT_KEY.format(FRAMEWORK=framework)

    return None


def load_cfg_from_registry(
    task_name: str, entry_point_key: str, unpack_callable: bool = True
) -> "AnyEnvCfg" | Dict[str, Any]:
    # Obtain the configuration entry point
    cfg_entry_point = gymnasium.spec(task_name).kwargs.get(entry_point_key)
    # Check if entry point exists
    if cfg_entry_point is None:
        raise ValueError(
            f"Could not find configuration for the environment: '{task_name}'."
            f" Please check that the gym registry has the entry point: '{entry_point_key}'."
            f" Found: {gymnasium.spec(task_name).kwargs}."
        )
    # Parse the default config file
    if isinstance(cfg_entry_point, str) and cfg_entry_point.endswith(".yaml"):
        if os.path.exists(cfg_entry_point):
            # Absolute path for the config file
            config_file = cfg_entry_point
        else:
            # Resolve path to the module location
            mod_name, file_name = cfg_entry_point.split(":")
            mod_path = os.path.dirname(importlib.import_module(mod_name).__file__)
            # Obtain the configuration file path
            config_file = os.path.join(mod_path, file_name)
        # Load the configuration
        logging.info(f"Parsing configuration from: {config_file}")
        with open(config_file, encoding="utf-8") as f:
            cfg = yaml.full_load(f)
    else:
        if unpack_callable and callable(cfg_entry_point):
            # Resolve path to the module location
            mod_path = inspect.getfile(cfg_entry_point)
            # Load the configuration
            cfg_cls = cfg_entry_point()
        elif isinstance(cfg_entry_point, str):
            # Resolve path to the module location
            mod_name, attr_name = cfg_entry_point.split(":")
            mod = importlib.import_module(mod_name)
            cfg_cls = getattr(mod, attr_name)
        else:
            cfg_cls = cfg_entry_point
        # Load the configuration
        logging.info(f"Parsing configuration from: {cfg_entry_point}")
        cfg = cfg_cls() if unpack_callable and callable(cfg_cls) else cfg_cls
    return cfg


def stamp_dir(directory: Path, timestamp_format: str = "%Y%m%d_%H%M%S") -> Path:
    return directory.joinpath(datetime.datetime.now().strftime(timestamp_format))


def new_logdir(
    env_id: str,
    workflow: str,
    root: Path = SRB_LOGS_DIR,
    timestamp_format: str = "%Y%m%d_%H%M%S",
) -> Path:
    return stamp_dir(
        root.joinpath(env_id.removeprefix("srb/")).joinpath(workflow),
        timestamp_format=timestamp_format,
    )


def last_logdir(
    env_id: str,
    workflow: str,
    root: Path = SRB_LOGS_DIR,
    modification_time: bool = False,
) -> Path:
    logdir_parent = root.joinpath(env_id.removeprefix("srb/")).joinpath(workflow)
    if not logdir_parent.is_dir():
        raise ValueError(
            f"Path {logdir_parent} is expected to be a directory with logdirs but it "
            + ("is a file" if logdir_parent.is_file() else "does not exist")
        )

    if last_logdir := last_dir(
        directory=logdir_parent, modification_time=modification_time
    ):
        logging.debug(
            f"Selecting {last_logdir} as the last logdir"
            + (" (based on modification time)" if modification_time else "")
        )
        return last_logdir
    else:
        raise FileNotFoundError(f"Path {logdir_parent} does not contain any logdirs")


def last_dir(directory: Path, modification_time: bool = False) -> Path | None:
    assert directory.is_dir()
    if dirs := sorted(
        filter(
            lambda p: p.is_dir(),
            (directory.joinpath(child) for child in os.listdir(directory)),
        ),
        key=os.path.getmtime if modification_time else None,
        reverse=True,
    ):
        return dirs[0]
    else:
        return None


def last_file(directory: Path, modification_time: bool = False) -> Path | None:
    assert directory.is_dir()
    if files := sorted(
        filter(
            lambda p: p.is_file(),
            (directory.joinpath(child) for child in os.listdir(directory)),
        ),
        key=os.path.getmtime if modification_time else None,
        reverse=True,
    ):
        return files[0]
    else:
        return None


# TODO[mid]: Improve hydra config handling by leveraging type hints
def register_task_to_hydra(
    task_name: str, agent_cfg_entry_point: str | None = None
) -> Tuple["AnyEnvCfg", Dict[str, Any]]:
    # load the configurations
    env_cfg = load_cfg_from_registry(task_name, "task_cfg")
    # replace gymnasium spaces with strings because OmegaConf does not support them.
    # this must be done before converting the env configs to dictionary to avoid internal reinterpretations
    env_cfg = replace_env_cfg_spaces_with_strings(env_cfg)
    # convert the configs to dictionary
    env_cfg_dict = env_cfg.to_dict()

    if agent_cfg_entry_point is None:
        agent_cfg = {}
        agent_cfg_dict = {}
    else:
        agent_cfg = load_cfg_from_registry(task_name, agent_cfg_entry_point)
        if isinstance(agent_cfg, dict):
            agent_cfg_dict = agent_cfg
        else:
            agent_cfg_dict = agent_cfg.to_dict()
    cfg_dict = {"env": env_cfg_dict, "agent": agent_cfg_dict}
    # replace slices with strings because OmegaConf does not support slices
    cfg_dict = replace_slices_with_strings(cfg_dict)
    # store the configuration to Hydra
    ConfigStore.instance().store(name=task_name.rsplit("/", 1)[1], node=cfg_dict)
    return env_cfg, agent_cfg


def hydra_task_config(
    task_name: str, agent_cfg_entry_point: str | None = None
) -> Callable:
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # register the task to Hydra
            env_cfg, agent_cfg = register_task_to_hydra(
                task_name, agent_cfg_entry_point
            )

            # define the new Hydra main function
            @hydra.main(
                config_path=None,
                config_name=task_name.rsplit("/", 1)[1],
                version_base="1.3",
            )
            def hydra_main(
                hydra_env_cfg: DictConfig, env_cfg=env_cfg, agent_cfg=agent_cfg
            ):
                # convert to a native dictionary
                hydra_env_cfg = OmegaConf.to_container(
                    hydra_env_cfg,
                    # structured_config_mode=SCMode.INSTANTIATE,
                    resolve=True,
                )
                # replace string with slices because OmegaConf does not support slices
                hydra_env_cfg = replace_strings_with_slices(hydra_env_cfg)
                # update the configs with the Hydra command line arguments
                # env_cfg.from_dict(hydra_env_cfg["env"])
                env_cfg = reconstruct_object(env_cfg, hydra_env_cfg["env"])
                # replace strings that represent gymnasium spaces because OmegaConf does not support them.
                # this must be done after converting the env configs from dictionary to avoid internal reinterpretations
                replace_strings_with_env_cfg_spaces(env_cfg)
                # get agent configs
                if isinstance(agent_cfg, dict):
                    agent_cfg = hydra_env_cfg["agent"]
                else:
                    # agent_cfg.from_dict(hydra_env_cfg["agent"])
                    agent_cfg = reconstruct_object(agent_cfg, hydra_env_cfg["agent"])
                # call the original function
                func(env_cfg, agent_cfg, *args, **kwargs)

            # call the new Hydra main function
            hydra_main()

        return wrapper

    return decorator


def reconstruct_object(obj: Any, updates: Any) -> Any:
    try:
        ## String-based updates that indirectly represent a type/instance
        if (
            (not isinstance(obj, str) or isinstance(obj, enum.Enum))
            and isinstance(updates, str)
            and all(c not in string.whitespace for c in updates)
        ):
            if ":" in updates and not callable(obj):
                ## Object updated via its full module path and name
                mod_name, attr_name = updates.split(":")
                if find_spec(mod_name) is None:
                    raise ModuleNotFoundError(f"Module '{mod_name}' not found.")
                mod = importlib.import_module(mod_name)
                if not hasattr(mod, attr_name):
                    raise AttributeError(
                        f"Attribute '{attr_name}' not found in '{mod_name}'."
                    )
                attr = getattr(mod, attr_name)
                if isinstance(attr, (Type, Callable)):
                    return attr()
                else:
                    return attr
            else:
                ## Asset variant updated via variant or asset name
                if isinstance(obj, AssetVariant):
                    if variant := AssetVariant.from_str(updates):
                        # Asset variant updated via its name
                        return variant
                    elif asset_class := AssetRegistry.get_by_name(updates):
                        # Asset variant updated via asset name
                        return asset_class()  # type: ignore

                ## Registered class updated via its name
                if isinstance(obj, Asset):
                    # Asset variant
                    if variant := AssetVariant.from_str(updates):
                        return variant

                    # Scenery
                    if isinstance(obj, Scenery):
                        if scenery_class := SceneryRegistry.get_by_name(updates):
                            return scenery_class()  # type: ignore
                        else:
                            logging.warning(
                                f'Asset "{updates}" is supposed to update an instance of "{Scenery.__name__}" but it is not registered under this type'
                            )

                    # Object
                    if isinstance(obj, Object):
                        if object_class := ObjectRegistry.get_by_name(updates):
                            return object_class()  # type: ignore
                        else:
                            logging.warning(
                                f'Asset "{updates}" is supposed to update an instance of "{Object.__name__}" but it is not registered under this type'
                            )

                    # Robot
                    if isinstance(obj, Robot):
                        # Mobile manipulator
                        if isinstance(obj, MobileManipulator):
                            if (
                                mobile_manipulator_class
                                := MobileManipulatorRegistry.get_by_name(updates)
                            ):
                                return mobile_manipulator_class()  # type: ignore
                            else:
                                logging.warning(
                                    f'Asset "{updates}" is supposed to update an instance of "{MobileManipulator.__name__}" but it is not registered under this type'
                                )

                        # Manipulator
                        if isinstance(obj, Manipulator):
                            if "+" in updates:
                                manipulator_name, end_effector_name = updates.split(
                                    "+", 1
                                )

                                # Find end_effector class if specified
                                if end_effector_name:
                                    if end_effector_class := next(
                                        (
                                            end_effector_class
                                            for end_effector_class in Tool.object_registry()
                                            if end_effector_class.name()
                                            == end_effector_name
                                        ),
                                        None,
                                    ):
                                        end_effector = end_effector_class()  # type: ignore
                                    else:
                                        raise ValueError(
                                            f'Asset "{end_effector_name}" is supposed to update an instance of "{Tool.__name__}" but it is not registered under this type'
                                        )
                                else:
                                    end_effector = None

                                # Handle end_effector-only update ("+end_effector_name")
                                if not manipulator_name and end_effector:
                                    obj.end_effector = end_effector
                                    return obj

                                # Handle robot update with optional end_effector ("robot_name+end_effector_name")
                                if manipulator_name:
                                    if manipulator_class := (
                                        ManipulatorRegistry.get_by_name(
                                            manipulator_name
                                        )
                                    ):
                                        manipulator = manipulator_class()  # type: ignore
                                    else:
                                        raise ValueError(
                                            f'Asset "{manipulator_name}" is supposed to update an instance of "{Manipulator.__name__}" but it is not registered under this type'
                                        )

                                    if end_effector:
                                        manipulator.end_effector = end_effector
                                    elif manipulator.end_effector is None:
                                        manipulator.end_effector = obj.end_effector

                                    return manipulator

                            # Case 2: Format is just "robot_name"
                            if manipulator_class := ManipulatorRegistry.get_by_name(
                                updates
                            ):
                                return manipulator_class()  # type: ignore
                            else:
                                logging.warning(
                                    f'Asset "{updates}" is supposed to update an instance of "{Manipulator.__name__}" but it is not registered under this type'
                                )

                        # Mobile robot
                        if isinstance(obj, MobileRobot):
                            if "+" in updates:
                                mobile_robot_name, payload_name = updates.split("+", 1)

                                # Find payload class if specified
                                if payload_name:
                                    if payload_class := next(
                                        (
                                            payload_class
                                            for payload_class in Payload.object_registry()
                                            if payload_class.name() == payload_name
                                        ),
                                        None,
                                    ):
                                        payload = payload_class()  # type: ignore
                                    else:
                                        raise ValueError(
                                            f'Asset "{payload_name}" is supposed to update an instance of "{Payload.__name__}" but it is not registered under this type'
                                        )
                                else:
                                    payload = None

                                # Handle payload-only update ("+payload_name")
                                if not mobile_robot_name and payload:
                                    obj.payload = payload
                                    return obj

                                # Handle robot update with optional payload ("robot_name+payload_name")
                                if mobile_robot_name:
                                    if mobile_robot_class := (
                                        MobileRobotRegistry.get_by_name(
                                            mobile_robot_name
                                        )
                                    ):
                                        mobile_robot = mobile_robot_class()  # type: ignore
                                    else:
                                        raise ValueError(
                                            f'Asset "{mobile_robot_name}" is supposed to update an instance of "{MobileRobot.__name__}" but it is not registered under this type'
                                        )

                                    if payload:
                                        mobile_robot.payload = payload
                                    elif mobile_robot.payload is None:
                                        mobile_robot.payload = obj.payload

                                    return mobile_robot

                            # Case 2: Format is just "robot_name"
                            if mobile_robot_class := MobileRobotRegistry.get_by_name(
                                updates
                            ):
                                return mobile_robot_class()  # type: ignore
                            else:
                                logging.warning(
                                    f'Asset "{updates}" is supposed to update an instance of "{MobileRobot.__name__}" but it is not registered under this type'
                                )

                        # Other robot
                        if robot_class := RobotRegistry.get_by_name(updates):
                            return robot_class()  # type: ignore
                        else:
                            logging.warning(
                                f'Asset "{updates}" is supposed to update an instance of "{Robot.__name__}" but it is not registered under this type'
                            )

                    # Other asset
                    if asset_class := AssetRegistry.get_by_name(updates):
                        return asset_class()  # type: ignore
                    else:
                        raise ValueError(f'Asset "{updates}" is not registered')

                # Action group
                if isinstance(obj, ActionGroup):
                    if action_group_class := ActionGroupRegistry.get_by_name(updates):
                        return action_group_class()
                    else:
                        raise ValueError(f'Action group "{updates}" is not registered')

                # SimForge asset
                if isinstance(obj, SimForgeAsset):
                    if asset_class := SimForgeAssetRegistry.get_by_name(updates):
                        return asset_class()
                    else:
                        raise ValueError(f'Asset "{updates}" is not registered')

        # Pydantic
        if isinstance(obj, BaseModel):
            try:
                type_hints = get_type_hints(obj.__class__)
            except Exception:
                type_hints = {k: type(v) for k, v in obj.__dict__.items()}
            new_kwargs = {}
            for field_name, field_type in type_hints.items():
                if field_name.startswith("__"):
                    continue
                current_value = getattr(obj, field_name, None)
                update_value = updates.get(field_name, None)
                if update_value is not None:
                    new_kwargs[field_name] = reconstruct_object(
                        current_value, update_value
                    )
                else:
                    new_kwargs[field_name] = current_value

            return obj.__class__(**new_kwargs)

        # Dataclass
        if is_dataclass(obj):
            try:
                type_hints = get_type_hints(obj.__class__)  # type: ignore
            except Exception:
                type_hints = {k: type(v) for k, v in obj.__dict__.items()}
            new_kwargs = {}
            for field_name, field_type in type_hints.items():
                if field_name.startswith("__"):
                    continue
                current_value = getattr(obj, field_name, None)
                update_value = updates.get(field_name, None)
                if update_value is not None:
                    new_kwargs[field_name] = reconstruct_object(
                        current_value, update_value
                    )
                else:
                    new_kwargs[field_name] = current_value

            return obj.__class__(**new_kwargs)  # type: ignore

        # Enum
        if isinstance(obj, enum.Enum):
            if isinstance(updates, str):
                return obj.__class__[updates.strip().upper()]
            if isinstance(updates, Mapping) and "_name_" in updates.keys():
                return obj.__class__[updates["_name_"]]
            if updates is None and hasattr(obj, "NONE"):
                # Handle enums with "NONE" value
                return obj.__class__.NONE

        # Dict
        if isinstance(obj, Dict) and isinstance(updates, Mapping):
            obj.update(
                {
                    key: reconstruct_object(obj.get(key, None), updates.get(key, None))
                    for key in set(obj) | set(updates)
                }
            )
            return obj

        # Set
        if isinstance(obj, Set) and isinstance(updates, Iterable):
            obj.update(updates)
            return obj

        # Mapping
        if isinstance(obj, Mapping) and isinstance(updates, Mapping):
            return obj.__class__(
                (  # type: ignore
                    key,
                    reconstruct_object(obj.get(key, None), updates.get(key, None)),
                )
                for key in set(obj) | set(updates)
            )

        # Sequence
        if (isinstance(obj, Sequence) and not isinstance(obj, str)) and isinstance(
            updates, Iterable
        ):
            result = obj.__class__(
                reconstruct_object(o, u)  # type: ignore
                for o, u in zip(obj, updates)
            )
            return result

        # Callable (e.g. function)
        if callable(obj):
            return obj

        # Other types
        if not isinstance(
            obj,
            (
                str,
                int,
                float,
                bool,
                slice,
                type(None),
                Path,
            ),
        ):
            logging.warning(
                f"Type '{type(obj)}' is not explicitly handled in the object reconstruction process"
            )
        return updates if updates is not None else obj
    except Exception as e:
        overrides = ", ".join(
            f'"{arg}"' if any(c in string.whitespace for c in arg) else arg
            for arg in sys.argv[1:]
        )
        logging.critical(
            f'Failed to apply the requested override of type "{type(updates)}" to object of type "{type(obj)}" with overrides: [{overrides}]'
        )
        raise e
