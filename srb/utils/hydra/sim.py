import enum
import functools
import importlib
import string
import sys
from dataclasses import is_dataclass
from importlib.util import find_spec
from pathlib import Path
from typing import (
    TYPE_CHECKING,
    Any,
    Callable,
    Dict,
    Iterable,
    Mapping,
    Sequence,
    Set,
    Tuple,
    Type,
    get_type_hints,
)

import hydra
from hydra.core.config_store import ConfigStore
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
from srb.utils.cfg import load_cfg_from_registry
from srb.utils.dict import replace_slices_with_strings, replace_strings_with_slices
from srb.utils.hydra.common import (
    replace_enums_with_strings,
    replace_strings_with_enums,
)
from srb.utils.spaces import (
    replace_env_cfg_spaces_with_strings,
    replace_strings_with_env_cfg_spaces,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


# TODO[mid]: Improve hydra config handling by leveraging type hints
def register_task_to_hydra(
    task_name: str, agent_cfg_entry_point: str | None = None
) -> Tuple["AnyEnvCfg", Dict[str, Any]]:
    # Load the configurations
    env_cfg = load_cfg_from_registry(task_name, "task_cfg")
    # Replace gymnasium spaces with strings because OmegaConf does not support them.
    # This must be done before converting the env configs to dictionary to avoid internal reinterpretations
    env_cfg = replace_env_cfg_spaces_with_strings(env_cfg)
    # Convert the configs to dictionary
    # NOTE: It's assumed that the .to_dict() method preserves enum objects. If it converts
    # Them to strings prematurely, the serialization below will not find them.
    env_cfg_dict = env_cfg.to_dict()  # type: ignore

    if agent_cfg_entry_point is None:
        agent_cfg = {}
        agent_cfg_dict = {}
    else:
        agent_cfg = load_cfg_from_registry(task_name, agent_cfg_entry_point)
        if isinstance(agent_cfg, dict):
            agent_cfg_dict = agent_cfg
        else:
            agent_cfg_dict = agent_cfg.to_dict()  # type: ignore
    cfg_dict = {"env": env_cfg_dict, "agent": agent_cfg_dict}
    # Replace complex types with serializable strings
    cfg_dict = replace_slices_with_strings(cfg_dict)
    cfg_dict = replace_enums_with_strings(cfg_dict)
    # Store the configuration to Hydra
    ConfigStore.instance().store(name=task_name.rsplit("/", 1)[1], node=cfg_dict)
    return env_cfg, agent_cfg  # type: ignore


def hydra_task_config(
    task_name: str,
    agent_cfg_entry_point: str | None = None,
    config_path: Path | str | None = None,
) -> Callable:
    config_path = Path(config_path) if config_path else None

    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # Register the task to Hydra
            env_cfg, agent_cfg = register_task_to_hydra(
                task_name, agent_cfg_entry_point
            )

            # Define the new Hydra main function
            @hydra.main(
                config_path=config_path.parent.as_posix() if config_path else None,
                config_name=task_name.rsplit("/", 1)[1]
                if config_path is None
                else config_path.stem,
                version_base=None,
            )
            def hydra_main(
                hydra_env_cfg: DictConfig, env_cfg=env_cfg, agent_cfg=agent_cfg
            ):
                # Convert to a native dictionary
                hydra_env_cfg = OmegaConf.to_container(  # type: ignore
                    hydra_env_cfg,
                    # structured_config_mode=SCMode.INSTANTIATE,
                    resolve=True,
                )
                # Replace serialized strings with their corresponding python objects
                hydra_env_cfg = replace_strings_with_slices(hydra_env_cfg)  # type: ignore
                hydra_env_cfg = replace_strings_with_enums(hydra_env_cfg)  # type: ignore

                # Update the configs with the Hydra command line arguments
                # Env_cfg.from_dict(hydra_env_cfg["env"])
                env_cfg = reconstruct_object(env_cfg, hydra_env_cfg["env"])
                # Replace strings that represent gymnasium spaces because OmegaConf does not support them.
                # This must be done after converting the env configs from dictionary to avoid internal reinterpretations
                replace_strings_with_env_cfg_spaces(env_cfg)
                # Get agent configs
                if isinstance(agent_cfg, dict):
                    agent_cfg = hydra_env_cfg["agent"]
                else:
                    # Agent_cfg.from_dict(hydra_env_cfg["agent"])
                    agent_cfg = reconstruct_object(agent_cfg, hydra_env_cfg["agent"])
                # Call the original function
                func(env_cfg, agent_cfg, *args, **kwargs)

            # Call the new Hydra main function
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

                                    if end_effector is not None:
                                        manipulator.end_effector = end_effector
                                    elif obj.end_effector is not None:
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
                update_value = updates.get(field_name, None)  # type: ignore
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
                update_value = updates.get(field_name, None)  # type: ignore
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
                return obj.__class__.NONE  # type: ignore

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
