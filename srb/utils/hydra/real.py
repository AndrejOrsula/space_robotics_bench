import enum
import functools
import importlib
import string
import sys
from dataclasses import is_dataclass
from importlib.util import find_spec
from pathlib import Path
from typing import (
    Any,
    Callable,
    Dict,
    Iterable,
    Mapping,
    Sequence,
    Set,
    Type,
    get_type_hints,
)

import hydra
from hydra.core.config_store import ConfigStore
from omegaconf import DictConfig, OmegaConf
from pydantic import BaseModel

from srb.utils import logging
from srb.utils.cfg import load_cfg_from_registry
from srb.utils.dict import replace_slices_with_strings, replace_strings_with_slices
from srb.utils.hydra.common import (
    replace_enums_with_strings,
    replace_strings_with_enums,
)


def register_task_to_hydra(
    task_name: str, agent_cfg_entry_point: str | None = None
) -> Dict[str, Any]:
    if agent_cfg_entry_point is None:
        agent_cfg = {}
        agent_cfg_dict = {}
    else:
        agent_cfg = load_cfg_from_registry(task_name, agent_cfg_entry_point)
        if isinstance(agent_cfg, dict):
            agent_cfg_dict = agent_cfg
        else:
            agent_cfg_dict = agent_cfg.to_dict()  # type: ignore
    cfg_dict = {"agent": agent_cfg_dict}
    # Replace complex types with serializable strings
    cfg_dict = replace_slices_with_strings(cfg_dict)
    cfg_dict = replace_enums_with_strings(cfg_dict)
    # Store the configuration to Hydra
    ConfigStore.instance().store(name=task_name.rsplit("/", 1)[1], node=cfg_dict)
    return agent_cfg  # type: ignore


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
            agent_cfg = register_task_to_hydra(task_name, agent_cfg_entry_point)

            # Define the new Hydra main function
            @hydra.main(
                config_path=config_path.parent.as_posix() if config_path else None,
                config_name=task_name.rsplit("/", 1)[1]
                if config_path is None
                else config_path.stem,
                version_base=None,
            )
            def hydra_main(hydra_env_cfg: DictConfig, agent_cfg=agent_cfg):
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
                # Get agent configs
                if isinstance(agent_cfg, dict):
                    agent_cfg = hydra_env_cfg["agent"]
                else:
                    # Agent_cfg.from_dict(hydra_env_cfg["agent"])
                    agent_cfg = reconstruct_object(agent_cfg, hydra_env_cfg["agent"])
                # Call the original function
                func(agent_cfg, *args, **kwargs)

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
