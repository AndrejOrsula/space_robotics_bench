import functools
from pathlib import Path
from typing import Any, Callable

import hydra
from hydra.core.config_store import ConfigStore
from omegaconf import DictConfig, OmegaConf

from srb.hydra.common import (
    replace_enums_with_strings,
    replace_strings_with_enums,
)
from srb.hydra.create import create_instance
from srb.utils.cfg import load_cfg_from_registry
from srb.utils.dict import replace_slices_with_strings, replace_strings_with_slices


def register_task_to_hydra(
    task_name: str, agent_cfg_entry_point: str | None = None
) -> dict[str, Any]:
    if agent_cfg_entry_point is None:
        agent_cfg = {}
        agent_cfg_dict = {}
    else:
        agent_cfg = load_cfg_from_registry(task_name, agent_cfg_entry_point)
        if isinstance(agent_cfg, dict):
            agent_cfg_dict = agent_cfg
        else:
            agent_cfg_dict = agent_cfg.to_dict()  # type: ignore[misc]
    cfg_dict = {"agent": agent_cfg_dict}
    # Replace complex types with serializable strings
    cfg_dict = replace_slices_with_strings(cfg_dict)
    cfg_dict = replace_enums_with_strings(cfg_dict)
    # Store the configuration to Hydra
    ConfigStore.instance().store(name=task_name.rsplit("/", 1)[1], node=cfg_dict)
    return agent_cfg  # type: ignore[return-value]


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
                hydra_env_cfg = OmegaConf.to_container(  # type: ignore[misc]
                    hydra_env_cfg,
                    resolve=True,
                )
                # Replace serialized strings with their corresponding python objects
                hydra_env_cfg = replace_strings_with_slices(hydra_env_cfg)  # type: ignore[misc]
                hydra_env_cfg = replace_strings_with_enums(hydra_env_cfg)  # type: ignore[misc]
                # Update the configs with the Hydra command line arguments
                # Get agent configs
                if isinstance(agent_cfg, dict):
                    agent_cfg = hydra_env_cfg.get("agent", agent_cfg)
                elif "agent" in hydra_env_cfg:
                    agent_cfg = create_instance(type(agent_cfg), hydra_env_cfg["agent"])
                # Call the original function
                func(agent_cfg, *args, **kwargs)

            # Call the new Hydra main function
            hydra_main()

        return wrapper

    return decorator
