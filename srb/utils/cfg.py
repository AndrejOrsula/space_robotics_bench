import importlib
import inspect
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any

import gymnasium
import yaml
from isaaclab.utils.configclass import configclass  # noqa: F401

from srb.utils import logging
from srb.utils.agent_config_catalog import (  # noqa: F401
    FRAMEWORK_CFG_ENTRYPOINT_KEY,
    FRAMEWORK_MULTI_ALGO_CFG_ENTRYPOINT_KEY,
    SUPPORTED_CFG_FILE_EXTENSIONS,
    SUPPORTED_FRAMEWORKS,
    agent_config_entry_points,
    identify_agent_config,
)
from srb.utils.config_introspection import (  # noqa: F401
    get_config_item,
    iter_config_items,
    iter_config_items_by_type,
    iter_config_values,
    iter_config_values_by_type,
    set_config_item,
)
from srb.utils.failures import InvalidInputError
from srb.utils.path import (  # noqa: F401
    DEFAULT_DATETIME_FORMAT,
    SRB_LOGS_DIR,
    new_logdir,
    stamp_dir,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg

# File-backed entry points, shared by both registry loaders.
_CFG_FILE_SUFFIXES = tuple(f".{ext}" for ext in SUPPORTED_CFG_FILE_EXTENSIONS)


def load_cfg_from_registry(
    task_name: str, entry_point_key: str, unpack_callable: bool = True
) -> "AnyEnvCfg | dict[str, Any]":
    cfg_entry_point = gymnasium.spec(task_name).kwargs.get(entry_point_key)
    if cfg_entry_point is None:
        raise ValueError(
            f"Could not find configuration for the environment: '{task_name}'."
            f" Check that the gym registry has the entry point: '{entry_point_key}'."
            f" Found: {gymnasium.spec(task_name).kwargs}."
        )
    if isinstance(cfg_entry_point, str) and cfg_entry_point.endswith(
        _CFG_FILE_SUFFIXES
    ):
        if os.path.exists(cfg_entry_point):
            config_file = cfg_entry_point
        else:
            # Resolve "module:relative/file"; a colon-less string here is a plain
            # filesystem path that simply does not exist -> report that clearly
            # instead of a cryptic "not enough values to unpack".
            if ":" not in cfg_entry_point:
                raise FileNotFoundError(f"Config file not found: {cfg_entry_point}")
            mod_name, file_name = cfg_entry_point.split(":", 1)
            mod_path = os.path.dirname(importlib.import_module(mod_name).__file__)  # type: ignore[misc]
            config_file = os.path.join(mod_path, file_name)
        logging.info(f"Parsing configuration from: {config_file}")
        if config_file.endswith(".json"):
            import json as _json

            with open(config_file, encoding="utf-8") as f:
                cfg = _json.load(f)
        elif config_file.endswith(".toml"):
            import tomllib as _toml

            with open(config_file, "rb") as f:
                cfg = _toml.load(f)
        else:
            with open(config_file, encoding="utf-8") as f:
                # Safe loading prevents Python tags from importing modules.
                cfg = yaml.safe_load(f)
    else:
        if unpack_callable and callable(cfg_entry_point):
            mod_path = inspect.getfile(cfg_entry_point)
            cfg_cls = cfg_entry_point()
        elif isinstance(cfg_entry_point, str):
            mod_name, attr_name = cfg_entry_point.split(":", 1)
            mod = importlib.import_module(mod_name)
            cfg_cls = getattr(mod, attr_name)
        else:
            cfg_cls = cfg_entry_point
        logging.info(f"Parsing configuration from: {cfg_entry_point}")
        cfg = cfg_cls() if unpack_callable and callable(cfg_cls) else cfg_cls
    return cfg  # type: ignore[return-value]


def load_cfg_class_from_registry(task_name: str, entry_point_key: str) -> type:
    """Load a config CLASS from the gymnasium registry without instantiation.

    Unlike :func:`load_cfg_from_registry`, this returns the **class** itself
    rather than an instance.  This avoids calling ``__init__``/``__post_init__``
    which may perform expensive computation or require Isaac Sim.
    """
    cfg_entry_point = gymnasium.spec(task_name).kwargs.get(entry_point_key)
    if cfg_entry_point is None:
        raise ValueError(
            f"Could not find configuration for '{task_name}' with key '{entry_point_key}'."
            f" Found: {gymnasium.spec(task_name).kwargs}."
        )
    if isinstance(cfg_entry_point, str) and cfg_entry_point.endswith(
        _CFG_FILE_SUFFIXES
    ):
        raise TypeError(
            f"Config file entry points cannot be loaded as classes: {cfg_entry_point}"
        )
    if isinstance(cfg_entry_point, str):
        mod_name, attr_name = cfg_entry_point.split(":", 1)
        mod = importlib.import_module(mod_name)
        return getattr(mod, attr_name)
    if callable(cfg_entry_point):
        return cfg_entry_point
    raise TypeError(f"Cannot extract class from entry point: {cfg_entry_point}")


def last_logdir(
    env_id: str,
    workflow: str,
    root: Path = SRB_LOGS_DIR,
    modification_time: bool = False,
    namespace: str = "srb",
) -> Path:
    logdir_parent = root.joinpath(env_id.removeprefix(f"{namespace}/")).joinpath(
        workflow
    )
    hint = "; train first or pass --model"
    if not logdir_parent.is_dir():
        raise InvalidInputError(
            f"Path {logdir_parent} is expected to be a directory with logdirs but it "
            + ("is a file" if logdir_parent.is_file() else "does not exist")
            + hint
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
        raise InvalidInputError(
            f"Path {logdir_parent} does not contain any logdirs{hint}"
        )


def last_dir(directory: Path, modification_time: bool = False) -> Path | None:
    if not directory.is_dir():
        raise NotADirectoryError(f"Expected a directory, got: {directory}")
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
