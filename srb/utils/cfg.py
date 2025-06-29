import datetime
import importlib
import inspect
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Mapping

import gymnasium
import yaml
from isaaclab.utils import configclass  # noqa: F401

from srb.utils import logging
from srb.utils.path import SRB_LOGS_DIR

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
            mod_path = os.path.dirname(importlib.import_module(mod_name).__file__)  # type: ignore
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
    return cfg  # type: ignore


def stamp_dir(directory: Path, timestamp_format: str = "%Y%m%d_%H%M%S") -> Path:
    return directory.joinpath(datetime.datetime.now().strftime(timestamp_format))


def new_logdir(
    env_id: str,
    workflow: str,
    root: Path = SRB_LOGS_DIR,
    timestamp_format: str = "%Y%m%d_%H%M%S",
    namespace: str = "srb",
) -> Path:
    return stamp_dir(
        root.joinpath(env_id.removeprefix(f"{namespace}/")).joinpath(workflow),
        timestamp_format=timestamp_format,
    )


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
