from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path
from typing import Mapping

from srb.utils.path import SRB_HYPERPARAMS_DIR

# Exclude task overrides and tuning artifacts from the global catalog walk.
_RESERVED_HYPERPARAMS_SUBDIRS = frozenset({"task", "tune"})


class AgentCfgNotFoundError(FileNotFoundError):
    """No agent config file exists at any applicable layer for (algo, obs, task)."""


def is_reserved_hyperparams_subdir(name: str) -> bool:
    """True for top-level ``hyperparams/`` subdirs excluded from the default walk."""
    return name in _RESERVED_HYPERPARAMS_SUBDIRS


SUPPORTED_FRAMEWORKS: Mapping[str, Mapping[str, bool]] = {
    "dreamer": {"multi_algo": False},
    "rsl_rl": {"multi_algo": True},
    "tdmpc2": {"multi_algo": False},
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


def agent_config_entry_points(cfg_dir: str | os.PathLike[str] | None) -> dict[str, str]:
    """Return gymnasium entry point kwargs for agent configs under *cfg_dir*.

    The catalog is cached by the supplied directory string. Callers receive a
    copy so accidental mutation cannot poison later registrations.
    """
    if cfg_dir is None:
        return {}
    return dict(_cached_agent_config_entry_points(os.fspath(cfg_dir)))


def clear_agent_config_catalog_cache() -> None:
    _cached_agent_config_entry_points.cache_clear()


def agent_config_entry_points_for_task(
    cfg_dir: str | os.PathLike[str] | None,
    task_id: str | None,
) -> dict[str, str]:
    """Return gym entry points for a task, layering L1 (task-specific) over L0.

    The global catalog under *cfg_dir* (reserved subdirs pruned) is overlaid
    with the per-task catalog at ``<cfg_dir>/task/<task_id>/`` when that
    directory exists, so a task-specific agent config shadows the global one
    for that task only. Isaac-free; the tuning driver reaches the same
    files via :func:`resolve_agent_cfg_path`.
    """
    base = agent_config_entry_points(cfg_dir)
    if cfg_dir is None or not task_id:
        return base
    task_dir = Path(cfg_dir) / "task" / task_id
    if not task_dir.is_dir():
        return base
    return {**base, **agent_config_entry_points(task_dir.as_posix())}


# Bounded for the shared root and per-task catalogs.
@lru_cache(maxsize=512)
def _cached_agent_config_entry_points(cfg_dir: str) -> tuple[tuple[str, str], ...]:
    algo_config: dict[str, str] = {}

    cfg_dir_abs = os.path.abspath(cfg_dir)
    for root, dirs, files in os.walk(cfg_dir):
        # Prune only at the root; task catalogs are also passed directly.
        if os.path.abspath(root) == cfg_dir_abs:
            dirs[:] = [d for d in dirs if not is_reserved_hyperparams_subdir(d)]
        for file_name in files:
            if not file_name.endswith(SUPPORTED_CFG_FILE_EXTENSIONS):
                continue

            file_path = os.path.join(root, file_name)
            key = identify_agent_config(root, file_path)
            if key is not None:
                algo_config[key] = file_path

    return tuple(sorted(algo_config.items()))


def identify_agent_config(root: str, file_path: str) -> str | None:
    basename = os.path.basename(file_path).split(".")[0]

    for framework, properties in SUPPORTED_FRAMEWORKS.items():
        if root.endswith(framework):
            if not properties["multi_algo"]:
                return None
            algo = basename.removeprefix(f"{framework}_")
            return FRAMEWORK_MULTI_ALGO_CFG_ENTRYPOINT_KEY.format(
                FRAMEWORK=framework,
                ALGO=algo,
            )
        if basename.startswith(framework):
            return agent_config_entry_point_key(basename, {framework: properties})

    return None


def agent_config_entry_point_key(
    algo_name: str,
    supported_frameworks: Mapping[str, Mapping[str, bool]] = SUPPORTED_FRAMEWORKS,
) -> str | None:
    for framework, properties in supported_frameworks.items():
        if not algo_name.startswith(framework):
            continue
        if properties["multi_algo"]:
            algo = algo_name.removeprefix(f"{framework}_")
            return FRAMEWORK_MULTI_ALGO_CFG_ENTRYPOINT_KEY.format(
                FRAMEWORK=framework,
                ALGO=algo,
            )
        # Preserve the visual suffix for single-algorithm frameworks.
        if algo_name.endswith("_visual"):
            return f"{framework}_visual_cfg"
        return FRAMEWORK_CFG_ENTRYPOINT_KEY.format(FRAMEWORK=framework)
    return None


def entry_key_for(algo: str, obs: str = "state") -> str | None:
    """Return the catalog entry-point key for an SRB algo *label* and obs mode.

    ``skrl_ppo`` + ``state`` → ``skrl_ppo_cfg``; ``skrl_ppo`` + ``visual`` →
    ``skrl_ppo_visual_cfg``; ``dreamer`` + ``visual`` → ``dreamer_visual_cfg``.
    Delegates to :func:`agent_config_entry_point_key` so the resolver and the
    catalog walk share one key-derivation rule. Returns ``None`` for an
    unknown algo label.
    """
    label = f"{algo}_visual" if obs == "visual" else algo
    return agent_config_entry_point_key(label)


def resolve_agent_cfg_path(
    algo: str,
    *,
    obs: str = "state",
    task: str | None = None,
    root: str | os.PathLike[str] = SRB_HYPERPARAMS_DIR,
) -> Path:
    """Resolve the agent-config file for (*algo*, *obs*, *task*).

    Precedence: task-specific (L1) → global (L0), both in the requested obs
    mode. ``obs='visual'`` never falls back to a state config — a state-net
    config is meaningless on image observations. Isaac-free; shared by the gym
    registry (task registration) and the tuning driver.

    :raises AgentCfgNotFoundError: if no file exists at any applicable layer.
    """
    root_path = Path(root)
    key = entry_key_for(algo, obs)
    if key is None:
        raise AgentCfgNotFoundError(f"Unknown algorithm label: {algo!r}")

    searched: list[str] = []
    # L1: task-specific override, if a task is given.
    if task:
        task_dir = root_path / "task" / task
        searched.append(task_dir.as_posix())
        catalog = agent_config_entry_points(task_dir.as_posix())
        if key in catalog:
            return Path(catalog[key])
    # L0: global default (reserved subdirs pruned from this walk).
    searched.append(root_path.as_posix())
    catalog = agent_config_entry_points(root_path.as_posix())
    if key in catalog:
        return Path(catalog[key])

    raise AgentCfgNotFoundError(
        f"No agent config for algo={algo!r} obs={obs!r} task={task!r} "
        f"(entry key {key!r}). Searched: {', '.join(searched)}"
    )
