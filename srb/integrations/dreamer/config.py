"""Shared helpers for Dreamer config updates."""

from __future__ import annotations

import copy
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from srb.utils import logging

#: SRB-only launcher controls that must never reach ``_safe_update_config``.
#: ``replay.save`` selects a persistence directory and ``replay.save_wait``
#: makes replay chunk writes synchronous before the checkpoint ``done``
#: marker is accepted; both configure the launcher-side ``Replay``
#: construction, not the upstream Dreamer config schema.
LAUNCHER_REPLAY_CONTROLS = ("save", "save_wait")


def _flatten_config_updates(
    updates: Mapping[str, Any], *, prefix: str = ""
) -> list[tuple[str, Any]]:
    """Flatten a nested mapping into dotted Dreamer config update keys."""
    flat: list[tuple[str, Any]] = []
    for key, value in updates.items():
        if not isinstance(key, str):
            continue
        full = f"{prefix}.{key}" if prefix else key
        if isinstance(value, Mapping):
            if not value:
                flat.append((full, value))
            else:
                flat.extend(_flatten_config_updates(value, prefix=full))
        else:
            flat.append((full, value))
    return flat


def _safe_update_config(
    config: Any, updates: Mapping[str, Any], *, strict: bool = False
) -> tuple[Any, list[str]]:
    """Apply updates while skipping unknown keys instead of failing hard.

    With ``strict=True`` a dropped (unknown or non-coercible) key is a fatal
    configuration error instead of a warning: a task-specific (L1) profile
    that silently loses one of its capacity/budget overrides would otherwise
    inherit the unsafe global defaults.

    Returns:
        (updated_config, dropped_keys)
    """
    dropped: list[str] = []
    if not isinstance(updates, Mapping):
        return config, dropped

    for key, value in _flatten_config_updates(updates):
        update_key = _canonical_update_key(config, key)
        try:
            config = config.update({update_key: value})
        except KeyError:
            dropped.append(key)
        except TypeError:
            # Known key whose value cannot coerce to the existing type (e.g. a
            # heterogeneous/empty list): skip rather than fail hard, matching the
            # KeyError contract instead of crashing _load_config.
            dropped.append(key)
        except AttributeError as error:
            raise RuntimeError(
                "Dreamer config object does not implement update()"
            ) from error

    if dropped:
        dropped = sorted(dropped)
        if strict:
            raise ValueError(
                "Task-specific Dreamer config contains keys the pinned "
                "upstream schema rejects (dropping them would silently "
                "inherit unsafe global defaults): " + ", ".join(dropped)
            )
        logging.warning(
            "Ignoring unknown or non-coercible Dreamer config keys: %s",
            ", ".join(dropped),
        )
    return config, dropped


def _canonical_update_key(config: Any, key: str) -> str:
    """Restore the upstream agent namespace stripped by parent composition."""

    flat = getattr(config, "flat", None)
    if not isinstance(flat, Mapping) or key in flat:
        return key
    agent_key = f"agent.{key}"
    return agent_key if agent_key in flat else key


def split_launcher_replay_controls(
    agent_cfg: Mapping[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Deep-copy *agent_cfg* and pop the SRB-only replay launcher controls.

    Returns ``(runtime_cfg, controls)`` where ``controls`` maps
    ``{"save": bool, "save_wait": bool}``. The catalog-owned config object is
    never mutated.
    """
    controls: dict[str, Any] = {"save": False, "save_wait": False}
    if not isinstance(agent_cfg, Mapping):
        return {}, controls

    runtime_cfg = copy.deepcopy(dict(agent_cfg))
    replay_cfg = runtime_cfg.get("replay")
    if isinstance(replay_cfg, Mapping):
        replay_cfg = dict(replay_cfg)
        for key in LAUNCHER_REPLAY_CONTROLS:
            if key in replay_cfg:
                controls[key] = bool(replay_cfg.pop(key))
        runtime_cfg["replay"] = replay_cfg
    return runtime_cfg, controls


def dreamer_task_config_is_strict(
    env_id: str,
    *,
    hyperparams_root: str | Path | None = None,
    agent_cfg: Mapping[str, Any] | None = None,
) -> bool:
    """True when the loaded Dreamer config is the task-specific (L1) profile.

    L1 overrides capacity/budget fields, so dropped keys are fatal; shared L0
    merges remain lenient. With *agent_cfg*, strictness also requires every
    catalog L1 key to be declared in the loaded config. Only key presence is
    checked, so command-line value overrides remain valid.
    """
    import yaml

    from srb.utils.agent_config_catalog import (
        AgentCfgNotFoundError,
        resolve_agent_cfg_path,
    )
    from srb.utils.path import SRB_HYPERPARAMS_DIR

    root = Path(hyperparams_root) if hyperparams_root else Path(SRB_HYPERPARAMS_DIR)
    task = str(env_id).rsplit("/", 1)[-1]
    if not task:
        return False
    try:
        resolved = resolve_agent_cfg_path("dreamer", task=task, root=root)
    except AgentCfgNotFoundError:
        return False
    try:
        resolved.resolve().relative_to((root / "task" / task).resolve())
    except ValueError:
        return False
    if agent_cfg is None:
        return True

    try:
        l1_cfg = yaml.safe_load(resolved.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # pragma: no cover - unreadable catalog file
        logging.warning(f"Could not read Dreamer L1 profile {resolved}: {exc}")
        return False
    # The launcher controls are popped from the runtime config before it
    # reaches here, so strip them from the reference profile too.
    l1_runtime, _controls = split_launcher_replay_controls(l1_cfg)
    loaded_keys = {key for key, _ in _flatten_config_updates(agent_cfg)}
    missing = sorted(
        key for key, _ in _flatten_config_updates(l1_runtime) if key not in loaded_keys
    )
    if missing:
        logging.warning(
            "Dreamer configuration for task %s does not declare the checked-in "
            "L1 profile keys %s; applying the lenient config merge instead of "
            "the strict L1 one.",
            task,
            ", ".join(missing),
        )
        return False
    return True
