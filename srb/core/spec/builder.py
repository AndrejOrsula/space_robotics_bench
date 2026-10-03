"""Env-level aggregation of EnvSpec."""

from __future__ import annotations

import datetime
import logging
from typing import Any

from srb.core.scene_introspection import iter_scene_articulations, iter_scene_sensors
from srb.core.spec.descriptors import (
    resolve_action_group_spec,
    resolve_observation_group_spec,
    resolve_sensor_entry,
)
from srb.core.spec.lifecycle import require_srb_spec_reset
from srb.core.spec.models import (
    ActionSpec,
    EnvSpec,
    GroupSpec,
    ObservationSpec,
    SensorSpec,
    TermSpec,
)

logger = logging.getLogger(__name__)


def _action_runtime_shapes(
    env: Any,
    group_name: str,
) -> dict[str, tuple[int, ...]] | None:
    """Resolve action-term shapes from the manager, then the Gym space.

    Isaac Lab exposes a flat action ``Box``, while ``action_manager`` provides
    per-term ``action_dim`` values. Test stubs may expose a Dict-of-Dict space.
    A successful manager probe can return ``{}``; unavailable probes return
    ``None`` only when no usable group space exists.
    """
    mgr = getattr(env, "action_manager", None)
    if mgr is not None:
        try:
            active_terms = list(mgr.active_terms)
            prefix = f"{group_name}/"
            out: dict[str, tuple[int, ...]] = {}
            for term_name in active_terms:
                if not term_name.startswith(prefix):
                    continue
                term = mgr.get_term(term_name)
                out[term_name[len(prefix) :]] = (int(term.action_dim),)
            return out
        except Exception as exc:  # noqa: BLE001
            logger.debug(
                "action_manager probe failed for group %s: %s; falling back",
                group_name,
                exc,
            )
    action_space = getattr(env, "action_space", None)
    if action_space is None:
        return None
    try:
        sub = action_space[group_name]
    except (AttributeError, IndexError, KeyError, TypeError):
        return None
    spaces = getattr(sub, "spaces", None)
    if spaces is None or not hasattr(spaces, "items"):
        return None
    return {
        term_name: tuple(int(d) for d in term_space.shape)
        for term_name, term_space in spaces.items()
    }


def _action_group_names(
    env: Any,
    art_action_groups: dict[str, Any],
    cfg_robots: dict[str, Any],
) -> list[str]:
    """List action groups deterministically, including manager-only subgroups.

    Combined robots can register several ``{group}/{term}`` prefixes although
    only the composite config carries ``action_group``. Merge manager prefixes,
    scene articulations, and configured robots to retain every group.
    """
    names: list[str] = []
    seen: set[str] = set()

    def _add(name: str) -> None:
        if name and name not in seen:
            seen.add(name)
            names.append(name)

    mgr = getattr(env, "action_manager", None)
    if mgr is not None:
        try:
            for term_name in mgr.active_terms:
                _add(str(term_name).split("/", 1)[0])
        except Exception as exc:  # noqa: BLE001
            logger.debug("action_manager.active_terms probe failed: %s", exc)
    for name in art_action_groups:
        _add(name)
    for name, robot in cfg_robots.items():
        if getattr(robot, "action_group", None) is not None:
            _add(name)
    return names


def _build_actions(env: Any) -> ActionSpec:
    groups: dict[str, GroupSpec] = {}
    # Runtime articulation cfgs omit the consumed ``Robot.action_group``;
    # recover it from the env's robot registry, keyed by scene name.
    cfg_robots = getattr(getattr(env, "cfg", None), "_robots", None) or {}
    art_action_groups: dict[str, Any] = {}
    for robot_name, robot in iter_scene_articulations(env.scene):
        ag = getattr(getattr(robot, "cfg", None), "action_group", None)
        if ag is not None:
            art_action_groups[robot_name] = ag

    for group_name in _action_group_names(env, art_action_groups, cfg_robots):
        runtime = _action_runtime_shapes(env, group_name)
        ag = art_action_groups.get(group_name)
        if ag is None:
            ag = getattr(cfg_robots.get(group_name), "action_group", None)
        if ag is not None:
            resolved = resolve_action_group_spec(ag, runtime)
            terms = resolved.terms
            if resolved.source == "fallback" and runtime:
                # Legacy/dummy configs may not expose shape-bearing fields for
                # every action manager term. Keep their reflected metadata and
                # fill only the terms that reflection could not describe.
                terms = {
                    **terms,
                    **{
                        term_name: TermSpec(
                            name=term_name,
                            shape=shape,
                            dtype="float32",
                            provenance="<runtime>",
                        )
                        for term_name, shape in runtime.items()
                        if term_name not in terms
                    },
                }
            schema_version = resolved.schema_version
            source = resolved.source
        elif runtime:
            # Combined-robot sub-articulations lack a reachable config; use the
            # manager's per-term dimensions to retain their group.
            terms = {
                term_name: TermSpec(
                    name=term_name,
                    shape=shape,
                    dtype="float32",
                    provenance="<runtime>",
                )
                for term_name, shape in runtime.items()
            }
            schema_version = 0
            source = "runtime"
        else:
            continue
        groups[group_name] = GroupSpec(
            name=group_name,
            terms=terms,
            schema_version=schema_version,
        )
        logger.info("action group %s resolved via %s", group_name, source)
    return ActionSpec(groups=groups)


def _build_observations(env: Any) -> ObservationSpec:
    groups: dict[str, GroupSpec] = {}
    obs_space = env.observation_space
    if not hasattr(obs_space, "spaces"):
        return ObservationSpec(groups=groups)
    for group_name, sub_space in obs_space.spaces.items():
        resolved = resolve_observation_group_spec(env, group_name, sub_space)
        groups[group_name] = GroupSpec(
            name=group_name,
            terms=resolved.terms,
            schema_version=resolved.schema_version,
        )
        logger.info("obs group %s resolved via %s", group_name, resolved.source)
    return ObservationSpec(groups=groups)


def _build_sensors(env: Any) -> SensorSpec:
    out = {}
    for name, sensor in iter_scene_sensors(env.scene):
        resolved = resolve_sensor_entry(name, sensor)
        out[name] = resolved.entry
        logger.info("sensor %s resolved via %s", name, resolved.source)
    return SensorSpec(sensors=out)


def _task_metadata(env: Any) -> dict[str, Any]:
    """Return optional task metadata, empty when the protocol is absent."""
    provider = getattr(env, "srb_spec_metadata", None)
    if not callable(provider):
        return {}
    metadata = provider()
    if not isinstance(metadata, dict):
        raise TypeError(
            f"srb_spec_metadata() must return a dict, got {type(metadata).__name__}"
        )
    return dict(metadata)


def build_env_spec(env: Any) -> EnvSpec:
    """Walk env's components, query SpecRegistry, assemble EnvSpec."""
    require_srb_spec_reset(env)
    obs = _build_observations(env)
    act = _build_actions(env)
    sen = _build_sensors(env)
    task = getattr(getattr(env, "cfg", None), "task_name", "") or ""
    metadata: dict[str, Any] = {
        "build_timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }
    metadata.update(_task_metadata(env))
    return EnvSpec(
        task=task,
        observation=obs,
        action=act,
        sensor=sen,
        metadata=metadata,
    )
