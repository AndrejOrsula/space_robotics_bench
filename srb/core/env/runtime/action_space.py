# SPDX-License-Identifier: Apache-2.0 OR MIT
"""Compatibility helpers backed by the runtime action-layout authority."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING, Any

import numpy as np

from srb.core.env.runtime.action_layout import (
    ActionLayout,
    classify_action_space,
    iter_action_space_children,
)
from srb.core.env.runtime.validation import validate_non_negative_int

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def _classify_action_space(action_space: Any) -> str:
    """Historical private name; dispatch is owned by ``action_layout``."""
    return classify_action_space(action_space)


def action_dim_for_space(action_space: Any) -> int:
    """Return the canonical flattened action dimension for a space."""
    return ActionLayout.from_structured_space(action_space).flat_dim


def flat_action_schema(action_dim: int) -> list[dict[str, Any]]:
    """Return schema metadata for an unstructured flat action vector."""
    dim = validate_non_negative_int(action_dim, name="action_dim")
    return [
        {
            "name": "action",
            "path": [],
            "shape": [dim],
            "dim": dim,
            "start": 0,
            "end": dim,
            "dtype": None,
        }
    ]


def action_schema_for_space(action_space: Any) -> list[dict[str, Any]]:
    """Return the layout-owned ordered flat-vector components."""
    return ActionLayout.from_structured_space(action_space).schema()


def flatten_action(action: Any, action_space: Any | None = None) -> np.ndarray:
    """Validate and flatten an action through one generic layout authority."""
    layout = ActionLayout.from_structured_space(action_space)
    if action_space is None:
        return layout.flatten_untyped(action)
    return layout.flatten(action)


def iter_mapping_action(
    action: Mapping[Any, Any],
    action_space: Any | None,
) -> list[tuple[Any, Any, Any | None]]:
    """Return mapping action values in layout-defined space order."""
    if _classify_action_space(action_space) != "mapping":
        return [(key, value, None) for key, value in action.items()]
    layout = ActionLayout.from_structured_space(action_space)
    layout.validate_mapping_keys(action)
    return [
        (key, action[key], space)
        for key, space in iter_action_space_children(action_space)
    ]


def validate_mapping_action_keys(
    action: Mapping[Any, Any], spaces: Mapping[Any, Any]
) -> None:
    """Raise when mapping action keys do not match the declared layout."""
    ActionLayout.from_structured_space(spaces).validate_mapping_keys(action)


def coerce_action_for_space(
    action: Any,
    action_space: Any | None,
    device: Any = None,
) -> Any:
    """Coerce an action to float32 Torch leaves through the layout authority."""
    layout = ActionLayout.from_structured_space(action_space)
    if action_space is None:
        return layout.coerce_untyped(action, device=device)
    return layout.coerce(action, device=device)


def unflatten_action(
    flat_action: Any,
    spaces: Mapping[Any, Any],
    device: Any = None,
) -> dict[Any, Any]:
    """Convert a strict flat vector into mapping-space ordered tensors."""
    layout = ActionLayout.from_structured_space(spaces)
    structured = layout.coerce(flat_action, device=device)
    if not isinstance(structured, dict):
        raise TypeError("mapping action layout did not produce a mapping")
    return structured


def coerce_policy_action_for_env(action: Any, env: "AnyEnv") -> Any:
    """Coerce a loaded-policy action for direct ``env.step`` evaluation."""
    unwrapped = getattr(env, "unwrapped", env)
    device = getattr(unwrapped, "device", None)
    for owner in (env, unwrapped):
        single_space = getattr(owner, "single_action_space", None)
        if single_space is not None:
            num_envs = getattr(owner, "num_envs", None)
            if num_envs is None:
                num_envs = getattr(unwrapped, "num_envs", 1)
            return coerce_policy_action(
                action, single_space, device, num_envs=int(num_envs)
            )
    return coerce_policy_action(action, env_action_space(env), device)


def coerce_policy_action(
    action: Any, action_space: Any, device: Any = None, *, num_envs: int = 1
) -> Any:
    """Validate policy output against a single-env space and ``num_envs``."""
    layout = ActionLayout.from_structured_space(action_space)
    if action_space is None:
        return layout.policy_action_untyped(action, device=device)
    return layout.policy_action(action, device=device, num_envs=num_envs)


def env_action_space(env: "AnyEnv") -> Any:
    """Return the effective outer action space used for one policy call."""
    unwrapped = getattr(env, "unwrapped", env)
    for owner in (env, unwrapped):
        action_space = getattr(owner, "action_space", None)
        if action_space is not None:
            return action_space
    return getattr(unwrapped, "single_action_space", None)


def env_action_shape(env: "AnyEnv") -> tuple[int, ...] | None:
    """Return the direct policy action shape for an environment."""
    return action_space_shape(env_action_space(env))


def action_space_shape(action_space: Any) -> tuple[int, ...] | None:
    """Return a normalized leaf action-space shape, if present."""
    shape = getattr(action_space, "shape", None)
    if shape is None:
        return None
    try:
        return tuple(int(dim) for dim in tuple(shape))
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"action space shape must be a sequence, got {shape!r}"
        ) from exc


def reshape_action(action: Any, target_shape: tuple[int, ...] | None) -> Any:
    """Add a single-env batch dimension only when the shape matches exactly."""
    if target_shape is None:
        return action
    action_shape = tuple(getattr(action, "shape", ()))
    if action_shape == target_shape:
        return action
    if target_shape == (1, *action_shape):
        return action.reshape(target_shape)
    raise ValueError(
        f"policy action shape must be {target_shape} or omit only a leading "
        f"single-env axis, got {action_shape}"
    )


__all__ = [
    "action_dim_for_space",
    "action_schema_for_space",
    "action_space_shape",
    "coerce_action_for_space",
    "coerce_policy_action",
    "coerce_policy_action_for_env",
    "env_action_shape",
    "env_action_space",
    "flat_action_schema",
    "flatten_action",
    "iter_mapping_action",
    "reshape_action",
    "unflatten_action",
    "validate_mapping_action_keys",
]
