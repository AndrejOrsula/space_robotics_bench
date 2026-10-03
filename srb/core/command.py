"""Environment-facing direct-command dispatch with an Isaac-free native path."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import torch

from srb.core.command_mapping import (
    CommandMapping,
    apply_command_mapping,
    revalidate_command_mapping,
)

if TYPE_CHECKING:
    from srb.core.action.action_group import ActionGroup


def _target_env(env: Any) -> Any:
    return getattr(env, "unwrapped", env)


def _action_group_type():
    # Import only after a configured action group exists. Importing this module
    # through srb.core.action would load Isaac Lab's package initializers.
    from srb.core.action.action_group import ActionGroup

    return ActionGroup


def _declared_mapping(target: Any) -> CommandMapping | None:
    mapping = getattr(target, "command_mapping", None)
    if mapping is None:
        return None
    if not isinstance(mapping, CommandMapping):
        raise TypeError("environment command_mapping must be a CommandMapping")
    return revalidate_command_mapping(mapping)


def env_action_group(env: Any) -> ActionGroup | None:
    """Return the SRB ActionGroup configured on an env, if one exists."""
    target = _target_env(env)
    cfg = getattr(target, "cfg", None)
    actions = getattr(cfg, "actions", None)
    if actions is None:
        return None
    action_group = _action_group_type()
    return actions if isinstance(actions, action_group) else None


def supports_direct_action_command(
    actions: Any,
    *,
    command_dim: int = 6,
) -> bool:
    """Return whether an action group maps a direct command into actions."""
    if actions is None:
        return False
    action_group = _action_group_type()
    if not isinstance(actions, action_group):
        return False
    try:
        actions.map_cmd_to_action(torch.zeros(command_dim), False)
    except NotImplementedError:
        return False
    return True


def env_supports_direct_action_command(
    env: Any,
    *,
    command_dim: int = 6,
) -> bool:
    """Return whether an env supports declared or simulator direct dispatch."""
    target = _target_env(env)
    if _declared_mapping(target) is not None:
        return True
    return supports_direct_action_command(
        env_action_group(target),
        command_dim=command_dim,
    )


def map_direct_action_command(
    env: Any,
    command: Any,
    event: bool,
    *,
    dtype: torch.dtype = torch.float32,
) -> torch.Tensor:
    """Map a direct command through a native declaration or simulator group."""
    target = _target_env(env)
    mapping = _declared_mapping(target)

    actions = None
    if mapping is None:
        actions = env_action_group(target)
        if actions is None:
            raise RuntimeError(
                "Environment does not expose an SRB direct action command"
            )

    tensor_kwargs: dict[str, Any] = {"dtype": dtype}
    device = getattr(target, "device", None)
    if device is not None:
        tensor_kwargs["device"] = device
    twist = torch.as_tensor(command, **tensor_kwargs)

    if mapping is not None:
        return apply_command_mapping(mapping, twist, event)

    assert actions is not None
    return actions.map_cmd_to_action(twist, event)


__all__ = [
    "env_action_group",
    "env_supports_direct_action_command",
    "map_direct_action_command",
    "supports_direct_action_command",
]
