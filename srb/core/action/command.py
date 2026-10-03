"""Compatibility exports for the core direct-command dispatcher."""

from srb.core.command import (
    env_action_group,
    env_supports_direct_action_command,
    map_direct_action_command,
    supports_direct_action_command,
)

__all__ = [
    "env_action_group",
    "env_supports_direct_action_command",
    "map_direct_action_command",
    "supports_direct_action_command",
]
