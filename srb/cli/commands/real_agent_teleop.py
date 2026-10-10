"""Native real-agent teleoperation preflight and input construction."""

from __future__ import annotations

from importlib import import_module
from typing import Any, Sequence

import gymnasium

from srb.core.command_mapping import (
    CommandMapping,
    revalidate_command_mapping,
    validate_command_mapping,
)
from srb.core.env.runtime.action_layout import ActionLayout


def _registered_real_env_class(env_id: str) -> type:
    """Load a registered environment class without constructing it."""
    spec = gymnasium.spec(env_id)
    entry_point = spec.entry_point
    if isinstance(entry_point, str):
        module_name, separator, attribute = entry_point.partition(":")
        if not separator or not module_name or not attribute:
            raise ValueError(f"Invalid RealEnv entry point for {env_id!r}")
        entry_point = getattr(import_module(module_name), attribute)
    if not isinstance(entry_point, type):
        raise TypeError(f"RealEnv entry point for {env_id!r} must resolve to a class")
    return entry_point


def _validate_generated_class(env_class: type) -> None:
    """Revalidate generated action domains and declared direct-command mapping."""
    layout = ActionLayout.from_spaces(
        env_class.DETAILED_ACTION_SPACE,
        ordering=env_class.ACTION_ORDERING,
        scale=env_class.ACTION_SCALE,
        single_space=env_class.SINGLE_ACTION_SPACE,
    )
    single_space = layout.single_space
    if (
        not isinstance(single_space, gymnasium.spaces.Box)
        or len(single_space.shape) != 1
    ):
        raise ValueError("Native real-agent teleop requires a flat single-action Box")
    declared = env_class.COMMAND_MAPPING
    if declared is None:
        raise ValueError("RealEnv has no declared COMMAND_MAPPING for native teleop")
    mapping = (
        revalidate_command_mapping(declared)
        if isinstance(declared, CommandMapping)
        else CommandMapping.model_validate(declared)
    )
    validate_command_mapping(mapping, layout.ordering, int(single_space.shape[0]))


def _validate_real_env_instance(env: Any) -> None:
    """Revalidate the constructed scalar action contract before input setup."""
    env_class = type(env)
    single_space = getattr(env, "single_action_space", None)
    if single_space is None:
        raise ValueError("Native real-agent teleop requires a single action space")
    layout = ActionLayout.from_spaces(
        env_class.DETAILED_ACTION_SPACE,
        ordering=env_class.ACTION_ORDERING,
        scale=env_class.ACTION_SCALE,
        single_space=single_space,
    )
    if (
        not isinstance(single_space, gymnasium.spaces.Box)
        or len(single_space.shape) != 1
    ):
        raise ValueError("Native real-agent teleop requires a flat single-action Box")
    declared = getattr(env, "command_mapping", None)
    if declared is None:
        raise ValueError("RealEnv has no declared COMMAND_MAPPING for native teleop")
    mapping = (
        revalidate_command_mapping(declared)
        if isinstance(declared, CommandMapping)
        else CommandMapping.model_validate(declared)
    )
    validate_command_mapping(mapping, layout.ordering, int(single_space.shape[0]))


def _resolve_native_devices(teleop_device: Sequence[str] | None) -> tuple[Any, ...]:
    """Resolve and validate the complete native input list without acquiring it."""
    from srb.interfaces.enums import TeleopDeviceType

    names = tuple(teleop_device or ()) or ("ros",)
    devices = []
    for name in names:
        device = (
            name
            if isinstance(name, TeleopDeviceType)
            else TeleopDeviceType.from_str(str(name))
        )
        if device is None:
            raise ValueError(f"Unsupported teleop device: {name!r}")
        if device not in (
            TeleopDeviceType.ROS,
            TeleopDeviceType.HAPTIC,
            TeleopDeviceType.SPACEMOUSE,
        ):
            raise ValueError(
                f"Native real-agent teleop does not support {device} input"
            )
        if device not in devices:
            devices.append(device)
    return tuple(devices)


def preflight_real_teleop(
    env_id: str, *, teleop_device: Sequence[str] | None = None
) -> tuple[Any, ...]:
    """Validate generated command metadata and all requested native devices."""
    env_class = _registered_real_env_class(env_id)
    _validate_generated_class(env_class)
    return _resolve_native_devices(teleop_device)


def prepare_real_teleop(
    env: Any,
    *,
    session: Any,
    teleop_device: Sequence[str] | None,
    pos_sensitivity: float,
    rot_sensitivity: float,
) -> Any:
    """Adopt a native input composite before it acquires device resources."""
    if session is None:
        raise ValueError("Native real-agent teleop requires a RealAgentSession")
    unwrapped = getattr(env, "unwrapped", env)
    _validate_real_env_instance(unwrapped)
    devices = _resolve_native_devices(teleop_device)
    from srb.interfaces.teleop.native import NativeTeleopInterface

    validator = getattr(NativeTeleopInterface, "validate_devices", None)
    if not callable(validator):
        raise TypeError("NativeTeleopInterface must expose validate_devices()")
    validator(devices)
    node = getattr(unwrapped, "ros_node", None)
    from srb.interfaces.enums import TeleopDeviceType

    if node is None and any(
        device in (TeleopDeviceType.ROS, TeleopDeviceType.HAPTIC) for device in devices
    ):
        raise ValueError("Native ROS teleop requires the RealEnv-owned ROS node")

    def adopt(owner: Any) -> None:
        session.adopt_command_source(
            owner,
            abort=lambda: unwrapped.abort_session("native teleoperation input stopped"),
        )

    return NativeTeleopInterface(
        devices,
        node=node,
        pos_sensitivity=pos_sensitivity,
        rot_sensitivity=rot_sensitivity,
        _adopt_cleanup_owner=adopt,
    )


__all__ = ("prepare_real_teleop", "preflight_real_teleop")
