"""Teleoperation interfaces with lazy simulator compatibility exports."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "CombinedTeleopInterface",
    "EventOmniKeyboardTeleopInterface",
    "GamepadTeleopInterface",
    "HapticROSTeleopInterface",
    "NativeTeleopInterface",
    "OmniKeyboardTeleopInterface",
    "ROSTeleopInterface",
    "SpacemouseTeleopInterface",
    "DeviceBase",
    "DeviceCfg",
    "DevicesCfg",
    "HaplyDevice",
    "HaplyDeviceCfg",
    "ManusVive",
    "ManusViveCfg",
    "OpenXRDevice",
    "OpenXRDeviceCfg",
    "RetargeterBase",
    "RetargeterCfg",
    "Se2Gamepad",
    "Se2GamepadCfg",
    "Se2Keyboard",
    "Se2KeyboardCfg",
    "Se2SpaceMouse",
    "Se2SpaceMouseCfg",
    "Se3Gamepad",
    "Se3GamepadCfg",
    "Se3Keyboard",
    "Se3KeyboardCfg",
    "Se3SpaceMouse",
    "Se3SpaceMouseCfg",
    "create_teleop_device",
]

_ISAAC_DEVICE_EXPORTS = frozenset(
    {
        "DeviceBase",
        "DeviceCfg",
        "DevicesCfg",
        "HaplyDevice",
        "HaplyDeviceCfg",
        "ManusVive",
        "ManusViveCfg",
        "OpenXRDevice",
        "OpenXRDeviceCfg",
        "RetargeterBase",
        "RetargeterCfg",
        "Se2Gamepad",
        "Se2GamepadCfg",
        "Se2Keyboard",
        "Se2KeyboardCfg",
        "Se2SpaceMouse",
        "Se2SpaceMouseCfg",
        "Se3Gamepad",
        "Se3GamepadCfg",
        "Se3Keyboard",
        "Se3KeyboardCfg",
        "Se3SpaceMouse",
        "Se3SpaceMouseCfg",
        "create_teleop_device",
    }
)

_EXPORTS = {
    "CombinedTeleopInterface": (".combined", "CombinedTeleopInterface"),
    "EventOmniKeyboardTeleopInterface": (
        ".keyboard_omni",
        "EventOmniKeyboardTeleopInterface",
    ),
    "GamepadTeleopInterface": (".gamepad", "GamepadTeleopInterface"),
    "OmniKeyboardTeleopInterface": (".keyboard_omni", "OmniKeyboardTeleopInterface"),
    "SpacemouseTeleopInterface": (".spacemouse", "SpacemouseTeleopInterface"),
    "ROSTeleopInterface": (".ros", "ROSTeleopInterface"),
    "HapticROSTeleopInterface": (".haptic", "HapticROSTeleopInterface"),
    "NativeTeleopInterface": (".native", "NativeTeleopInterface"),
}


def __getattr__(name: str) -> Any:
    if name in _ISAAC_DEVICE_EXPORTS:
        devices = import_module("isaaclab.devices")
        return getattr(devices, name)
    try:
        module_name, attr_name = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(name) from exc
    return getattr(import_module(module_name, __name__), attr_name)


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
