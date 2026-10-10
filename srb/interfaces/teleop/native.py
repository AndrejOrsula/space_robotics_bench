"""Isaac-free direct teleoperation input composition for real workflows."""

from __future__ import annotations

import threading
from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from srb.interfaces.enums import TeleopDeviceType
from srb.utils.failures import attempt_cleanup


class NativeTeleopInterface:
    """Compose supported native input devices into one flat six-axis command."""

    def __init__(
        self,
        devices: Sequence[TeleopDeviceType],
        node: Any = None,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 3.1415927,
        _adopt_cleanup_owner: Callable[[NativeTeleopInterface], None] | None = None,
    ):
        self.device_types = self.validate_devices(devices)
        self._closed = False
        self._close_lock = threading.RLock()
        self._cleanup_in_progress = False
        self._closed_interfaces: set[int] = set()
        self.interfaces: list[Any] = []
        self.ft_feedback_interfaces: list[Any] = []
        self.gain = 1.0
        self._close_gripper = False
        self._prev_gripper_cmds: list[bool] = []
        self._node = node
        if _adopt_cleanup_owner is not None:
            _adopt_cleanup_owner(self)

        def initialize() -> None:
            for device in self.device_types:
                interface = self._allocate(device)
                # Retain the cleanup owner before __init__ can acquire resources.
                self.interfaces.append(interface)
                if device is TeleopDeviceType.HAPTIC:
                    self.ft_feedback_interfaces.append(interface)
                interface.__init__(
                    **self._kwargs(device, pos_sensitivity, rot_sensitivity)
                )
                self.gain = 1.0
            self._prev_gripper_cmds = [False] * len(self.interfaces)

        _, construction_error = attempt_cleanup(initialize)
        if construction_error is not None:
            _, cleanup_error = attempt_cleanup(self.close)
            if cleanup_error is not None:
                raise BaseExceptionGroup(
                    "native teleop construction and cleanup failed",
                    [construction_error, cleanup_error],
                ) from construction_error
            raise construction_error

    @staticmethod
    def validate_devices(
        devices: Sequence[TeleopDeviceType],
    ) -> tuple[TeleopDeviceType, ...]:
        try:
            normalized = tuple(
                item if isinstance(item, TeleopDeviceType) else TeleopDeviceType(item)
                for item in devices
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid teleop device list: {devices!r}") from exc
        if not normalized:
            raise ValueError("native teleop requires at least one device")
        supported = {
            TeleopDeviceType.ROS,
            TeleopDeviceType.HAPTIC,
            TeleopDeviceType.SPACEMOUSE,
        }
        unsupported = tuple(item for item in normalized if item not in supported)
        if unsupported:
            names = ", ".join(str(item) for item in unsupported)
            raise ValueError(f"native teleop does not support device(s): {names}")
        return normalized

    @staticmethod
    def _allocate(device: TeleopDeviceType) -> Any:
        if device is TeleopDeviceType.SPACEMOUSE:
            try:
                from srb.interfaces.teleop._spacemouse_impl import (
                    SpacemouseImplementation,
                )
            except ImportError as exc:
                raise ImportError(
                    "native SpaceMouse teleop requires the pyspacemouse package"
                ) from exc
            return SpacemouseImplementation.__new__(SpacemouseImplementation)
        try:
            if device is TeleopDeviceType.ROS:
                from srb.interfaces.teleop._ros_impl import (
                    ROSTeleopInterfaceImplementation,
                )

                return ROSTeleopInterfaceImplementation.__new__(
                    ROSTeleopInterfaceImplementation
                )
            from srb.interfaces.teleop._haptic_impl import (
                HapticROSTeleopInterfaceImplementation,
            )

            return HapticROSTeleopInterfaceImplementation.__new__(
                HapticROSTeleopInterfaceImplementation
            )
        except ImportError as exc:
            raise ImportError(
                f"native {device} teleop requires the ROS 2 Python packages and message types"
            ) from exc

    def _kwargs(
        self, device: TeleopDeviceType, pos: float, rot: float
    ) -> dict[str, Any]:
        if device is TeleopDeviceType.ROS:
            return {
                "node": self._node,
                "pos_sensitivity": 10.0 * pos,
                "rot_sensitivity": 8.0 * rot,
            }
        if device is TeleopDeviceType.HAPTIC:
            return {
                "node": self._node,
                "pos_sensitivity": 8.0 * pos,
                "rot_sensitivity": rot,
            }
        return {
            "pos_sensitivity": 2.0 * pos,
            "rot_sensitivity": 0.8 * rot,
            "_strict": True,
        }

    def __str__(self) -> str:
        msg = "Native Teleop Interface\n"
        msg += f"Devices: {', '.join(str(device) for device in self.device_types)}\n"
        for interface in self.interfaces:
            msg += str(interface)
        return msg

    @property
    def callback_keys(self) -> frozenset[str]:
        keys: set[str] = set()
        for device in self.device_types:
            if device is TeleopDeviceType.SPACEMOUSE:
                keys.update(("L", "R", "LR"))
        return frozenset(keys)

    def add_callback(self, key: str, func: Callable[..., Any]) -> None:
        with self._close_lock:
            if self._closed:
                return
            for device, interface in zip(self.device_types, self.interfaces):
                if device is not TeleopDeviceType.SPACEMOUSE or key not in {
                    "L",
                    "R",
                    "LR",
                }:
                    continue
                add = getattr(interface, "add_callback", None)
                if callable(add):
                    add(key=key, func=func)

    def advance(self) -> tuple[np.ndarray, bool]:
        with self._close_lock:
            if self._closed:
                return np.zeros(6, dtype=np.float64), False
            raw_actions = [interface.advance() for interface in self.interfaces]
            twist = self.gain * np.sum(np.stack([a[0] for a in raw_actions]), axis=0)
            for index, previous in enumerate(self._prev_gripper_cmds):
                if previous != raw_actions[index][1]:
                    self._close_gripper = not self._close_gripper
                    break
            self._prev_gripper_cmds = [a[1] for a in raw_actions]
            return twist, self._close_gripper

    def reset(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            for interface in self.interfaces:
                interface.reset()
            self._close_gripper = False
            self._prev_gripper_cmds = [False] * len(self.interfaces)

    def set_ft_feedback(self, ft_feedback: Any) -> None:
        with self._close_lock:
            if self._closed:
                return
            for interface in self.ft_feedback_interfaces:
                interface.set_ft_feedback(ft_feedback)

    def close(self) -> None:
        self._close_lock.acquire()
        if self._cleanup_in_progress:
            self._close_lock.release()
            raise RuntimeError("native teleop cleanup already in progress")
        self._cleanup_in_progress = True
        try:
            self._closed = True
            errors: list[BaseException] = []
            for index, interface in enumerate(self.interfaces):
                if index in self._closed_interfaces:
                    continue
                close = getattr(interface, "close", None)
                if not callable(close):
                    self._closed_interfaces.add(index)
                    continue
                _, error = attempt_cleanup(close)
                if error is None:
                    self._closed_interfaces.add(index)
                else:
                    errors.append(error)
            if errors:
                if len(errors) == 1:
                    raise errors[0]
                raise BaseExceptionGroup("native teleop cleanup incomplete", errors)
        finally:
            self._cleanup_in_progress = False
            self._close_lock.release()
