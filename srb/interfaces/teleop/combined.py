import threading
from collections.abc import Callable
from functools import partial
from typing import TYPE_CHECKING, Sequence

import numpy as np
import torch

from srb.core.action import ActionGroup
from srb.interfaces._ros_node import (
    RosCleanupError,
    RosCleanupScope,
    acquire_ros_node_runtime,
    attempt_ros_cleanup,
)
from srb.interfaces.enums import TeleopDeviceType
from srb.interfaces.teleop import DeviceBase
from srb.interfaces.teleop.gamepad import GamepadTeleopInterface
from srb.interfaces.teleop.keyboard_omni import OmniKeyboardTeleopInterface
from srb.interfaces.teleop.spacemouse import SpacemouseTeleopInterface
from srb.utils.failures import attempt_cleanup

if TYPE_CHECKING:
    from rclpy.node import Node


class CombinedTeleopInterface(DeviceBase):
    def __init__(
        self,
        devices: Sequence[TeleopDeviceType],
        node: "Node | None" = None,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 3.1415927,
        actions: ActionGroup | None = None,
    ):
        if not devices:
            raise ValueError("combined teleop requires at least one device")
        self._closed = False
        self._close_lock = threading.RLock()
        self._cleanup_in_progress = False
        self._closed_interfaces: set[int] = set()
        self._runtime_closed = False
        self._ros_runtime = None
        self._node = node
        self._actions = actions
        self.interfaces = []
        self.ft_feedback_interfaces = []
        self.gain = 1.0
        self._close_gripper = False
        self._prev_gripper_cmds: list[bool] = []

        with RosCleanupScope(self, "Combined teleop construction cleanup incomplete"):
            if TeleopDeviceType.ROS in devices or TeleopDeviceType.HAPTIC in devices:
                if node is None:
                    from srb.utils.ros import enable_ros2_bridge

                    enable_ros2_bridge()
                try:
                    self._ros_runtime = acquire_ros_node_runtime(node, spin=True)
                except RosCleanupError as error:
                    owner = error.cleanup_owner
                    if (
                        owner is not None
                        and owner is not self
                        and callable(getattr(owner, "close", None))
                    ):
                        self._ros_runtime = owner
                        self._node = getattr(owner, "node", self._node)
                    raise
                self._node = self._ros_runtime.node

            for device in devices:
                try:
                    match device:
                        case TeleopDeviceType.KEYBOARD:
                            self.interfaces.append(
                                OmniKeyboardTeleopInterface(
                                    pos_sensitivity=0.6 * pos_sensitivity,
                                    rot_sensitivity=0.4 * rot_sensitivity,
                                )
                            )
                        case TeleopDeviceType.SPACEMOUSE:
                            self.interfaces.append(
                                SpacemouseTeleopInterface(
                                    pos_sensitivity=2.0 * pos_sensitivity,
                                    rot_sensitivity=0.8 * rot_sensitivity,
                                )
                            )
                        case TeleopDeviceType.GAMEPAD:
                            self.interfaces.append(
                                GamepadTeleopInterface(
                                    pos_sensitivity=10.0 * pos_sensitivity,
                                    rot_sensitivity=8.0 * rot_sensitivity,
                                )
                            )
                        case TeleopDeviceType.ROS:
                            from srb.interfaces.teleop.ros import ROSTeleopInterface

                            self.interfaces.append(
                                ROSTeleopInterface(
                                    node=self._node,
                                    # ROS gains assume messages originate from a gamepad.
                                    pos_sensitivity=10.0 * pos_sensitivity,
                                    rot_sensitivity=8.0 * rot_sensitivity,
                                )
                            )
                        case TeleopDeviceType.HAPTIC:
                            from srb.interfaces.teleop.haptic import (
                                HapticROSTeleopInterface,
                            )

                            interface = HapticROSTeleopInterface(
                                node=self._node,
                                pos_sensitivity=8.0 * pos_sensitivity,
                                rot_sensitivity=1.0 * rot_sensitivity,
                            )
                            self.interfaces.append(interface)
                            self.ft_feedback_interfaces.append(interface)
                        case _:
                            raise ValueError(f"Invalid device interface '{device}'.")
                except RosCleanupError as error:
                    owner = error.cleanup_owner
                    if (
                        owner is not None
                        and owner is not self
                        and callable(getattr(owner, "close", None))
                    ):
                        self.interfaces.append(owner)
                    raise

            # Register once after every device exists: add_callback attaches to
            # all keyboards, so per-device registration would stack multipliers.
            def cb_gain_decrease():
                self.gain *= 0.75
                print(f"Gain: {self.gain}")

            self.add_callback("O", cb_gain_decrease)

            def cb_gain_increase():
                self.gain *= 1.25
                print(f"Gain: {self.gain}")

            self.add_callback("P", cb_gain_increase)

            self._prev_gripper_cmds = [False] * len(self.interfaces)

    def __del__(self):
        attempt_ros_cleanup(self.close)

    def close(self) -> None:
        close_lock = getattr(self, "_close_lock", None)
        if close_lock is None:
            close_lock = threading.RLock()
            self._close_lock = close_lock

        if not close_lock.acquire(blocking=False):
            raise RuntimeError("combined teleop cleanup already in progress")
        if getattr(self, "_cleanup_in_progress", False):
            close_lock.release()
            raise RuntimeError("combined teleop cleanup already in progress")
        self._cleanup_in_progress = True
        try:
            self._closed = True
            closed_interfaces = getattr(self, "_closed_interfaces", None)
            if closed_interfaces is None:
                closed_interfaces = set()
                self._closed_interfaces = closed_interfaces
            # Continue cleanup after errors; failed resources remain retryable.
            errors: list[BaseException] = []

            for index, interface in enumerate(getattr(self, "interfaces", ())):
                if index in closed_interfaces:
                    continue

                def close_child(child: object = interface) -> None:
                    # Include lookup failures so remaining resources still close.
                    close = getattr(child, "close", None)
                    if not callable(close):
                        close = getattr(child, "__del__", None)
                    if callable(close):
                        close()

                _, error = attempt_cleanup(close_child)
                if error is None:
                    closed_interfaces.add(index)
                else:
                    errors.append(error)

            runtime = getattr(self, "_ros_runtime", None)
            runtime_closed = getattr(self, "_runtime_closed", False)
            if not runtime_closed:
                if runtime is None:
                    self._runtime_closed = True
                else:
                    _, error = attempt_cleanup(runtime.close)
                    if error is not None:
                        errors.append(error)
                    else:
                        self._runtime_closed = True
                        node_handle = getattr(runtime, "node_handle", None)
                        if getattr(runtime, "owns", False) and getattr(
                            node_handle, "released", False
                        ):
                            shared_node = getattr(runtime, "node", None)
                            for index, interface in enumerate(
                                getattr(self, "interfaces", ())
                            ):
                                mark_shared_node_destroyed = getattr(
                                    interface,
                                    "_mark_shared_node_destroyed",
                                    None,
                                )
                                if not callable(mark_shared_node_destroyed):
                                    continue
                                marked, error = attempt_cleanup(
                                    partial(mark_shared_node_destroyed, shared_node)
                                )
                                if error is not None:
                                    errors.append(error)
                                elif marked:
                                    closed_interfaces.add(index)

            if errors:
                if len(errors) == 1:
                    raise errors[0]
                # Preserve cancellation and other BaseException subclasses.
                raise BaseExceptionGroup("combined teleop cleanup failed", errors)
        finally:
            self._cleanup_in_progress = False
            close_lock.release()

    def __str__(self) -> str:
        from srb.interfaces.teleop.keyboard_omni import OmniKeyboardTeleopInterface

        msg = "Combined Interface\n"
        msg += f"Devices: {', '.join([interface.__class__.__name__ for interface in self.interfaces])}\n"

        for interface in self.interfaces:
            if (
                isinstance(interface, OmniKeyboardTeleopInterface)
                and self._actions is not None
            ):
                msg += self._keyboard_control_scheme()
                continue
            msg += "\n"
            msg += interface.__str__()

        return msg

    """
    Operations
    """

    def reset(self):
        close_lock = getattr(self, "_close_lock", None)
        if close_lock is None:
            close_lock = threading.RLock()
            self._close_lock = close_lock
        with close_lock:
            if getattr(self, "_closed", False):
                return
            for interface in self.interfaces:
                interface.reset()

            self._close_gripper = False
            self._prev_gripper_cmds = [False] * len(self.interfaces)

    def add_callback(self, key: str, func: Callable):
        for interface in self.interfaces:
            if isinstance(interface, OmniKeyboardTeleopInterface):
                interface.add_callback(key=key, func=func)
            if isinstance(interface, SpacemouseTeleopInterface) and key in [
                "L",
                "R",
                "LR",
            ]:
                interface.add_callback(key=key, func=func)

    def advance(self) -> tuple[np.ndarray, bool]:
        close_lock = getattr(self, "_close_lock", None)
        if close_lock is None:
            close_lock = threading.RLock()
            self._close_lock = close_lock
        with close_lock:
            if getattr(self, "_closed", False):
                return np.zeros(6, dtype=np.float64), False
            raw_actions = [interface.advance() for interface in self.interfaces]

            twist = self.gain * np.sum(
                np.stack([a[0] for a in raw_actions], axis=0), axis=0
            )

            for i, prev_gripper_cmd in enumerate(self._prev_gripper_cmds):
                if prev_gripper_cmd != raw_actions[i][1]:
                    self._close_gripper = not self._close_gripper
                    break
            self._prev_gripper_cmds = [a[1] for a in raw_actions]

            return twist, self._close_gripper

    def set_ft_feedback(self, ft_feedback: np.ndarray | torch.Tensor):
        close_lock = getattr(self, "_close_lock", None)
        if close_lock is None:
            close_lock = threading.RLock()
            self._close_lock = close_lock
        with close_lock:
            if getattr(self, "_closed", False):
                return
            for interface in self.ft_feedback_interfaces:
                interface.set_ft_feedback(ft_feedback)

    @staticmethod
    def _keyboard_control_scheme() -> str:
        return """
+------------------------------------------------+
|  Keyboard Scheme (focus the Isaac Sim window)  |
+------------------------------------------------+
+------------------------------------------------+
| Reset: [ L ]                                   |
| Decrease Gain [ O ]   | Increase Gain: [ P ]   |
| Event: [ R / K ]                               |
+------------------------------------------------+
| Translation                                    |
|             [ W ] (+X)            [ Q ] (+Z)   |
|               ↑                     ↑          |
|               |                     |          |
|  (+Y) [ A ] ← + → [ D ] (-Y)        +          |
|               |                     |          |
|               ↓                     ↓          |
|             [ S ] (-X)            [ E ] (-Z)   |
|------------------------------------------------|
| Rotation                                       |
|       [ Z ] ←--------(±X)--------→ [ X ]       |
|                                                |
|       [ T ] ↻--------(±Y)--------↺ [ G ]       |
|                                                |
|       [ C ] ↺--------(±Z)--------↻ [ V ]       |
+------------------------------------------------+
        """
