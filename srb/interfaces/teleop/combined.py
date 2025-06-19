import threading
from collections.abc import Callable
from typing import TYPE_CHECKING, Sequence

import numpy
import torch

from srb.core.action import ActionGroup
from srb.interfaces.enums import TeleopDeviceType
from srb.interfaces.teleop import DeviceBase, Se3Gamepad
from srb.interfaces.teleop.keyboard_omni import OmniKeyboardTeleopInterface
from srb.interfaces.teleop.spacemouse import SpacemouseTeleopInterface

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
        if not node and (
            TeleopDeviceType.ROS in devices or TeleopDeviceType.HAPTIC in devices
        ):
            from srb.utils.ros import enable_ros2_bridge

            enable_ros2_bridge()
            import rclpy
            from rclpy.node import Node

            rclpy.init()
            self._node = Node("sim", namespace="srb", start_parameter_services=False)  # type: ignore
        else:
            self._node = node

        self._actions = actions
        self.interfaces = []
        self.ft_feedback_interfaces = []
        for device in devices:
            match device.lower():
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
                        Se3Gamepad(
                            pos_sensitivity=10.0 * pos_sensitivity,
                            rot_sensitivity=8.0 * rot_sensitivity,
                        )
                    )
                case TeleopDeviceType.ROS:
                    from srb.interfaces.teleop.ros import ROSTeleopInterface

                    self.interfaces.append(
                        ROSTeleopInterface(
                            node=self._node,
                            # Note: Gains assume that ROS messages originate from a gamepad
                            pos_sensitivity=10.0 * pos_sensitivity,
                            rot_sensitivity=8.0 * rot_sensitivity,
                        )
                    )
                case TeleopDeviceType.HAPTIC:
                    from srb.interfaces.teleop.haptic import HapticROSTeleopInterface

                    interface = HapticROSTeleopInterface(
                        node=self._node,
                        pos_sensitivity=8.0 * pos_sensitivity,
                        rot_sensitivity=1.0 * rot_sensitivity,
                    )
                    self.interfaces.append(interface)
                    self.ft_feedback_interfaces.append(interface)
                case _:
                    raise ValueError(f"Invalid device interface '{device}'.")

            self.gain = 1.0

            def cb_gain_decrease():
                self.gain *= 0.75
                print(f"Gain: {self.gain}")

            self.add_callback("O", cb_gain_decrease)

            def cb_gain_increase():
                self.gain *= 1.25
                print(f"Gain: {self.gain}")

            self.add_callback("P", cb_gain_increase)

        # Run a thread for listening to device
        if not node and self._node is not None:
            from rclpy.executors import MultiThreadedExecutor

            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin)
            self._thread.daemon = True
            self._thread.start()

    def __del__(self):
        for interface in self.interfaces:
            interface.__del__()

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

    def advance(self) -> tuple[numpy.ndarray, bool]:
        raw_actions = [interface.advance() for interface in self.interfaces]

        twist = self.gain * numpy.sum(
            numpy.stack([a[0] for a in raw_actions], axis=0), axis=0
        )

        for i, prev_gripper_cmd in enumerate(self._prev_gripper_cmds):
            if prev_gripper_cmd != raw_actions[i][1]:
                self._close_gripper = not self._close_gripper
                break
        self._prev_gripper_cmds = [a[1] for a in raw_actions]

        return twist, self._close_gripper

    def set_ft_feedback(self, ft_feedback: numpy.ndarray | torch.Tensor):
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
|  (-Y) [ A ] ← + → [ D ] (+Y)        +          |
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
