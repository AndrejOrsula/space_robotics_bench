from collections.abc import Callable
from typing import List, Optional, Union

import numpy as np
from omni.isaac.lab.devices import DeviceBase

from space_robotics_bench.core.actions import (
    ManipulatorTaskSpaceActionCfg,
    MultiCopterActionGroupCfg,
    SpacecraftActionGroupCfg,
    WheeledRoverActionGroupCfg,
)
from space_robotics_bench.core.teleop_devices import (
    Se3Gamepad,
    Se3Keyboard,
    Se3ROS2,
    Se3SpaceMouse,
    Se3Touch,
)
from space_robotics_bench.utils.ros import enable_ros2_bridge


class CombinedInterface(DeviceBase):
    def __init__(
        self,
        devices: List[str],
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 1.0,
        action_cfg: Optional[
            Union[
                ManipulatorTaskSpaceActionCfg,
                MultiCopterActionGroupCfg,
                WheeledRoverActionGroupCfg,
            ]
        ] = None,
        node: Optional[object] = None,
    ):
        enable_ros2_bridge()
        import rclpy
        from rclpy.node import Node

        if node is None:
            rclpy.init(args=None)
            self._node = Node("srb_teleop_combined")
        else:
            self._node = node

        self._action_cfg = action_cfg
        self.interfaces = []
        for device in devices:
            if device.lower() == "keyboard":
                self.interfaces.append(
                    Se3Keyboard(
                        pos_sensitivity=0.05 * pos_sensitivity,
                        rot_sensitivity=10.0 * rot_sensitivity,
                    )
                )
            elif device.lower() == "ros2":
                self.interfaces.append(
                    Se3ROS2(
                        node=self._node,
                        pos_sensitivity=1.0 * pos_sensitivity,
                        rot_sensitivity=1.0 * rot_sensitivity,
                    )
                )
            elif device.lower() == "touch":
                self.interfaces.append(
                    Se3Touch(
                        node=self._node,
                        pos_sensitivity=1.0 * pos_sensitivity,
                        rot_sensitivity=0.15 * rot_sensitivity,
                    )
                )
            elif device.lower() == "spacemouse":
                self.interfaces.append(
                    Se3SpaceMouse(
                        pos_sensitivity=0.1 * pos_sensitivity,
                        rot_sensitivity=0.05 * rot_sensitivity,
                    )
                )
            elif device.lower() == "gamepad":
                self.interfaces.append(
                    Se3Gamepad(
                        pos_sensitivity=0.1 * pos_sensitivity,
                        rot_sensitivity=0.1 * rot_sensitivity,
                    )
                )
            else:
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

    def __del__(self):
        for interface in self.interfaces:
            interface.__del__()

    def __str__(self) -> str:
        msg = "Combined Interface\n"
        msg += f"Devices: {', '.join([interface.__class__.__name__ for interface in self.interfaces])}\n"

        for interface in self.interfaces:
            if isinstance(interface, Se3Keyboard) and self._action_cfg is not None:
                msg += self._keyboard_control_scheme(self._action_cfg)
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
            if isinstance(interface, Se3Keyboard):
                interface.add_callback(key=key, func=func)
            if isinstance(interface, Se3SpaceMouse) and key in ["L", "R", "LR"]:
                interface.add_callback(key=key, func=func)

    def advance(self) -> tuple[np.ndarray, bool]:
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

    def set_ft_feedback(self, ft_feedback: np.ndarray):
        for interface in self.interfaces:
            if isinstance(interface, Se3Touch):
                interface.set_ft_feedback(ft_feedback)

    @staticmethod
    def _keyboard_control_scheme(
        action_cfg: Union[
            ManipulatorTaskSpaceActionCfg,
            MultiCopterActionGroupCfg,
            WheeledRoverActionGroupCfg,
        ],
    ) -> str:
        if isinstance(action_cfg, ManipulatorTaskSpaceActionCfg):
            return """
+------------------------------------------------+
|  Keyboard Scheme (focus the Isaac Sim window)  |
+------------------------------------------------+
+------------------------------------------------+
| Reset: [ L ]                                   |
| Decrease Gain [ O ]   | Increase Gain: [ P ]   |
| Toggle Gripper: [ R / K ]                      |
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
        elif isinstance(action_cfg, MultiCopterActionGroupCfg):
            return """
+------------------------------------------------+
|  Keyboard Scheme (focus the Isaac Sim window)  |
+------------------------------------------------+
+------------------------------------------------+
| Decrease Gain [ O ]   | Increase Gain: [ P ]   |
| Reset: [ L ]                                   |
+------------------------------------------------+
|                  Translation                   |
|             [ W ] (+X)            [ Q ] (+Z)   |
|               ↑                     ↑          |
|               |                     |          |
|  (-Y) [ A ] ← + → [ D ] (+Y)        +          |
|               |                     |          |
|               ↓                     ↓          |
|             [ S ] (-X)            [ E ] (-Z)   |
|------------------------------------------------|
|                    Rotation                    |
|       [ C ] ↺--------(±Z)--------↻ [ V ]       |
+------------------------------------------------+
        """
        elif isinstance(action_cfg, WheeledRoverActionGroupCfg):
            return """
+------------------------------------------------+
|  Keyboard Scheme (focus the Isaac Sim window)  |
+------------------------------------------------+
+------------------------------------------------+
| Decrease Gain [ O ]   | Increase Gain: [ P ]   |
| Reset: [ L ]                                   |
+------------------------------------------------+
| Planar Motion                                  |
|                     [ W ] (+X)                 |
|                       ↑                        |
|                       |                        |
|          (-Y) [ A ] ← + → [ D ] (+Y)           |
|                       |                        |
|                       ↓                        |
|                     [ S ] (-X)                 |
+------------------------------------------------+
        """
        elif isinstance(action_cfg, SpacecraftActionGroupCfg):
            return """
+------------------------------------------------+
|  Keyboard Scheme (focus the Isaac Sim window)  |
+------------------------------------------------+
+------------------------------------------------+
| Decrease Gain [ O ]   | Increase Gain: [ P ]   |
| Reset: [ L ]                                   |
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
        else:
            raise ValueError(f"Invalid action configuration '{action_cfg}'.")
