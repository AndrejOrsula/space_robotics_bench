import os
import threading
from collections.abc import Callable
from typing import Optional

import numpy as np
from rclpy.node import Node

from space_robotics_bench.core.teleop_devices import DeviceBase
from space_robotics_bench.utils.ros import enable_ros2_bridge


class Se3ROS2(DeviceBase, Node):
    def __init__(
        self,
        node: Optional[object] = None,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 1.0,
    ):
        enable_ros2_bridge()
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float64

        if node is None:
            rclpy.init(args=None)
            self._node = Node("srb_teleop_ros2")
        else:
            self._node = node

        # Store inputs
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        self.sub_cmd_bel = self._node.create_subscription(
            Twist, "cmd_vel", self.cb_twist, 1
        )
        self.sub_gripper = self._node.create_subscription(
            Bool, "gripper", self.cb_event, 1
        )
        self.sub_latency = self._node.create_subscription(
            Float64, "gui/latency", self.cb_latency, 1
        )

        self.latency = 0.0
        self.command_queue = []
        self.last_command = None

        # Command buffers
        self._close_gripper = False
        self._delta_pos = np.zeros(3)  # (x, y, z)
        self._delta_rot = np.zeros(3)  # (roll, pitch, yaw)

        # Run a thread for listening to device
        if node is None:
            self._thread = threading.Thread(target=rclpy.spin, args=(self._node,))
            self._thread.daemon = True
            self._thread.start()

    def cb_twist(self, msg):
        self._delta_pos[0] = self.pos_sensitivity * msg.linear.x
        self._delta_pos[1] = self.pos_sensitivity * msg.linear.y
        self._delta_pos[2] = self.pos_sensitivity * msg.linear.z

        self._delta_rot[0] = self.rot_sensitivity * msg.angular.x
        self._delta_rot[1] = self.rot_sensitivity * msg.angular.y
        self._delta_rot[2] = self.rot_sensitivity * msg.angular.z

    def cb_event(self, msg):
        if msg.data:
            self._close_gripper = not self._close_gripper

    def cb_latency(self, msg):
        if msg.data != self.latency:
            self.latency = msg.data
            self.command_queue = []
            self.feedback_queue = []
            self.last_command = None
            self.last_feedback = None

    def __del__(self):
        if hasattr(self, "_thread"):
            self._thread.join()

    def __str__(self) -> str:
        msg = f"ROS 2 Interface ({self.__class__.__name__})\n"
        msg += f"Listenining on ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 0)}\n"
        return msg

    def reset(self):
        self._close_gripper = False
        self._delta_pos = np.zeros(3)
        self._delta_rot = np.zeros(3)
        self.command_queue = []
        self.feedback_queue = []
        self.last_command = None
        self.last_feedback = None

    def add_callback(self, key: str, func: Callable):
        raise NotImplementedError

    def advance(self) -> tuple[np.ndarray, bool]:
        commands = (
            np.concatenate([self._delta_pos, self._delta_rot]),
            self._close_gripper,
        )

        if self.latency == 0.0:
            return commands
        else:
            system_time = self._node.get_clock().now()
            self.command_queue.append((system_time, commands))

            # Find the last viable command
            last_viable_command = None
            for i, (t, _) in enumerate(self.command_queue):
                if (system_time - t).nanoseconds / 1e9 > self.latency:
                    last_viable_command = i
                else:
                    break

            if last_viable_command is not None:
                _, self.last_command = self.command_queue[last_viable_command]
                self.command_queue = self.command_queue[last_viable_command + 1 :]
                return self.last_command
            else:
                if self.last_command is not None:
                    return self.last_command
                else:
                    return np.zeros(6), commands[1]
