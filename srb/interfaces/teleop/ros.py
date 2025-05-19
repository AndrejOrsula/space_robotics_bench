try:
    import rclpy
except ImportError:
    from srb.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()

import os
import threading
from collections.abc import Callable

import numpy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from srb.interfaces.teleop import DeviceBase


class ROSTeleopInterface(DeviceBase, Node):
    def __init__(
        self,
        node: Node | None = None,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 3.1415927,
    ):
        if not node:
            rclpy.init()
            self._node = Node("sim", namespace="srb", start_parameter_services=False)  # type: ignore
        else:
            self._node = node

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        self.sub_cmd_bel = self._node.create_subscription(
            Twist, "/cmd_vel", self.cb_twist, 1
        )
        self.sub_gripper = self._node.create_subscription(
            Bool, "/gripper", self.cb_event, 1
        )
        self.sub_latency = self._node.create_subscription(
            Float64, "/gui/latency", self.cb_latency, 1
        )

        self.latency = 0.0
        self.command_queue = []
        self.last_command = None

        # Command buffers
        self._close_gripper = False
        self._delta_pos = numpy.zeros(3)  # (x, y, z)
        self._delta_rot = numpy.zeros(3)  # (roll, pitch, yaw)

        # Run a thread for listening to device
        if not node:
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin)
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
        msg = f"ROS Interface ({self.__class__.__name__})\n"
        msg += f"Listenining on ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 0)}\n"
        return msg

    def reset(self):
        self._close_gripper = False
        self._delta_pos = numpy.zeros(3)
        self._delta_rot = numpy.zeros(3)
        self.command_queue = []
        self.feedback_queue = []
        self.last_command = None
        self.last_feedback = None

    def add_callback(self, key: str, func: Callable):
        raise NotImplementedError

    def advance(self) -> tuple[numpy.ndarray, bool]:
        commands = (
            numpy.concatenate([self._delta_pos, self._delta_rot]),
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
                    return numpy.zeros(6), commands[1]
