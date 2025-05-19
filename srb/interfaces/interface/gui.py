try:
    import rclpy
except ImportError:
    from srb.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()

import threading
from queue import Queue
from typing import TYPE_CHECKING

import numpy
import rclpy
from pxr import Gf
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float64

from .base import InterfaceBase

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class GuiInterface(InterfaceBase):
    def __init__(self, env: "AnyEnv", node: Node | None = None, *args, **kwargs):
        self._env = env

        ## Initialize node
        if not node:
            rclpy.init()
            self._node = Node("sim", namespace="srb", start_parameter_services=False)  # type: ignore
        else:
            self._node = node

        ## Execution queue for actions and services that must be executed in the main thread between environment steps via `update()`
        self._exec_queue = Queue()

        ## Subscribers
        self._sub_reset = self._node.create_subscription(
            Empty, "/gui/reset_discard_dataset", self._cb_reset, 1
        )
        self._sub_shutdown_process = self._node.create_subscription(
            Empty, "/gui/shutdown_process", self._cb_shutdown_process, 1
        )
        self._sub_gravity = self._node.create_subscription(
            Float64, "/gui/gravity", self._cb_gravity, 1
        )

        # Run a thread for listening to device
        if not node:
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin)
            self._thread.daemon = True
            self._thread.start()

    def __del__(self):
        if hasattr(self, "_thread"):
            self._thread.join()

    def update(self, *args, **kwargs):
        while not self._exec_queue.empty():
            request, request_kwargs = self._exec_queue.get()
            request(**request_kwargs)

    def reset(self):
        self._env.reset()

    def shutdown(self):
        exit(0)

    def set_gravity(self, gravity: float):
        self._env.unwrapped.sim.cfg.gravity = (0.0, 0.0, -gravity)  # type: ignore

        physics_scene = self._env.unwrapped.sim._physics_context._physics_scene  # type: ignore

        gravity = numpy.asarray(self._env.unwrapped.sim.cfg.gravity)  # type: ignore
        gravity_magnitude = numpy.linalg.norm(gravity)

        # Avoid division by zero
        if gravity_magnitude != 0.0:
            gravity_direction = gravity / gravity_magnitude
        else:
            gravity_direction = gravity

        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(*gravity_direction))
        physics_scene.CreateGravityMagnitudeAttr(gravity_magnitude)

    def _cb_reset(self, msg: Bool):
        self._exec_queue.put((self.reset, {}))

    def _cb_shutdown_process(self, msg: Bool):
        self._exec_queue.put((self.shutdown, {}))

    def _cb_gravity(self, msg: Float64):
        self._exec_queue.put((self.set_gravity, {"gravity": msg.data}))
