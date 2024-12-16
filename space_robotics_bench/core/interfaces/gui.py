import threading
from queue import Queue
from typing import Optional, Union

import numpy as np
import rclpy
from pxr import Gf
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float64

from space_robotics_bench.core.envs import BaseEnv
from space_robotics_bench.envs import (
    BaseAerialRoboticsEnv,
    BaseManipulationEnv,
    BaseMobileRoboticsEnv,
)


class GuiInterface:
    def __init__(
        self,
        env: Union[
            BaseEnv,
            BaseAerialRoboticsEnv,
            BaseManipulationEnv,
            BaseMobileRoboticsEnv,
        ],
        node: Optional[object] = None,
    ):
        self._env = env

        ## Initialize node
        if node is None:
            rclpy.init(args=None)
            self._node = Node("srb")
        else:
            self._node = node

        ## Execution queue for actions and services that must be executed in the main thread between environment steps via `update()`
        self._exec_queue = Queue()

        ## Subscribers
        self._sub_reset = self._node.create_subscription(
            Empty, "gui/reset_discard_dataset", self._cb_reset, 1
        )
        self._sub_shutdown_process = self._node.create_subscription(
            Empty, "gui/shutdown_process", self._cb_shutdown_process, 1
        )
        self._sub_gravity = self._node.create_subscription(
            Float64, "gui/gravity", self._cb_gravity, 1
        )

        # Run a thread for listening to device
        if node is None:
            self._thread = threading.Thread(target=rclpy.spin, args=(self._node,))
            self._thread.daemon = True
            self._thread.start()

    def __del__(self):
        if hasattr(self, "_thread"):
            self._thread.join()

    def reset(self):
        self._env.reset()

    def shutdown(self):
        exit(0)

    def set_gravity(self, gravity: float):
        self._env.unwrapped.sim.cfg.gravity = (0.0, 0.0, -gravity)

        physics_scene = self._env.unwrapped.sim._physics_context._physics_scene

        gravity = np.asarray(self._env.unwrapped.sim.cfg.gravity)
        gravity_magnitude = np.linalg.norm(gravity)

        # Avoid division by zero
        if gravity_magnitude != 0.0:
            gravity_direction = gravity / gravity_magnitude
        else:
            gravity_direction = gravity

        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(*gravity_direction))
        physics_scene.CreateGravityMagnitudeAttr(gravity_magnitude)

    def update(self):
        while not self._exec_queue.empty():
            request, kwargs = self._exec_queue.get()
            request(**kwargs)

    def _cb_reset(self, msg: Bool):
        self._exec_queue.put((self.reset, {}))

    def _cb_shutdown_process(self, msg: Bool):
        self._exec_queue.put((self.shutdown, {}))

    def _cb_gravity(self, msg: Float64):
        self._exec_queue.put((self.set_gravity, {"gravity": msg.data}))
