try:
    import rclpy  # noqa: F401
except ImportError:
    from srb.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()

from queue import Queue
from typing import TYPE_CHECKING

from pxr import Gf, UsdPhysics
from rclpy.node import Node
from std_msgs.msg import Empty, Float64

from srb.interfaces._ros_node import acquire_ros_node_runtime, destroy_ros_endpoint

from ._gravity import parse_gravity_magnitude
from .base import InterfaceBase

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class GuiInterface(InterfaceBase):
    def __init__(self, env: "AnyEnv", node: Node | None = None, *args, **kwargs):
        self._env = env

        ## Node
        self._ros_runtime = acquire_ros_node_runtime(node, spin=True)
        self._node = self._ros_runtime.node
        self._owned_endpoints = []

        ## Main-thread request queue, drained by `update()`
        self._exec_queue = Queue()

        ## Subscribers
        self._sub_reset = self._node.create_subscription(
            Empty, "/gui/reset_discard_dataset", self._cb_reset, 1
        )
        self._owned_endpoints.append((self._sub_reset, "subscription"))
        self._sub_shutdown_process = self._node.create_subscription(
            Empty, "/gui/shutdown_process", self._cb_shutdown_process, 1
        )
        self._owned_endpoints.append((self._sub_shutdown_process, "subscription"))
        self._sub_gravity = self._node.create_subscription(
            Float64, "/gui/gravity", self._cb_gravity, 1
        )
        self._owned_endpoints.append((self._sub_gravity, "subscription"))

        ## Publishers
        self._pub_reward = self._node.create_publisher(Float64, "/gui/reward", 1)
        self._owned_endpoints.append((self._pub_reward, "publisher"))

    def __del__(self):
        if hasattr(self, "_ros_runtime"):
            self.close()

    def close(self):
        while self._owned_endpoints:
            endpoint, kind = self._owned_endpoints[0]
            destroy_ros_endpoint(self._node, endpoint, kind=kind)
            self._owned_endpoints.pop(0)
        self._ros_runtime.close()

    def update(self, observation=None, reward=None, *args, **kwargs):
        if reward is not None:
            if hasattr(reward, "reshape"):
                reward = reward.reshape(-1)[0]
            self._pub_reward.publish(Float64(data=float(reward)))
        while not self._exec_queue.empty():
            request, request_kwargs = self._exec_queue.get()
            request(**request_kwargs)

    def reset(self):
        self._env.reset()

    def shutdown(self):
        exit(0)

    def set_gravity(self, gravity: float):
        magnitude = parse_gravity_magnitude(gravity)
        if magnitude is None:
            raise ValueError(
                f"gravity must be a finite magnitude >= 0 m/s^2, got {gravity!r}"
            )
        self._env.unwrapped.sim.cfg.gravity = (0.0, 0.0, -magnitude)  # type: ignore[attr-defined]
        physics_scene = UsdPhysics.Scene.Get(
            self._env.unwrapped.sim.stage,  # type: ignore[attr-defined]
            self._env.unwrapped.sim.cfg.physics_prim_path,  # type: ignore[attr-defined]
        )
        # Keep the zero vector when gravity is zero.
        gravity_direction = (0.0, 0.0, -1.0 if magnitude else 0.0)
        physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(*gravity_direction))
        physics_scene.CreateGravityMagnitudeAttr(magnitude)

    def _cb_reset(self, msg: Empty):
        self._exec_queue.put((self.reset, {}))

    def _cb_shutdown_process(self, msg: Empty):
        self._exec_queue.put((self.shutdown, {}))

    def _cb_gravity(self, msg: Float64):
        magnitude = parse_gravity_magnitude(msg.data)
        if magnitude is None:
            self._node.get_logger().warn(
                f"Ignoring /gui/gravity value {msg.data!r}: expected a finite "
                "magnitude >= 0 m/s^2"
            )
            return
        self._exec_queue.put((self.set_gravity, {"gravity": magnitude}))
