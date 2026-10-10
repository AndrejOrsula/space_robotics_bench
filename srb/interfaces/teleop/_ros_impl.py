"""Base-free shared implementation for native and simulator teleop."""

import os
import threading
import time
from collections.abc import Callable
from math import isfinite

import numpy as np
from geometry_msgs.msg import Twist

try:
    from rclpy.node import Node
except ImportError as exc:
    raise ImportError("native ROS teleop requires the ROS 2 Python packages") from exc
from std_msgs.msg import Bool, Float64

from srb.interfaces._ros_node import (
    RosCleanupError,
    RosCleanupScope,
    acquire_ros_node_runtime,
    attempt_ros_cleanup,
    destroy_ros_endpoint,
)

_ROS_ENDPOINTS = ("sub_cmd_bel", "sub_gripper", "sub_latency")


class ROSTeleopInterfaceImplementation:
    def __init__(
        self,
        node: Node | None = None,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 3.1415927,
        command_timeout_s: float = 0.25,
        clock: Callable[[], float] | None = None,
    ):
        if (
            isinstance(command_timeout_s, bool)
            or not isfinite(command_timeout_s)
            or command_timeout_s <= 0.0
        ):
            raise ValueError("command_timeout_s must be finite and positive")
        self._closed = False
        self._lock = threading.RLock()
        self._cleanup_lock = threading.RLock()
        self._cleanup_in_progress = False
        self._cleanup_complete = False
        self._closed_endpoints: set[str] = set()
        self._runtime_closed = False
        self._clock = time.monotonic if clock is None else clock
        self.command_timeout_s = float(command_timeout_s)
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # Initialize callback state before a subscription can deliver a message.
        self.latency = 0.0
        self.command_queue = []
        self.last_command = None
        self._last_applied_gripper = False
        self._last_twist_s: float | None = None

        # Command buffers
        self._close_gripper = False
        self._delta_pos = np.zeros(3)  # (x, y, z)
        self._delta_rot = np.zeros(3)  # (roll, pitch, yaw)

        self._ros_runtime = None
        self._node = None
        for name in _ROS_ENDPOINTS:
            setattr(self, name, None)

        with RosCleanupScope(self, "ROS teleop construction cleanup incomplete"):
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
                    self._node = getattr(owner, "node", None)
                raise
            self._node = self._ros_runtime.node
            self.sub_cmd_bel = self._node.create_subscription(
                Twist, "/cmd_vel", self.cb_twist, 1
            )
            self.sub_gripper = self._node.create_subscription(
                Bool, "/gripper", self.cb_event, 1
            )
            self.sub_latency = self._node.create_subscription(
                Float64, "/gui/latency", self.cb_latency, 1
            )

    def _clear_pending_motion(self) -> None:
        for name in ("_delta_pos", "_delta_rot"):
            delta = getattr(self, name, None)
            if delta is not None:
                delta.fill(0.0)
        self._last_twist_s = None
        queue = getattr(self, "command_queue", None)
        if queue is not None:
            queue.clear()
        self.last_command = None

    def cb_twist(self, msg):
        with self._lock:
            if self._closed:
                return
            values = np.asarray(
                (
                    self.pos_sensitivity * msg.linear.x,
                    self.pos_sensitivity * msg.linear.y,
                    self.pos_sensitivity * msg.linear.z,
                    self.rot_sensitivity * msg.angular.x,
                    self.rot_sensitivity * msg.angular.y,
                    self.rot_sensitivity * msg.angular.z,
                ),
                dtype=np.float64,
            )
            if not np.all(np.isfinite(values)):
                # Clear delayed motion too; invalid input must not replay stale commands.
                self._clear_pending_motion()
                return
            self._delta_pos[:] = values[:3]
            self._delta_rot[:] = values[3:]
            self._last_twist_s = self._clock()

    def cb_event(self, msg):
        with self._lock:
            if not self._closed and msg.data:
                self._close_gripper = not self._close_gripper

    def cb_latency(self, msg):
        with self._lock:
            if self._closed:
                return
            latency = float(msg.data)
            if not isfinite(latency) or latency < 0.0:
                return
            if latency != self.latency:
                self.latency = latency
                self.command_queue = []
                self.last_command = None

    def __del__(self):
        attempt_ros_cleanup(self.close)

    def _mark_shared_node_destroyed(self, node: object) -> bool:
        """Mark endpoints closed after the owning runtime destroys the node."""
        if node is None or getattr(self, "_node", None) is not node:
            return False
        if not self._cleanup_lock.acquire(blocking=False):
            raise RuntimeError("ROS teleop cleanup already in progress")
        try:
            with self._lock:
                self._closed = True
                self._clear_pending_motion()
                self._closed_endpoints = set(_ROS_ENDPOINTS)
                self._runtime_closed = True
                self._cleanup_complete = True
        finally:
            self._cleanup_lock.release()
        return True

    def close(self) -> None:
        lock = getattr(self, "_lock", None)
        if lock is None:
            lock = self._lock = threading.RLock()
        cleanup_lock = getattr(self, "_cleanup_lock", None)
        if cleanup_lock is None:
            cleanup_lock = self._cleanup_lock = threading.RLock()
        if not cleanup_lock.acquire(blocking=False):
            raise RuntimeError("ROS teleop cleanup already in progress")
        if getattr(self, "_cleanup_in_progress", False):
            cleanup_lock.release()
            raise RuntimeError("ROS teleop cleanup already in progress")
        self._cleanup_in_progress = True
        try:
            with lock:
                if getattr(self, "_cleanup_complete", False):
                    return
                self._closed = True
                self._clear_pending_motion()
            closed_endpoints = getattr(self, "_closed_endpoints", None)
            if closed_endpoints is None:
                closed_endpoints = self._closed_endpoints = set()
            errors: list[Exception] = []
            node = getattr(self, "_node", None)
            for name in _ROS_ENDPOINTS:
                if name in closed_endpoints:
                    continue
                try:
                    destroy_ros_endpoint(
                        node, getattr(self, name, None), kind="subscription"
                    )
                except (
                    AttributeError,
                    ExceptionGroup,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(exc)
                else:
                    closed_endpoints.add(name)
            if not getattr(self, "_runtime_closed", False):
                runtime = getattr(self, "_ros_runtime", None)
                try:
                    if runtime is not None:
                        runtime.close()
                except (
                    AttributeError,
                    ExceptionGroup,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(exc)
                else:
                    self._runtime_closed = True
                    node_handle = getattr(runtime, "node_handle", None)
                    if getattr(runtime, "owns", False) and getattr(
                        node_handle, "released", False
                    ):
                        self._mark_shared_node_destroyed(node)
            if errors:
                if len(errors) == 1:
                    raise errors[0]
                raise ExceptionGroup("ROS teleop cleanup incomplete", errors)
            self._cleanup_complete = True
        finally:
            self._cleanup_in_progress = False
            cleanup_lock.release()

    def __str__(self) -> str:
        msg = f"ROS Interface ({self.__class__.__name__})\n"
        msg += f"Listening on ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}\n"
        return msg

    def reset(self):
        with self._lock:
            if self._closed:
                return
            self._close_gripper = False
            self._last_applied_gripper = False
            self._clear_pending_motion()

    def add_callback(self, key: str, func: Callable):
        raise NotImplementedError

    def advance(self) -> tuple[np.ndarray, bool]:
        with self._lock:
            if self._closed:
                return np.zeros(6, dtype=np.float64), False
            # Do not replay buffered input after the owned executor fails.
            raise_if_failed = getattr(
                getattr(self, "_ros_runtime", None), "raise_if_failed", None
            )
            if callable(raise_if_failed):
                raise_if_failed()
            now = self._clock()
            fresh = (
                self._last_twist_s is not None
                and 0.0 <= now - self._last_twist_s <= self.command_timeout_s
            )
            motion = (
                np.concatenate([self._delta_pos, self._delta_rot]).copy()
                if fresh
                else np.zeros(6, dtype=np.float64)
            )
            commands = (motion, self._close_gripper)
            if self.latency == 0.0:
                self._last_applied_gripper = self._close_gripper
                return commands

            self.command_queue.append((now, self._last_twist_s, commands))
            selected = None
            selected_index = -1
            for i, (queued_at, source_at, queued) in enumerate(self.command_queue):
                if now - queued_at < self.latency:
                    break
                selected_index = i
                # Judge freshness when queued; latency itself is not staleness.
                motion_fresh = (
                    source_at is not None
                    and 0.0 <= queued_at - source_at <= self.command_timeout_s
                )
                # Compare against the latest selection so a later reversal wins.
                gripper = (
                    self._last_applied_gripper if selected is None else selected[1]
                )
                if motion_fresh or queued[1] != gripper:
                    selected = (
                        queued[0] if motion_fresh else np.zeros(6, dtype=np.float64),
                        queued[1],
                    )
            if selected_index >= 0:
                self.command_queue = self.command_queue[selected_index + 1 :]
            self.last_command = selected
            if selected is None:
                return np.zeros(6, dtype=np.float64), self._last_applied_gripper
            self._last_applied_gripper = selected[1]
            return selected
