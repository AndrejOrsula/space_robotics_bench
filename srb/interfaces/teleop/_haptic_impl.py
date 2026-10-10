"""Base-free shared implementation for native and simulator teleop."""

import os
import threading
import time
from collections.abc import Callable
from math import isfinite

import numpy as np
import torch
from geometry_msgs.msg import Twist, Vector3

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

_HAPTIC_ENDPOINTS = (
    ("sub_twist", "subscription"),
    ("sub_button", "subscription"),
    ("sub_event", "subscription"),
    ("sub_latency", "subscription"),
    ("sub_motion_sensitivity", "subscription"),
    ("sub_force_feedback_sensitivity", "subscription"),
    ("pub_force_feedback", "publisher"),
)


class HapticROSTeleopInterfaceImplementation:
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
        self._cleanup_complete = False
        self._closed_endpoints: set[str] = set()
        self._runtime_closed = False
        self._lock = threading.RLock()
        self._cleanup_lock = threading.RLock()
        self._cleanup_in_progress = False
        self._clock = time.monotonic if clock is None else clock
        self.command_timeout_s = float(command_timeout_s)
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # Initialize callback state before a provider can deliver a message.
        self.is_motion_enabled = False
        self.gain = 1.0
        self.inactive_feedback_decay = 1.0
        self.latency = 0.0
        self.command_queue = []
        self.feedback_queue = []
        self.last_command = None
        self.last_feedback = None
        self._last_twist_s: float | None = None
        self._last_clock_s: float | None = None
        self.force_feedback_sensitivity = 1.0
        self._close_gripper = False
        self._last_applied_gripper = False
        self._delta_pos = np.zeros(3)  # (x, y, z)
        self._delta_rot = np.zeros(3)  # (roll, pitch, yaw)

        self._ros_runtime = None
        self._node = None
        for name, _kind in _HAPTIC_ENDPOINTS:
            setattr(self, name, None)

        with RosCleanupScope(self, "Haptic ROS teleop construction cleanup incomplete"):
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
            self.sub_twist = self._node.create_subscription(
                Twist, "/touch/twist_stylus_current", self.cb_twist, 1
            )
            self.sub_button = self._node.create_subscription(
                Bool, "/touch/button", self.cb_button, 1
            )
            self.sub_event = self._node.create_subscription(
                Bool, "/touch/event", self.cb_event, 1
            )
            self.pub_force_feedback = self._node.create_publisher(
                Vector3, "/touch/force_feedback", 1
            )

            self.sub_latency = self._node.create_subscription(
                Float64, "/gui/latency", self.cb_latency, 1
            )
            self.sub_motion_sensitivity = self._node.create_subscription(
                Float64, "/gui/motion_sensitivity", self.cb_motion_sensitivity, 1
            )
            self.sub_force_feedback_sensitivity = self._node.create_subscription(
                Float64,
                "/gui/force_feedback_sensitivity",
                self.cb_force_feedback_sensitivity,
                1,
            )

    def _clear_pending_motion(self) -> None:
        """Clear buffered motion while preserving gripper state."""
        delta_pos = getattr(self, "_delta_pos", None)
        if delta_pos is not None:
            delta_pos.fill(0.0)
        delta_rot = getattr(self, "_delta_rot", None)
        if delta_rot is not None:
            delta_rot.fill(0.0)
        self._last_twist_s = None
        command_queue = getattr(self, "command_queue", None)
        if command_queue is not None:
            # Keep delayed gripper toggles; drop only their motion.
            command_queue[:] = [
                (queued_at, None, (np.zeros(6, dtype=np.float64), gripper))
                for queued_at, _source_at, (_motion, gripper) in command_queue
            ]
        self.last_command = None

    def _now(self) -> float:
        """Read the clock and clear motion if it moves backward."""
        now = float(self._clock())
        if not isfinite(now):
            self._clear_pending_motion()
            self._last_clock_s = None
            return now
        if self._last_clock_s is not None and now < self._last_clock_s:
            self._clear_pending_motion()
        self._last_clock_s = now
        return now

    def cb_twist(self, msg):
        with self._lock:
            if self._closed:
                return
            if not self.is_motion_enabled:
                self._clear_pending_motion()
                return
            sample_at = self._now()
            if not isfinite(sample_at):
                return
            values = np.asarray(
                (
                    -self.pos_sensitivity * self.gain * msg.linear.z,
                    -self.pos_sensitivity * self.gain * msg.linear.x,
                    self.pos_sensitivity * self.gain * msg.linear.y,
                    self.rot_sensitivity * self.gain * msg.angular.z,
                    self.rot_sensitivity * self.gain * msg.angular.x,
                    self.rot_sensitivity * self.gain * msg.angular.y,
                ),
                dtype=np.float64,
            )
            if not np.all(np.isfinite(values)):
                self._clear_pending_motion()
                return
            self._delta_pos[:] = values[:3]
            self._delta_rot[:] = values[3:]
            self._last_twist_s = sample_at

    def cb_button(self, msg):
        with self._lock:
            if self._closed:
                return
            enabled = bool(msg.data)
            was_enabled = self.is_motion_enabled
            self.is_motion_enabled = enabled
            if not enabled or not was_enabled:
                self._clear_pending_motion()

    def cb_event(self, msg):
        with self._lock:
            if not self._closed and msg.data:
                self._close_gripper = not self._close_gripper

    def cb_latency(self, msg):
        latency = float(msg.data)
        if not isfinite(latency) or latency < 0.0:
            return
        with self._lock:
            if self._closed:
                return
            if latency != self.latency:
                self.latency = latency
                self.command_queue = []
                self.feedback_queue = []
                self.last_command = None
                self.last_feedback = None

    def cb_motion_sensitivity(self, msg):
        gain = float(msg.data)
        if isfinite(gain) and gain >= 0.0:
            with self._lock:
                if self._closed:
                    return
                self.gain = gain

    def cb_force_feedback_sensitivity(self, msg):
        sensitivity = float(msg.data)
        if isfinite(sensitivity) and sensitivity >= 0.0:
            with self._lock:
                if self._closed:
                    return
                self.force_feedback_sensitivity = sensitivity

    def __del__(self):
        attempt_ros_cleanup(self.close)

    def _mark_shared_node_destroyed(self, node: object) -> bool:
        """Mark endpoints closed after their owning composite destroys ``node``.

        The owner's node release removes every endpoint, including ones whose
        direct destruction previously failed.
        """
        if node is None or getattr(self, "_node", None) is not node:
            return False
        cleanup_lock = getattr(self, "_cleanup_lock", None)
        if cleanup_lock is None:
            cleanup_lock = threading.RLock()
            self._cleanup_lock = cleanup_lock
        lock = getattr(self, "_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._lock = lock
        with cleanup_lock:
            with lock:
                self._closed = True
                self._clear_pending_motion()
                if hasattr(self, "feedback_queue"):
                    self.feedback_queue.clear()
                self.last_feedback = None
                self.is_motion_enabled = False
                self._closed_endpoints = {name for name, _kind in _HAPTIC_ENDPOINTS}
                self._runtime_closed = True
                self._cleanup_complete = True
        return True

    def close(self) -> None:
        lock = getattr(self, "_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._lock = lock
        cleanup_lock = getattr(self, "_cleanup_lock", None)
        if cleanup_lock is None:
            cleanup_lock = threading.RLock()
            self._cleanup_lock = cleanup_lock

        if not cleanup_lock.acquire(blocking=False):
            raise RuntimeError("haptic ROS cleanup already in progress")
        if getattr(self, "_cleanup_in_progress", False):
            cleanup_lock.release()
            raise RuntimeError("haptic ROS cleanup already in progress")
        self._cleanup_in_progress = True
        try:
            with lock:
                if getattr(self, "_cleanup_complete", False):
                    return
                # Disable motion first so failed cleanup cannot leave a command.
                self._closed = True
                self._clear_pending_motion()
                if hasattr(self, "feedback_queue"):
                    self.feedback_queue.clear()
                self.last_feedback = None
                self.is_motion_enabled = False

            closed_endpoints = getattr(self, "_closed_endpoints", None)
            if closed_endpoints is None:
                closed_endpoints = set()
                self._closed_endpoints = closed_endpoints
            errors: list[Exception] = []
            node = getattr(self, "_node", None)

            for name, kind in _HAPTIC_ENDPOINTS:
                if name in closed_endpoints:
                    continue
                try:
                    endpoint = getattr(self, name, None)
                    destroy_ros_endpoint(node, endpoint, kind=kind)
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

            runtime = getattr(self, "_ros_runtime", None)
            if not getattr(self, "_runtime_closed", False):
                if runtime is None:
                    self._runtime_closed = True
                else:
                    try:
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
                            self._mark_shared_node_destroyed(
                                getattr(runtime, "node", None)
                            )

            if errors:
                if len(errors) == 1:
                    raise errors[0]
                raise ExceptionGroup("haptic ROS cleanup incomplete", errors)
            self._cleanup_complete = True
        finally:
            self._cleanup_in_progress = False
            cleanup_lock.release()

    def __str__(self) -> str:
        msg = f"Haptic ROS Interface  ({self.__class__.__name__})\n"
        msg += f"Listening on ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}\n"
        msg += "Hold to enable motion: Dark grey button\n"
        msg += "Press to toggle the gripper: Light grey button\n"
        return msg

    def reset(self):
        with self._lock:
            if self._closed:
                return
            self._close_gripper = False
            self._last_applied_gripper = False
            self._clear_pending_motion()
            self.command_queue = []
            self.feedback_queue = []
            self.last_feedback = None

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
            now = self._now()
            if not isfinite(now):
                return np.zeros(6, dtype=np.float64), self._close_gripper
            fresh = (
                self.is_motion_enabled
                and self._last_twist_s is not None
                and isfinite(self._last_twist_s)
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
                motion_fresh = (
                    self.is_motion_enabled
                    and source_at is not None
                    and isfinite(source_at)
                    # Judge freshness when queued; latency itself is not staleness.
                    and 0.0 <= queued_at - source_at <= self.command_timeout_s
                )
                # Delay gripper toggles too; a later reversal wins.
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

    def set_ft_feedback(self, ft_feedback: np.ndarray | torch.Tensor):
        from geometry_msgs.msg import Vector3

        values = np.asarray(
            [float(ft_feedback[index]) for index in range(3)], dtype=np.float64
        )
        if not np.all(np.isfinite(values)):
            values.fill(0.0)
        with self._lock:
            if self._closed:
                return
            now = self._now()
            if self.latency != 0.0:
                self.feedback_queue.append((now, values.copy()))
                selected = None
                selected_index = -1
                for i, (queued_at, queued) in enumerate(self.feedback_queue):
                    if now - queued_at < self.latency:
                        break
                    selected_index = i
                    selected = queued
                if selected_index >= 0:
                    self.feedback_queue = self.feedback_queue[selected_index + 1 :]
                if selected is None:
                    return
                self.last_feedback = selected
                values = selected
            # Decay feedback while motion is disabled, delayed or not.
            if self.is_motion_enabled:
                output = values * self.force_feedback_sensitivity
                self.inactive_feedback_decay = 1.0
            elif self.inactive_feedback_decay > 0.0:
                self.inactive_feedback_decay = max(
                    0.0, 0.1 * self.inactive_feedback_decay
                )
                output = (
                    values
                    * self.inactive_feedback_decay
                    * self.force_feedback_sensitivity
                )
            else:
                return

            # Publish under the lock so close cannot destroy the publisher.
            self.pub_force_feedback.publish(
                Vector3(x=float(output[0]), y=float(output[1]), z=float(output[2]))
            )
