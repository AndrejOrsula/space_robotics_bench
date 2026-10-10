from __future__ import annotations

import math
import time
from collections.abc import Callable

import gymnasium
import numpy as np
from pydantic import Field, model_validator

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)


class RosFormationFollowingObservationInterfaceCfg(HardwareInterfaceCfg):
    robot_names: tuple[str, ...] = ("robot", "robot_1", "robot_2")
    target_frame: str = "formation_center"
    formation_center_frame: str = "formation_center"
    source_frame_for_velocity: str = "world"
    timeout_duration: float = 0.2
    max_robots: int = 3
    # Per-robot slot offsets (metres), parallel to ``robot_names``. The policy
    # tracks each assigned slot (centre + offset), matching the sim task's
    # ``formation_offsets * formation_scale``. Zero offset preserves legacy
    # robot-to-centre behavior.
    slot_offsets: tuple[tuple[float, float], ...] = (
        (1.5, 0.0),
        (-0.75, 1.3),
        (-0.75, -1.3),
    )
    max_velocity_dt_s: float = Field(default=0.5, gt=0.0, allow_inf_nan=False)
    """Largest TF stamp gap differenced into a velocity.  A larger gap or a
    backward stamp reseeds the anchor and reports zero.  A repeated stamp holds
    the last velocity for at most this long on the local monotonic clock; a
    TF stream frozen for longer reports zero."""

    @model_validator(mode="after")
    def _validate_robot_names(self) -> RosFormationFollowingObservationInterfaceCfg:
        if self.max_robots < 1:
            raise ValueError("max_robots must be positive")
        if not self.robot_names or len(self.robot_names) > self.max_robots:
            raise ValueError("robot_names must contain 1..max_robots names")
        if any(not name.strip() for name in self.robot_names):
            raise ValueError("robot_names must not contain empty names")
        if len(set(self.robot_names)) != len(self.robot_names):
            raise ValueError("robot_names must be unique")
        return self


class _StampedVelocity:
    """Planar finite difference over TF sample stamps for one frame.

    The anchor is the last *new* sample.  A duplicate stamp (sync faster than
    the TF source) holds the last velocity without moving the anchor, so the
    next new sample is differenced over its true stamp interval rather than
    aliasing to zero and then twice the speed.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._anchor_pos: np.ndarray | None = None
        self._anchor_stamp_s: float | None = None
        self._anchor_local_s: float | None = None
        self._velocity = np.zeros(2, dtype=np.float32)

    def _seed(self, pos: np.ndarray, stamp_s: float, now_s: float) -> np.ndarray:
        self._anchor_pos = pos
        self._anchor_stamp_s = stamp_s
        self._anchor_local_s = now_s
        self._velocity = np.zeros(2, dtype=np.float32)
        return self._velocity.copy()

    def update(
        self,
        pos: np.ndarray,
        stamp_s: float | None,
        now_s: float,
        max_dt_s: float,
    ) -> np.ndarray:
        if stamp_s is None or not math.isfinite(stamp_s) or stamp_s <= 0.0:
            # No usable sample time: no derivative, and no anchor to resume from.
            self.reset()
            return self._velocity.copy()
        if self._anchor_pos is None or self._anchor_stamp_s is None:
            return self._seed(pos, stamp_s, now_s)
        dt = stamp_s - self._anchor_stamp_s
        if dt == 0.0:
            if (
                self._anchor_local_s is not None
                and now_s - self._anchor_local_s > max_dt_s
            ):
                # The source stopped publishing; do not hold motion forever.
                self._velocity = np.zeros(2, dtype=np.float32)
            return self._velocity.copy()
        if dt < 0.0 or dt > max_dt_s:
            return self._seed(pos, stamp_s, now_s)
        self._velocity = ((pos - self._anchor_pos) / dt).astype(np.float32)
        self._anchor_pos = pos
        self._anchor_stamp_s = stamp_s
        self._anchor_local_s = now_s
        return self._velocity.copy()


class RosFormationFollowingObservationInterface(HardwareInterface):
    """Emit formation-following state observations for sim-to-real rollout."""

    cfg: RosFormationFollowingObservationInterfaceCfg

    _POS_KEY = "state/tf_pos2d_robots_to_targets"
    _ROT_KEY = "state/tf_rot2dtrigyaw_robots_to_targets"
    _VEL_KEY = "state/formation_center_vel_2d"
    _ROBOT_VEL_KEY = "state_dyn/robot_vel_xy"

    def __init__(
        self,
        cfg: RosFormationFollowingObservationInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ):
        if cfg is None:
            cfg = RosFormationFollowingObservationInterfaceCfg()
        super().__init__(cfg)
        self._clock = clock
        self.robot_names: tuple[str, ...] = tuple(cfg.robot_names)[: cfg.max_robots]
        self._slot_offsets: tuple[tuple[float, float], ...] = tuple(cfg.slot_offsets)

        self._obs = {
            self._POS_KEY: np.zeros(cfg.max_robots * 2, dtype=np.float32),
            self._ROT_KEY: np.zeros(cfg.max_robots * 2, dtype=np.float32),
            self._VEL_KEY: np.zeros(2, dtype=np.float32),
            self._ROBOT_VEL_KEY: np.zeros(cfg.max_robots * 2, dtype=np.float32),
        }
        self._center_velocity = _StampedVelocity()
        self._robot_velocity = {name: _StampedVelocity() for name in self.robot_names}

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformException, TransformListener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.ros_node)
        self._transform_exception = TransformException
        self.tf_timeout_duration = Duration(
            seconds=0
            if self.cfg.timeout_duration < 1
            else int(self.cfg.timeout_duration),
            nanoseconds=int((self.cfg.timeout_duration % 1) * 1_000_000_000),
        )

    def close(self):
        super().close()
        if hasattr(self, "tf_listener"):
            self.tf_listener.unregister()
        if hasattr(self, "tf_buffer"):
            self.tf_buffer.clear()

    def reset(self):
        super().reset()
        self._obs[self._POS_KEY] = np.zeros_like(self._obs[self._POS_KEY])
        self._obs[self._ROT_KEY] = np.zeros_like(self._obs[self._ROT_KEY])
        self._obs[self._VEL_KEY] = np.zeros_like(self._obs[self._VEL_KEY])
        self._obs[self._ROBOT_VEL_KEY] = np.zeros_like(self._obs[self._ROBOT_VEL_KEY])
        self._center_velocity.reset()
        for velocity in self._robot_velocity.values():
            velocity.reset()

    def sync(self):
        super().sync()
        now_s = self._clock()
        self._update_robot_offsets()
        self._update_center_velocity(now_s)
        self._update_robot_velocities(now_s)

    @staticmethod
    def _stamp_s(tf_stamped: object) -> float | None:
        stamp = getattr(getattr(tf_stamped, "header", None), "stamp", None)
        try:
            return (
                float(getattr(stamp, "sec")) + float(getattr(stamp, "nanosec")) * 1e-9
            )
        except (AttributeError, TypeError, ValueError):
            return None

    def _update_robot_offsets(self) -> None:
        pos_entries: list[float] = []
        rot_entries: list[float] = []
        from rclpy.time import Time

        for idx, robot_name in enumerate(self.robot_names):
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    robot_name,
                    self.cfg.target_frame,
                    Time(),
                    self.tf_timeout_duration,
                )
                # Pose of the formation centre expressed in this robot's frame.
                center_x = float(tf_stamped.transform.translation.x)
                center_y = float(tf_stamped.transform.translation.y)
                yaw = self._quat_to_yaw(tf_stamped.transform.rotation)
                # The policy tracks each robot -> its assigned SLOT, not the bare
                # centre: rotate the slot offset (defined in the centre frame) by
                # the centre yaw and add it to the centre position.
                offset_x, offset_y = (
                    self._slot_offsets[idx]
                    if idx < len(self._slot_offsets)
                    else (0.0, 0.0)
                )
                slot_x, slot_y = self._slot_position_in_robot_frame(
                    center_x, center_y, yaw, offset_x, offset_y
                )
                pos_entries.extend((slot_x, slot_y))
                # Slot orientation matches the formation-centre orientation.
                rot_entries.extend((float(math.sin(yaw)), float(math.cos(yaw))))
            except self._transform_exception:
                pos_entries.extend((0.0, 0.0))
                rot_entries.extend((0.0, 1.0))

        while len(pos_entries) < self.cfg.max_robots * 2:
            pos_entries.extend((0.0, 0.0))
            rot_entries.extend((0.0, 1.0))

        self._obs[self._POS_KEY] = np.array(
            pos_entries[: self.cfg.max_robots * 2], dtype=np.float32
        )
        self._obs[self._ROT_KEY] = np.array(
            rot_entries[: self.cfg.max_robots * 2], dtype=np.float32
        )

    def _update_center_velocity(self, now_s: float) -> None:
        from rclpy.time import Time

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.cfg.source_frame_for_velocity,
                self.cfg.formation_center_frame,
                Time(),
                self.tf_timeout_duration,
            )
            center_pos = np.array(
                (
                    float(tf_stamped.transform.translation.x),
                    float(tf_stamped.transform.translation.y),
                ),
                dtype=np.float32,
            )
            self._obs[self._VEL_KEY] = self._center_velocity.update(
                center_pos,
                self._stamp_s(tf_stamped),
                now_s,
                self.cfg.max_velocity_dt_s,
            )
        except self._transform_exception:
            # Reseed after a lookup failure: the next sample only anchors.
            self._center_velocity.reset()
            self._obs[self._VEL_KEY] = np.zeros(2, dtype=np.float32)

    def _update_robot_velocities(self, now_s: float) -> None:
        from rclpy.time import Time

        vel_entries: list[float] = []
        for robot_name in self.robot_names:
            velocity = self._robot_velocity[robot_name]
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.cfg.source_frame_for_velocity,
                    robot_name,
                    Time(),
                    self.tf_timeout_duration,
                )
                robot_pos = np.array(
                    (
                        float(tf_stamped.transform.translation.x),
                        float(tf_stamped.transform.translation.y),
                    ),
                    dtype=np.float32,
                )
                robot_vel = velocity.update(
                    robot_pos,
                    self._stamp_s(tf_stamped),
                    now_s,
                    self.cfg.max_velocity_dt_s,
                )
                vel_entries.extend((float(robot_vel[0]), float(robot_vel[1])))
            except self._transform_exception:
                velocity.reset()
                vel_entries.extend((0.0, 0.0))

        while len(vel_entries) < self.cfg.max_robots * 2:
            vel_entries.extend((0.0, 0.0))

        self._obs[self._ROBOT_VEL_KEY] = np.array(
            vel_entries[: self.cfg.max_robots * 2],
            dtype=np.float32,
        )

    @staticmethod
    def _quat_to_yaw(quat: object) -> float:
        x = float(quat.x)
        y = float(quat.y)
        z = float(quat.z)
        w = float(quat.w)
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def _slot_position_in_robot_frame(
        center_x: float,
        center_y: float,
        yaw: float,
        offset_x: float,
        offset_y: float,
    ) -> tuple[float, float]:
        """2D position of a formation SLOT expressed in the robot frame.

        ``center_*`` and ``yaw`` are the formation-centre pose expressed in the
        robot frame (from a ``robot -> formation_center`` TF lookup); ``offset_*``
        is the slot offset defined in the centre frame. The sim formation centre
        evolves yaw-only, so the slot is the standard 2D rotation
        ``R(yaw) @ offset + centre`` -- equal to the sim's
        ``subtract_frame_transforms(robot, centre + quat_apply(centre_quat, offset))``.
        """
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        slot_x = center_x + offset_x * cos_yaw - offset_y * sin_yaw
        slot_y = center_y + offset_x * sin_yaw + offset_y * cos_yaw
        return slot_x, slot_y

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self._obs

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self._obs)


__all__ = [
    "RosFormationFollowingObservationInterface",
    "RosFormationFollowingObservationInterfaceCfg",
]
