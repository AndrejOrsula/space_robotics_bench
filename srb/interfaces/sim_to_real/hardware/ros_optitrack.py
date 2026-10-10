"""OptiTrack TF interface for planar base pose and optional body velocity.

ROS imports are deferred until ``start()``; vertical velocity is always zero.
"""

from __future__ import annotations

import math
from typing import Sequence

import gymnasium
import numpy as np
from pydantic import Field

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.terrain.landscaping_pose import (
    quat_xyzw_to_yaw,
)
from srb.utils import logging
from srb.utils.failures import HealthEvent, HealthSeverity, HealthStatus


class RosOptitrackInterfaceCfg(HardwareInterfaceCfg):
    """Configuration for the OptiTrack mocap hardware interface."""

    world_frame: str = "world"
    """Mocap world (fixed) frame id. Override if your mocap driver publishes
    under a different frame name (e.g. ``"mocap"`` / ``"optitrack"``)."""
    base_frame: str = "base_link"
    """Robot body frame id tracked by the mocap (child of the tf2 lookup)."""
    tf_timeout_duration: float = Field(default=0.05, gt=0.0, allow_inf_nan=False)
    """Max seconds to wait when querying tf2."""
    name: str = "ros_optitrack"
    """Hardware interface instance name (inherited slot)."""

    # --- Velocity derivation ---
    emit_base_vel_body: bool = True
    """If True, emit proprio/base_vel_body (3,) float32 = body-frame linear
    velocity, computed by numerically differentiating the mocap pose and
    rotating into body frame via yaw. Disable for pose-only deployments."""
    vel_ema_alpha: float = Field(default=0.0, ge=0.0, le=1.0, allow_inf_nan=False)
    """EMA smoothing factor for the velocity derivative [0, 1]. 0 = no
    smoothing (pure finite-difference, matches sim-side unsmoothed
    root_lin_vel_b); 1 = hold last value. Empirical retune if deploy
    measurements show jitter; sim-side uses no smoothing."""
    vel_max_dt_s: float = Field(default=0.2, gt=0.0, allow_inf_nan=False)
    """If the pose timestamp gap between two consecutive sync() calls exceeds
    this, reject the derivative (assume dropped frames) and hold last-good
    velocity."""
    clock_jump_tolerance_s: float = Field(default=0.0, ge=0.0, allow_inf_nan=False)
    """How far, in seconds, a mocap transform stamp may lead the ROS clock
    before health reports the clock as invalid (fails closed, like
    ``RosTfInterfaceCfg``)."""


class RosOptitrackInterface(HardwareInterface):
    """Produces ``proprio/base_pose_world_optitrack`` from a mocap TF stream."""

    cfg: RosOptitrackInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = (
        ("proprio/base_pose_world_optitrack", "base_pose_world_optitrack"),
        ("proprio/base_vel_body", "base_vel_body"),
    )

    def __init__(
        self,
        cfg: RosOptitrackInterfaceCfg | None = None,
    ):
        if cfg is None:
            cfg = RosOptitrackInterfaceCfg()
        super().__init__(cfg)

        # Output buffer returned via the ``observation`` property.
        self.obs: dict[str, np.ndarray] = {
            "proprio/base_pose_world_optitrack": np.zeros((3,), dtype=np.float32),
        }

        # Body-frame linear velocity derivation state.
        if self.cfg.emit_base_vel_body:
            self.obs["proprio/base_vel_body"] = np.zeros((3,), dtype=np.float32)
            self._prev_pose_xy: np.ndarray | None = None
            self._prev_timestamp: float | None = None
            self._latest_vel_body: np.ndarray = np.zeros((3,), dtype=np.float32)

        # ROS handles (set in ``start``).
        self.tf_buffer = None
        self._tf_listener = None
        self._tf_timeout = None
        self._last_tf_stamp = None

    # ----------------------------------------------------------------- lifecycle

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformListener

        self._last_tf_stamp = None
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self.ros_node)
        self._tf_timeout = Duration(
            seconds=math.floor(self.cfg.tf_timeout_duration),
            nanoseconds=int((self.cfg.tf_timeout_duration % 1) * 1e9),
        )
        logging.info(
            f"[{self.name}] Listening for mocap TF: "
            f"{self.cfg.world_frame} -> {self.cfg.base_frame}"
        )

    def close(self):
        super().close()
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        if self.tf_buffer is not None:
            self.tf_buffer.clear()
            self.tf_buffer = None

    def health(self) -> HealthEvent | dict | None:
        """Return staleness event if last mocap pose is older than 500 ms."""
        if self.tf_buffer is None:
            return None
        if self._last_tf_stamp is None:
            return HealthEvent(
                status=HealthStatus.UNHEALTHY,
                severity=HealthSeverity.ERROR,
                component=self.name,
                code=f"{self.name}.pre_message",
                message="waiting for the first mocap transform",
                details={
                    "state": "pre_message",
                    "ready": False,
                    "world_frame": self.cfg.world_frame,
                    "base_frame": self.cfg.base_frame,
                },
            )
        from rclpy.time import Time

        if not getattr(self, "_ros_node", None):
            return None
        now = self._ros_node.get_clock().now()
        age_s = (now - Time.from_msg(self._last_tf_stamp)).nanoseconds / 1e9
        if age_s < -self.cfg.clock_jump_tolerance_s:
            # A stamp from the future gives a negative age that would pass the
            # staleness check below, so freshness would go unverified.
            return HealthEvent(
                status=HealthStatus.UNHEALTHY,
                severity=HealthSeverity.ERROR,
                component=self.name,
                code=f"{self.name}.clock_invalid",
                message="mocap transform timestamp is ahead of the ROS clock",
                details={
                    "state": "clock_invalid",
                    "ready": False,
                    "age_s": age_s,
                    "world_frame": self.cfg.world_frame,
                    "base_frame": self.cfg.base_frame,
                },
            )
        if age_s > 0.5:
            return {
                "kind": "topic_stale",
                "interface": self.name,
                "age_ms": age_s * 1000.0,
            }
        return None

    def reset(self):
        super().reset()
        self._last_tf_stamp = None
        self.obs = {
            "proprio/base_pose_world_optitrack": np.zeros((3,), dtype=np.float32),
        }
        if self.cfg.emit_base_vel_body:
            self.obs["proprio/base_vel_body"] = np.zeros((3,), dtype=np.float32)
            self._prev_pose_xy = None
            self._prev_timestamp = None
            self._latest_vel_body = np.zeros((3,), dtype=np.float32)

    # -------------------------------------------------------------- main stepping

    def sync(self):
        super().sync()
        if self.tf_buffer is None:
            return

        from rclpy.time import Time

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.cfg.world_frame,
                self.cfg.base_frame,
                Time(),
                self._tf_timeout,
            )
        except Exception as e:  # noqa: BLE001 - tf not ready yet is normal at startup
            # Leave obs at its last-successful value to avoid flicker during
            # brief tf dropouts; log at debug since this is the common
            # start-up transient.
            logging.debug(
                f"[{self.name}] tf2 lookup {self.cfg.world_frame} -> "
                f"{self.cfg.base_frame} failed: {e}"
            )
            return

        self._last_tf_stamp = tf_stamped.header.stamp
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        yaw = quat_xyzw_to_yaw(q.x, q.y, q.z, q.w)
        self.obs["proprio/base_pose_world_optitrack"] = np.array(
            (t.x, t.y, yaw), dtype=np.float32
        )

        # Derive body-frame linear velocity via finite-difference when enabled.
        if self.cfg.emit_base_vel_body:
            # dt is derived from the tf header stamp (mocap capture time), NOT
            # wall-clock -- see _update_base_vel_body.
            stamp = tf_stamped.header.stamp
            t_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            self._update_base_vel_body(
                float(t.x), float(t.y), float(yaw), t_stamp=t_stamp
            )

    # -------------------------------------------------------------------- property

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self.obs)

    # --------------------------------------------------------------------- helpers

    def _update_base_vel_body(
        self, x: float, y: float, yaw: float, t_stamp: float
    ) -> None:
        """Estimate body velocity from pose differences and smooth with EMA.

        Uses mocap capture timestamps. Duplicate or backward stamps hold the
        last velocity without advancing the anchor; gaps over ``vel_max_dt_s``
        reseed it. The first sample also seeds history, and vertical velocity
        remains zero.
        """
        if self._prev_pose_xy is None or self._prev_timestamp is None:
            # First successful pose after reset / start - seed anchor.
            self._prev_pose_xy = np.array([x, y], dtype=np.float32)
            self._prev_timestamp = t_stamp
            # Keep last-good vel (initial zero).
            self.obs["proprio/base_vel_body"] = self._latest_vel_body.copy()
            return

        dt = t_stamp - self._prev_timestamp
        if dt == 0.0:
            # Duplicate stamp (stale tf repeat): hold last-good velocity.
            # Do NOT advance the anchor so that a subsequent new stamp
            # computes velocity against the original anchor.
            self.obs["proprio/base_vel_body"] = self._latest_vel_body.copy()
            return
        if dt < 0.0:
            # Backward stamp (clock step / bag replay): hold last-good.
            logging.debug(
                f"[{self.name}] tf header stamp went backwards "
                f"(dt={dt:.6f}s); holding last-good velocity."
            )
            self.obs["proprio/base_vel_body"] = self._latest_vel_body.copy()
            return
        if dt > self.cfg.vel_max_dt_s:
            # Gap-too-large: reject derivative, reseed anchor to current
            # sample (as if first call after reset) so the next sample can
            # derive a valid dt.
            self._prev_pose_xy = np.array([x, y], dtype=np.float32)
            self._prev_timestamp = t_stamp
            self.obs["proprio/base_vel_body"] = self._latest_vel_body.copy()
            return

        # Finite-difference in world frame.
        vx_world = (x - float(self._prev_pose_xy[0])) / dt
        vy_world = (y - float(self._prev_pose_xy[1])) / dt

        # Rotate into body frame via current yaw.
        # v_body = R_z(-yaw) @ v_world
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        vx_body = cy * vx_world + sy * vy_world
        vy_body = -sy * vx_world + cy * vy_world

        raw = np.array([vx_body, vy_body, 0.0], dtype=np.float32)

        # EMA smoothing. Pin float32 to guard against silent dtype promotion
        # under future numpy versions.
        alpha = float(self.cfg.vel_ema_alpha)
        self._latest_vel_body = (
            alpha * self._latest_vel_body + (1.0 - alpha) * raw
        ).astype(np.float32, copy=False)

        self.obs["proprio/base_vel_body"] = self._latest_vel_body.copy()

        # Advance anchor.
        self._prev_pose_xy = np.array([x, y], dtype=np.float32)
        self._prev_timestamp = t_stamp
