"""ROS input interface for validated planar scoop targets."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal, Sequence

import gymnasium
import numpy as np

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.utils import logging

if TYPE_CHECKING:
    from geometry_msgs.msg import Pose2D
    from std_msgs.msg import Float32MultiArray

_DEFAULT_XY_M: tuple[float, float] = (0.55, 0.0)
_DEFAULT_YAW_RAD: float = 0.0
_DEFAULT_FRAME: str = "kinova_base_link"
_DEFAULT_BOUNDS_M: tuple[tuple[float, float], tuple[float, float]] = (
    (0.30, 0.85),
    (-0.35, 0.35),
)

_OBS_XY = "command/scoop_target_xy"
_OBS_YAW = "command/scoop_target_yaw_sin_cos"


@dataclass(frozen=True)
class _ValidationReport:
    valid: bool
    errors: list[str] = field(default_factory=list)


class RosScoopTargetInterfaceCfg(HardwareInterfaceCfg):
    """Configuration for operator-selected single-scoop excavation targets."""

    topic: str = "scoop_target"
    message_type: Literal["pose2d", "float32_multi_array"] = "pose2d"
    frame: str = _DEFAULT_FRAME
    default_xy_m: tuple[float, float] = _DEFAULT_XY_M
    default_yaw_rad: float = _DEFAULT_YAW_RAD
    workspace_bounds_m: tuple[tuple[float, float], tuple[float, float]] = (
        _DEFAULT_BOUNDS_M
    )


class RosScoopTargetInterface(HardwareInterface):
    cfg: RosScoopTargetInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    def __init__(
        self,
        cfg: RosScoopTargetInterfaceCfg | None = None,
    ):
        if cfg is None:
            cfg = RosScoopTargetInterfaceCfg()
        super().__init__(cfg)
        self._last_update_time_s: float | None = None
        self._last_target_valid = True
        self._last_target_error = ""
        self._sub = None
        default_xy_m = (float(cfg.default_xy_m[0]), float(cfg.default_xy_m[1]))
        default_yaw_rad = float(cfg.default_yaw_rad)
        # The default is published as a target too, so it must pass the same
        # checks as an operator-selected one.
        report = self._validate(xy_m=default_xy_m, yaw_rad=default_yaw_rad)
        if not report.valid:
            raise ValueError(
                f"[{self.name}] invalid default scoop target: "
                + "; ".join(report.errors)
            )
        self._set_target(
            xy_m=default_xy_m,
            yaw_rad=default_yaw_rad,
            mark_update=False,
        )

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        msg_type: type[Pose2D] | type[Float32MultiArray]
        if self.cfg.message_type == "pose2d":
            from geometry_msgs.msg import Pose2D

            msg_type = Pose2D
        elif self.cfg.message_type == "float32_multi_array":
            from std_msgs.msg import Float32MultiArray

            msg_type = Float32MultiArray
        else:
            raise ValueError(
                f"Unsupported scoop target message type: {self.cfg.message_type}"
            )

        self._sub = self.ros_node.create_subscription(
            msg_type,
            self.cfg.topic,
            self._target_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        logging.info(f"[{self.name}] Subscribing to scoop target: {self.cfg.topic}")

    def close(self):
        super().close()
        if self._sub is not None:
            self._sub.destroy()
            self._sub = None

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self._obs

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self._obs)

    @property
    def info(self) -> dict[str, Any]:
        return {
            "topic": self.cfg.topic,
            "message_type": self.cfg.message_type,
            "frame": self.cfg.frame,
            "last_update_time_s": self._last_update_time_s,
            "last_target_valid": self._last_target_valid,
            "last_target_error": self._last_target_error,
        }

    def _target_callback(self, msg: "Pose2D | Float32MultiArray") -> None:
        try:
            xy_m, yaw_rad = self._target_from_msg(msg)
        except ValueError as exc:
            self._last_target_valid = False
            self._last_target_error = str(exc)
            logging.warning(f"[{self.name}] Rejected scoop target: {exc}")
            return

        report = self._validate(xy_m=xy_m, yaw_rad=yaw_rad)
        if not report.valid:
            self._last_target_valid = False
            self._last_target_error = "; ".join(report.errors)
            logging.warning(
                f"[{self.name}] Rejected scoop target: {self._last_target_error}"
            )
            return

        self._set_target(xy_m=xy_m, yaw_rad=yaw_rad, mark_update=True)
        logging.debug(
            f"[{self.name}] Accepted scoop target "
            f"xy=({xy_m[0]:.3f}, {xy_m[1]:.3f}) yaw={yaw_rad:.3f}"
        )

    def _target_from_msg(
        self,
        msg: "Pose2D | Float32MultiArray",
    ) -> tuple[tuple[float, float], float]:
        if hasattr(msg, "x") and hasattr(msg, "y"):
            yaw = getattr(msg, "theta", self.cfg.default_yaw_rad)
            return (float(msg.x), float(msg.y)), float(yaw)

        data = getattr(msg, "data", None)
        if data is not None:
            if len(data) not in (2, 3):
                raise ValueError(
                    "float32_multi_array scoop target must contain "
                    "[x, y] or [x, y, yaw]"
                )
            yaw = float(data[2]) if len(data) == 3 else float(self.cfg.default_yaw_rad)
            return (float(data[0]), float(data[1])), yaw

        raise ValueError(f"unsupported scoop target message type: {type(msg).__name__}")

    def _validate(
        self,
        *,
        xy_m: tuple[float, float],
        yaw_rad: float,
    ) -> _ValidationReport:
        errors: list[str] = []
        (x_min, x_max), (y_min, y_max) = self.cfg.workspace_bounds_m
        x, y = xy_m

        if not math.isfinite(x) or not math.isfinite(y):
            errors.append("target xy must be finite")
        else:
            if not x_min <= x <= x_max:
                errors.append(
                    f"target x {x:.3f} outside workspace bounds "
                    f"[{x_min:.3f}, {x_max:.3f}]"
                )
            if not y_min <= y <= y_max:
                errors.append(
                    f"target y {y:.3f} outside workspace bounds "
                    f"[{y_min:.3f}, {y_max:.3f}]"
                )
        if not math.isfinite(yaw_rad):
            errors.append("target yaw must be finite")
        elif not -math.pi <= yaw_rad <= math.pi:
            errors.append(
                f"target yaw {yaw_rad:.3f} outside bounds "
                f"[{-math.pi:.3f}, {math.pi:.3f}]"
            )

        return _ValidationReport(valid=not errors, errors=errors)

    def _set_target(
        self,
        *,
        xy_m: tuple[float, float],
        yaw_rad: float,
        mark_update: bool,
    ) -> None:
        self._obs = {
            _OBS_XY: np.asarray(xy_m, dtype=np.float32),
            _OBS_YAW: np.asarray(
                (math.sin(yaw_rad), math.cos(yaw_rad)),
                dtype=np.float32,
            ),
        }
        self._last_target_valid = True
        self._last_target_error = ""
        if mark_update:
            self._last_update_time_s = time.monotonic()
