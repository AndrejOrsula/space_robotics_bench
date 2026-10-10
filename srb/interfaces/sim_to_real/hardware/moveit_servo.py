import time
from collections.abc import Callable
from enum import Enum, auto
from math import isfinite
from typing import Sequence

import gymnasium
import numpy as np
from pydantic import Field
from typing_extensions import Self

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareDispatchNotReadyError,
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.utils.failures import HealthEvent, HealthSeverity, HealthStatus

from ._rotation import quat_to_rotmat as _quat_to_rotmat


class RotationRepresentation(Enum):
    QUAT = auto()
    ROTMAT = auto()
    ROT_6D = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class MoveitServoInterfaceCfg(HardwareInterfaceCfg):
    frame_id: str = "end_effector"
    rotation_repr: Sequence[RotationRepresentation] = (RotationRepresentation.ROT_6D,)
    ## MoveIt2 configuration for joint state and FK computation
    joint_names: Sequence[str] = ()
    base_link_name: str = "base_link"
    end_effector_name: str = "end_effector"
    group_name: str = "arm"
    max_joint_state_age_s: float = Field(default=0.5, gt=0.0, allow_inf_nan=False)
    """Reject servo commands without a recent, newly received JointState.

    Freshness starts at first receipt. A newer nonzero header stamp, or a new
    message object when unstamped, counts; rereading cached data does not.
    """
    ## Joint limits for normalization (lower, upper) per joint
    joint_limits_lower: Sequence[float] = ()
    joint_limits_upper: Sequence[float] = ()


class MoveitServoInterface(HardwareInterface):
    cfg: MoveitServoInterfaceCfg

    def __init__(
        self,
        cfg: MoveitServoInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ):
        if cfg is None:
            cfg = MoveitServoInterfaceCfg()
        if (len(cfg.joint_limits_lower) > 0) != (len(cfg.joint_limits_upper) > 0):
            raise ValueError(
                "joint_limits_lower and joint_limits_upper must be configured together"
            )
        super().__init__(cfg)
        self._clock = clock
        self._last_joint_state_s: float | None = None
        # Key by usable stamp, else object identity. Retain the message so its
        # object id cannot be reused.
        self._joint_state_key: tuple | None = None
        self._joint_state_msg: object | None = None
        # JointState key for the cached pose and latest FK failure state.
        self._fk_joint_state_key: tuple | None = None
        self._fk_failed = False
        self._servo_disabled = False
        self.obs: dict[str, np.ndarray] = {
            "proprio/fk_pos_end_effector": np.array((0.0, 0.0, 0.0), dtype=np.float32),
            "proprio_dyn/joint_pos_robot_normalized": ...,
        }

        ## Rotation
        if RotationRepresentation.QUAT in self.cfg.rotation_repr:
            # Quaternion observation
            self.obs["proprio/fk_quat_end_effector"] = np.array(
                (0.0, 0.0, 0.0, 1.0), dtype=np.float32
            )
        if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
            # Rotation matrix observation
            self.obs["proprio/fk_rotmat_end_effector"] = np.eye(
                3, dtype=np.float32
            ).flatten()
        if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
            # 6D rotation observation
            # Match _rotmat_to_rot6d's row-major identity representation.
            self.obs["proprio/fk_rot6d_end_effector"] = np.array(
                (1.0, 0.0, 0.0, 1.0, 0.0, 0.0), dtype=np.float32
            )

    def start(self, **kwargs):
        action_scale = kwargs.get("action_scale", {})
        linear_speed = self._action_scale_value(
            "robot/delta_twist_position", "robot/delta_twist_linear", action_scale
        )
        angular_speed = self._action_scale_value(
            "robot/delta_twist_orientation", "robot/delta_twist_angular", action_scale
        )
        super().start(**kwargs)
        from pymoveit2 import MoveIt2, MoveIt2Servo

        self.moveit_servo = MoveIt2Servo(
            self.ros_node,
            frame_id=self.cfg.frame_id,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
        )
        self._servo_disabled = False
        self.moveit_servo.servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))

        # Create MoveIt2 instance for joint state and FK computation
        self.moveit2 = MoveIt2(
            node=self.ros_node,
            joint_names=list(self.cfg.joint_names),
            base_link_name=self.cfg.base_link_name,
            end_effector_name=self.cfg.end_effector_name,
            group_name=self.cfg.group_name,
        )

    def close(self):
        super().close()

        servo = getattr(self, "moveit_servo", None)
        if servo is not None and not self._servo_disabled:
            servo.disable()
            self._servo_disabled = True

    def pause(self) -> None:
        """Publish a physical zero twist before latching the pause."""
        servo = getattr(self, "moveit_servo", None)
        if servo is not None:
            zero = np.zeros(3, dtype=np.float32)
            servo.servo(linear=zero, angular=zero)
        super().pause()

    def _joint_state_age_s(self) -> float | None:
        if self._last_joint_state_s is None:
            return None
        return max(0.0, self._clock() - self._last_joint_state_s)

    def health(self) -> HealthEvent:
        age_s = self._joint_state_age_s()
        fk_current = (
            self._joint_state_key is not None
            and self._fk_joint_state_key == self._joint_state_key
        )
        if age_s is None:
            state, message = "pre_message", "waiting for the first joint state"
        elif age_s > self.cfg.max_joint_state_age_s:
            state, message = "stale", "latest joint state is stale"
        elif not fk_current and self._fk_failed:
            state, message = (
                "fk_failed",
                "forward kinematics failed for the current joint state",
            )
        elif not fk_current:
            state, message = (
                "fk_stale",
                "end-effector pose was not computed from the current joint state",
            )
        else:
            state, message = "healthy", "joint state and end-effector pose are fresh"
        healthy = state == "healthy"
        return HealthEvent(
            status=HealthStatus.HEALTHY if healthy else HealthStatus.UNHEALTHY,
            severity=HealthSeverity.INFO if healthy else HealthSeverity.ERROR,
            component=self.name,
            code=f"{self.name}.{state}",
            message=message,
            details={
                "state": state,
                "ready": healthy,
                "age_ms": None if age_s is None else age_s * 1000.0,
                "max_joint_state_age_ms": self.cfg.max_joint_state_age_s * 1000.0,
                "fk_current": fk_current,
                "fk_failed": self._fk_failed,
            },
        )

    def reset(self):
        super().reset()

    def sync(self):
        super().sync()
        joint_state = self.moveit2.joint_state
        self._update_joint_pos(joint_state)
        self._update_fk(joint_state)

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/delta_twist": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(6,), dtype=np.float32
                )
            }
        )

    @property
    def action_scale_linear(self) -> float:
        return self._action_scale_value(
            "robot/delta_twist_position", "robot/delta_twist_linear", self._action_scale
        )

    @property
    def action_scale_angular(self) -> float:
        return self._action_scale_value(
            "robot/delta_twist_orientation",
            "robot/delta_twist_angular",
            self._action_scale,
        )

    def _action_scale_value(
        self,
        canonical_key: str,
        legacy_key: str,
        action_scale: dict[str, float],
    ) -> float:
        """Resolve a scale, preferring canonical keys over legacy aliases.

        Explicit zero is valid. Empty maps retain the unit default; non-empty
        maps require a matching key for each dimension.
        """
        if not action_scale:
            return 1.0

        for key in (canonical_key, legacy_key, "robot/delta_twist"):
            if key not in action_scale:
                continue
            value = action_scale[key]
            try:
                factor = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Action scale {key!r} must be a finite number; got {value!r}"
                ) from exc
            if not isfinite(factor):
                raise ValueError(
                    f"Action scale {key!r} must be a finite number; got {value!r}"
                )
            return factor
        raise ValueError(
            f"Missing action scale {canonical_key!r} (or supported alias {legacy_key!r})"
        )

    def apply_action(self, action: dict[str, np.ndarray]):
        if "robot/delta_twist" not in action:
            raise ValueError(f"[{self.name}] Missing action key 'robot/delta_twist'")
        if action["robot/delta_twist"].shape != (6,):
            raise ValueError(
                f"[{self.name}] Action shape mismatch: expected (6,), "
                f"got {action['robot/delta_twist'].shape}"
            )

        act = action["robot/delta_twist"]
        if not np.all(np.isfinite(act)):
            raise ValueError(f"[{self.name}] Action contains non-finite values")
        self.validate_dispatch_ready()
        age_s = self._joint_state_age_s()
        if age_s is None:
            raise HardwareDispatchNotReadyError(self.name, "no joint state received")
        if age_s > self.cfg.max_joint_state_age_s:
            raise HardwareDispatchNotReadyError(
                self.name,
                f"joint state stale ({age_s * 1000.0:.0f} ms > "
                f"{self.cfg.max_joint_state_age_s * 1000.0:.0f} ms)",
            )
        self.moveit_servo.servo(linear=act[0:3], angular=act[3:6])

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs.copy()

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        spaces = dict(
            observation_spaces_from_values(
                {
                    key: value
                    for key, value in self.obs.items()
                    if isinstance(value, np.ndarray)
                }
            ).spaces
        )
        spaces["proprio_dyn/joint_pos_robot_normalized"] = gymnasium.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(len(self.cfg.joint_names),),
            dtype=np.float32,
        )
        return gymnasium.spaces.Dict(spaces)

    @staticmethod
    def _message_key(joint_state: object) -> tuple:
        """Identify a JointState by its header stamp, else by object identity."""
        stamp = getattr(getattr(joint_state, "header", None), "stamp", None)
        try:
            sec = int(getattr(stamp, "sec"))
            nanosec = int(getattr(stamp, "nanosec"))
        except (AttributeError, TypeError, ValueError):
            return ("object", id(joint_state))
        if sec <= 0 and nanosec <= 0:
            return ("object", id(joint_state))
        return ("stamp", sec, nanosec)

    def _is_new_joint_state(self, key: tuple) -> bool:
        previous = self._joint_state_key
        if previous is None:
            return True
        if key[0] == "stamp" and previous[0] == "stamp":
            # Repeated or rolled-back stamps cannot renew freshness.
            return key[1:] > previous[1:]
        return key != previous

    def _update_joint_pos(self, joint_state: object | None = None):
        if joint_state is None:
            joint_state = self.moveit2.joint_state
        if joint_state is None:
            return
        key = self._message_key(joint_state)
        if not self._is_new_joint_state(key):
            # Cached messages do not prove the publisher is still live.
            return

        # Get joint positions for the configured joints
        positions = []
        for joint_name in self.cfg.joint_names:
            if joint_name not in joint_state.name:
                return  # Skip if joint not found
            idx = joint_state.name.index(joint_name)
            if idx >= len(joint_state.position):
                return  # Malformed message: names without positions
            positions.append(joint_state.position[idx])

        positions = np.array(positions, dtype=np.float32)
        if not np.all(np.isfinite(positions)):
            # A non-finite sample must not renew freshness.
            return
        self._last_joint_state_s = self._clock()
        self._joint_state_key = key
        self._joint_state_msg = joint_state
        self._fk_failed = False

        # Normalize joint positions if limits are configured
        if self.cfg.joint_limits_lower and self.cfg.joint_limits_upper:
            lower = np.array(self.cfg.joint_limits_lower, dtype=np.float32)
            upper = np.array(self.cfg.joint_limits_upper, dtype=np.float32)
            # Normalize to [-1, 1]
            range_size = upper - lower
            range_size = np.where(np.abs(range_size) < 1e-6, 1.0, range_size)
            normalized = 2.0 * (positions - lower) / range_size - 1.0
            self.obs["proprio_dyn/joint_pos_robot_normalized"] = normalized
        else:
            # Store raw positions if no limits configured
            self.obs["proprio_dyn/joint_pos_robot_normalized"] = positions

    def _update_fk(self, joint_state: object | None = None):
        if joint_state is None:
            joint_state = self.moveit2.joint_state
        if joint_state is None or joint_state is not self._joint_state_msg:
            # Only the accepted joint state may produce the published pose.
            return
        if self._fk_joint_state_key == self._joint_state_key:
            # The cached pose already belongs to this joint state.
            return

        # Avoid an unverified timeout argument. Health stays unready until FK
        # succeeds for this state, so stale poses are never reported.
        pose_stamped = self.moveit2.compute_fk()
        if pose_stamped is None:
            self._fk_failed = True
            return

        pose = pose_stamped.pose
        position = np.array(
            (pose.position.x, pose.position.y, pose.position.z), dtype=np.float64
        )
        quat = np.array(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            dtype=np.float64,
        )
        if (
            not np.all(np.isfinite(position))
            or not np.all(np.isfinite(quat))
            or np.linalg.norm(quat) < 1e-6
        ):
            # Reject non-finite poses and degenerate quaternions.
            self._fk_failed = True
            return

        # Update position observation
        self.obs["proprio/fk_pos_end_effector"] = np.array(
            (pose.position.x, pose.position.y, pose.position.z),
            dtype=np.float32,
        )

        # Update rotation observations based on configured representations
        if RotationRepresentation.QUAT in self.cfg.rotation_repr:
            self.obs["proprio/fk_quat_end_effector"] = np.array(
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                dtype=np.float32,
            )

        if (
            RotationRepresentation.ROTMAT in self.cfg.rotation_repr
            or RotationRepresentation.ROT_6D in self.cfg.rotation_repr
        ):
            rotmat = _quat_to_rotmat(pose.orientation)
            if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
                self.obs["proprio/fk_rotmat_end_effector"] = rotmat.flatten()
            if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
                self.obs["proprio/fk_rot6d_end_effector"] = self._rotmat_to_rot6d(
                    rotmat
                )

        self._fk_joint_state_key = self._joint_state_key
        self._fk_failed = False

    @staticmethod
    def _rotmat_to_rot6d(rotmat: np.ndarray) -> np.ndarray:
        return rotmat[:, :2].flatten()
