from enum import Enum, auto
from functools import cached_property
from typing import TYPE_CHECKING, Dict, Sequence

import gymnasium
import numpy
from typing_extensions import Self

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)

if TYPE_CHECKING:
    from geometry_msgs.msg import Quaternion


class RotationRepresentation(Enum):
    QUAT_WXYZ = auto()
    ROTMAT = auto()
    ROT_6D = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class MoveitServoCfg(HardwareInterfaceCfg):
    frame_id: str = "end_effector"
    rotation_repr: Sequence[RotationRepresentation] = (RotationRepresentation.ROT_6D,)


class MoveitServo(HardwareInterface):
    cfg: MoveitServoCfg

    def __init__(self, cfg: MoveitServoCfg = MoveitServoCfg()):
        super().__init__(cfg)
        self.obs: Dict[str, numpy.ndarray] = {
            "proprio/fk_pos_end_effector": numpy.array(
                (0.0, 0.0, 0.0), dtype=numpy.float32
            ),
            "proprio_dyn/joint_pos_robot_normalized": ...,
        }

        ## Rotation
        if RotationRepresentation.QUAT_WXYZ in self.cfg.rotation_repr:
            # Quaternion observation
            self.obs["proprio/fk_quat_end_effector"] = numpy.array(
                (1.0, 0.0, 0.0, 0.0), dtype=numpy.float32
            )
        if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
            # Rotation matrix observation
            self.obs["proprio/fk_rotmat_end_effector"] = numpy.eye(
                3, dtype=numpy.float32
            ).flatten()
        if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
            # 6D rotation observation
            self.obs["proprio/fk_rot6d_end_effector"] = numpy.array(
                (1.0, 0.0, 0.0, 0.0, 1.0, 0.0), dtype=numpy.float32
            )

    def start(self, **kwargs):
        super().start(**kwargs)
        from pymoveit2 import MoveIt2Servo

        self.moveit_servo = MoveIt2Servo(
            self.ros_node,
            frame_id=self.cfg.frame_id,
            linear_speed=self.action_scale_linear,
            angular_speed=self.action_scale_angular,
        )
        self.moveit_servo.servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))

    def close(self):
        super().close()

        self.moveit_servo.disable()

    def reset(self):
        super().reset()

    def sync(self):
        super().sync()
        self._update_joint_pos()
        self._update_fk()

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "robot/delta_twist": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(6,), dtype=numpy.float32
                )
            }
        )

    @cached_property
    def action_scale_linear(self) -> float:
        return self._action_scale.get(
            "robot/delta_twist_linear"
        ) or self._action_scale.get("robot/delta_twist", 1.0)

    @cached_property
    def action_scale_angular(self) -> float:
        return self._action_scale.get(
            "robot/delta_twist_angular"
        ) or self._action_scale.get("robot/delta_twist", 1.0)

    def apply_action(self, action: Dict[str, numpy.ndarray]):
        assert "robot/delta_twist" in action.keys() and action[
            "robot/delta_twist"
        ].shape == (6,)

        act = action["robot/delta_twist"]
        self.moveit_servo.servo(linear=act[0:3], angular=act[3:6])

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        return self.obs.copy()

    def _update_joint_pos(self):
        # TODO[high]: Implement joint position update logic
        raise NotImplementedError()

    def _update_fk(self):
        # TODO[high]: Implement joint position update logic
        raise NotImplementedError()

    @staticmethod
    def _quat_to_rotmat(quat: "Quaternion") -> numpy.ndarray:
        r, i, j, k = quat.w, quat.x, quat.y, quat.z
        two_s = 2.0 / (r * r + i * i + j * j + k * k)
        return numpy.array(
            (
                1 - two_s * (j * j + k * k),
                two_s * (i * j - k * r),
                two_s * (i * k + j * r),
                two_s * (i * j + k * r),
                1 - two_s * (i * i + k * k),
                two_s * (j * k - i * r),
                two_s * (i * k - j * r),
                two_s * (j * k + i * r),
                1 - two_s * (i * i + j * j),
            ),
            dtype=numpy.float32,
        ).reshape((3, 3))

    @staticmethod
    def _rotmat_to_rot6d(rotmat: numpy.ndarray) -> numpy.ndarray:
        return rotmat[:, :2].flatten()
