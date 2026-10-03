import math
from dataclasses import MISSING
from typing import TYPE_CHECKING, ClassVar, Sequence

import torch

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass
from srb.utils.math import euler_xyz_from_quat, quat_from_euler_xyz

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation


class MulticopterBodyAccelerationAction(ActionTerm):
    """Velocity-level multicopter control in the body frame.

    The term writes a root velocity rather than rotor thrusts: the rotors are
    spun at a nominal RPM for appearance and the vehicle's attitude is authored
    to look like a tilt appropriate to its motion. This is a kinematic
    controller, not flight dynamics, and it is declared as such on the
    modelling envelope page. It does what its name says:

    * the linear command is interpreted in the **body** frame and rotated into
      the world frame by the vehicle's yaw before it is blended, so "forward"
      follows the vehicle's heading rather than world +x;
    * the authored tilt is clamped. An unbounded ``tilt_magnitude * speed``
      would pass pi radians above 3.14 m/s and roll the vehicle past vertical.
    """

    ACTION_DIM: ClassVar[int] = 4

    cfg: "MulticopterBodyAccelerationActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation"

    def __init__(self, cfg: "MulticopterBodyAccelerationActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore[misc]
        )

        self._body_index = self._asset.find_bodies(self.cfg.frame_base)[0]
        self._rotor_indices, rotor_names = self._asset.find_joints(
            self.cfg.regex_rotor_joints
        )

        if isinstance(self.cfg.nominal_rpm, dict):
            self._nominal_rpm = torch.tensor(
                [self.cfg.nominal_rpm[name] for name in rotor_names],
                device=self.device,
                dtype=torch.float32,
            ).repeat(self._asset.num_instances, 1)
        else:
            self._nominal_rpm = torch.tensor(
                [self.cfg.nominal_rpm] * len(rotor_names),
                device=self.device,
                dtype=torch.float32,
            ).repeat(self._asset.num_instances, 1)

    @property
    def action_dim(self) -> int:
        return self.ACTION_DIM

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        # Clamp the normalized action to [-1, 1] before scaling so an
        # out-of-range policy output cannot write an unbounded velocity to sim
        # (matches the clamp every other action term applies).
        self._processed_actions = self.raw_actions.clamp(-1.0, 1.0) * self.cfg.scale

    @staticmethod
    def body_to_world_xy(command_b: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
        """Rotate a body-frame linear command into the world frame by yaw.

        Only the heading is applied. The vehicle's roll and pitch are authored
        for appearance by this same term, so rotating by the full attitude
        would feed the decoration back into the dynamics.
        """
        cos_yaw = torch.cos(yaw)
        sin_yaw = torch.sin(yaw)
        return torch.stack(
            (
                command_b[:, 0] * cos_yaw - command_b[:, 1] * sin_yaw,
                command_b[:, 0] * sin_yaw + command_b[:, 1] * cos_yaw,
                command_b[:, 2],
            ),
            dim=-1,
        )

    @staticmethod
    def tilt_from_motion(
        applied_velocity_lin_w: torch.Tensor,
        speed_xy: torch.Tensor,
        yaw: torch.Tensor,
        *,
        tilt_magnitude: float,
        max_tilt: float,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Return the (roll, pitch) authored to depict the vehicle's motion.

        The magnitude is clamped to ``max_tilt``; without the clamp the tilt
        grows without bound with speed and inverts the vehicle past pi.
        """
        direction = (
            torch.atan2(applied_velocity_lin_w[:, 1], applied_velocity_lin_w[:, 0])
            - yaw
        )
        magnitude = (tilt_magnitude * speed_xy).clamp(max=max_tilt)
        return -torch.sin(direction) * magnitude, torch.cos(direction) * magnitude

    def apply_actions(self):
        self._asset.set_joint_velocity_target(
            self._nominal_rpm, joint_ids=self._rotor_indices
        )

        current_velocity = self._asset._data.body_vel_w[:, self._body_index].squeeze(1)
        current_yaw = euler_xyz_from_quat(self._asset._data.root_quat_w)[2]

        command_w = self.body_to_world_xy(self.processed_actions[:, :3], current_yaw)
        applied_velocity_lin = (
            self.cfg.controller_damping * current_velocity[:, :3]
            + (1 - self.cfg.controller_damping) * command_w
        )
        applied_velocity_rot_yaw = (
            self.cfg.controller_damping * current_velocity[:, 5]
            + (1 - self.cfg.controller_damping) * self.processed_actions[:, 3]
        )
        applied_velocities = torch.cat(
            (
                applied_velocity_lin,
                torch.zeros_like(applied_velocity_rot_yaw)[:, None],
                torch.zeros_like(applied_velocity_rot_yaw)[:, None],
                applied_velocity_rot_yaw[:, None],
            ),
            dim=1,
        )
        if self.cfg.noise > 0:
            applied_velocities += self.cfg.noise * torch.randn_like(applied_velocities)
        self._asset.write_root_velocity_to_sim(applied_velocities)

        target_roll, target_pitch = self.tilt_from_motion(
            applied_velocity_lin,
            torch.norm(applied_velocity_lin[:, :2], dim=1),
            current_yaw,
            tilt_magnitude=self.cfg.tilt_magnitude,
            max_tilt=self.cfg.max_tilt,
        )
        self._asset.write_root_pose_to_sim(
            torch.cat(
                (
                    self._asset._data.root_pos_w,
                    quat_from_euler_xyz(target_roll, target_pitch, current_yaw),
                ),
                dim=1,
            )
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        pass


@configclass
class MulticopterBodyAccelerationActionCfg(ActionTermCfg):
    class_type: type = MulticopterBodyAccelerationAction  # type: ignore[misc]
    frame_base: str = MISSING  # type: ignore[assignment]
    regex_rotor_joints: str = MISSING  # type: ignore[assignment]
    nominal_rpm: float | dict[str, float] = 1000.0
    tilt_magnitude: float = 1.0
    max_tilt: float = math.pi / 6.0
    """Upper bound on the authored tilt, in radians.

    30 deg is a conventional multicopter attitude limit. Without a bound the
    tilt is ``tilt_magnitude * speed`` and passes pi above 3.14 m/s, at which
    point the vehicle renders inverted.
    """
    controller_damping: float = 0.98

    scale: float = 1.0
    noise: float = 0.0025
