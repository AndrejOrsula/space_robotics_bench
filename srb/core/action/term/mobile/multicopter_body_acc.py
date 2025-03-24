from dataclasses import MISSING
from typing import TYPE_CHECKING, Dict, Sequence, Type

import torch

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass
from srb.utils.math import euler_xyz_from_quat, quat_from_euler_xyz

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation


class MulticopterBodyAccelerationAction(ActionTerm):
    cfg: "MulticopterBodyAccelerationActionCfg"
    _asset: "Articulation"

    def __init__(self, cfg: "MulticopterBodyAccelerationActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

        self._body_index = self._asset.find_bodies(self.cfg.frame_base)[0]
        self._rotor_indices, rotor_names = self._asset.find_joints(
            self.cfg.regex_rotor_joints
        )

        if isinstance(self.cfg.nominal_rpm, Dict):
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
        return 4

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = self.raw_actions * self.cfg.scale

    def apply_actions(self):
        self._asset.set_joint_velocity_target(
            self._nominal_rpm, joint_ids=self._rotor_indices
        )

        current_velocity = self._asset._data.body_vel_w[:, self._body_index].squeeze(1)
        current_yaw = euler_xyz_from_quat(self._asset._data.root_quat_w)[2]

        applied_velocity_lin = (
            self.cfg.controller_damping * current_velocity[:, :3]
            + (1 - self.cfg.controller_damping) * self.processed_actions[:, :3]
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

        target_tilt = (
            torch.atan2(applied_velocity_lin[:, 1], applied_velocity_lin[:, 0])
            - current_yaw
        )
        tilt_factor = self.cfg.tilt_magnitude * torch.norm(
            current_velocity[:, :2], dim=1
        )
        target_roll = -torch.sin(target_tilt) * tilt_factor
        target_pitch = torch.cos(target_tilt) * tilt_factor
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
    class_type: Type = MulticopterBodyAccelerationAction  # type: ignore

    frame_base: str = MISSING  # type: ignore
    regex_rotor_joints: str = MISSING  # type: ignore

    nominal_rpm: float | Dict[str, float] = 1000.0
    tilt_magnitude: float = 1.0
    controller_damping: float = 0.98

    scale: float = 1.0
    noise: float = 0.0025
