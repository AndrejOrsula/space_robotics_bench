from dataclasses import MISSING
from typing import TYPE_CHECKING, List, Sequence, Tuple, Type

import torch

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation


class WheeledRoverDriveAction(ActionTerm):
    cfg: "WheeledRoverDriveActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation"

    def __init__(self, cfg: "WheeledRoverDriveActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

        self._steering_joint_indices = self._asset.find_joints(
            self.cfg.steering_joint_names, preserve_order=True
        )[0]
        self._drive_joint_indices = self._asset.find_joints(
            self.cfg.drive_joint_names, preserve_order=True
        )[0]

    @property
    def action_dim(self) -> int:
        return 2

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = self.raw_actions * self.cfg.scale
        self._processed_actions[:, 1] /= torch.pi

    def apply_actions(self):
        steer_joint_positions, drive_joint_velocities = self._process_actions(
            velocity_lin=self._processed_actions[:, 0],
            velocity_ang=self._processed_actions[:, 1],
            wheelbase=self.cfg.wheelbase,
            wheelbase_mid=self.cfg.wheelbase_mid or self.cfg.wheelbase[1],
            wheel_radius=self.cfg.wheel_radius,
        )
        self._asset.set_joint_position_target(
            steer_joint_positions, joint_ids=self._steering_joint_indices
        )
        self._asset.set_joint_velocity_target(
            drive_joint_velocities, joint_ids=self._drive_joint_indices
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        pass

    @staticmethod
    @torch.jit.script
    def _process_actions(
        *,
        velocity_lin: torch.Tensor,
        velocity_ang: torch.Tensor,
        wheelbase: Tuple[float, float],
        wheelbase_mid: float,
        wheel_radius: float,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Computes the steering joint positions and drive joint velocities from linear and angular velocities of a wheeled rover.

        Adapted from: <https://github.com/abmoRobotics/RLRoverLab>
        """
        num_envs = velocity_lin.size(0)
        device = velocity_lin.device

        abs_velocity_lin = torch.abs(velocity_lin)
        abs_velocity_ang = torch.abs(velocity_ang)
        turn_direction = torch.sign(velocity_ang)
        turning_radius = torch.where(
            torch.logical_not(abs_velocity_ang == 0)
            | torch.logical_not(abs_velocity_lin == 0),
            abs_velocity_lin / abs_velocity_ang,
            torch.tensor(10e10, device=device),
        )
        is_point_turn = turning_radius < wheelbase_mid / 2.0

        r_front_left = turning_radius - (wheelbase[0] / 2) * turn_direction
        steer_angle_front_left = torch.where(
            is_point_turn,
            torch.tensor(-torch.pi / 4, device=device),
            turn_direction
            * torch.atan2(
                (torch.tensor(wheelbase[1], device=device).repeat(num_envs) / 2),
                r_front_left,
            ),
        )
        r_front_right = turning_radius + (wheelbase[0] / 2) * turn_direction
        steer_angle_front_right = torch.where(
            is_point_turn,
            torch.tensor(torch.pi / 4, device=device),
            turn_direction
            * torch.atan2(
                (torch.tensor(wheelbase[1], device=device).repeat(num_envs) / 2),
                r_front_right,
            ),
        )
        r_rear_left = turning_radius - (wheelbase[0] / 2) * turn_direction
        steer_angle_rear_left = torch.where(
            is_point_turn,
            torch.tensor(torch.pi / 4, device=device),
            -turn_direction
            * torch.atan2(
                (torch.tensor(wheelbase[1], device=device).repeat(num_envs) / 2),
                r_rear_left,
            ),
        )
        r_rear_right = turning_radius + (wheelbase[0] / 2) * turn_direction
        steer_angle_rear_right = torch.where(
            is_point_turn,
            torch.tensor(-torch.pi / 4, device=device),
            -turn_direction
            * torch.atan2(
                (torch.tensor(wheelbase[1], device=device).repeat(num_envs) / 2),
                r_rear_right,
            ),
        )
        steer_joint_positions = torch.stack(
            [
                steer_angle_front_left,
                steer_angle_front_right,
                steer_angle_rear_left,
                steer_angle_rear_right,
            ],
            dim=1,
        )

        drive_direction = torch.sign(velocity_lin)
        drive_direction = torch.where(
            drive_direction == 0, drive_direction + 1, drive_direction
        )
        velocity_lin_front_left = torch.where(
            is_point_turn,
            -turn_direction * abs_velocity_ang,
            drive_direction
            * torch.where(
                abs_velocity_ang == 0,
                abs_velocity_lin,
                (r_front_left * abs_velocity_ang),
            ),
        )
        velocity_lin_front_right = torch.where(
            is_point_turn,
            turn_direction * abs_velocity_ang,
            drive_direction
            * torch.where(
                abs_velocity_ang == 0,
                abs_velocity_lin,
                (r_front_right * abs_velocity_ang),
            ),
        )
        velocity_lin_rear_left = torch.where(
            is_point_turn,
            -turn_direction * abs_velocity_ang,
            drive_direction
            * torch.where(
                abs_velocity_ang == 0,
                abs_velocity_lin,
                (r_rear_left * abs_velocity_ang),
            ),
        )
        velocity_lin_rear_right = torch.where(
            is_point_turn,
            turn_direction * abs_velocity_ang,
            drive_direction
            * torch.where(
                abs_velocity_ang == 0,
                abs_velocity_lin,
                (r_rear_right * abs_velocity_ang),
            ),
        )
        velocity_lin_mid_left = torch.where(
            is_point_turn,
            -turn_direction * abs_velocity_ang,
            drive_direction
            * torch.where(
                abs_velocity_ang == 0,
                abs_velocity_lin,
                (
                    turning_radius
                    - (wheelbase_mid / 2) * turn_direction * abs_velocity_ang
                ),
            ),
        )
        velocity_lin_mid_right = torch.where(
            is_point_turn,
            turn_direction * abs_velocity_ang,
            drive_direction
            * torch.where(
                abs_velocity_ang == 0,
                abs_velocity_lin,
                (
                    turning_radius
                    + (wheelbase_mid / 2) * turn_direction * abs_velocity_ang
                ),
            ),
        )
        drive_joint_velocities = torch.stack(
            [
                velocity_lin_front_left,
                velocity_lin_front_right,
                velocity_lin_rear_left,
                velocity_lin_rear_right,
                velocity_lin_mid_left,
                velocity_lin_mid_right,
            ],
            dim=1,
        ) / (2.0 * wheel_radius)

        return steer_joint_positions, drive_joint_velocities


@configclass
class WheeledRoverDriveActionCfg(ActionTermCfg):
    class_type: Type = WheeledRoverDriveAction

    scale: float = 1.0

    steering_joint_names: List[str] = MISSING  # type: ignore
    drive_joint_names: List[str] = MISSING  # type: ignore

    wheelbase: Tuple[float, float] = MISSING  # type: ignore
    wheelbase_mid: float | None = None

    wheel_radius: float = MISSING  # type: ignore
