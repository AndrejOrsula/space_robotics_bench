from enum import Enum, auto
from typing import TYPE_CHECKING, ClassVar, Sequence

import torch

from srb.core.action.drive_geometry import (
    SteeringAxle,
    ackermann_steering_axle_sign,
    ackermann_steering_wheel_indices,
)
from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import radians

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation


#: Absolute linear-speed threshold (m/s) for point turns, above float noise.
POINT_TURN_LINEAR_VELOCITY_EPS: float = 1.0e-4


class WheeledDriveAction(ActionTerm):
    #: linear_velocity, angular_velocity
    ACTION_DIM: ClassVar[int] = 2

    cfg: "WheeledDriveActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation"

    def __init__(self, cfg: "WheeledDriveActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore[misc]
        )

        if self.cfg.drive_type is None:
            self.drive_type = self._autodetect_drive_type()
            logging.debug(f"Auto-detected drive type: {self.drive_type}")
        else:
            self.drive_type = self.cfg.drive_type

        self._setup_wheel_configuration()

    @property
    def action_dim(self) -> int:
        return self.ACTION_DIM

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions = actions
        # Bound normalized policy outputs even when velocity caps are unset.
        self._processed_actions = actions.clamp(-1.0, 1.0)
        self._processed_actions[:, 0] *= self.cfg.scale_linear
        self._processed_actions[:, 1] *= self.cfg.scale_angular
        if self.cfg.max_linear_velocity is not None:
            self._processed_actions[:, 0].clamp_(
                -self.cfg.max_linear_velocity, self.cfg.max_linear_velocity
            )
        if self.cfg.max_angular_velocity is not None:
            self._processed_actions[:, 1].clamp_(
                -self.cfg.max_angular_velocity, self.cfg.max_angular_velocity
            )

    def apply_actions(self):
        match self.drive_type:
            case DriveType.SKID_STEER:
                drive_velocities, steer_angles = self._skid_steer_drive(
                    self.processed_actions[:, 0], self.processed_actions[:, 1]
                )
            case DriveType.ACKERMANN:
                drive_velocities, steer_angles = self._ackermann_drive(
                    self.processed_actions[:, 0], self.processed_actions[:, 1]
                )
            case DriveType.ROVER:
                drive_velocities, steer_angles = self._rover_drive(
                    self.processed_actions[:, 0], self.processed_actions[:, 1]
                )

        self._asset.set_joint_velocity_target(
            drive_velocities, joint_ids=self._drive_joint_indices
        )

        if steer_angles is not None:
            self._asset.set_joint_position_target(
                steer_angles, joint_ids=self._steering_joint_indices
            )

    def reset(self, env_ids: Sequence[int] | None = None):
        super().reset(env_ids)

    def _skid_steer_drive(
        self, v_lin: torch.Tensor, v_ang: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor | None]:
        drive_velocities = torch.zeros(
            (self.num_envs, len(self._drive_wheels)), device=self.device
        )
        for i, wheel_idx in enumerate(self._drive_wheels):
            y_offset = self._wheel_positions[wheel_idx, 1]
            drive_velocities[:, i] = (v_lin + v_ang * y_offset) / self.cfg.wheel_radius

        return drive_velocities, None

    def _ackermann_drive(
        self, v_lin: torch.Tensor, v_ang: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor | None]:
        drive_velocities = torch.zeros(
            (self.num_envs, len(self._drive_wheels)), device=self.device
        )
        steer_angles = torch.zeros(
            (self.num_envs, len(self._steering_wheels)), device=self.device
        )

        turn_radius = torch.where(
            v_ang != 0.0,
            torch.abs(v_lin) / torch.abs(v_ang),
            torch.ones_like(v_lin) * 1.0e10,
        )
        turn_dir = torch.sign(v_ang)
        # Reverse steering with signed travel velocity so yaw follows the command;
        # use forward sign at zero speed to keep the steering target defined.
        # The target flips at v_lin=0; joint dynamics smooth the transition.
        drive_dir = torch.sign(v_lin)
        drive_dir = torch.where(drive_dir == 0, torch.ones_like(drive_dir), drive_dir)

        wheelbase_length = torch.tensor(self.cfg.wheelbase[0], device=self.device)

        for i, wheel_idx in enumerate(self._steering_wheels):
            wheel_pos = self._wheel_positions[wheel_idx]
            effective_radius = turn_radius - turn_dir * wheel_pos[1]
            # Rear steering reverses the angle needed for the same body yaw.
            axle_sign = ackermann_steering_axle_sign(self.cfg.steering_axle)
            angle = (
                axle_sign
                * drive_dir
                * turn_dir
                * torch.atan2(wheelbase_length, effective_radius)
            )
            angle = torch.clamp(
                angle, -self._steering_limits[i], self._steering_limits[i]
            )
            steer_angles[:, i] = angle

        for i, wheel_idx in enumerate(self._drive_wheels):
            wheel_y = self._wheel_positions[wheel_idx, 1]
            wheel_path_radius = turn_radius - turn_dir * wheel_y
            drive_velocities[:, i] = (
                torch.where(
                    v_ang != 0,
                    torch.abs(v_ang) * wheel_path_radius * torch.sign(v_lin),
                    v_lin,
                )
                / self.cfg.wheel_radius
            )

        return drive_velocities, steer_angles

    def _rover_drive(
        self, v_lin: torch.Tensor, v_ang: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor | None]:
        drive_velocities = torch.zeros(
            (self.num_envs, len(self._drive_wheels)), device=self.device
        )
        steer_angles = torch.zeros(
            (self.num_envs, len(self._steering_wheels)), device=self.device
        )

        abs_v_lin = torch.abs(v_lin)
        abs_v_ang = torch.abs(v_ang)
        turn_dir = torch.sign(v_ang)
        drive_dir = torch.sign(v_lin)
        drive_dir = torch.where(drive_dir == 0, torch.ones_like(drive_dir), drive_dir)

        half_width = torch.tensor(self.cfg.wheelbase[1] / 2, device=self.device)

        turning_radius = torch.where(
            abs_v_ang > 0,
            abs_v_lin / abs_v_ang,
            torch.tensor(1.0e10, device=self.device),
        )

        # Compare raw speed, not computed radius; tiny outputs still count as point turns.
        is_point_turn = (abs_v_lin <= POINT_TURN_LINEAR_VELOCITY_EPS) & (abs_v_ang > 0)

        r_left = turning_radius - half_width * turn_dir
        r_right = turning_radius + half_width * turn_dir

        # Steering slots use -y for the inner wheel; joint order maps physical sides.
        for i, idx in enumerate(self._steering_wheels):
            wheel_pos = self._wheel_positions[idx]
            is_front = wheel_pos[0] > 0
            is_left = wheel_pos[1] < 0

            point_angle_val = 0.78539816 * (-1 if (is_front == is_left) else 1)

            base_angle = torch.atan2(half_width, r_left if is_left else r_right)
            scaled_angle = base_angle * 0.3

            # Reverse steering with travel direction so yaw follows the command.
            normal_angle = drive_dir * turn_dir * scaled_angle * (1 if is_front else -1)

            steer_angles[:, i] = torch.where(
                is_point_turn,
                point_angle_val,
                normal_angle,
            )

        # Wheel speeds remain linear (m/s) until divided by wheel_radius.
        # Joint order sets the side convention for v_lin - v_ang*y.
        for i, wheel_idx in enumerate(self._drive_wheels):
            wheel_pos = self._wheel_positions[wheel_idx]

            side_factor = 1.0 if wheel_pos[1] < 0 else -1.0
            wheel_radial = torch.sqrt(wheel_pos[0] ** 2 + wheel_pos[1] ** 2)
            point_turn_vel = side_factor * turn_dir * abs_v_ang * wheel_radial

            normal_vel = v_lin - v_ang * wheel_pos[1]

            wheel_vel = torch.where(is_point_turn, point_turn_vel, normal_vel)
            drive_velocities[:, i] = wheel_vel / self.cfg.wheel_radius

        return drive_velocities, steer_angles

    def _autodetect_drive_type(self) -> "DriveType":
        match len(self.cfg.steering_joint_names):
            case 0:
                return DriveType.SKID_STEER
            case _len if _len < 4:
                return DriveType.ACKERMANN
            case _:
                return DriveType.ROVER

    def _setup_wheel_configuration(self):
        # Validate explicitly because python -O disables asserts.
        if not self.cfg.drive_joint_names:
            raise ValueError(
                f"{type(self).__name__} requires at least one drive joint, but "
                "cfg.drive_joint_names is empty"
            )
        self._drive_joint_indices = self._asset.find_joints(
            self.cfg.drive_joint_names, preserve_order=True
        )[0]
        self._steering_joint_indices = (
            self._asset.find_joints(self.cfg.steering_joint_names, preserve_order=True)[
                0
            ]
            if self.cfg.steering_joint_names
            else None
        )

        num_wheels = len(self._drive_joint_indices)

        positions = self._generate_wheel_positions(num_wheels)
        logging.debug(f"Generated wheel positions for {num_wheels} wheels")

        self._wheel_positions = torch.tensor(positions, device=self.device)

        self._drive_wheels = list(range(num_wheels))

        if not self.cfg.steering_joint_names:
            self._steering_wheels = []
        else:
            self._steering_wheels = self._map_steering_to_drive_wheels(positions)
            if len(self._steering_wheels) != len(self._steering_joint_indices):
                raise ValueError(
                    f"{self.drive_type.name} drive maps "
                    f"{len(self._steering_wheels)} of {num_wheels} drive wheels to "
                    f"{len(self._steering_joint_indices)} steering joints"
                )

        if self.drive_type == DriveType.ROVER:
            steering_limit = radians(45)
        else:
            steering_limit = radians(35)

        self._steering_limits = (
            torch.ones(len(self._steering_wheels), device=self.device) * steering_limit
        )

    def _generate_wheel_positions(self, num_wheels):
        """Return slots front-to-rear, with ``-y`` before ``+y`` in each pair.

        Joint names map to slots by list order. The ``y`` sign's physical side
        depends on drive type; assets order drive and steering joints to match
        their geometry and joint axes.
        """
        half_length = self.cfg.wheelbase[0] / 2
        half_width = self.cfg.wheelbase[1] / 2

        if num_wheels == 2:
            return [
                (0, -half_width, 0),  # -y
                (0, half_width, 0),  # +y
            ]
        elif num_wheels == 4:
            return [
                (half_length, -half_width, 0),  # front, -y
                (half_length, half_width, 0),  # front, +y
                (-half_length, -half_width, 0),  # rear, -y
                (-half_length, half_width, 0),  # rear, +y
            ]
        elif num_wheels == 6:
            mid_width = (self.cfg.wheelbase_mid or self.cfg.wheelbase[1]) / 2
            return [
                (half_length, -half_width, 0),  # front, -y
                (half_length, half_width, 0),  # front, +y
                (0, -mid_width, 0),  # middle, -y
                (0, mid_width, 0),  # middle, +y
                (-half_length, -half_width, 0),  # rear, -y
                (-half_length, half_width, 0),  # rear, +y
            ]
        else:
            # Place pairs evenly from front to rear; odd counts end at centerline.
            positions: list[tuple[float, float, float]] = []
            stations = (num_wheels + 1) // 2
            for index in range(stations):
                x = (
                    half_length - index * (2 * half_length) / (stations - 1)
                    if stations > 1
                    else 0.0
                )
                if num_wheels - len(positions) >= 2:
                    positions.append((x, -half_width, 0))  # -y
                    positions.append((x, half_width, 0))  # +y
                else:
                    positions.append((x, 0.0, 0))  # Centreline wheel

            return positions

    def _map_steering_to_drive_wheels(self, drive_positions):
        steering_wheels = []

        if self.drive_type == DriveType.ACKERMANN:
            steering_wheels = ackermann_steering_wheel_indices(
                drive_positions,
                steering_count=len(self.cfg.steering_joint_names),
                steering_axle=self.cfg.steering_axle,
            )
        elif self.drive_type == DriveType.ROVER:
            if len(drive_positions) >= 6:
                steering_wheels = [0, 1, 4, 5]
            else:
                x_coords = [pos[0] for pos in drive_positions]
                y_coords = [pos[1] for pos in drive_positions]

                min_x, max_x = min(x_coords), max(x_coords)
                min_y, max_y = min(y_coords), max(y_coords)

                for i, (x, y, _) in enumerate(drive_positions):
                    if (abs(x - min_x) < 0.1 or abs(x - max_x) < 0.1) and (
                        abs(y - min_y) < 0.1 or abs(y - max_y) < 0.1
                    ):
                        steering_wheels.append(i)

        steering_wheels = steering_wheels[: len(self.cfg.steering_joint_names)]

        return steering_wheels


class DriveType(Enum):
    SKID_STEER = auto()
    ACKERMANN = auto()
    ROVER = auto()


@configclass
class WheeledDriveActionCfg(ActionTermCfg):
    class_type: type = WheeledDriveAction

    scale_linear: float = 1.0
    scale_angular: float = 1.0

    drive_type: DriveType | None = None

    drive_joint_names: list[str] = []
    steering_joint_names: list[str] = []
    steering_axle: SteeringAxle = "front"

    wheelbase: tuple[float, float] = (0.8, 0.6)  # (length, width)
    wheel_radius: float = 0.1
    wheelbase_mid: float | None = None  # Middle wheels width (for rovers)

    max_linear_velocity: float | None = None
    max_angular_velocity: float | None = None

    def __post_init__(self) -> None:
        # Wheel speeds divide by the radius.
        if not self.wheel_radius > 0.0 or self.wheel_radius == float("inf"):
            raise ValueError(f"wheel_radius must be positive, got {self.wheel_radius}")
        if (
            self.drive_type in (DriveType.ACKERMANN, DriveType.ROVER)
            and not self.steering_joint_names
        ):
            raise ValueError(
                f"{self.drive_type.name} drive requires at least one steering joint"
            )
