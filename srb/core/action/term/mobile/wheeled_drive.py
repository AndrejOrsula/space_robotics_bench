from enum import Enum, auto
from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple, Type

import torch
from pydantic import BaseModel

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import deg_to_rad

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation


class WheeledDriveAction(ActionTerm):
    cfg: "WheeledDriveActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation"

    def __init__(self, cfg: "WheeledDriveActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

        ## Auto-detect drive type if not specified
        if self.cfg.drive_type is None:
            self.drive_type = self._autodetect_drive_type()
            logging.debug(f"Auto-detected drive type: {self.drive_type}")
        else:
            self.drive_type = self.cfg.drive_type

        ## Prepare wheel configuration
        self._setup_wheel_configuration()

    @property
    def action_dim(self) -> int:
        return 2  # linear_velocity, angular_velocity

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions = actions
        self._processed_actions = actions.clone()
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
        ## Calculate drive commands based on drive type
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

        ## Apply drive commands
        self._asset.set_joint_velocity_target(
            drive_velocities, joint_ids=self._drive_joint_indices
        )

        ## Apply steering commands
        if steer_angles is not None:
            self._asset.set_joint_position_target(
                steer_angles, joint_ids=self._steering_joint_indices
            )

    def reset(self, env_ids: Sequence[int] | None = None):
        super().reset(env_ids)

    def _skid_steer_drive(
        self, v_lin: torch.Tensor, v_ang: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor | None]:
        drive_velocities = torch.zeros(
            (self.num_envs, len(self._drive_wheels)), device=self.device
        )
        for i, wheel_idx in enumerate(self._drive_wheels):
            y_offset = self._wheel_positions[wheel_idx, 1]
            drive_velocities[:, i] = (v_lin + v_ang * y_offset) / self.cfg.wheel_radius

        return drive_velocities, None

    def _ackermann_drive(
        self, v_lin: torch.Tensor, v_ang: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor | None]:
        drive_velocities = torch.zeros(
            (self.num_envs, len(self._drive_wheels)), device=self.device
        )
        steer_angles = torch.zeros(
            (self.num_envs, len(self._steering_wheels)), device=self.device
        )

        ## Turning radius and direction
        turn_radius = torch.where(
            v_ang != 0.0,
            torch.abs(v_lin) / torch.abs(v_ang),
            torch.ones_like(v_lin) * 1.0e10,
        )
        turn_dir = torch.sign(v_ang)

        # Wheelbase parameters
        wheelbase_length = torch.tensor(self.cfg.wheelbase[0], device=self.device)

        # Calculate steering angles for front wheels
        for i, wheel_idx in enumerate(self._steering_wheels):
            wheel_pos = self._wheel_positions[wheel_idx]
            # Ackermann geometry
            effective_radius = turn_radius - turn_dir * wheel_pos[1]
            angle = turn_dir * torch.atan2(wheelbase_length, effective_radius)
            # Apply steering limits
            angle = torch.clamp(
                angle, -self._steering_limits[i], self._steering_limits[i]
            )
            steer_angles[:, i] = angle

        # Calculate wheel velocities
        for i, wheel_idx in enumerate(self._drive_wheels):
            wheel_y = self._wheel_positions[wheel_idx, 1]
            # Path radius for this wheel
            wheel_path_radius = turn_radius - turn_dir * wheel_y
            # Velocity calculation
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
    ) -> Tuple[torch.Tensor, torch.Tensor | None]:
        drive_velocities = torch.zeros(
            (self.num_envs, len(self._drive_wheels)), device=self.device
        )
        steer_angles = torch.zeros(
            (self.num_envs, len(self._steering_wheels)), device=self.device
        )

        # Key parameters
        abs_v_lin = torch.abs(v_lin)
        abs_v_ang = torch.abs(v_ang)
        turn_dir = torch.sign(v_ang)
        drive_dir = torch.sign(v_lin)
        drive_dir = torch.where(drive_dir == 0, torch.ones_like(drive_dir), drive_dir)

        # Wheelbase parameters
        half_width = torch.tensor(self.cfg.wheelbase[1] / 2, device=self.device)

        # Calculate turning radius
        turning_radius = torch.where(
            abs_v_ang > 0,
            abs_v_lin / abs_v_ang,
            torch.tensor(1.0e10, device=self.device),
        )

        # Detect point turn mode (tight radius)
        is_point_turn = turning_radius == 0.0

        # Calculate wheel path radii
        r_left = turning_radius - half_width * turn_dir
        r_right = turning_radius + half_width * turn_dir

        # Calculate steering angles for corner wheels
        for i, idx in enumerate(self._steering_wheels):
            wheel_pos = self._wheel_positions[idx]
            is_front = wheel_pos[0] > 0
            is_left = wheel_pos[1] < 0

            # Point turn: wheels turn inward/outward
            point_angle_val = 0.78539816 * (-1 if (is_front == is_left) else 1)

            # Normal driving: Reduce steering angle for rover mode
            # Calculate the base angle but scale it down to prevent extreme angles
            base_angle = torch.atan2(half_width, r_left if is_left else r_right)
            # Apply scaling factor to reduce steering angle (0.3 = ~30% of the original angle)
            scaled_angle = base_angle * 0.3

            # Front wheels turn in direction of turn, rear wheels in opposite direction
            # This is critical for proper rover steering geometry
            normal_angle = turn_dir * scaled_angle * (1 if is_front else -1)

            # Select angle based on mode
            steer_angles[:, i] = torch.where(
                is_point_turn,
                point_angle_val,
                normal_angle,
            )

        # Calculate wheel velocities
        for i, wheel_idx in enumerate(self._drive_wheels):
            wheel_pos = self._wheel_positions[wheel_idx]
            is_left = wheel_pos[1] < 0

            # Directional factor for point turns
            side_factor = -1.0 if is_left else 1.0

            # Velocity profiles for different modes
            point_turn_vel = side_factor * turn_dir * abs_v_ang
            normal_vel = drive_dir * torch.where(
                abs_v_ang == 0,
                abs_v_lin,  # Straight motion
                abs_v_ang,  # Normal turning
            )

            # Final wheel velocity
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
        # Get joint indices
        assert len(self.cfg.drive_joint_names) > 0, "No drive joints specified"
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

        # Determine number of wheels from drive joints
        num_wheels = len(self._drive_joint_indices)

        # Generate positions based on wheelbase dimensions
        positions = self._generate_wheel_positions(num_wheels)
        logging.debug(f"Generated wheel positions for {num_wheels} wheels")

        # Store wheel positions as tensor
        self._wheel_positions = torch.tensor(positions, device=self.device)

        # All wheels are drive wheels
        self._drive_wheels = list(range(num_wheels))

        # Determine which wheels have steering capability
        if not self.cfg.steering_joint_names:
            self._steering_wheels = []
        else:
            # Map steering joints to corresponding drive wheels
            self._steering_wheels = self._map_steering_to_drive_wheels(positions)

        # Set steering limits based on drive type
        # More angle for point turns, less for normal driving
        if self.drive_type == DriveType.ROVER:
            steering_limit = deg_to_rad(45)
        else:
            steering_limit = deg_to_rad(35)

        self._steering_limits = (
            torch.ones(len(self._steering_wheels), device=self.device) * steering_limit
        )

        # Set a separate limit for normal driving in rover mode (used in _rover_drive)
        if self.drive_type == DriveType.ROVER:
            self._normal_steering_limit = deg_to_rad(22.5)

    def _generate_wheel_positions(self, num_wheels):
        half_length = self.cfg.wheelbase[0] / 2
        half_width = self.cfg.wheelbase[1] / 2

        if num_wheels == 2:
            # Simple 2-wheel differential drive
            return [
                (0, -half_width, 0),  # Left
                (0, half_width, 0),  # Right
            ]
        elif num_wheels == 4:
            # 4-wheel layout (common car-like or differential)
            return [
                (half_length, -half_width, 0),  # Front left
                (half_length, half_width, 0),  # Front right
                (-half_length, -half_width, 0),  # Rear left
                (-half_length, half_width, 0),  # Rear right
            ]
        elif num_wheels == 6:
            # 6-wheel rover layout
            mid_width = (self.cfg.wheelbase_mid or self.cfg.wheelbase[1]) / 2
            return [
                (half_length, -half_width, 0),  # Front left
                (half_length, half_width, 0),  # Front right
                (0, -mid_width, 0),  # Middle left
                (0, mid_width, 0),  # Middle right
                (-half_length, -half_width, 0),  # Rear left
                (-half_length, half_width, 0),  # Rear right
            ]
        else:
            # For other wheel counts, distribute evenly
            positions = []
            if num_wheels % 2 == 0:
                # Even number of wheels - arrange symmetrically in pairs
                pairs = num_wheels // 2
                for i in range(pairs):
                    x = half_length - i * (2 * half_length) / (
                        pairs - 1 if pairs > 1 else 1
                    )
                    positions.append((x, -half_width, 0))  # Left
                    positions.append((x, half_width, 0))  # Right
            else:
                # Odd number of wheels - include a center wheel
                positions.append((0, 0, 0))  # Center wheel
                pairs = (num_wheels - 1) // 2
                for i in range(pairs):
                    x_offset = (i + 1) * half_length / pairs
                    positions.append((x_offset, -half_width, 0))  # Front left
                    positions.append((x_offset, half_width, 0))  # Front right
                    positions.append((-x_offset, -half_width, 0))  # Rear left
                    positions.append((-x_offset, half_width, 0))  # Rear right

            return positions[
                :num_wheels
            ]  # Ensure we return exactly num_wheels positions

    def _map_steering_to_drive_wheels(self, drive_positions):
        steering_wheels = []

        # Use heuristics based on drive type
        if self.drive_type == DriveType.ACKERMANN:
            # For Ackermann, assume front wheels steer
            # Find the wheels with largest x coordinate (assuming x is forward)
            x_coords = [pos[0] for pos in drive_positions]
            front_indices = sorted(range(len(x_coords)), key=lambda i: -x_coords[i])
            steering_wheels = front_indices[: len(self.cfg.steering_joint_names)]
        elif self.drive_type == DriveType.ROVER:
            # For rover, assume corner wheels steer
            if len(drive_positions) >= 6:
                # Typical 6-wheel rover has corners at 0, 1, 4, 5
                steering_wheels = [0, 1, 4, 5]
            else:
                # For other configurations, find the corner wheels
                x_coords = [pos[0] for pos in drive_positions]
                y_coords = [pos[1] for pos in drive_positions]

                # Find min/max x and y to identify corners
                min_x, max_x = min(x_coords), max(x_coords)
                min_y, max_y = min(y_coords), max(y_coords)

                for i, (x, y, _) in enumerate(drive_positions):
                    # If close to extreme x and y values, consider it a corner
                    if (abs(x - min_x) < 0.1 or abs(x - max_x) < 0.1) and (
                        abs(y - min_y) < 0.1 or abs(y - max_y) < 0.1
                    ):
                        steering_wheels.append(i)

        # Ensure we don't have more steering wheels than steering joints
        steering_wheels = steering_wheels[: len(self.cfg.steering_joint_names)]

        return steering_wheels


class DriveType(Enum):
    SKID_STEER = auto()
    ACKERMANN = auto()
    ROVER = auto()


class WheelConfig(BaseModel):
    position: Tuple[float, float, float]
    radius: float
    drive_joint_name: str
    steering_joint_name: Optional[str] = None
    steering_limit: Optional[float] = None


@configclass
class WheeledDriveActionCfg(ActionTermCfg):
    class_type: Type = WheeledDriveAction

    # Scaling factors
    scale_linear: float = 1.0
    scale_angular: float = 1.0

    # Drive configuration (auto-detected if None)
    drive_type: Optional[DriveType] = None

    # Joint configuration
    drive_joint_names: List[str] = []
    steering_joint_names: List[str] = []

    # Vehicle parameters
    wheelbase: Tuple[float, float] = (0.8, 0.6)  # (length, width)
    wheel_radius: float = 0.1
    wheelbase_mid: Optional[float] = None  # Middle wheels width (for rovers)

    # Velocity limits
    max_linear_velocity: Optional[float] = None
    max_angular_velocity: Optional[float] = None
