from typing import TYPE_CHECKING, Sequence, Tuple, Type

import torch
from pydantic import BaseModel

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.core.marker import ARROW_CFG, VisualizationMarkers
from srb.core.sim import PreviewSurfaceCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_euler,
    normalize,
    quat_apply,
    quat_from_matrix,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import RigidObject


class ThrustAction(ActionTerm):
    cfg: "ThrustActionCfg"
    _env: "AnyEnv"
    _asset: "RigidObject"

    def __init__(self, cfg: "ThrustActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

        ## Pre-process thrusters
        thruster_offset = []
        thruster_direction = []
        thruster_power = []
        thruster_gimbal_limits = []
        for thruster_cfg in cfg.thrusters:
            thruster_offset.append(thruster_cfg.offset)
            direction_norm = (
                thruster_cfg.direction[0] ** 2
                + thruster_cfg.direction[1] ** 2
                + thruster_cfg.direction[2] ** 2
            ) ** 0.5
            assert direction_norm > 0, (
                "Thruster direction must have a non-zero magnitude"
            )
            direction = (
                thruster_cfg.direction[0] / direction_norm,
                thruster_cfg.direction[1] / direction_norm,
                thruster_cfg.direction[2] / direction_norm,
            )
            thruster_direction.append(direction)
            thruster_gimbal_limits.append(thruster_cfg.gimbal_limits or (0.0, 0.0))
            thruster_power.append(thruster_cfg.power)
        self._thruster_offset = torch.tensor(thruster_offset, device=env.device)
        self._thruster_direction = torch.tensor(thruster_direction, device=env.device)
        self._thruster_power = torch.tensor(thruster_power, device=env.device)
        self._thruster_gimbal_limits = torch.tensor(
            thruster_gimbal_limits, device=env.device
        )

        ## Set up action indices
        self._num_thrusters = len(cfg.thrusters)
        self._action_indices_thrust = torch.arange(
            self._num_thrusters, device=env.device
        )
        self._num_thrusters_with_gimbal = len(
            [limits for limits in thruster_gimbal_limits if limits != (0.0, 0.0)]
        )
        action_indices_gimbal: Sequence[Tuple[int, int]] = []
        for i, limits in enumerate(thruster_gimbal_limits):
            if limits == (0.0, 0.0):
                action_indices_gimbal.append((-1, -1))
            else:
                start_idx = self._num_thrusters + 2 * i
                action_indices_gimbal.append((start_idx, start_idx + 1))
        self._action_indices_gimbal = torch.tensor(
            action_indices_gimbal, device=env.device
        )

        ## Initialize fuel & mass
        self._remaining_fuel = cfg.fuel_capacity * torch.ones(
            env.num_envs, device=env.device
        )
        self._dry_masses = self._asset.root_physx_view.get_masses().clone()

        ## Set up visualization markers
        if self.cfg.debug_vis:
            self._setup_visualization_markers()

    @property
    def action_dim(self) -> int:
        return self._num_thrusters + 2 * self._num_thrusters_with_gimbal

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @property
    def remaining_fuel(self) -> torch.Tensor:
        return self._remaining_fuel

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = actions.clone()
        self._processed_actions[:, self._action_indices_thrust] = torch.clamp(
            self._processed_actions[:, self._action_indices_thrust], 0.0, 1.0
        )

    def apply_actions(self):
        ## Apply gimbal rotations to thruster directions
        thruster_directions = (
            self._thruster_direction.unsqueeze(0).expand(self.num_envs, -1, -1).clone()
        )
        for i in range(self._num_thrusters):
            gimbal_i = self._action_indices_gimbal[i]

            # Skip thrusters without gimbal
            if gimbal_i[0] < 0:
                continue

            # Extract gimbal actions and map them to limits
            lim = self._thruster_gimbal_limits[i]
            gimbal_x = lim[0] * self._processed_actions[:, gimbal_i[0]].clamp(-1.0, 1.0)
            gimbal_y = lim[1] * self._processed_actions[:, gimbal_i[1]].clamp(-1.0, 1.0)

            # Convert gimbal angles to rotation matrices
            euler_angles = torch.zeros((self.num_envs, 3), device=self.device)
            euler_angles[:, 0] = gimbal_x
            euler_angles[:, 1] = gimbal_y
            rotation_matrices = matrix_from_euler(euler_angles, convention="XYZ")

            # Rotate the direction vectors
            nominal_dir = (
                self._thruster_direction[i].unsqueeze(0).expand(self.num_envs, -1)
            )
            rotated_dir = torch.bmm(
                rotation_matrices, nominal_dir.unsqueeze(2)
            ).squeeze(2)

            # Update the direction for this thruster
            thruster_directions[:, i, :] = rotated_dir

        ## Compute thrust force magnitude for each thruster
        thrust_actions = self._processed_actions[:, self._action_indices_thrust]
        thrust_magnitudes = thrust_actions * self._thruster_power.unsqueeze(0)

        ## Disable thrust if fuel is depleted
        thrust_magnitudes *= self._remaining_fuel.unsqueeze(-1) > 0.0

        ## Compute force vector for each thruster
        # Note: The force is applied in the opposite direction of the thrust vector
        thruster_forces = (
            -self.cfg.scale * thrust_magnitudes.unsqueeze(-1) * thruster_directions
            # / self._env.cfg.agent_rate
        )

        ## Get center of mass positions [num_envs, 3]
        thruster_offsets = self._thruster_offset.unsqueeze(0).expand(
            self.num_envs, -1, -1
        )
        com_positions = self._asset.root_physx_view.get_coms()[:, :3].unsqueeze(1)
        thruster_offsets_com = thruster_offsets - com_positions

        ## Calculate torques resulting from thruster forces
        thruster_torques = torch.cross(thruster_offsets_com, thruster_forces, dim=2)

        ## Apply forces and torques at center of mass in the local frame
        self._asset.root_physx_view.apply_forces_and_torques_at_position(
            force_data=thruster_forces.sum(dim=1),
            torque_data=thruster_torques.sum(dim=1),
            position_data=com_positions,
            indices=self._asset._ALL_INDICES,
            is_global=False,
        )

        ## Update fuel and mass
        self._remaining_fuel -= (
            self.cfg.fuel_consumption_rate
            * thrust_magnitudes.sum(dim=1)
            * self._env.cfg.agent_rate
        )
        self._remaining_fuel.clamp_(min=0.0)
        masses = self._dry_masses + self._remaining_fuel.unsqueeze(-1)
        mass_decrease_ratio = masses / self._asset.root_physx_view.get_masses()
        self._asset.root_physx_view.set_masses(masses, indices=self._asset._ALL_INDICES)
        self._asset.root_physx_view.set_inertias(
            mass_decrease_ratio * self._asset.root_physx_view.get_inertias(),
            indices=self._asset._ALL_INDICES,
        )

        ## Update visualization markers
        if self.cfg.debug_vis:
            self._update_visualization_markers(
                thruster_offsets=thruster_offsets,
                thruster_directions=thruster_directions,
                thrust_magnitudes=thrust_magnitudes,
            )

    def _setup_visualization_markers(self):
        # TODO[low]: Support custom visualization markers for thrusters
        self._thruster_markers = []
        for i in range(self._num_thrusters):
            cfg = ARROW_CFG.copy().replace(  # type: ignore
                prim_path=f"/Visuals/thrusters/thruster{i}"
            )
            cfg.markers["arrow"].tail_radius = 0.25 * 0.1
            cfg.markers["arrow"].tail_length = 0.25 * 1.0
            cfg.markers["arrow"].head_radius = 0.25 * 0.2
            cfg.markers["arrow"].head_length = 0.25 * 0.5

            # Use a different color for each thruster (gradient from red to blue)
            blue = i / max(self._num_thrusters - 1, 1)
            cfg.markers["arrow"].visual_material = PreviewSurfaceCfg(
                emissive_color=(1.0 - blue, 0.2, blue)
            )

            # Create the marker and add to list
            self._thruster_markers.append(VisualizationMarkers(cfg))

    def _update_visualization_markers(
        self,
        thruster_offsets: torch.Tensor,
        thruster_directions: torch.Tensor,
        thrust_magnitudes: torch.Tensor,
    ):
        asset_pos = self._asset.data.root_pos_w
        asset_quat = self._asset.data.root_quat_w

        for i in range(self._num_thrusters):
            # Transform thruster position to world frame
            thruster_pos_w, _ = combine_frame_transforms(
                t01=asset_pos,
                q01=asset_quat,
                t12=thruster_offsets[:, i, :],
            )

            # Orient the marker with the thrust vector
            thrust_dir_world = quat_apply(asset_quat, thruster_directions[:, i, :])
            thrust_dir_world = normalize(thrust_dir_world)

            # Create rotation matrix where x-axis is aligned with thrust direction
            x_axis = thrust_dir_world

            # Choose any perpendicular vector for y-axis
            y_axis = torch.zeros_like(x_axis)
            # Find index of smallest component in x_axis to create orthogonal vector
            min_idx = torch.argmin(torch.abs(x_axis), dim=1)
            for env_idx in range(self.num_envs):
                y_axis[env_idx, min_idx[env_idx]] = 1.0

            # Make y_axis perpendicular to x_axis
            y_axis = normalize(
                y_axis - x_axis * torch.sum(x_axis * y_axis, dim=1, keepdim=True)
            )

            # Get z_axis from cross product
            z_axis = normalize(torch.cross(x_axis, y_axis, dim=1))

            # Create rotation matrix
            rot_matrix = torch.zeros((self.num_envs, 3, 3), device=self.device)
            rot_matrix[:, :, 0] = x_axis
            rot_matrix[:, :, 1] = y_axis
            rot_matrix[:, :, 2] = z_axis

            # Convert to quaternion
            thrust_dir_quat_w = quat_from_matrix(rot_matrix)

            # Scale the marker based on the thrust magnitude
            marker_scale = torch.ones((self.num_envs, 3), device=self.device)
            marker_scale[:, :] = (
                thrust_magnitudes[:, i] * (1.0 / self._thruster_power.max().item())
            ).unsqueeze(1)

            # Visualize the marker
            self._thruster_markers[i].visualize(
                thruster_pos_w, thrust_dir_quat_w, marker_scale
            )

    def reset(self, env_ids: Sequence[int] | None = None):
        super().reset(env_ids)
        self._remaining_fuel[env_ids] = self.cfg.fuel_capacity


class ThrusterCfg(BaseModel):
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    direction: Tuple[float, float, float] = (0.0, 0.0, -1.0)
    gimbal_limits: Tuple[float, float] | None = None
    power: float = 1.0


@configclass
class ThrustActionCfg(ActionTermCfg):
    class_type: Type = ThrustAction
    scale: float = 1.0

    thrusters: Sequence[ThrusterCfg] = (ThrusterCfg(),)
    fuel_capacity: float = 1.0
    fuel_consumption_rate: float = 0.1
