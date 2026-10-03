import math
from typing import TYPE_CHECKING, Sequence

import torch

try:
    import warp as wp

    def _to_warp(t: torch.Tensor) -> "wp.array":
        return wp.from_torch(t.contiguous())

except ImportError:
    wp = None  # type: ignore[assignment]

    def _to_warp(t: torch.Tensor):  # type: ignore[misc]
        return t


from srb.core.manager import ActionTerm, ActionTermCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.sim import ArrowCfg, PreviewSurfaceCfg, SpawnerCfg
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
    from srb.core.asset import Articulation, RigidObject


def _host_indices(indices):
    """Return asset indices on the host for the PhysX mass/inertia setters.

    Isaac Lab keeps ``_ALL_INDICES`` as a Warp array on the simulation
    device; PhysX rejects a CUDA index tensor there just as it rejects CUDA
    values ("Incompatible device of index tensor in function setMasses").
    """
    if isinstance(indices, torch.Tensor):
        return _to_warp(indices.cpu())
    if wp is not None and isinstance(indices, wp.array):
        return _to_warp(wp.to_torch(indices).cpu())
    return indices


class ThrustAction(ActionTerm):
    cfg: "ThrustActionCfg"
    _env: "AnyEnv"
    _asset: "RigidObject | Articulation"

    def __init__(self, cfg: "ThrustActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore[misc]
        )

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
            if not math.isfinite(direction_norm) or direction_norm <= 0:
                raise ValueError(
                    "Thruster direction must have a finite non-zero magnitude"
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

        self._thruster_body_indices: list[int | None] = []
        for thruster_cfg in cfg.thrusters:
            if thruster_cfg.link_name:
                body_ids, body_names = self._asset.find_bodies(thruster_cfg.link_name)
                if len(body_ids) != 1:
                    raise ValueError(
                        f"Expected one match for thruster link '{thruster_cfg.link_name}'. "
                        f"Found {len(body_ids)}: {body_names}"
                    )
                self._thruster_body_indices.append(body_ids[0])
            else:
                self._thruster_body_indices.append(None)
        self._num_thrusters = len(cfg.thrusters)
        self._action_indices_thrust = torch.arange(
            self._num_thrusters, device=env.device
        )
        self._num_thrusters_with_gimbal = len(
            [limits for limits in thruster_gimbal_limits if limits != (0.0, 0.0)]
        )
        action_indices_gimbal: Sequence[tuple[int, int]] = []
        gimbal_action_offset = 0
        for limits in thruster_gimbal_limits:
            if limits == (0.0, 0.0):
                action_indices_gimbal.append((-1, -1))
            else:
                start_idx = self._num_thrusters + 2 * gimbal_action_offset
                action_indices_gimbal.append((start_idx, start_idx + 1))
                gimbal_action_offset += 1
        # Gimballed thrusters and their (x, y) action columns, resolved once on
        # the host so apply_actions() never branches on a device tensor.
        gimbal_thrusters = [
            i for i, limits in enumerate(thruster_gimbal_limits) if limits != (0.0, 0.0)
        ]
        self._num_gimbal_thrusters = len(gimbal_thrusters)
        self._gimbal_thruster_ids = torch.tensor(
            gimbal_thrusters, dtype=torch.long, device=env.device
        )
        self._gimbal_action_cols_x = torch.tensor(
            [action_indices_gimbal[i][0] for i in gimbal_thrusters],
            dtype=torch.long,
            device=env.device,
        )
        self._gimbal_action_cols_y = torch.tensor(
            [action_indices_gimbal[i][1] for i in gimbal_thrusters],
            dtype=torch.long,
            device=env.device,
        )

        self._remaining_fuel = cfg.fuel_capacity * torch.ones(
            env.num_envs, device=env.device
        )
        _masses = self._asset.root_view.get_masses()
        self._dry_masses = (
            _masses.to(env.device).clone()
            if isinstance(_masses, torch.Tensor)
            else torch.as_tensor(_masses, dtype=torch.float32, device=env.device)
        )
        self._num_bodies = self._dry_masses.shape[-1]
        self._fuel_body_idx = next(
            (idx for idx in self._thruster_body_indices if idx is not None), 0
        )
        # Fuel level last written into the PhysX masses; None forces the first
        # synchronisation.
        self._mass_synced_fuel: torch.Tensor | None = None

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
        # Fuel burns per physics substep in apply_actions(), but the PhysX mass
        # and inertia round trips run here, once per env step, and only when
        # the fuel level changed since the last write.
        self._sync_fuel_mass()

    def apply_actions(self):
        thruster_directions = (
            self._thruster_direction.unsqueeze(0).expand(self.num_envs, -1, -1).clone()
        )
        if self._num_gimbal_thrusters:
            limits = self._thruster_gimbal_limits[self._gimbal_thruster_ids]
            gimbal_x = limits[:, 0] * self._processed_actions[
                :, self._gimbal_action_cols_x
            ].clamp(-1.0, 1.0)
            gimbal_y = limits[:, 1] * self._processed_actions[
                :, self._gimbal_action_cols_y
            ].clamp(-1.0, 1.0)

            # Convert gimbal angles to rotation matrices: (num_envs * G, 3, 3)
            euler_angles = torch.stack(
                (gimbal_x, gimbal_y, torch.zeros_like(gimbal_x)), dim=-1
            ).reshape(-1, 3)
            rotation_matrices = matrix_from_euler(euler_angles, convention="XYZ")

            nominal_dir = (
                self._thruster_direction[self._gimbal_thruster_ids]
                .unsqueeze(0)
                .expand(self.num_envs, -1, -1)
                .reshape(-1, 3, 1)
            )
            rotated_dir = torch.bmm(rotation_matrices, nominal_dir).reshape(
                self.num_envs, self._num_gimbal_thrusters, 3
            )
            thruster_directions[:, self._gimbal_thruster_ids, :] = rotated_dir

        thruster_offsets = self._thruster_offset.unsqueeze(0).expand(
            self.num_envs, -1, -1
        )

        thrust_actions = self._processed_actions[:, self._action_indices_thrust]
        thrust_magnitudes = thrust_actions * self._thruster_power.unsqueeze(0)

        thrust_magnitudes *= self._remaining_fuel.unsqueeze(-1) > 0.0

        thruster_forces = (
            -self.cfg.scale * thrust_magnitudes.unsqueeze(-1) * thruster_directions
            # / self._env.cfg.agent_rate
        )

        _coms = self._asset.root_view.get_coms()
        coms = (
            _coms.to(self.device)
            if isinstance(_coms, torch.Tensor)
            else torch.as_tensor(_coms, dtype=torch.float32, device=self.device)
        )
        if self._num_bodies > 1:
            # Articulation: accumulate per-body forces and torques in each body's local frame
            padded_forces = torch.zeros(
                self.num_envs, self._num_bodies, 3, device=self.device
            )
            padded_torques = torch.zeros(
                self.num_envs, self._num_bodies, 3, device=self.device
            )
            for i in range(self._num_thrusters):
                body_idx = (
                    self._thruster_body_indices[i]
                    if self._thruster_body_indices[i] is not None
                    else 0
                )
                body_com = coms[:, body_idx, :3]
                force_i = thruster_forces[:, i, :]
                offset_from_com = thruster_offsets[:, i, :] - body_com
                torque_i = torch.cross(offset_from_com, force_i, dim=1)
                padded_forces[:, body_idx, :] += force_i
                padded_torques[:, body_idx, :] += torque_i
            self._asset.root_view.apply_forces_and_torques_at_position(
                force_data=_to_warp(padded_forces.reshape(-1, 3)),
                torque_data=_to_warp(padded_torques.reshape(-1, 3)),
                position_data=None,
                indices=self._asset._ALL_INDICES,
                is_global=False,
            )
        else:
            # RigidObject: apply all forces/torques to root body
            com_positions = coms[:, :3].unsqueeze(1)
            thruster_offsets_com = thruster_offsets - com_positions
            thruster_torques = torch.cross(thruster_offsets_com, thruster_forces, dim=2)
            self._asset.root_view.apply_forces_and_torques_at_position(
                force_data=_to_warp(thruster_forces.sum(dim=1)),
                torque_data=_to_warp(thruster_torques.sum(dim=1)),
                position_data=None,
                indices=self._asset._ALL_INDICES,
                is_global=False,
            )

        # apply_actions() runs once per physics substep (`decimation` times per
        # env step), so the burn must use the substep duration (physics_dt),
        # not the full env-step duration (agent_rate) -- otherwise fuel/mass
        # deplete `decimation`x too fast whenever decimation > 1. Burn follows
        # the applied thrust, including ``scale``.
        self._remaining_fuel -= (
            self.cfg.fuel_consumption_rate
            * thrust_magnitudes.sum(dim=1)
            * abs(self.cfg.scale)
            * self._env.physics_dt
        )
        self._remaining_fuel.clamp_(min=0.0)

        if self.cfg.debug_vis:
            self._update_visualization_markers(
                thruster_offsets=thruster_offsets,
                thruster_directions=thruster_directions,
                thrust_magnitudes=thrust_magnitudes,
            )

    def _sync_fuel_mass(self) -> None:
        """Write dry mass plus fuel to PhysX when changed; avoid per-substep writes.
        Mass lags fuel burn by at most one environment step.
        """
        if self._mass_synced_fuel is not None and torch.equal(
            self._mass_synced_fuel, self._remaining_fuel
        ):
            return
        self._mass_synced_fuel = self._remaining_fuel.clone()
        new_masses = self._dry_masses.clone()
        new_masses[:, self._fuel_body_idx] += self._remaining_fuel
        _cm = self._asset.root_view.get_masses()
        current_masses = (
            _cm.to(self.device)
            if isinstance(_cm, torch.Tensor)
            else torch.as_tensor(_cm, dtype=torch.float32, device=self.device)
        )
        # A massless body would turn its inertia into NaN; keep it unscaled.
        mass_ratio = torch.where(
            current_masses > 0.0,
            new_masses / current_masses,
            torch.ones_like(new_masses),
        )
        # The PhysX tensor API keeps masses and inertias on the host even when
        # the simulation runs on the GPU: setMasses/setInertias reject a CUDA
        # tensor ("Incompatible device of mass tensor ... expected device -1"),
        # so values and indices are moved to the CPU before the write.
        _all_idx_warp = _host_indices(self._asset._ALL_INDICES)
        self._asset.root_view.set_masses(
            _to_warp(new_masses.cpu()), indices=_all_idx_warp
        )
        _ci = self._asset.root_view.get_inertias()
        current_inertias = (
            _ci.to(self.device)
            if isinstance(_ci, torch.Tensor)
            else torch.as_tensor(_ci, dtype=torch.float32, device=self.device)
        )
        if current_inertias.ndim == 3:
            # Articulation: (num_envs, num_bodies, 9)
            new_inertias = mass_ratio.unsqueeze(-1) * current_inertias
        else:
            # RigidObject: (num_envs, 9)
            new_inertias = mass_ratio * current_inertias
        self._asset.root_view.set_inertias(
            _to_warp(new_inertias.cpu()), indices=_all_idx_warp
        )

    def _setup_visualization_markers(self):
        self._thruster_markers = []
        for i in range(self._num_thrusters):
            thruster_cfg = self.cfg.thrusters[i]

            if thruster_cfg.marker_cfg is not None:
                # Use custom marker configuration if provided
                cfg = VisualizationMarkersCfg(
                    prim_path=f"/Visuals/thrusters/thruster{i}",
                    markers={"marker": thruster_cfg.marker_cfg},
                )
            else:
                # Default to arrow marker with automatic scaling and coloring
                marker_scale = 0.1 * self._thruster_power.max().sqrt().item()

                # Use a different color for each thruster (gradient from red to blue)
                blue = i / max(self._num_thrusters - 1, 1)

                cfg = VisualizationMarkersCfg(
                    prim_path=f"/Visuals/thrusters/thruster{i}",
                    markers={
                        "marker": ArrowCfg(
                            tail_radius=marker_scale * 0.1,
                            tail_length=marker_scale * 1.0,
                            head_radius=marker_scale * 0.2,
                            head_length=marker_scale * 0.5,
                            visual_material=PreviewSurfaceCfg(
                                emissive_color=(1.0 - blue, 0.2, blue)
                            ),
                        )
                    },
                )

            # Create the marker and add to list
            self._thruster_markers.append(VisualizationMarkers(cfg))

    def _update_visualization_markers(
        self,
        thruster_offsets: torch.Tensor,
        thruster_directions: torch.Tensor,
        thrust_magnitudes: torch.Tensor,
    ):
        for i in range(self._num_thrusters):
            # Get the world pose of the body this thruster is attached to
            body_idx = self._thruster_body_indices[i]
            if body_idx is not None and self._num_bodies > 1:
                body_pos_w = self._asset.data.body_pos_w[:, body_idx]
                body_quat_w = self._asset.data.body_quat_w[:, body_idx]
            else:
                body_pos_w = self._asset.data.root_pos_w
                body_quat_w = self._asset.data.root_quat_w

            # Transform thruster position to world frame
            thruster_pos_w, _ = combine_frame_transforms(
                t01=body_pos_w,
                q01=body_quat_w,
                t12=thruster_offsets[:, i, :],
            )

            # Orient the marker with the thrust vector
            thrust_dir_world = quat_apply(body_quat_w, thruster_directions[:, i, :])
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


@configclass
class ThrusterCfg:
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    direction: tuple[float, float, float] = (0.0, 0.0, -1.0)
    gimbal_limits: tuple[float, float] | None = None
    power: float = 1.0
    marker_cfg: SpawnerCfg | None = None
    link_name: str = ""


@configclass
class ThrustActionCfg(ActionTermCfg):
    class_type: type = ThrustAction
    scale: float = 1.0

    thrusters: Sequence[ThrusterCfg] = (ThrusterCfg(),)
    fuel_capacity: float = 1.0
    fuel_consumption_rate: float = 0.1

    def __post_init__(self) -> None:
        # Fuel observations and rewards divide by the capacity.
        if not self.fuel_capacity > 0.0 or self.fuel_capacity == float("inf"):
            raise ValueError(
                f"fuel_capacity must be positive, got {self.fuel_capacity}"
            )
