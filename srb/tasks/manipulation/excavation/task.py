"""Single-scoop excavation task for a Husky, Kinova arm, and scoop."""

from __future__ import annotations

import logging as _logging
import math
from dataclasses import dataclass
from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.assets.object.tool.scoop import ScoopCustom3
from srb.assets.robot.manipulation.kinova import KinovaGen3n7
from srb.assets.robot.mobile.ground.husky import Husky
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.env.manipulation import ManipulationRewardCfg
from srb.core.env.runtime import episode_outcome_info
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import (
    randomize_joint_parameters,
    randomize_particle_material_uniform,
    randomize_rigid_body_mass,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    GaussianRewardTerm,
    GaussianTrackingRewardTerm,
    HyperbolicRewardTerm,
    LinearRewardTerm,
    RewardNormalization,
    ScaledRewardTerm,
)
from srb.core.reward.normalize import RewardNormalizer, TukeyRewardTerm
from srb.core.scene_introspection import find_scene_sensor
from srb.core.sensor import Imu, ImuCfg
from srb.core.sensor.category import SensorCategory
from srb.core.sensor.heightmap import bin_points_to_heightmap_grid
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d, scale_transform

from .event_camera_wiring import build_wrist_event_camera_cfg
from .excavation_command import (
    DEFAULT_SCOOP_TARGET_FRAME,
    DEFAULT_SCOOP_TARGET_XY_M,
    DEFAULT_SCOOP_TARGET_YAW_RAD,
    DEFAULT_TARGET_WORKSPACE_BOUNDS_M,
    ScoopTargetCommand,
    ScoopTargetCommandLock,
    validate_scoop_target_command,
)
from .excavation_utils import (
    DEFAULT_SCOOP_TILT_RAD,
    compute_scoop_capacity,
    mask_lift_reward,
    particle_mean_pos_relative_base,
    rot6d_target_forward_tilt,
)
from .heightmap_spec import EXCAVATION_HEIGHTMAP_SPEC as _EXCAVATION_HEIGHTMAP_SPEC
from .heightmap_spec import (
    heightmap_spec_for_target_xy as _heightmap_spec_for_target_xy,
)

try:
    from srb.core.sensor.heightmap import HeightmapSensor

    _HAS_HEIGHTMAP = True
except (ImportError, AttributeError):
    _HAS_HEIGHTMAP = False

_log = _logging.getLogger(__name__)


_PILE_FORWARD_M: float = float(DEFAULT_SCOOP_TARGET_XY_M[0])


@dataclass(eq=False)
class _PerEnvScoopTargetLock(ScoopTargetCommandLock):
    """Adopt targets at reset while preserving active episodes."""

    retarget_on_reset: bool = False

    def apply_pending(self) -> bool:
        if super().apply_pending():
            self.retarget_on_reset = True
        return self.retarget_on_reset


class RewardCfg(ManipulationRewardCfg):
    """Excavation shaping, smoothness, and terminal rewards."""

    # Task rewards are already scaled; NONE prevents a second runtime normalization
    # that would crush the signal.
    normalize: RewardNormalization = RewardNormalization.NONE

    reward_approach_pile: GaussianRewardTerm = GaussianRewardTerm(
        weight=1.5, scale=0.15
    )
    """Gaussian reward for EE XY distance to the commanded scoop target."""

    reward_scoop_depth: TukeyRewardTerm = TukeyRewardTerm(
        weight=2.0, sweet_spot_m=0.02, tolerance_m=0.07
    )
    """Flat-top reward for EE depth relative to pile surface (±2 cm sweet spot)."""

    reward_scoop_orientation: GaussianTrackingRewardTerm = GaussianTrackingRewardTerm(
        weight=1.0,
        scale=0.26,
    )
    """Rewards the forward-scoop tilt orientation (rot6d tracking)."""

    reward_lift: HyperbolicRewardTerm = HyperbolicRewardTerm(weight=2.0, scale=0.2)
    """Rewards EE height above pile-surface maximum (encourage lifting)."""

    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-0.05, limit=4.0)
    penalty_jerk: LinearRewardTerm = LinearRewardTerm(weight=-0.02, limit=2.0)
    penalty_joint_torque: LinearRewardTerm = LinearRewardTerm(weight=-1e-5, limit=1.0e5)
    penalty_joint_acc: LinearRewardTerm = LinearRewardTerm(weight=-1e-6, limit=1.0e6)
    penalty_undesired_contacts: LinearRewardTerm = LinearRewardTerm(
        weight=-0.5, limit=200.0
    )

    penalty_joint_acceleration: LinearRewardTerm = LinearRewardTerm(
        weight=0.0, limit=1.0
    )  # type: ignore[assignment]
    penalty_undesired_robot_contacts: LinearRewardTerm = LinearRewardTerm(
        weight=0.0, limit=1.0
    )  # type: ignore[assignment]

    terminal_particle_count_scaled: ScaledRewardTerm = ScaledRewardTerm(
        weight=10.0, limit=10.0
    )
    terminal_success_bonus: ScaledRewardTerm = ScaledRewardTerm(weight=50.0, limit=50.0)


@configclass
class SceneCfg(ManipulationSceneCfg):
    """Regolith pile, heightmap, and contact sensor scene."""

    env_spacing: float = 6.0


@configclass
class EventCfg(ManipulationEventCfg):
    """Domain randomization for robot and regolith physics."""

    randomize_robot_mass: SrbEventTermCfg = SrbEventTermCfg(
        func=randomize_rigid_body_mass,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "mass_distribution_params": (0.9, 1.1),
            "operation": "scale",
        },
    )

    randomize_kinova_joint_friction: SrbEventTermCfg = SrbEventTermCfg(
        func=randomize_joint_parameters,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "friction_distribution_params": (0.7, 1.3),
            "operation": "abs",
        },
    )

    randomize_regolith_material: SrbEventTermCfg = SrbEventTermCfg(
        func=randomize_particle_material_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "friction_range": (0.6, 1.0),
            "density_range": (2700.0, 3100.0),
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    """Simulation configuration for the excavation task."""

    robot: KinovaGen3n7 = KinovaGen3n7(  # type: ignore[assignment]
        end_effector=ScoopCustom3(),
        action_mode="osc_fixed",
    )

    pedestal: Husky = Husky()  # type: ignore[assignment]

    scene: SceneCfg = SceneCfg()
    events: EventCfg = EventCfg()
    reward: RewardCfg = RewardCfg()

    env_rate: float = 1.0 / 200.0
    episode_length_s: float = 8.0
    is_finite_horizon: bool = False

    scoop_target_xy_m: tuple[float, float] = DEFAULT_SCOOP_TARGET_XY_M
    scoop_target_yaw_rad: float = DEFAULT_SCOOP_TARGET_YAW_RAD
    scoop_target_frame: str = DEFAULT_SCOOP_TARGET_FRAME
    scoop_target_workspace_bounds_m: tuple[
        tuple[float, float],
        tuple[float, float],
    ] = DEFAULT_TARGET_WORKSPACE_BOUNDS_M

    particles: bool = True
    particles_systems: int = 1
    particles_spawner: str = "pyramid"
    particles_size: float = 0.007
    particles_ratio: float = 0.6
    particles_density: float = 2900.0
    particles_friction: float = 0.8
    particles_damping: float = 0.15
    particles_cohesion: float = 0.0
    particles_adhesion: float = 0.0
    particles_pile_center_jitter: float = 0.05
    particles_update_interval: float = 0.1

    expose_privileged: bool = False

    n_scoop_capacity: int = -1

    scoop_capture_half_extents_m: tuple[float, float, float] = (0.06, 0.04, 0.02)

    scoop_contact_radius_half_extents_m: tuple[float, float, float] = (
        0.10,
        0.06,
        0.05,
    )

    @property
    def n_success_scoop_particles(self) -> int:
        return int(0.6 * self.n_scoop_capacity)

    @n_success_scoop_particles.setter
    def n_success_scoop_particles(self, value: object) -> None:
        # IsaacLab's configclass re-assigns inherited properties while building
        # subclasses; accept that echo but reject real overrides.
        if value != self.n_success_scoop_particles:
            raise AttributeError(
                "n_success_scoop_particles is derived from n_scoop_capacity; "
                "set n_scoop_capacity instead"
            )

    def __post_init__(self):
        if hasattr(self.pedestal, "sensor_cfgs"):
            self.pedestal.sensor_cfgs = {}

        self._husky_chassis_top_z = 0.3684
        _init_pos = self.robot.asset_cfg.init_state.pos or (0.0, 0.0, 0.0)
        self.robot.asset_cfg.init_state.pos = (
            _init_pos[0],
            _init_pos[1],
            _init_pos[2] + self._husky_chassis_top_z,
        )

        if not (self.particles):
            raise ValueError(
                "Particles must be enabled for the excavation task; set env.particles=true"
            )

        _target = ScoopTargetCommand(
            xy_m=tuple(self.scoop_target_xy_m),
            yaw_rad=float(self.scoop_target_yaw_rad),
            frame=str(self.scoop_target_frame),
        )
        _target_report = validate_scoop_target_command(
            _target,
            workspace_bounds_m=self.scoop_target_workspace_bounds_m,
        )
        if not _target_report.valid:
            raise ValueError(
                "Invalid excavation scoop target: " + "; ".join(_target_report.errors)
            )

        _pile_width_m = 0.35
        _pile_height_m = 0.4
        self.particles_dim_xy = max(1, round(_pile_width_m / self.particles_size))
        self.particles_height = max(1, round(_pile_height_m / self.particles_size))
        self.particles_init_pos = (
            self.scoop_target_xy_m[0],
            self.scoop_target_xy_m[1],
            0.5 * self.particles_size,
        )

        super().__post_init__()

        _ped_pos = self.scene.pedestal.init_state.pos
        self.scene.pedestal.init_state.pos = (_ped_pos[0], _ped_pos[1], 0.0)

        if self.n_scoop_capacity <= 0:
            self.n_scoop_capacity = compute_scoop_capacity(
                self.scoop_capture_half_extents_m,
                self.particles_size,
            )

        if self.scene.num_envs == 1:
            # Use the pedestal’s rewritten path; stacked tasks place it outside
            # the environment namespace.
            self.env_sensor_cfgs["imu_pedestal"] = ImuCfg(
                prim_path=f"{self.scene.pedestal.prim_path}/chassis",
            )
        else:
            from srb.utils import logging

            logging.warning(
                "imu_pedestal is disabled for num_envs=%d; set env.num_envs=1 to "
                "enable it. Observations that read "
                "self._imu will report zeros.",
                self.scene.num_envs,
            )

        self.active_sensors.add(SensorCategory.TERRAIN)

        self._particle_update_interval_n_steps: int = max(
            1, round(self.particles_update_interval / self.agent_rate)
        )

        _log.info(
            "Excavation heightmap: no raycast sensor (PhysX particle sets are "
            "not raycastable on Isaac 6); the heightmap observation is derived "
            "by binning the regolith particle positions into the sensor grid."
        )

        self.env_sensor_cfgs["event_camera_wrist"] = build_wrist_event_camera_cfg(
            f"{{ENV_REGEX_NS}}/robot/{self._robot.frame_flange.prim_relpath}",
            preset_name=self.event_camera_preset,
            backend=self.event_camera_backend,
            sub_render_hz=self.event_camera_sub_render_hz,
            resolution=self.event_camera_resolution,
        )

        self._settle_down_particles()


class _TaskMixin:
    """Lifecycle-independent excavation behavior."""

    cfg: TaskCfg

    def _horizon_reached(self) -> torch.Tensor:
        """Truncate and score terminal rewards at the episode horizon."""
        if self.cfg.truncate_episodes:
            return self.episode_length_buf >= self.max_episode_length
        return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._heightmap: "HeightmapSensor | None" = find_scene_sensor(
            self.scene, "heightmap"
        )

        self._imu: Imu | None = find_scene_sensor(self.scene, "imu_pedestal")

        self._regolith_ps = self.get_particle_system(
            "particles0", fallback_to_first=True
        )
        if self._regolith_ps is None:
            _log.warning(
                "ExcavationTask: no particle system found. "
                "Particle-based reward terms will be disabled."
            )

        self._action_group = None
        for _, term in self.action_manager.iter_terms():
            if hasattr(term, "emitted_action"):
                self._action_group = term
                break
        if self._action_group is None:
            _log.warning(
                "ExcavationTask: no action group with .emitted_action found; "
                "prev_action observation will use action_manager.prev_action."
            )

        self._reward_normalizer = RewardNormalizer(
            max_episode_steps=int(self.max_episode_length),
            terms={
                "reward_approach_pile": cfg.reward.reward_approach_pile,
                "reward_scoop_depth": cfg.reward.reward_scoop_depth,
                "reward_scoop_orientation": cfg.reward.reward_scoop_orientation,
                "reward_lift": cfg.reward.reward_lift,
                "penalty_action_rate": cfg.reward.penalty_action_rate,
                "penalty_jerk": cfg.reward.penalty_jerk,
                "penalty_joint_torque": cfg.reward.penalty_joint_torque,
                "penalty_joint_acc": cfg.reward.penalty_joint_acc,
                "penalty_undesired_contacts": cfg.reward.penalty_undesired_contacts,
                "terminal_particle_count_scaled": cfg.reward.terminal_particle_count_scaled,
                "terminal_success_bonus": cfg.reward.terminal_success_bonus,
            },
        )

        act_dim = self.action_manager.total_action_dim
        self._prev_prev_action = torch.zeros(
            (self.num_envs, act_dim), dtype=torch.float32, device=self.device
        )
        n_joints = self._robot.data.joint_pos.shape[1]
        self._prev_jvel = torch.zeros(
            (self.num_envs, n_joints), dtype=torch.float32, device=self.device
        )
        self._step_action_rate = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )
        self._step_joint_acc = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )

        self._particle_update_counter = torch.full(
            (self.num_envs,),
            cfg._particle_update_interval_n_steps,
            dtype=torch.long,
            device=self.device,
        )
        self._cached_particles_pos: torch.Tensor = torch.zeros(
            (self.num_envs, 1, 3), dtype=torch.float32, device=self.device
        )
        self._cached_particles_vel: torch.Tensor = torch.zeros(
            (self.num_envs, 1, 3), dtype=torch.float32, device=self.device
        )

        self._scoop_target_rot6d = rot6d_target_forward_tilt(
            target_rad=DEFAULT_SCOOP_TILT_RAD,
            device=self.device,
            dtype=torch.float32,
        ).unsqueeze(0)

        _mount_xy = cfg.pedestal.frame_manipulator_mount.offset.pos[:2]
        self._husky_mount_offset_xy: torch.Tensor = torch.tensor(
            _mount_xy, dtype=torch.float32, device=self.device
        )

        self._heightmap_spec_values = _heightmap_spec_for_target_xy(
            tuple(cfg.scoop_target_xy_m)
        )
        self._heightmap_offsets_b = torch.zeros(
            (self.num_envs, 2), dtype=torch.float32, device=self.device
        )
        self._scoop_target_xy_b = torch.zeros(
            (self.num_envs, 2), dtype=torch.float32, device=self.device
        )
        self._scoop_target_yaw_sin_cos = torch.zeros(
            (self.num_envs, 2), dtype=torch.float32, device=self.device
        )
        _initial_target = ScoopTargetCommand(
            xy_m=tuple(cfg.scoop_target_xy_m),
            yaw_rad=float(cfg.scoop_target_yaw_rad),
            frame=str(cfg.scoop_target_frame),
        )
        self._target_lock = _PerEnvScoopTargetLock(
            current=_initial_target,
            workspace_bounds_m=tuple(cfg.scoop_target_workspace_bounds_m),
        )
        self.set_scoop_target_command(_initial_target)

    def queue_scoop_target_command(self, command: ScoopTargetCommand) -> None:
        """Validate and apply a command at each environment reset."""
        self._target_lock.queue(command)

    def set_scoop_target_command(self, command: ScoopTargetCommand) -> None:
        """Validate and apply a command to every environment immediately."""
        report = validate_scoop_target_command(
            command,
            workspace_bounds_m=self.cfg.scoop_target_workspace_bounds_m,
        )
        if not report.valid:
            raise ValueError("Invalid scoop target: " + "; ".join(report.errors))

        self._target_lock.current = command
        all_envs = torch.arange(self.num_envs, device=self.device)
        self._adopt_scoop_target(all_envs)
        self._target_lock.retarget_on_reset = False

    def _adopt_scoop_target(self, env_ids: torch.Tensor) -> None:
        """Write the locked target into the selected environment rows."""
        command = self._target_lock.current
        spec = _heightmap_spec_for_target_xy(tuple(command.xy_m))
        target_xy = torch.tensor(command.xy_m, dtype=torch.float32, device=self.device)
        target_yaw_sin_cos = torch.tensor(
            [math.sin(command.yaw_rad), math.cos(command.yaw_rad)],
            dtype=torch.float32,
            device=self.device,
        )
        grid_offsets = torch.tensor(
            [float(spec["forward_offset_m"]), float(spec["lateral_offset_m"])],
            dtype=torch.float32,
            device=self.device,
        )
        env_ids = env_ids.to(device=self.device, dtype=torch.long)

        scoop_target_xy_b = self._scoop_target_xy_b.clone()
        scoop_target_xy_b[env_ids] = target_xy
        self._scoop_target_xy_b = scoop_target_xy_b
        scoop_target_yaw_sin_cos = self._scoop_target_yaw_sin_cos.clone()
        scoop_target_yaw_sin_cos[env_ids] = target_yaw_sin_cos
        self._scoop_target_yaw_sin_cos = scoop_target_yaw_sin_cos
        self._heightmap_offsets_b[env_ids] = grid_offsets

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        env_ids_t = torch.as_tensor(env_ids, device=self.device)

        if self._target_lock.apply_pending():
            self._adopt_scoop_target(env_ids_t)

        self._prev_prev_action[env_ids] = 0.0
        self._prev_jvel[env_ids] = 0.0
        self._step_action_rate[env_ids] = 0.0
        self._step_joint_acc[env_ids] = 0.0

        self._particle_update_counter[env_ids_t] = (
            self.cfg._particle_update_interval_n_steps
        )

        if self._regolith_ps is not None:
            if self._regolith_ps.data.has_default_state:
                default_pos = self._regolith_ps.data.default_particle_pos_w
                default_vel = self._regolith_ps.data.default_particle_vel_w
                if self._cached_particles_pos.shape != default_pos.shape:
                    count = min(
                        self._cached_particles_pos.shape[1], default_pos.shape[1]
                    )
                    pos = torch.zeros_like(default_pos)
                    vel = torch.zeros_like(default_vel)
                    pos[:, :count] = self._cached_particles_pos[:, :count]
                    vel[:, :count] = self._cached_particles_vel[:, :count]
                    self._cached_particles_pos = pos
                    self._cached_particles_vel = vel
                self._cached_particles_pos[env_ids_t] = default_pos[env_ids_t]
                self._cached_particles_vel[env_ids_t] = default_vel[env_ids_t]

    def _emitted_previous_action(self) -> torch.Tensor:
        if self._action_group is not None:
            return self._action_group.emitted_action
        return self.action_manager.prev_action

    def advance_step_state(self) -> None:
        """Update per-step caches and particle-cache cadence."""
        self._particle_update_counter -= 1
        due_mask = self._particle_update_counter <= 0
        if torch.any(due_mask):
            if self._regolith_ps is not None:
                self._regolith_ps.update_state_from_sim()
                self._cached_particles_pos[due_mask] = (
                    self._regolith_ps.data.particle_pos_w[due_mask]
                )
                self._cached_particles_vel[due_mask] = (
                    self._regolith_ps.data.particle_vel_w[due_mask]
                )
            self._particle_update_counter[due_mask] = (
                self.cfg._particle_update_interval_n_steps
            )

        prev_action = self._emitted_previous_action()
        jvel = self._robot.data.joint_vel
        self._step_action_rate = torch.mean(
            torch.square(prev_action - self._prev_prev_action), dim=-1
        )
        dt = self.step_dt
        self._step_joint_acc = (
            ((jvel - self._prev_jvel) / (dt + 1e-8)).square().mean(dim=-1)
        )
        self._prev_prev_action = prev_action.clone().detach()
        self._prev_jvel = jvel.clone().detach()

    def extract_step_return(self) -> StepReturn:  # noqa: C901 (complexity)
        """Build observations, rewards, and episode outcomes."""
        rew_cfg = self.cfg.reward
        num_envs = self.num_envs
        device = self.device
        dtype = torch.float32

        jpos = self._robot.data.joint_pos
        jvel = self._robot.data.joint_vel
        jpos_soft = self._robot.data.soft_joint_pos_limits

        has_finite_limits = torch.all(torch.isfinite(jpos_soft))
        if has_finite_limits:
            jpos_n = scale_transform(jpos, jpos_soft[..., 0], jpos_soft[..., 1])
        else:
            jpos_n = jpos

        if self._tf_end_effector is not None:
            ee_pos_b = self._tf_end_effector.data.target_pos_source[:, 0, :]
            ee_quat_b = self._tf_end_effector.data.target_quat_source[:, 0, :]
        else:
            ee_pos_b = torch.zeros((num_envs, 3), dtype=dtype, device=device)
            ee_quat_b = torch.zeros((num_envs, 4), dtype=dtype, device=device)
            ee_quat_b[:, 3] = 1.0

        ee_rotmat_b = matrix_from_quat(ee_quat_b)
        ee_rot6d_b = rotmat_to_rot6d(ee_rotmat_b)

        if self._imu is not None:
            imu_lin_acc = self._imu.data.lin_acc_b
            imu_ang_vel = self._imu.data.ang_vel_b
        else:
            imu_lin_acc = torch.zeros((num_envs, 3), dtype=dtype, device=device)
            imu_ang_vel = torch.zeros((num_envs, 3), dtype=dtype, device=device)

        kinova_pos_w = (
            self._robot.data.root_pos_w[:, :2] - self.scene.env_origins[:, :2]
        )
        base_quat_w = self._robot.data.root_quat_w
        q_w = base_quat_w[:, 3]
        q_x = base_quat_w[:, 0]
        q_y = base_quat_w[:, 1]
        q_z = base_quat_w[:, 2]
        yaw = torch.atan2(
            2.0 * (q_w * q_z + q_x * q_y),
            1.0 - 2.0 * (q_y * q_y + q_z * q_z),
        )
        _mox = self._husky_mount_offset_xy[0]
        _moy = self._husky_mount_offset_xy[1]
        _cos_y = torch.cos(yaw)
        _sin_y = torch.sin(yaw)
        chassis_x = kinova_pos_w[:, 0] - (_cos_y * _mox - _sin_y * _moy)
        chassis_y = kinova_pos_w[:, 1] - (_sin_y * _mox + _cos_y * _moy)
        base_pos_w = torch.stack([chassis_x, chassis_y], dim=-1)
        base_pose_ot = torch.cat([base_pos_w, yaw.unsqueeze(-1)], dim=-1)

        ground_z_b = self.scene.env_origins[:, 2] - self._robot.data.root_pos_w[:, 2]

        particle_coords = (
            self._particle_grid_coords(self._robot.data.root_pos_w, yaw)
            if self._regolith_ps is not None
            else None
        )

        if self._heightmap is not None:
            hm = self._heightmap.data
            heights = hm.heights
            valid_mask = hm.valid_mask.float()
        elif particle_coords is not None:
            heights, valid_mask = self._particle_heightmap(particle_coords)
        else:
            _ny = _EXCAVATION_HEIGHTMAP_SPEC["ny"]
            _nx = _EXCAVATION_HEIGHTMAP_SPEC["nx"]
            _max_range = _EXCAVATION_HEIGHTMAP_SPEC["max_range_m"]
            heights = torch.full(
                (num_envs, _ny, _nx), _max_range, dtype=dtype, device=device
            )
            valid_mask = torch.zeros((num_envs, _ny, _nx), dtype=dtype, device=device)

        prev_action = self._emitted_previous_action()

        ee_pos_w = (
            self._tf_end_effector.data.target_pos_w[:, 0, :]
            if self._tf_end_effector is not None
            else torch.zeros((num_envs, 3), dtype=dtype, device=device)
        )
        if self._tf_end_effector is not None:
            ee_quat_w = self._tf_end_effector.data.target_quat_w[:, 0, :]
        else:
            ee_quat_w = torch.zeros((num_envs, 4), dtype=dtype, device=device)
            ee_quat_w[:, 3] = 1.0
        ee_rot6d_w = rotmat_to_rot6d(matrix_from_quat(ee_quat_w))
        # Env-local, so replicas at different origins observe alike.
        ee_pose_w = torch.cat([ee_pos_w - self.scene.env_origins, ee_rot6d_w], dim=-1)

        if self._regolith_ps is not None:
            in_scoop, carried = self._particles_near_scoop(self._cached_particles_pos)
            pcnt_scoop = in_scoop.float().sum(dim=-1, keepdim=True)
            pile_top_z, has_pile = self._pile_top_height(particle_coords, carried)
            pmean_rel = self._particle_mean_pos_relative(self._cached_particles_pos)
            hm_gt = (
                self._heightmap.data_ground_truth()
                if self._heightmap is not None
                else heights.clone()
            )
            pen_depth = self._ee_penetration_depth_into_pile(self._cached_particles_pos)
        else:
            _g16 = heights.clone()
            pcnt_scoop = torch.zeros((num_envs, 1), dtype=dtype, device=device)
            pile_top_z = self._pile_max_height(heights, valid_mask, ground_z_b)
            has_pile = valid_mask.bool().flatten(1).any(dim=-1)
            pmean_rel = torch.zeros((num_envs, 3), dtype=dtype, device=device)
            hm_gt = _g16
            pen_depth = torch.zeros((num_envs, 1), dtype=dtype, device=device)

        ee_xy = ee_pos_b[:, :2]
        approach_dist = torch.norm(self._scoop_target_xy_b - ee_xy, dim=-1)
        r_approach = rew_cfg.reward_approach_pile(approach_dist)

        surf_z = self._pile_surface_height_at_ee(
            heights, valid_mask, ee_pos_b, ground_z_b
        )
        depth_err = surf_z - ee_pos_b[:, 2]
        r_depth = rew_cfg.reward_scoop_depth(depth_err)

        target_rot6d = self._scoop_target_rot6d.expand_as(ee_rot6d_b)
        orient_err = torch.norm(ee_rot6d_b - target_rot6d, dim=-1)
        r_orient = rew_cfg.reward_scoop_orientation(orient_err)

        pile_max_z = torch.where(has_pile, pile_top_z, ground_z_b)
        lift_val = ee_pos_b[:, 2] - pile_max_z
        lift_err = (0.2 - lift_val).clamp(min=0.0)
        r_lift = rew_cfg.reward_lift(lift_err)
        r_lift = mask_lift_reward(r_lift, has_pile.view(-1, 1, 1))

        act_rate = self._step_action_rate
        p_ar = rew_cfg.penalty_action_rate(act_rate)

        if (
            self._action_group is not None
            and hasattr(self._action_group, "_jerk")
            and hasattr(self._action_group._jerk, "clip_frequency")
        ):
            jerk_val = self._action_group._jerk.clip_frequency.reshape(num_envs)
        else:
            jerk_val = torch.zeros(num_envs, dtype=dtype, device=device)
        p_jerk = rew_cfg.penalty_jerk(jerk_val)

        torque_val = self._robot.data.applied_torque.square().mean(dim=-1)
        p_torque = rew_cfg.penalty_joint_torque(torque_val)

        jacc_val = self._step_joint_acc
        p_jacc = rew_cfg.penalty_joint_acc(jacc_val)

        if self._contacts_robot is not None:
            undesired_cf = self._contacts_robot.data.net_forces_w.norm(dim=-1).max(
                dim=1
            )[0]
        else:
            undesired_cf = torch.zeros(num_envs, dtype=dtype, device=device)
        p_contacts = rew_cfg.penalty_undesired_contacts(undesired_cf)

        last_step = self._horizon_reached()
        scoop_ang_vel_norm = self._scoop_ang_vel_norm()

        terminal_ok = (
            (pcnt_scoop.squeeze(-1) >= self.cfg.n_success_scoop_particles)
            & (lift_val >= 0.2)
            & (scoop_ang_vel_norm < 0.5)
        )

        r_t_count = rew_cfg.terminal_particle_count_scaled(
            last_step.float()
            * (pcnt_scoop.squeeze(-1) / max(1, self.cfg.n_scoop_capacity)).clamp(
                0.0, 1.0
            )
        )
        r_t_bonus = rew_cfg.terminal_success_bonus(
            last_step.float() * terminal_ok.float()
        )

        _reward_total, per_term = self._reward_normalizer.compute(
            {
                "reward_approach_pile": r_approach,
                "reward_scoop_depth": r_depth,
                "reward_scoop_orientation": r_orient,
                "reward_lift": r_lift,
                "penalty_action_rate": p_ar,
                "penalty_jerk": p_jerk,
                "penalty_joint_torque": p_torque,
                "penalty_joint_acc": p_jacc,
                "penalty_undesired_contacts": p_contacts,
                "terminal_particle_count_scaled": r_t_count,
                "terminal_success_bonus": r_t_bonus,
            }
        )

        obs = {
            ObservationGroup.PROPRIO: {
                "joint_pos_normalized": jpos_n,
                "joint_vel": jvel,
                "fk_pos_end_effector_base": ee_pos_b,
                "fk_rot6d_end_effector_base": ee_rot6d_b,
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
                "base_pose_world_optitrack": base_pose_ot,
            },
            ObservationGroup.PROPRIO_DYN: {
                "heightmap": heights,
                "heightmap_valid_mask": valid_mask,
                "prev_action": prev_action,
            },
            ObservationGroup.COMMAND: {
                "scoop_target_xy": self._scoop_target_xy_b,
                "scoop_target_yaw_sin_cos": self._scoop_target_yaw_sin_cos,
            },
            ObservationGroup.STATE: {
                "ee_pose_world": ee_pose_w,
            },
        }

        if self.cfg.expose_privileged:
            obs[ObservationGroup.STATE_PRIVILEGED] = {
                "particle_count_in_scoop_volume": pcnt_scoop,
                "particle_mean_pos_relative": pmean_rel,
                "particle_heightmap_gt": hm_gt,
                "ee_penetration_depth_into_pile": pen_depth,
            }

        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)

        return StepReturn(
            obs,
            per_term,
            termination,
            last_step,
            info=episode_outcome_info(
                success=last_step & terminal_ok,
                metrics={
                    "scoop_particle_count": pcnt_scoop.squeeze(-1),
                    "scoop_lift_clearance_m": lift_val,
                    "scoop_angular_velocity_norm": scoop_ang_vel_norm,
                },
            ),
        )

    def _particle_grid_coords(
        self, base_pos_w: torch.Tensor, yaw: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        p_w = self._cached_particles_pos
        dx = p_w[..., 0] - base_pos_w[:, 0:1]
        dy = p_w[..., 1] - base_pos_w[:, 1:2]
        dz = p_w[..., 2] - base_pos_w[:, 2:3]
        cos_y = torch.cos(yaw).unsqueeze(1)
        sin_y = torch.sin(yaw).unsqueeze(1)
        ax = cos_y * dx + sin_y * dy - self._heightmap_offsets_b[:, 0:1]
        ay = -sin_y * dx + cos_y * dy - self._heightmap_offsets_b[:, 1:2]
        return ax, ay, dz

    def _particle_heightmap(
        self, coords: tuple[torch.Tensor, torch.Tensor, torch.Tensor]
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Build an ego-frame heightmap from cached particle positions."""
        spec = self._heightmap_spec_values
        ax, ay, dz = coords
        return bin_points_to_heightmap_grid(
            ax,
            ay,
            dz,
            torch.isfinite(dz),
            ny=int(spec["ny"]),
            nx=int(spec["nx"]),
            cell_size_m=float(spec["cell_size_m"]),
            forward_offset_m=0.0,
            lateral_offset_m=0.0,
            max_range_m=float(spec["max_range_m"]),
        )

    def _pile_top_height(
        self,
        coords: tuple[torch.Tensor, torch.Tensor, torch.Tensor],
        exclude: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        spec = self._heightmap_spec_values
        ax, ay, dz = coords
        cell = float(spec["cell_size_m"])
        half_width = 0.5 * int(spec["nx"]) * cell
        pile = (
            torch.isfinite(dz)
            & ~exclude
            & (ax >= 0.0)
            & (ax < int(spec["ny"]) * cell)
            & (ay >= -half_width)
            & (ay < half_width)
        )
        top_z = torch.where(pile, dz, torch.full_like(dz, float("-inf"))).amax(-1)
        return top_z, torch.isfinite(top_z)

    def _pile_surface_height_at_ee(
        self,
        heights: torch.Tensor,
        valid_mask: torch.Tensor,
        ee_pos_b: torch.Tensor,
        ground_z_b: torch.Tensor,
    ) -> torch.Tensor:
        if not _HAS_HEIGHTMAP:
            return ground_z_b.to(heights.dtype)

        spec_vals = self._heightmap_spec_values
        ny, nx = spec_vals["ny"], spec_vals["nx"]
        cell_sz = spec_vals["cell_size_m"]
        fwd_off = self._heightmap_offsets_b[:, 0]
        lat_off = self._heightmap_offsets_b[:, 1]

        num_envs = heights.shape[0]
        device = heights.device

        ee_x = ee_pos_b[:, 0]
        ee_y = ee_pos_b[:, 1]

        row = ((ee_x - fwd_off) / cell_sz).long().clamp(0, ny - 1)
        col = ((ee_y - lat_off + nx * cell_sz / 2.0) / cell_sz).long().clamp(0, nx - 1)

        env_idx = torch.arange(num_envs, device=device)
        h = heights[env_idx, row, col]
        v = valid_mask[env_idx, row, col]

        return torch.where(v.bool(), h, ground_z_b.to(h.dtype))

    def _pile_max_height(
        self,
        heights: torch.Tensor,
        valid_mask: torch.Tensor,
        ground_z_b: torch.Tensor,
    ) -> torch.Tensor:
        masked = torch.where(
            valid_mask.bool(),
            heights,
            torch.full_like(heights, float("-inf")),
        )
        max_val = masked.view(heights.shape[0], -1).max(dim=-1).values
        return torch.where(torch.isinf(max_val), ground_z_b.to(max_val.dtype), max_val)

    def _particles_near_scoop(
        self, particle_pos: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Return masks for particles inside and carried by the scoop."""
        if self._tf_end_effector is None:
            none = torch.zeros(
                particle_pos.shape[:2], dtype=torch.bool, device=particle_pos.device
            )
            return none, none

        ee_pos_w = self._tf_end_effector.data.target_pos_w[:, 0, :]
        rel = (particle_pos - ee_pos_w.unsqueeze(1)).abs()

        capture_half = torch.tensor(
            list(self.cfg.scoop_capture_half_extents_m),
            dtype=rel.dtype,
            device=rel.device,
        )
        in_capture = (rel <= capture_half).all(dim=-1)

        near_x, near_y, near_z = self.cfg.scoop_contact_radius_half_extents_m
        near_xy = max(float(near_x), float(near_y))
        near_half = torch.tensor(
            [near_xy, near_xy, float(near_z)], dtype=rel.dtype, device=rel.device
        )
        carried = in_capture | (rel <= near_half).all(dim=-1)
        return in_capture, carried

    def _particle_mean_pos_relative(self, particle_pos: torch.Tensor) -> torch.Tensor:
        if self._tf_end_effector is not None:
            ee_pos_w = self._tf_end_effector.data.target_pos_w[:, 0, :]
        else:
            ee_pos_w = self._robot.data.root_pos_w

        base_quat_w = self._robot.data.root_quat_w

        return particle_mean_pos_relative_base(particle_pos, ee_pos_w, base_quat_w)

    def _ee_penetration_depth_into_pile(
        self, particle_pos: torch.Tensor
    ) -> torch.Tensor:
        if particle_pos.shape[1] == 0:
            return torch.zeros((self.num_envs, 1), device=particle_pos.device)

        pile_z_world = torch.quantile(particle_pos[:, :, 2], q=0.95, dim=1)

        if self._tf_end_effector is not None:
            ee_z_w = self._tf_end_effector.data.target_pos_w[:, 0, 2]
        else:
            ee_z_w = self._robot.data.root_pos_w[:, 2]

        pen = pile_z_world - ee_z_w
        return pen.unsqueeze(-1)

    def _scoop_ang_vel_norm(self) -> torch.Tensor:
        if self._end_effector is not None:
            try:
                scoop_ang_vel = self._end_effector.data.root_ang_vel_w
                return torch.norm(scoop_ang_vel, dim=-1)
            except AttributeError:
                pass

        try:
            body_ang_vel = self._robot.data.body_ang_vel_w
            scoop_ang_vel = body_ang_vel[:, -1, :]
        except AttributeError:
            scoop_ang_vel = self._robot.data.root_ang_vel_w

        return torch.norm(scoop_ang_vel, dim=-1)


class Task(_TaskMixin, SrbEnv, ManipulationEnv):
    """Single-scoop excavation task using the SRB lifecycle."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(
            cfg, runtime_mode=kwargs.pop("runtime_mode", "isaac"), **kwargs
        )
