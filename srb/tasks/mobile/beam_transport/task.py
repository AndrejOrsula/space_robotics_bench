from typing import NamedTuple, Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    AssetVariant,
    GroundRobot,
    RigidObject,
    RigidObjectCfg,
    Robot,
    Terrain,
)
from srb.core.domain import Domain
from srb.core.env import (
    GroundEnv,
    GroundEnvCfg,
    GroundEventCfg,
    GroundSceneCfg,
    ViewerCfg,
)
from srb.core.env.mobile import GroundMobileRewardCfg
from srb.core.env.runtime import episode_outcome_info
from srb.core.env.srb_env import SrbEnv
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.randomization import SrbEventTermCfg
from srb.core.reward import GaussianRewardTerm, LinearRewardTerm, ScaledRewardTerm
from srb.core.reward.transport import (
    compute_planar_target_progress_velocity,
    compute_two_robot_beam_support_error,
    compute_two_robot_mounted_beam_pose,
)
from srb.core.scene_introspection import get_scene_rigid_object
from srb.core.sim import CollisionPropertiesCfg, PreviewSurfaceCfg, SphereCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d


class RewardCfg(GroundMobileRewardCfg):
    """Reward terms for collaborative beam transport."""

    ## Dense shaping
    reward_beam_position: GaussianRewardTerm = GaussianRewardTerm(
        weight=16.0, scale=1.5
    )

    reward_beam_midpoint_alignment: GaussianRewardTerm = GaussianRewardTerm(
        weight=10.0, scale=0.25
    )

    reward_robot_support_slots: GaussianRewardTerm = GaussianRewardTerm(
        weight=12.0, scale=0.20
    )

    reward_beam_progress: ScaledRewardTerm = ScaledRewardTerm(weight=8.0, limit=4.0)
    """Rewards signed beam velocity toward the target and penalizes moving away."""

    reward_attachment_balance: GaussianRewardTerm = GaussianRewardTerm(
        weight=4.0, scale=0.15
    )

    # limit must be >= |weight|: this term is fed a 0/1 indicator, so a limit
    # of 1.0 would clamp the intended 24.0 bonus down to 1.0 (weaker than every
    # dense term). limit == weight delivers the full bonus (cf. modular_assembly).
    reward_success: LinearRewardTerm = LinearRewardTerm(weight=24.0, limit=24.0)

    ## Safety/robustness penalties
    penalty_beam_tilt: LinearRewardTerm = LinearRewardTerm(weight=-4.0, limit=4.0)

    penalty_beam_drop: LinearRewardTerm = LinearRewardTerm(weight=-16.0, limit=1.0)

    penalty_beam_speed: LinearRewardTerm = LinearRewardTerm(weight=-2.0, limit=2.0)

    ## Locomotion stability shaping
    penalty_joint_torque: LinearRewardTerm = LinearRewardTerm(
        weight=-0.000025, limit=4.0
    )

    penalty_joint_acceleration: LinearRewardTerm = LinearRewardTerm(
        weight=-0.00000025, limit=2.0
    )

    penalty_undesired_lin_vel_z: LinearRewardTerm = LinearRewardTerm(
        weight=-0.5, limit=4.0
    )

    penalty_undesired_ang_vel_xy: LinearRewardTerm = LinearRewardTerm(
        weight=-0.1, limit=4.0
    )

    penalty_gravity_rotation_alignment: LinearRewardTerm = LinearRewardTerm(
        weight=-2.0, limit=8.0
    )


@configclass
class SceneCfg(GroundSceneCfg):
    beam: RigidObjectCfg = assets.Beam().asset_cfg


@configclass
class EventCfg(GroundEventCfg):
    # The base single-robot randomizer breaks the mounted pair. The task reset
    # applies one shared rigid transform after the robots receive their layout.
    randomize_robot_state: SrbEventTermCfg | None = None


def _make_beam_transport_moon_surface() -> Terrain:
    """Provide terrain covering the spawn area and 4 m target.

    The 5 m default misses the target by 1.5 m; 12 m covers it, while 6 m levels spawn.
    """
    return assets.MoonSurface(scale=(12.0, 12.0, 0.5), flat_area_size=6.0)


@configclass
class TaskCfg(GroundEnvCfg):
    """Two coordinated rovers transport a shared beam to a target point."""

    ## Scenario
    domain: Domain = Domain.MOON

    ## Assets
    robot: GroundRobot | AssetVariant = assets.Spot()
    extra_robots: dict[str, Robot | AssetVariant] = {}
    scenery: Terrain | AssetVariant | None = _make_beam_transport_moon_surface()

    ## Scene
    scene: SceneCfg = SceneCfg()

    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": SphereCfg(
                radius=0.05,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 30.0
    is_finite_horizon: bool = True
    truncate_episodes: bool = True

    ## Robot layout
    robot_positions: tuple[tuple[float, float], ...] | None = (
        (0.0, -0.45),
        (0.0, 0.45),
    )

    ## Transport task
    target_pos: tuple[float, float, float] = (4.0, 0.0, 0.0)
    target_zone_radius_xy: float = 0.20
    target_speed_threshold_xy: float = 0.35
    success_hold_steps: int = 8
    beam_spawn_height: float = 0.62
    beam_kinematic_mount_enabled: bool = True
    beam_support_half_length: float = 0.45
    beam_attachment_radius_xy: float = 0.30
    beam_tilt_tolerance: float = 0.20
    beam_drop_threshold: float = 0.02

    ## Viewer
    # Keep both rovers, the beam, and target in view across the spawn area.
    viewer: ViewerCfg = ViewerCfg(
        eye=(1.0, -8.5, 4.0),
        lookat=(1.0, 0.0, 0.3),
        origin_type="env",
    )

    def __post_init__(self):
        self.extra_robots = dict(self.extra_robots)
        if not self.extra_robots:
            self.extra_robots["robot_1"] = assets.Spot()
        super().__post_init__()


class _BeamGeometry(NamedTuple):
    robot_midpoint: torch.Tensor
    robot_to_beam: torch.Tensor
    dist_beam_to_target: torch.Tensor
    dist_beam_to_midpoint: torch.Tensor
    dist_attachment_balance: torch.Tensor
    beam_speed_xy: torch.Tensor
    beam_speed_excess_xy: torch.Tensor
    beam_progress_velocity: torch.Tensor
    beam_rotmat: torch.Tensor
    beam_support_error: torch.Tensor
    beam_tilt_metric: torch.Tensor
    beam_height_metric: torch.Tensor


class _BeamTransportTaskMixin:
    """Shared beam construction, state transitions, and extraction logic."""

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        # The kinematically mounted beam is a non-colliding payload proxy.
        # Set this after config reconstruction, which restores asset defaults.
        if cfg.beam_kinematic_mount_enabled and cfg.scene.beam.spawn is not None:
            cfg.scene.beam.spawn.collision_props = CollisionPropertiesCfg(
                collision_enabled=False
            )
        super().__init__(cfg, **kwargs)

        self._initialize_beam_transport()

    def _initialize_beam_transport(self) -> None:
        ## Robots (primary + one extra by design)
        self._robot_names: list[str] = ["robot"] + sorted(self.cfg.extra_robots.keys())
        self._robots = [self._robots[name] for name in self._robot_names]
        self._num_robots = len(self._robots)
        if self._num_robots != 2:
            raise ValueError(
                f"beam_transport requires exactly 2 robots; configure one robot in "
                f"env.extra_robots (got {self._num_robots})"
            )

        ## Scene assets
        self._beam: RigidObject = get_scene_rigid_object(self.scene, "beam")
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )
        self._target_rot = torch.tensor(
            (0.0, 0.0, 0.0, 1.0), dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

        ## Episode state
        self._target_pos = self.scene.env_origins + torch.tensor(
            self.cfg.target_pos, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1).to(self.device)
        self._min_beam_distance_to_target = torch.full(
            (self.num_envs,), float("inf"), dtype=torch.float32, device=self.device
        )
        self._beam_hold_steps = torch.zeros(
            self.num_envs, dtype=torch.int32, device=self.device
        )
        self._beam_mount_pos_prev = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        self._beam_mount_quat_prev = torch.tensor(
            (0.0, 0.0, 0.0, 1.0), dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        # Written during state advance and read by pure extraction.
        self._mounted_beam_lin_vel = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        self._mounted_beam_ang_vel = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        self._target_marker.visualize(self._target_pos, self._target_rot)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        if len(env_ids) == 0:
            return

        env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        robot_pos = self._randomize_paired_spawn(env_ids_t)
        # A free beam also starts aligned with the randomized carrier pair.
        beam_pos, beam_quat = compute_two_robot_mounted_beam_pose(
            robot_xyz=robot_pos,
            beam_z=self._beam_z(env_ids_t),
            fallback_pos=self._beam_mount_pos_prev[env_ids_t],
            fallback_quat=self._beam_mount_quat_prev[env_ids_t],
        )
        self._beam.write_root_pose_to_sim(
            torch.cat((beam_pos, beam_quat), dim=-1),
            env_ids=env_ids_t,
        )
        self._beam.write_root_velocity_to_sim(
            torch.zeros((len(env_ids_t), 6), dtype=torch.float32, device=self.device),
            env_ids=env_ids_t,
        )

        self._min_beam_distance_to_target[env_ids_t] = float("inf")
        self._beam_hold_steps[env_ids_t] = 0
        self._beam_mount_pos_prev[env_ids_t] = beam_pos
        self._beam_mount_quat_prev[env_ids_t] = beam_quat
        self._mounted_beam_lin_vel[env_ids_t] = 0.0
        self._mounted_beam_ang_vel[env_ids_t] = 0.0

    def _randomize_paired_spawn(self, env_ids: torch.Tensor) -> torch.Tensor:
        """Apply one random planar rigid transform to both mounted carriers."""
        positions = torch.stack(
            [robot.data.root_pos_w[env_ids, :3] for robot in self._robots], dim=1
        )
        quaternions = torch.stack(
            [robot.data.root_quat_w[env_ids] for robot in self._robots], dim=1
        )
        midpoint = positions.mean(dim=1, keepdim=True)
        yaw = torch.empty(len(env_ids), device=self.device).uniform_(
            -torch.pi, torch.pi
        )
        cosine = torch.cos(yaw)
        sine = torch.sin(yaw)
        relative_xy = positions[:, :, :2] - midpoint[:, :, :2]
        rotated_xy = torch.stack(
            (
                cosine[:, None] * relative_xy[..., 0]
                - sine[:, None] * relative_xy[..., 1],
                sine[:, None] * relative_xy[..., 0]
                + cosine[:, None] * relative_xy[..., 1],
            ),
            dim=-1,
        )
        offset_xy = torch.empty((len(env_ids), 1, 2), device=self.device).uniform_(
            -1.5, 1.5
        )
        positions[:, :, :2] = midpoint[:, :, :2] + rotated_xy + offset_xy

        half_yaw = yaw[:, None] * 0.5
        yaw_z = torch.sin(half_yaw)
        yaw_w = torch.cos(half_yaw)
        x, y, z, w = quaternions.unbind(dim=-1)
        quaternions = torch.stack(
            (
                yaw_w * x - yaw_z * y,
                yaw_w * y + yaw_z * x,
                yaw_w * z + yaw_z * w,
                yaw_w * w - yaw_z * z,
            ),
            dim=-1,
        )
        for index, robot in enumerate(self._robots):
            robot.write_root_pose_to_sim(
                torch.cat((positions[:, index], quaternions[:, index]), dim=-1),
                env_ids=env_ids,
            )
        return positions

    def _sync_mounted_beam(
        self, robot_pos: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        beam_pos, beam_quat = compute_two_robot_mounted_beam_pose(
            robot_xyz=robot_pos,
            beam_z=self._beam_z(),
            fallback_pos=self._beam_mount_pos_prev,
            fallback_quat=self._beam_mount_quat_prev,
        )
        beam_lin_vel = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        step_dt = max(float(getattr(self, "step_dt", 1.0)), 1.0e-6)
        beam_lin_vel[:, :2] = (
            beam_pos[:, :2] - self._beam_mount_pos_prev[:, :2]
        ) / step_dt
        # The mounted pose is yaw-only (xyzw), so its rate is the wrapped yaw
        # change about z; body and world z coincide for a level beam.
        beam_ang_vel = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        yaw = 2.0 * torch.atan2(beam_quat[:, 2], beam_quat[:, 3])
        yaw_prev = 2.0 * torch.atan2(
            self._beam_mount_quat_prev[:, 2], self._beam_mount_quat_prev[:, 3]
        )
        yaw_delta = (
            torch.remainder(yaw - yaw_prev + torch.pi, 2.0 * torch.pi) - torch.pi
        )
        beam_ang_vel[:, 2] = yaw_delta / step_dt

        self._beam.write_root_pose_to_sim(torch.cat((beam_pos, beam_quat), dim=-1))
        self._beam.write_root_velocity_to_sim(
            torch.cat((beam_lin_vel, beam_ang_vel), dim=-1)
        )
        self._beam_mount_pos_prev[:] = beam_pos
        self._beam_mount_quat_prev[:] = beam_quat
        self._mounted_beam_lin_vel[:] = beam_lin_vel
        self._mounted_beam_ang_vel[:] = beam_ang_vel
        return beam_pos, beam_quat, beam_lin_vel, beam_ang_vel

    def _beam_z(self, env_ids: torch.Tensor | None = None) -> torch.Tensor:
        """Return the world-frame beam height above each env origin."""
        origins_z = self.scene.env_origins[:, 2]
        if env_ids is not None:
            origins_z = origins_z[env_ids]
        return origins_z + self.cfg.beam_spawn_height

    def _robot_positions(self) -> torch.Tensor:
        return torch.stack(
            [robot.data.root_pos_w[:, :3] for robot in self._robots], dim=1
        )

    def _beam_state(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return the current beam pose and velocity without side effects.

        A kinematic pose uses carrier positions, the cached mount pose as
        fallback, and cached velocity; otherwise read state from simulation.
        """
        if self.cfg.beam_kinematic_mount_enabled:
            beam_pos, beam_quat = compute_two_robot_mounted_beam_pose(
                robot_xyz=self._robot_positions(),
                beam_z=self._beam_z(),
                fallback_pos=self._beam_mount_pos_prev,
                fallback_quat=self._beam_mount_quat_prev,
            )
            beam_lin_vel = self._mounted_beam_lin_vel
            beam_ang_vel = self._mounted_beam_ang_vel
            return beam_pos, beam_quat, beam_lin_vel, beam_ang_vel
        beam_pos = self._beam.data.root_pos_w
        beam_quat = self._beam.data.root_quat_w
        # Use world-frame velocity to match target directions and mounted motion.
        beam_lin_vel = self._beam.data.root_lin_vel_w
        beam_ang_vel = self._beam.data.root_ang_vel_b
        return beam_pos, beam_quat, beam_lin_vel, beam_ang_vel

    def _beam_geometry(
        self,
        robot_pos: torch.Tensor,
        beam_pos: torch.Tensor,
        beam_quat: torch.Tensor,
        beam_lin_vel: torch.Tensor,
    ) -> "_BeamGeometry":
        """Pure per-step geometry shared by the state advance and the extract."""
        robot_midpoint = robot_pos.mean(dim=1)
        robot_to_beam = beam_pos[:, None, :2] - robot_pos[:, :, :2]
        dist_robot_to_beam = torch.norm(robot_to_beam, dim=-1)
        dist_beam_to_target = torch.norm(
            beam_pos[:, :2] - self._target_pos[:, :2], dim=1
        )
        dist_beam_to_midpoint = torch.norm(
            beam_pos[:, :2] - robot_midpoint[:, :2], dim=1
        )
        dist_attachment_balance = torch.abs(
            dist_robot_to_beam[:, 0] - dist_robot_to_beam[:, 1]
        )
        beam_speed_xy = torch.norm(beam_lin_vel[:, :2], dim=1)
        beam_speed_excess_xy = torch.relu(
            beam_speed_xy - self.cfg.target_speed_threshold_xy
        )
        beam_progress_velocity = compute_planar_target_progress_velocity(
            current_xy=beam_pos[:, :2],
            target_xy=self._target_pos[:, :2],
            velocity_xy=beam_lin_vel[:, :2],
        )
        beam_rotmat = matrix_from_quat(beam_quat)
        beam_yaw = torch.atan2(beam_rotmat[:, 1, 0], beam_rotmat[:, 0, 0])
        beam_support_error = compute_two_robot_beam_support_error(
            robot_xy=robot_pos[:, :, :2],
            beam_xy=beam_pos[:, :2],
            beam_yaw=beam_yaw,
            half_length=self.cfg.beam_support_half_length,
        )
        # 1 - |cos(tilt)| uprightness gap (dimensionless, NOT radians): 0 = upright,
        # grows with tilt from vertical. beam_tilt_tolerance=0.20 => tilt <~ 37 deg.
        beam_tilt_metric = 1.0 - torch.abs(beam_rotmat[:, 2, 2])
        beam_height_metric = torch.relu(
            self.cfg.beam_drop_threshold
            - (beam_pos[:, 2] - self.scene.env_origins[:, 2])
        )
        return _BeamGeometry(
            robot_midpoint=robot_midpoint,
            robot_to_beam=robot_to_beam,
            dist_beam_to_target=dist_beam_to_target,
            dist_beam_to_midpoint=dist_beam_to_midpoint,
            dist_attachment_balance=dist_attachment_balance,
            beam_speed_xy=beam_speed_xy,
            beam_speed_excess_xy=beam_speed_excess_xy,
            beam_progress_velocity=beam_progress_velocity,
            beam_rotmat=beam_rotmat,
            beam_support_error=beam_support_error,
            beam_tilt_metric=beam_tilt_metric,
            beam_height_metric=beam_height_metric,
        )

    def advance_step_state(self) -> None:
        robot_pos = self._robot_positions()
        if self.cfg.beam_kinematic_mount_enabled:
            beam_pos, beam_quat, beam_lin_vel, _ = self._sync_mounted_beam(robot_pos)
        else:
            beam_pos, beam_quat, beam_lin_vel, _ = self._beam_state()
        geometry = self._beam_geometry(robot_pos, beam_pos, beam_quat, beam_lin_vel)
        dist_beam_to_target = geometry.dist_beam_to_target
        dist_beam_to_midpoint = geometry.dist_beam_to_midpoint
        beam_support_error = geometry.beam_support_error
        beam_speed_xy = geometry.beam_speed_xy
        beam_tilt_metric = geometry.beam_tilt_metric

        self._min_beam_distance_to_target = torch.minimum(
            self._min_beam_distance_to_target, dist_beam_to_target
        )
        # Success condition: hold the target pose for N consecutive steps
        on_target_xy = dist_beam_to_target <= self.cfg.target_zone_radius_xy
        near_midpoint = dist_beam_to_midpoint <= self.cfg.beam_attachment_radius_xy
        support_aligned = beam_support_error <= self.cfg.beam_attachment_radius_xy
        gentle_speed = beam_speed_xy <= self.cfg.target_speed_threshold_xy
        gentle_tilt = beam_tilt_metric <= self.cfg.beam_tilt_tolerance
        if self.cfg.beam_kinematic_mount_enabled:
            gentle_tilt = torch.ones_like(on_target_xy)
        at_goal_step = (
            on_target_xy & near_midpoint & support_aligned & gentle_speed & gentle_tilt
        )
        self._beam_hold_steps = torch.where(
            at_goal_step,
            torch.clamp(self._beam_hold_steps + 1, max=self.cfg.success_hold_steps),
            torch.zeros_like(self._beam_hold_steps),
        )

        # Visualize target
        self._target_marker.visualize(self._target_pos, self._target_rot)

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
        device = self.device
        rew = self.cfg.reward

        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        # Robot state
        robot_pos = self._robot_positions()
        robot_vel = torch.stack(
            [robot.data.root_lin_vel_b[:, :3] for robot in self._robots], dim=1
        )
        robot_ang_vel = torch.stack(
            [robot.data.root_ang_vel_b[:, :3] for robot in self._robots], dim=1
        )
        robot_projected_gravity = torch.stack(
            [robot.data.projected_gravity_b[:, :3] for robot in self._robots], dim=1
        )
        robot_joint_acc = torch.stack(
            [robot.data.joint_acc for robot in self._robots], dim=1
        )
        robot_joint_torque = torch.stack(
            [robot.data.applied_torque for robot in self._robots], dim=1
        )

        # Mount sync and success bookkeeping happen in ``advance_step_state``.
        beam_pos, beam_quat, beam_lin_vel, beam_ang_vel = self._beam_state()
        geometry = self._beam_geometry(robot_pos, beam_pos, beam_quat, beam_lin_vel)
        robot_midpoint = geometry.robot_midpoint
        robot_to_beam = geometry.robot_to_beam
        dist_beam_to_target = geometry.dist_beam_to_target
        dist_beam_to_midpoint = geometry.dist_beam_to_midpoint
        dist_attachment_balance = geometry.dist_attachment_balance
        beam_speed_excess_xy = geometry.beam_speed_excess_xy
        beam_progress_velocity = geometry.beam_progress_velocity
        beam_rotmat = geometry.beam_rotmat
        beam_support_error = geometry.beam_support_error
        beam_tilt_metric = geometry.beam_tilt_metric
        beam_height_metric = geometry.beam_height_metric

        # Action-rate shaping
        action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)

        # Locomotion stability shaping averaged over both carriers
        joint_torque_sq = torch.mean(
            torch.sum(torch.square(robot_joint_torque), dim=-1), dim=1
        )
        joint_acc_sq = torch.mean(
            torch.sum(torch.square(robot_joint_acc), dim=-1), dim=1
        )
        undesired_lin_vel_z = torch.mean(torch.square(robot_vel[:, :, 2]), dim=1)
        undesired_ang_vel_xy = torch.mean(
            torch.sum(torch.square(robot_ang_vel[:, :, :2]), dim=-1), dim=1
        )
        gravity_alignment = torch.mean(
            torch.sum(torch.square(robot_projected_gravity[:, :, :2]), dim=-1)
            + torch.square(robot_projected_gravity[:, :, 2] + 1.0),
            dim=1,
        )

        # Core rewards
        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate),
            "reward_beam_position": rew.reward_beam_position(dist_beam_to_target),
            "reward_beam_midpoint_alignment": rew.reward_beam_midpoint_alignment(
                dist_beam_to_midpoint
            ),
            "reward_robot_support_slots": rew.reward_robot_support_slots(
                beam_support_error
            ),
            "reward_beam_progress": rew.reward_beam_progress(beam_progress_velocity),
            "reward_attachment_balance": rew.reward_attachment_balance(
                dist_attachment_balance
            ),
            "penalty_beam_tilt": (
                rew.penalty_beam_tilt(beam_tilt_metric)
                if not self.cfg.beam_kinematic_mount_enabled
                else torch.zeros_like(beam_tilt_metric)
            ),
            "penalty_beam_drop": (
                rew.penalty_beam_drop(beam_height_metric)
                if not self.cfg.beam_kinematic_mount_enabled
                else torch.zeros_like(beam_height_metric)
            ),
            "penalty_beam_speed": rew.penalty_beam_speed(beam_speed_excess_xy),
            "penalty_joint_torque": rew.penalty_joint_torque(joint_torque_sq),
            "penalty_joint_acceleration": rew.penalty_joint_acceleration(joint_acc_sq),
            "penalty_undesired_lin_vel_z": rew.penalty_undesired_lin_vel_z(
                undesired_lin_vel_z
            ),
            "penalty_undesired_ang_vel_xy": rew.penalty_undesired_ang_vel_xy(
                undesired_ang_vel_xy
            ),
            "penalty_gravity_rotation_alignment": rew.penalty_gravity_rotation_alignment(
                gravity_alignment
            ),
        }

        termination = self._beam_hold_steps >= self.cfg.success_hold_steps

        reward["reward_success"] = rew.reward_success(termination.float())

        # Observations
        robot_pos_local = robot_pos - self.scene.env_origins[:, None, :]
        beam_pos_local = beam_pos - self.scene.env_origins
        robot_midpoint_local = robot_midpoint - self.scene.env_origins
        robot_pos_xy_flat = robot_pos_local[:, :, :2].reshape(num_envs, -1)
        robot_vel_xy_flat = robot_vel[:, :, :2].reshape(num_envs, -1)
        robot_beam_xy_flat = robot_to_beam.reshape(num_envs, -1)
        beam_rot6d = rotmat_to_rot6d(beam_rotmat)
        beam_to_target = self._target_pos - beam_pos

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "robot_pos_xy": robot_pos_xy_flat,
                    "robot_vel_xy": robot_vel_xy_flat,
                    "beam_pos": beam_pos_local[:, :3],
                    "beam_rot6d": beam_rot6d,
                    "beam_to_target": beam_to_target[:, :3],
                    "robot_to_beam_xy": robot_beam_xy_flat,
                    "dist_beam_to_target": dist_beam_to_target[:, None],
                    "beam_support_error": beam_support_error[:, None],
                },
                ObservationGroup.STATE_DYN: {
                    "beam_lin_vel_xy": beam_lin_vel[:, :2],
                    "beam_ang_vel": beam_ang_vel[:, :3],
                    "robot_pos_midpoint": robot_midpoint_local[:, :3],
                },
                ObservationGroup.STATE_PRIVILEGED: {
                    "beam_tilt": beam_tilt_metric[:, None],
                    "beam_hold_steps": self._beam_hold_steps.float()[:, None],
                },
            },
            reward,
            termination,
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device),
            info=episode_outcome_info(
                success=termination,
                metrics={
                    "min_beam_distance_to_target": self._min_beam_distance_to_target,
                    "success_hold_steps": self._beam_hold_steps.float(),
                    "target_zone_radius_xy": torch.full(
                        (num_envs,), self.cfg.target_zone_radius_xy, device=device
                    ),
                },
            ),
        )


class Task(_BeamTransportTaskMixin, SrbEnv, GroundEnv):
    """Two-robot beam transport on the owned SRB lifecycle."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)
