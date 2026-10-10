from enum import Enum
from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.assets.scenery.space_station import EVA_WAYPOINT_PATH
from srb.core.asset import (
    AssetVariant,
    ExtravehicularScenery,
    Humanoid,
    IntravehicularScenery,
)
from srb.core.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
    ViewerCfg,
)
from srb.core.env.runtime import episode_outcome_info
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    BaseRewardCfg,
    GaussianAlignmentRewardTerm,
    GaussianRewardTerm,
    LinearRewardTerm,
    ScaledRewardTerm,
    SoftThresholdRewardTerm,
)
from srb.core.reward.spacewalk import (
    advance_waypoints,
    compute_approach_alignment,
    compute_nearest_handrail_distance,
    compute_reset_pose_range,
    compute_signed_progress,
    legacy_handrail_specs_to_segments,
    update_success_streak,
)
from srb.core.sim import ArrowCfg, PreviewSurfaceCfg
from srb.utils.cfg import configclass
from srb.utils.failures import InvalidInputError
from srb.utils.math import (
    matrix_from_quat,
    radians,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)


class SpacewalkMode(str, Enum):
    """Selects the physical context for the spacewalk task."""

    EVA = "eva"
    """Extravehicular Activity — exterior around the ISS+Gateway station."""
    IVA = "iva"
    """Intravehicular Activity — inside the Kibou module."""


# Keep the EVA path aligned with scenery handrails; IVA stays inside Kibou.
IVA_WAYPOINT_PATH: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (0.8, 0.0, 1.0),
    (0.8, 0.8, 1.0),
    (0.0, 0.8, 1.0),
)


class RewardCfg(BaseRewardCfg):
    """Dense waypoint approach rewards, goal stabilization, and drift control."""

    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-0.5, limit=8.0)
    """Penalises large joint-command changes."""

    penalty_angular_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=-2.0, limit=8.0
    )
    """Penalises angular velocity to limit tumbling."""

    reward_distance_to_target: GaussianRewardTerm = GaussianRewardTerm(
        weight=16.0, scale=2.5
    )
    """Gaussian distance reward with a 2.5 m scale."""

    reward_progress: ScaledRewardTerm = ScaledRewardTerm(weight=8.0, limit=8.0)
    """Rewards signed per-step progress while preserving its direction."""

    reward_approach_velocity: GaussianAlignmentRewardTerm = GaussianAlignmentRewardTerm(
        weight=12.0, scale=0.7854
    )
    """Aligns world-frame velocity with the target direction (45° scale)."""

    reward_reaching: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=32.0, threshold=0.5, operator="<", sharpness=4.0
    )
    """Soft bonus inside 0.5 m, independent of ``success_radius``."""

    reward_stability: GaussianRewardTerm = GaussianRewardTerm(weight=8.0, scale=0.1)
    """Rewards low world-frame speed inside the final goal region."""

    reward_success_hold: LinearRewardTerm = LinearRewardTerm(weight=10.0, limit=10.0)
    """Rewards sustained presence inside the final goal region."""

    penalty_drift_from_spacecraft: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=-6.0, threshold=9.0, operator=">", sharpness=2.0
    )
    """Penalises drift beyond 9 m from the waypoint-path centre."""


@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 16.0


@configclass
class EventCfg(OrbitalEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
                "roll": (-radians(5.0), radians(5.0)),
                "pitch": (-radians(5.0), radians(5.0)),
                "yaw": (-radians(5.0), radians(5.0)),
            },
        },
    )


@configclass
class TaskCfg(OrbitalEnvCfg):
    """Configure a humanoid waypoint traversal in EVA or IVA mode."""

    mode: SpacewalkMode = SpacewalkMode.EVA
    """EVA = exterior (open space); IVA = interior (Kibou module)."""

    robot: Humanoid | AssetVariant = assets.UnitreeG1()
    scenery: ExtravehicularScenery | IntravehicularScenery | AssetVariant | None = None

    scene: SceneCfg = SceneCfg()

    events: EventCfg = EventCfg()

    reward: RewardCfg = RewardCfg()

    episode_length_s: float = 20.0
    is_finite_horizon: bool = True

    # The 5 ms step keeps stiff finger joints stable and the policy at 25 Hz.
    env_rate: float = 1.0 / 200.0

    target_pos_range: float = 4.0

    use_waypoint_path: bool = True
    """When true, advance through ``waypoint_path`` instead of sampling one target."""

    waypoint_path: tuple[tuple[float, float, float], ...] = EVA_WAYPOINT_PATH
    """EVA path shared with ISSGatewayScenery handrails via ``EVA_WAYPOINT_PATH``."""

    no_handrail_distance: float = 10.0
    """Fallback rail-distance observation when the active scenery has no handrails."""

    success_radius: float = 0.5
    """Episode is successful when robot stays inside this radius around target."""

    success_stability_steps: int = 10

    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": ArrowCfg(
                tail_radius=0.02,
                tail_length=0.3,
                head_radius=0.1,
                head_length=0.15,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    viewer: ViewerCfg = ViewerCfg(
        eye=(0.0, -24.0, 26.0),
        lookat=(0.0, -5.0, 19.0),
        origin_type="env",
    )

    def __post_init__(self):
        # Validate success bounds and target sampling extent before reset.
        if self.success_radius <= 0.0:
            raise InvalidInputError("success_radius must be positive")
        if self.success_stability_steps < 1:
            raise InvalidInputError("success_stability_steps must be >= 1")
        if self.target_pos_range <= 0.0:
            raise InvalidInputError("target_pos_range must be positive")

        # A mode-only IVA override must use reachable interior waypoints.
        if (
            self.mode == SpacewalkMode.IVA
            and tuple(self.waypoint_path) == EVA_WAYPOINT_PATH
        ):
            self.waypoint_path = IVA_WAYPOINT_PATH

        if self.scenery is None:
            if self.mode == SpacewalkMode.IVA:
                self.scenery = assets.Kibou()
            else:
                self.scenery = assets.ISSGatewayScenery()

        super().__post_init__()
        self._sync_reset_pose_to_spacewalk_mode()

    def _sync_reset_pose_to_spacewalk_mode(self) -> None:
        params = dict(self.events.randomize_robot_state.params)
        params["pose_range"] = compute_reset_pose_range(
            base_pose_range=params["pose_range"],
            interior_origin=self.mode == SpacewalkMode.IVA,
            first_waypoint=(
                self.waypoint_path[0]
                if self.use_waypoint_path and self.waypoint_path
                else None
            ),
        )
        self.events.randomize_robot_state.params = params


@configclass
class TaskCfgEVA(TaskCfg):
    """Entrypoint config pre-selecting EVA mode."""

    mode: SpacewalkMode = SpacewalkMode.EVA


@configclass
class TaskCfgIVA(TaskCfg):
    """Entrypoint config pre-selecting IVA mode."""

    mode: SpacewalkMode = SpacewalkMode.IVA
    viewer: ViewerCfg = ViewerCfg(
        eye=(0.0, -2.1, 1.1),
        lookat=(0.0, 0.4, 0.7),
        origin_type="env",
    )
    waypoint_path: tuple[tuple[float, float, float], ...] = IVA_WAYPOINT_PATH
    """IVA traversal path close to Kibou interior origin for launch-time usability."""


class Task(SrbEnv, OrbitalEnv):
    """Humanoid spacewalk: traverse a rail-aligned waypoint path near a spacecraft.

    Observation:
        Robot orientation (rot6d), body-frame velocity, joint states, IMU,
        body-frame (egocentric) relative vector from robot to target waypoint,
        distance to the work-volume centre, traversal progress, and body-distance
        to the nearest handrail.

    Action:
        Joint position targets for the selected humanoid/jointed agent.

    Reward:
        Dense distance and progress shaping, approach alignment, reaching,
        final-goal stability and success-hold bonuses, and drift penalty.
    """

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._goal = torch.zeros(self.num_envs, 3, device=self.device)
        self._goal[:] = self.scene.env_origins
        self._waypoints_local = torch.tensor(
            self.cfg.waypoint_path,
            dtype=torch.float32,
            device=self.device,
        ).reshape(-1, 3)
        # Measure drift from the path centre so translated paths retain a zero
        # penalty throughout their working envelope.
        self._work_volume_centre_local = (
            self._waypoints_local.mean(dim=0)
            if self.cfg.use_waypoint_path and self._waypoints_local.shape[0] > 0
            else torch.zeros(3, dtype=torch.float32, device=self.device)
        )
        self._waypoint_index = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self._path_complete = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        (
            self._handrail_starts_local,
            self._handrail_ends_local,
            self._handrail_radii,
        ) = self._build_handrail_observation_tensors()

        self._prev_dist_to_target = torch.zeros(self.num_envs, device=self.device)
        # Cached before updating previous distance; consumed by extraction.
        self._progress_metric = torch.zeros(self.num_envs, device=self.device)
        self._success_streak = torch.zeros(
            self.num_envs, dtype=torch.int64, device=self.device
        )
        self._has_reached_goal = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._min_distance_to_target = torch.full(
            (self.num_envs,), float("inf"), device=self.device
        )
        # Track final-goal approach separately from the advancing current goal.
        self._min_distance_to_final = torch.full(
            (self.num_envs,), float("inf"), device=self.device
        )
        self._success_stability_steps = torch.tensor(
            self.cfg.success_stability_steps, dtype=torch.int64, device=self.device
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        num_reset = len(env_ids)
        half_range = self.cfg.target_pos_range

        self._waypoint_index[env_ids] = 0
        if self.cfg.use_waypoint_path and self._waypoints_local.numel() > 0:
            self._path_complete[env_ids] = self._waypoints_local.shape[0] <= 1
            self._goal[env_ids] = (
                self.scene.env_origins[env_ids] + self._waypoints_local[0]
            )
        else:
            self._path_complete[env_ids] = True
            self._goal[env_ids] = self.scene.env_origins[env_ids] + torch.empty(
                num_reset, 3, device=self.device
            ).uniform_(-half_range, half_range)

        # Seed distance so the first post-reset progress value is zero.
        robot_pos = _to_torch(self._robot.data.root_pos_w)[env_ids]
        self._prev_dist_to_target[env_ids] = torch.norm(
            robot_pos - self._goal[env_ids], dim=-1
        )
        self._progress_metric[env_ids] = 0.0
        self._success_streak[env_ids] = 0
        self._has_reached_goal[env_ids] = False
        self._min_distance_to_target[env_ids] = float("inf")
        self._min_distance_to_final[env_ids] = float("inf")

    def _uninitialized_step_return(self) -> StepReturn:
        """Return shape-valid zeros while pre-reset articulation buffers are
        placeholders.
        """
        n, device = self.num_envs, self.device
        n_joints = int(self._robot.num_joints)

        def z(*shape: int) -> torch.Tensor:
            return torch.zeros((n, *shape), device=device)

        bz = torch.zeros(n, dtype=torch.bool, device=device)
        reward = {
            key: z()
            for key in (
                "penalty_action_rate",
                "penalty_angular_velocity",
                "reward_distance_to_target",
                "reward_progress",
                "reward_approach_velocity",
                "reward_reaching",
                "reward_stability",
                "reward_success_hold",
                "penalty_drift_from_spacecraft",
            )
        }
        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_rot6d_robot": z(6),
                    "tf_pos_robot_to_target": z(3),
                    "dist_to_spacecraft": z(1),
                    "dist_to_nearest_handrail": z(1),
                    "waypoint_progress": z(1),
                },
                ObservationGroup.STATE_DYN: {
                    "vel_lin_robot": z(3),
                    "vel_ang_robot": z(3),
                },
                ObservationGroup.PROPRIO: {
                    "joint_pos": z(n_joints),
                    "imu_lin_acc": z(3),
                    "imu_ang_vel": z(3),
                },
                ObservationGroup.PROPRIO_DYN: {
                    "joint_vel": z(n_joints),
                },
            },
            reward,
            bz,
            bz,
        )

    def _robot_to_target(
        self, tf_pos_robot: torch.Tensor, tf_quat_robot: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Robot -> current goal (body frame), its norm and the near-target mask."""
        tf_pos_robot_to_target, _ = subtract_frame_transforms(
            t01=tf_pos_robot,
            q01=tf_quat_robot,
            t02=self._goal,
            q02=None,
        )
        dist_robot_to_target = torch.norm(tf_pos_robot_to_target, dim=-1)
        is_near_target = dist_robot_to_target <= self.cfg.success_radius
        return tf_pos_robot_to_target, dist_robot_to_target, is_near_target

    def advance_step_state(self) -> None:
        num_envs = self.num_envs
        device = self.device

        # Articulation buffers are placeholders until the first reset.
        tf_pos_robot = _to_torch(self._robot.data.root_pos_w)
        if tf_pos_robot.ndim != 2 or tf_pos_robot.shape[-1] != 3:
            return
        tf_quat_robot = _to_torch(self._robot.data.root_quat_w)

        _, dist_robot_to_target, is_near_target = self._robot_to_target(
            tf_pos_robot, tf_quat_robot
        )
        advanced_waypoint = torch.zeros(num_envs, dtype=torch.bool, device=device)
        if self.cfg.use_waypoint_path and self._waypoints_local.shape[0] > 1:
            last_waypoint = self._waypoints_local.shape[0] - 1
            self._waypoint_index, should_advance = advance_waypoints(
                waypoint_index=self._waypoint_index,
                is_near_target=is_near_target,
                last_index=last_waypoint,
            )
            if torch.any(should_advance):
                advanced_waypoint = should_advance
                self._goal = (
                    self.scene.env_origins + self._waypoints_local[self._waypoint_index]
                )
                _, dist_robot_to_target, is_near_target = self._robot_to_target(
                    tf_pos_robot, tf_quat_robot
                )
            self._path_complete = self._waypoint_index >= last_waypoint

        in_success_state = is_near_target & self._path_complete
        self._success_streak = update_success_streak(
            success_streak=self._success_streak,
            in_success_state=in_success_state,
            max_steps=self.cfg.success_stability_steps,
        )
        self._has_reached_goal |= self._success_streak >= self._success_stability_steps
        self._min_distance_to_target = torch.minimum(
            self._min_distance_to_target,
            dist_robot_to_target,
        )
        # Track the last path waypoint, or the single sampled goal.
        if self.cfg.use_waypoint_path and self._waypoints_local.shape[0] > 0:
            final_target = self.scene.env_origins + self._waypoints_local[-1]
        else:
            final_target = self._goal
        self._min_distance_to_final = torch.minimum(
            self._min_distance_to_final,
            torch.norm(tf_pos_robot - final_target, dim=-1),
        )

        self._progress_metric = compute_signed_progress(
            previous_dist=self._prev_dist_to_target,
            current_dist=dist_robot_to_target,
            advanced_mask=advanced_waypoint,
        )
        self._prev_dist_to_target = dist_robot_to_target.detach().clone()

        identity_quat = torch.zeros(num_envs, 4, device=device)
        identity_quat[:, 3] = 1.0  # xyzw identity (markers are xyzw in this fork)
        self._target_marker.visualize(self._goal, identity_quat)

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward
        num_envs = self.num_envs
        device = self.device

        # Convert ProxyArray before checking shape; its native shape omits the
        # vector dimension and otherwise triggers the placeholder path always.
        tf_pos_robot = _to_torch(self._robot.data.root_pos_w)
        if tf_pos_robot.ndim != 2 or tf_pos_robot.shape[-1] != 3:
            return self._uninitialized_step_return()

        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        tf_quat_robot = _to_torch(self._robot.data.root_quat_w)
        vel_lin_robot_b = _to_torch(
            self._robot.data.root_lin_vel_b
        )  # body frame (for obs)
        vel_ang_robot_b = _to_torch(self._robot.data.root_ang_vel_b)  # body frame
        vel_lin_robot_w = _to_torch(
            self._robot.data.root_lin_vel_w
        )  # world frame (for approach vel)

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        imu_lin_acc = _to_torch(self._imu_robot.data.lin_acc_b)
        imu_ang_vel = _to_torch(self._imu_robot.data.ang_vel_b)

        joint_pos = _to_torch(self._robot.data.joint_pos)
        joint_vel = _to_torch(self._robot.data.joint_vel)

        tf_pos_robot_to_target, dist_robot_to_target, is_near_target = (
            self._robot_to_target(tf_pos_robot, tf_quat_robot)
        )
        waypoint_progress = self._waypoint_index.float() / max(
            float(self._waypoints_local.shape[0] - 1),
            1.0,
        )
        dist_to_nearest_handrail = self._distance_to_nearest_handrail(tf_pos_robot)
        in_success_state = is_near_target & self._path_complete
        success_progress = self._success_streak.float() / max(
            float(self.cfg.success_stability_steps), 1.0
        )

        dist_to_spacecraft = torch.norm(
            tf_pos_robot - (self.scene.env_origins + self._work_volume_centre_local),
            dim=-1,
        )

        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        angular_velocity_metric = torch.norm(vel_ang_robot_b, dim=-1)

        progress_metric = self._progress_metric

        approach_cos_sim = compute_approach_alignment(
            position=tf_pos_robot,
            goal=self._goal,
            velocity=vel_lin_robot_w,
        )

        lin_vel_magnitude = torch.norm(vel_lin_robot_w, dim=-1)

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_angular_velocity": rew.penalty_angular_velocity(
                angular_velocity_metric
            ),
            "reward_distance_to_target": rew.reward_distance_to_target(
                dist_robot_to_target
            ),
            "reward_progress": rew.reward_progress(progress_metric),
            "reward_approach_velocity": rew.reward_approach_velocity(approach_cos_sim),
            "reward_reaching": rew.reward_reaching(dist_robot_to_target),
            "reward_stability": torch.where(
                in_success_state,
                rew.reward_stability(lin_vel_magnitude),
                torch.zeros_like(lin_vel_magnitude),
            ),
            "reward_success_hold": rew.reward_success_hold(success_progress),
            "penalty_drift_from_spacecraft": rew.penalty_drift_from_spacecraft(
                dist_to_spacecraft
            ),
        }

        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )
        last_step = self.episode_length_buf >= self.max_episode_length

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "tf_pos_robot_to_target": tf_pos_robot_to_target,
                    "dist_to_spacecraft": dist_to_spacecraft.unsqueeze(-1),
                    "dist_to_nearest_handrail": dist_to_nearest_handrail.unsqueeze(-1),
                    "waypoint_progress": waypoint_progress.unsqueeze(-1),
                },
                ObservationGroup.STATE_DYN: {
                    "vel_lin_robot": vel_lin_robot_b,
                    "vel_ang_robot": vel_ang_robot_b,
                },
                ObservationGroup.PROPRIO: {
                    "joint_pos": joint_pos,
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                },
                ObservationGroup.PROPRIO_DYN: {
                    "joint_vel": joint_vel,
                },
            },
            reward,
            termination,
            truncation,
            info=episode_outcome_info(
                success=self._has_reached_goal,
                metrics={
                    "min_distance_to_target": self._min_distance_to_target,
                    "min_distance_to_final_target": self._min_distance_to_final,
                    "waypoint_index": self._waypoint_index.float(),
                    "waypoint_progress": waypoint_progress,
                    "dist_to_nearest_handrail": dist_to_nearest_handrail,
                    "success_radius": self.cfg.success_radius,
                    "success_stability_steps": self.cfg.success_stability_steps,
                    "last_step": last_step,
                },
            ),
        )

    def _build_handrail_observation_tensors(
        self,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        scenery = self.cfg.scenery
        has_handrails = bool(getattr(scenery, "add_handrails", False))
        segment_specs = (
            getattr(scenery, "handrail_segment_specs", ()) if has_handrails else ()
        )
        legacy_specs = getattr(scenery, "handrail_specs", ()) if has_handrails else ()
        segments = list(segment_specs)
        # Expand legacy specs the same way as the scenery spawner.
        segments.extend(legacy_handrail_specs_to_segments(tuple(legacy_specs)))
        if not segments:
            # Match populated tensor ranks for callers that skip the empty guard.
            empty_segments = torch.empty(0, 3, device=self.device)
            empty_radii = torch.empty(0, device=self.device)
            return empty_segments, empty_segments, empty_radii

        starts = torch.tensor(
            [spec[0] for spec in segments],
            dtype=torch.float32,
            device=self.device,
        )
        ends = torch.tensor(
            [spec[1] for spec in segments],
            dtype=torch.float32,
            device=self.device,
        )
        radii = torch.tensor(
            [spec[2] for spec in segments],
            dtype=torch.float32,
            device=self.device,
        )
        return starts, ends, radii

    def _distance_to_nearest_handrail(self, robot_pos_w: torch.Tensor) -> torch.Tensor:
        if self._handrail_starts_local.numel() == 0:
            return torch.full(
                (self.num_envs,),
                self.cfg.no_handrail_distance,
                dtype=torch.float32,
                device=self.device,
            )

        starts = (
            self.scene.env_origins[:, None, :] + self._handrail_starts_local[None, :, :]
        )
        ends = (
            self.scene.env_origins[:, None, :] + self._handrail_ends_local[None, :, :]
        )
        return compute_nearest_handrail_distance(
            points=robot_pos_w,
            seg_starts=starts,
            seg_ends=ends,
            seg_radii=self._handrail_radii,
            fallback=self.cfg.no_handrail_distance,
        )


def _to_torch(value: torch.Tensor) -> torch.Tensor:
    """Unwrap ProxyArray reads so helpers receive torch shapes, not warp shapes."""

    return value.torch if hasattr(value, "torch") else value
