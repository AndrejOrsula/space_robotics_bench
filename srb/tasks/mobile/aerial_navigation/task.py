from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.assets.scenery import MarsSurface
from srb.core.asset import Scenery
from srb.core.env import AerialEnv, AerialEnvCfg, AerialEventCfg, AerialSceneCfg
from srb.core.env.mobile import AerialMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.core.marker import CYLINDER_CFG, VisualizationMarkers
from srb.core.reward import (
    HyperbolicBarrierRewardTerm,
    HyperbolicRewardTerm,
    HyperbolicTrackingRewardTerm,
    LinearRewardTerm,
    OffsetLinearRewardTerm,
    ThresholdRewardTerm,
)
from srb.core.sim import PreviewSurfaceCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d


class RewardCfg(AerialMobileRewardCfg):
    reward_altitude_tracking: HyperbolicTrackingRewardTerm = (
        HyperbolicTrackingRewardTerm(weight=4.0, target=0.0, scale=1.0)
    )
    reward_waypoint_approach: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=16.0, scale=2.0
    )
    penalty_tilt_angle: OffsetLinearRewardTerm = OffsetLinearRewardTerm(
        weight=-2.0, limit=8.0, threshold=0.35, direction=">"
    )
    reward_terrain_clearance: HyperbolicBarrierRewardTerm = HyperbolicBarrierRewardTerm(
        weight=8.0, scale=0.5, threshold=1.0, direction="<"
    )
    penalty_energy: LinearRewardTerm = LinearRewardTerm(weight=-0.5, limit=4.0)
    reward_waypoint_reached: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=32.0, threshold=0.5, operator="<"
    )


@configclass
class SceneCfg(AerialSceneCfg):
    terrain: MarsSurface = MarsSurface()


@configclass
class EventCfg(AerialEventCfg):
    pass  # Using manual waypoint generation in _reset_idx


@configclass
class TaskCfg(AerialEnvCfg):
    scene: SceneCfg = SceneCfg()
    events: EventCfg = EventCfg()
    reward: RewardCfg = RewardCfg()
    episode_length_s: float = 60.0
    is_finite_horizon: bool = True
    command_vis: bool = True

    def __post_init__(self):
        super().__post_init__()
        # Convert raw scenery to a spawnable config; cloning also requires a plain prim name.
        terrain = self.scene.terrain
        if isinstance(terrain, Scenery):
            self.scene.terrain = terrain.as_asset_base_cfg()  # type: ignore[assignment]


class Task(SrbEnv, AerialEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)
        self._command = torch.zeros(self.num_envs, 3, device=self.device)
        # Distance to the pre-advance waypoint for this step's rewards.
        self._reward_dist_to_waypoint = torch.zeros(self.num_envs, device=self.device)
        self._altimeter = getattr(self, "_altimeter", None)
        if self._altimeter is None and hasattr(self.scene, "sensors"):
            self._altimeter = self.scene.sensors.get("altimeter")

        if self.cfg.command_vis or self.cfg.debug_vis:
            self._setup_visualization_markers()

    def _setup_visualization_markers(self):
        cfg = CYLINDER_CFG.copy()
        cfg.prim_path = "/Visuals/command/target_waypoint"
        cfg.markers["cylinder"].radius = 0.5
        cfg.markers["cylinder"].height = 0.1
        cfg.markers["cylinder"].visual_material = PreviewSurfaceCfg(
            emissive_color=(0.0, 1.0, 0.0)
        )
        self._marker_target_waypoint = VisualizationMarkers(cfg)

    def _update_visualization_markers(self):
        marker_pos = torch.zeros(
            (self.cfg.scene.num_envs, 3), dtype=torch.float32, device=self.device
        )
        marker_orientation = torch.zeros(
            (self.cfg.scene.num_envs, 4), dtype=torch.float32, device=self.device
        )
        marker_orientation[:, 3] = 1.0  # w (xyzw identity, w at last index)
        marker_scale = torch.ones(
            (self.cfg.scene.num_envs, 3), dtype=torch.float32, device=self.device
        )
        marker_pos[:, :2] = self._command[:, :2]
        marker_pos[:, 2] = self._command[:, 2]  # At target altitude
        self._marker_target_waypoint.visualize(
            marker_pos, marker_orientation, marker_scale
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        if len(env_ids) > 0:
            rand_xy = torch.rand((len(env_ids), 2), device=self.device) * 10.0 - 5.0
            # Clamp in the environment's local frame, not the world frame.
            origin_xy = self.scene.env_origins[env_ids, :2]
            self._command[env_ids, :2] = origin_xy + (
                self._robot.data.root_pos_w[env_ids, :2] - origin_xy + rand_xy
            ).clamp(-10.0, 10.0)
            self._command[env_ids, 2] = 5.0
            if self.cfg.command_vis or self.cfg.debug_vis:
                self._update_visualization_markers()

    def advance_step_state(self) -> None:
        # Record reward distance before assigning new waypoints.
        tf_pos_robot = self._robot.data.root_pos_w
        dist_to_waypoint = torch.norm(
            self._command[:, :2] - tf_pos_robot[:, :2], dim=-1
        )
        self._reward_dist_to_waypoint = dist_to_waypoint
        waypoint_reached = dist_to_waypoint < 0.5
        if waypoint_reached.any():
            reached_ids = waypoint_reached.nonzero(as_tuple=False).squeeze(-1)
            rand_xy = torch.rand((len(reached_ids), 2), device=self.device) * 10.0 - 5.0
            # Keep the new waypoint within the environment's local bounds.
            origin_xy = self.scene.env_origins[reached_ids, :2]
            self._command[reached_ids, :2] = origin_xy + (
                tf_pos_robot[reached_ids, :2] - origin_xy + rand_xy
            ).clamp(-10.0, 10.0)

        # Draw the marker after advancing so it matches the observed command.
        if self.cfg.command_vis or self.cfg.debug_vis:
            self._update_visualization_markers()

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
        device = self.device
        rew = self.cfg.reward

        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        tf_pos_robot = self._robot.data.root_pos_w
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_b
        vel_ang_robot = self._robot.data.root_ang_vel_b

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        # Terrain clearance via altimeter heights; fall back to env-local height.
        fallback_clearance = tf_pos_robot[:, 2] - self.scene.env_origins[:, 2]
        if (
            self._altimeter is not None
            and getattr(self._altimeter.data, "heights", None) is not None
        ):
            heights = self._altimeter.data.heights
            if len(heights.shape) >= 2 and heights.shape[1] > 0:
                # heights are attach-frame z, negative for downward hits -> abs for
                # positive clearance; amin over all altimeter cells picks the
                # nearest terrain and is robust to a >1-cell altimeter (crashes as
                # reshape(num_envs) otherwise), matching flight/moonfall.
                abs_h = torch.abs(heights.reshape(num_envs, -1))
                valid = getattr(self._altimeter.data, "valid_mask", None)
                if valid is not None:
                    # Ignore invalid cells, whose sentinel equals max_range.
                    vm = valid.reshape(num_envs, -1).bool()
                    abs_h = torch.where(vm, abs_h, torch.full_like(abs_h, float("inf")))
                    clearance = torch.where(
                        vm.any(dim=1), abs_h.amin(dim=1), fallback_clearance
                    )
                else:
                    clearance = abs_h.amin(dim=1)
            else:
                clearance = fallback_clearance
        else:
            clearance = fallback_clearance

        dist_to_waypoint = self._reward_dist_to_waypoint
        dist_to_target_alt = torch.abs(clearance - self._command[:, 2])

        local_up = tf_rotmat_robot[:, :, 2]
        tilt_angle = torch.acos(local_up[:, 2].clamp(-1.0, 1.0))

        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        energy_metric = torch.sum(torch.square(act_current), dim=1)

        reward = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "reward_altitude_tracking": rew.reward_altitude_tracking(
                dist_to_target_alt
            ),
            "reward_waypoint_approach": rew.reward_waypoint_approach(dist_to_waypoint),
            "penalty_tilt_angle": rew.penalty_tilt_angle(tilt_angle),
            "reward_terrain_clearance": rew.reward_terrain_clearance(clearance),
            "penalty_energy": rew.penalty_energy(energy_metric),
            "reward_waypoint_reached": rew.reward_waypoint_reached(dist_to_waypoint),
        }

        # Terminate if hitting the ground or excessive tilt
        termination = (clearance < 0.05) | (tilt_angle > 1.0)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    # Env-local, so replicas at different origins observe alike.
                    "tf_pos_robot": tf_pos_robot - self.scene.env_origins,
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "rel_waypoint": self._command[:, :2] - tf_pos_robot[:, :2],
                },
                ObservationGroup.PROPRIO: {
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                    "clearance": clearance.unsqueeze(-1),
                },
            },
            reward,
            termination,
            truncation,
        )
