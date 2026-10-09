from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import Articulation, AssetVariant, GroundRobot, Robot, Terrain
from srb.core.env import GroundEnv, GroundEnvCfg, GroundEventCfg, GroundSceneCfg
from srb.core.env.mobile import GroundMobileRewardCfg
from srb.core.env.runtime import episode_outcome_info
from srb.core.env.srb_env import SrbEnv
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import offset_pose_natural, reset_pose_natural
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import LinearRewardTerm, ScaledRewardTerm
from srb.core.reward.formation import (
    compute_mean_planar_formation_progress,
    compute_mean_planar_velocity_tracking,
)
from srb.core.sim import PreviewSurfaceCfg
from srb.core.sim.spawners.shapes.extras.cfg import PinnedArrowCfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, quat_apply, subtract_frame_transforms


class RewardCfg(GroundMobileRewardCfg):
    """Reward configuration for formation tracking."""

    penalty_formation_tracking: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=4.0
    )

    reward_formation_progress: ScaledRewardTerm = ScaledRewardTerm(
        weight=4.0, limit=2.0
    )

    reward_formation_velocity: LinearRewardTerm = LinearRewardTerm(
        weight=2.0, limit=2.0
    )

    reward_point_towards_target: LinearRewardTerm = LinearRewardTerm(
        weight=0.5, limit=0.5
    )

    reward_formation_precision: LinearRewardTerm = LinearRewardTerm(
        weight=4.0, limit=4.0
    )

    reward_in_zone: LinearRewardTerm = LinearRewardTerm(weight=0.0, limit=10.0)

    reward_formation_orientation: LinearRewardTerm = LinearRewardTerm(
        weight=4.0, limit=4.0
    )

    reward_action_rate_in_formation: LinearRewardTerm = LinearRewardTerm(
        weight=32.0, limit=32.0
    )
    tanh_std_point_towards_target: float = 0.7854
    tanh_std_formation_velocity: float = 0.20
    tanh_std_formation_precision: float = 0.2
    tanh_std_formation_orientation: float = 0.3491
    tanh_std_action_rate_in_formation: float = 0.2


@configclass
class SceneCfg(GroundSceneCfg):
    pass


@configclass
class EventCfg(GroundEventCfg):
    formation_center_evolution: SrbEventTermCfg = SrbEventTermCfg(
        func=offset_pose_natural,
        mode="interval",
        interval_range_s=(0.05, 0.05),
        is_global_time=True,
        category=RandomizationCategory.TASK,
        params={
            "env_attr_name": "_formation_center",
            "pos_axes": ("x", "y"),
            "pos_step_range": (0.003, 0.006),
            "pos_smoothness": 0.995,
            "pos_step_smoothness": 0.85,
            "pos_bounds": {
                "x": MISSING,
                "y": MISSING,
            },
            "orient_yaw_only": True,
            "orient_smoothness": 0.9,
        },
    )


@configclass
class TaskCfg(GroundEnvCfg):
    """Configuration for robots following a moving formation center."""

    scenery: Terrain | AssetVariant | None = assets.GroundPlane()
    robot: GroundRobot | AssetVariant = assets.Cadre()
    extra_robots: dict[str, Robot | AssetVariant] = {}

    scene: SceneCfg = SceneCfg()
    stack: bool = True

    events: EventCfg = EventCfg()

    reward: RewardCfg = RewardCfg()

    episode_length_s: float = 60.0
    is_finite_horizon: bool = False
    success_hold_steps: int = 8
    formation_success_position_tolerance_xy: float = 0.20
    formation_success_yaw_tolerance: float = 0.3490658503988659

    # Local-frame (x, y) offsets, one per robot.
    formation_offsets: tuple[tuple[float, float], ...] = (
        (1.5, 0.0),  # Front position
        (-0.75, 1.3),  # Rear-left position
        (-0.75, -1.3),  # Rear-right position
    )
    randomize_formation_assignment: bool = False
    formation_scale: float = 1.0
    # Single-rover training uses a shared per-rover policy; evaluation reconstructs the team.
    single_robot: bool = False
    # Keep the decentralized episode running after success so shaping rewards continue.
    terminate_on_success: bool = True
    # Differential-drive single-rover control cannot always hold slot and yaw while the center drifts.
    require_yaw_match: bool = True
    # 1.0 uses default drift; 0.0 holds a static center for station-keeping.
    center_drift_scale: float = 1.0

    target_pos_range_ratio: float = 0.9
    formation_center_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/formation_center",
        markers={
            "center": PinnedArrowCfg(
                pin_radius=0.015,
                pin_length=2.5,
                tail_radius=0.015,
                tail_length=0.25,
                head_radius=0.05,
                head_length=0.1,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.8, 0.2, 0.2)),
            )
        },
    )
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": PinnedArrowCfg(
                pin_radius=0.01,
                pin_length=2.0,
                tail_radius=0.01,
                tail_length=0.2,
                head_radius=0.04,
                head_length=0.08,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        self.extra_robots = dict(self.extra_robots)
        if not self.extra_robots and not self.single_robot:
            self.extra_robots["robot_1"] = assets.Cadre()
            self.extra_robots["robot_2"] = assets.Cadre()

        super().__post_init__()

        if self.center_drift_scale != 1.0:
            lo, hi = self.events.formation_center_evolution.params["pos_step_range"]  # type: ignore[attr-defined]
            self.events.formation_center_evolution.params["pos_step_range"] = (  # type: ignore[attr-defined]
                lo * self.center_drift_scale,
                hi * self.center_drift_scale,
            )

        if (
            "hardcoded"
            not in self.events.formation_center_evolution.params["pos_bounds"].keys()  # type: ignore[attr-defined]
        ):
            spacing = (
                self.spacing if self.spacing is not None else self.scene.env_spacing
            )
            assert spacing is not None
            max_formation_radius = max(
                (x**2 + y**2) ** 0.5 for x, y in self.formation_offsets
            )
            effective_range = self.target_pos_range_ratio * spacing - (
                2.0 * max_formation_radius * self.formation_scale
            )
            for dim in ("x", "y"):
                self.events.formation_center_evolution.params["pos_bounds"][dim] = (  # type: ignore[attr-defined]
                    -0.5 * effective_range,
                    0.5 * effective_range,
                )


@configclass
class TaskCfgSingle(TaskCfg):
    """Single-rover variant for decentralized, slot-agnostic control."""

    single_robot: bool = True
    terminate_on_success: bool = False
    require_yaw_match: bool = False


class _FormationTaskMixin:
    """Shared target, state, and reward logic for formation tracking."""

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._formation_center_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.formation_center_marker_cfg
        )
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._all_robot_names: list[str] = ["robot"] + sorted(
            self.cfg.extra_robots.keys()
        )
        self._all_robots: list[Articulation] = [
            self._robots[name] for name in self._all_robot_names
        ]
        self._num_robots: int = len(self._all_robots)

        # Single-rover training uses the first slot; trailing offsets are irrelevant.
        formation_offsets = self.cfg.formation_offsets
        if self.cfg.single_robot:
            formation_offsets = formation_offsets[: self._num_robots]
        if self._num_robots != len(formation_offsets):
            raise ValueError(
                f"Number of robots ({self._num_robots}) must match the number of "
                f"formation positions ({len(formation_offsets)})"
            )
        self._formation_offsets_base = torch.tensor(
            formation_offsets, device=self.device, dtype=torch.float32
        )
        self._formation_offsets_base *= self.cfg.formation_scale

        self._formation_center = torch.zeros(self.num_envs, 7, device=self.device)
        self._formation_center[:, 0:3] = self.scene.env_origins
        self._formation_center[:, 6] = 1.0

        # Anchor at the last actual move because the global-time event skips steps.
        self._formation_center_prev = self._formation_center.clone()
        self._steps_since_center_change = torch.zeros(
            self.num_envs, dtype=torch.int32, device=self.device
        )
        self._formation_center_vel = torch.zeros(self.num_envs, 3, device=self.device)
        self._formation_center_tf_reanchor = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )

        self._formation_assignment = (
            torch.arange(self._num_robots, device=self.device)
            .unsqueeze(0)
            .expand(self.num_envs, -1)
            .clone()
        )

        self._goals = torch.zeros(
            self.num_envs, self._num_robots, 7, device=self.device
        )
        self._goals[:, :, 6] = 1.0

        self._success_hold_steps = torch.zeros(
            self.num_envs, dtype=torch.int32, device=self.device
        )
        # Preserve success through truncation when success does not terminate the episode.
        self._episode_success_latch = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._prev_dist2d_to_target = torch.zeros(
            self.num_envs, self._num_robots, device=self.device
        )
        # Extraction may repeat after selective reset; keep its progress reference unchanged.
        self._prev_dist2d_to_target_pre = self._prev_dist2d_to_target.clone()

        self._episodic_noise_tf_pos2d = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._episodic_noise_tf_yaw = torch.zeros(self.num_envs, device=self.device)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        num_reset_envs = len(env_ids)

        self._formation_center[env_ids, 0:3] = self.scene.env_origins[env_ids]
        self._formation_center[env_ids, 3:7] = torch.tensor(
            [0.0, 0.0, 0.0, 1.0], device=self.device
        )
        self._formation_center_prev[env_ids] = self._formation_center[env_ids].clone()
        self._steps_since_center_change[env_ids] = 0
        self._formation_center_vel[env_ids] = 0.0
        self._formation_center_tf_reanchor[env_ids] = hasattr(
            self, "tf_buffer"
        ) or hasattr(self.unwrapped, "ros_node")
        # Clear natural-motion state to avoid carrying drift and yaw across episodes.
        reset_pose_natural(self, env_ids, "_formation_center")

        if self.cfg.randomize_formation_assignment:
            for eid in env_ids:
                self._formation_assignment[eid] = torch.randperm(
                    self._num_robots, device=self.device
                )
        else:
            self._formation_assignment[env_ids] = (
                torch.arange(self._num_robots, device=self.device)
                .unsqueeze(0)
                .expand(num_reset_envs, -1)
            )

        self._success_hold_steps[env_ids] = 0
        self._episode_success_latch[env_ids] = False

        self._update_goal_positions(env_ids)
        self._reset_progress_distance_buffer(env_ids)

        self._episodic_noise_tf_pos2d[env_ids] = torch.normal(
            mean=0.0,
            std=0.01,
            size=(num_reset_envs, 2),
            device=self.device,
        )
        self._episodic_noise_tf_yaw[env_ids] = torch.normal(
            mean=0.0,
            std=0.043633231,  # 2.5 deg
            size=(num_reset_envs,),
            device=self.device,
        )

        if hasattr(self.unwrapped, "ros_node") and not hasattr(self, "tf_buffer"):
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer,
                self.unwrapped.ros_node,  # type: ignore[misc]
            )
            logging.info("TF listener initialized for formation center.")
            # Disable formation center evolution event if using ROS TF
            self.cfg.events.formation_center_evolution.interval_range_s = (
                1.0e9,
                1.0e9,
            )
            self.event_manager.set_term_cfg(
                "formation_center_evolution",
                self.cfg.events.formation_center_evolution,
            )

    def _update_goal_positions(self, env_ids: Sequence[int] | None = None):
        """Compute assigned target poses from the center pose and local offsets."""
        if env_ids is None:
            env_ids_tensor = torch.arange(self.num_envs, device=self.device)
        else:
            env_ids_tensor = torch.as_tensor(
                env_ids,
                dtype=torch.long,
                device=self.device,
            )

        n = len(env_ids_tensor)

        assignment = self._formation_assignment[env_ids_tensor]  # [n, num_robots]
        offsets_2d = self._formation_offsets_base[assignment]  # [n, num_robots, 2]

        offsets_3d = torch.zeros(n, self._num_robots, 3, device=self.device)
        offsets_3d[:, :, 0:2] = offsets_2d

        center_quat = self._formation_center[env_ids_tensor, 3:7]  # [n, 4]
        center_quat_flat = center_quat.unsqueeze(1).expand(-1, self._num_robots, -1)
        center_quat_flat = center_quat_flat.reshape(-1, 4)
        offsets_3d_flat = offsets_3d.reshape(-1, 3)
        rotated_offsets = quat_apply(center_quat_flat, offsets_3d_flat).reshape(
            n, self._num_robots, 3
        )

        center_pos = self._formation_center[env_ids_tensor, 0:3]  # [n, 3]
        self._goals[env_ids_tensor, :, 0:3] = center_pos.unsqueeze(1) + rotated_offsets
        self._goals[env_ids_tensor, :, 3:7] = center_quat.unsqueeze(1).expand(
            -1, self._num_robots, -1
        )

    def _reset_progress_distance_buffer(self, env_ids: Sequence[int]):
        env_ids_tensor = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        tf_pos_robots = torch.stack(
            [
                robot.data.root_link_pose_w[env_ids_tensor, 0:3]
                for robot in self._all_robots
            ],
            dim=1,
        )
        tf_quat_robots = torch.stack(
            [
                robot.data.root_link_pose_w[env_ids_tensor, 3:7]
                for robot in self._all_robots
            ],
            dim=1,
        )
        num_envs = len(env_ids_tensor)
        tf_pos_to_target_flat, _ = subtract_frame_transforms(
            t01=tf_pos_robots.reshape(num_envs * self._num_robots, 3),
            q01=tf_quat_robots.reshape(num_envs * self._num_robots, 4),
            t02=self._goals[env_ids_tensor, :, 0:3].reshape(
                num_envs * self._num_robots, 3
            ),
            q02=self._goals[env_ids_tensor, :, 3:7].reshape(
                num_envs * self._num_robots, 4
            ),
        )
        tf_pos_to_target = tf_pos_to_target_flat.reshape(num_envs, self._num_robots, 3)
        distance = torch.linalg.norm(tf_pos_to_target[:, :, 0:2], dim=-1)
        self._prev_dist2d_to_target[env_ids_tensor] = distance
        self._prev_dist2d_to_target_pre[env_ids_tensor] = distance

    def _current_dist2d_to_target(self) -> torch.Tensor:
        """Measure robot-frame planar slot distances without changing state."""

        tf_pos_robots = torch.stack(
            [robot.data.root_link_pose_w[:, 0:3] for robot in self._all_robots],
            dim=1,
        )
        tf_quat_robots = torch.stack(
            [robot.data.root_link_pose_w[:, 3:7] for robot in self._all_robots],
            dim=1,
        )
        tf_pos_flat = tf_pos_robots.reshape(self.num_envs * self._num_robots, 3)
        tf_quat_flat = tf_quat_robots.reshape(self.num_envs * self._num_robots, 4)
        tf_pos_targets_flat = self._goals[:, :, 0:3].reshape(
            self.num_envs * self._num_robots, 3
        )
        tf_pos_to_target_flat, _ = subtract_frame_transforms(
            t01=tf_pos_flat,
            q01=tf_quat_flat,
            t02=tf_pos_targets_flat,
            q02=self._goals[:, :, 3:7].reshape(self.num_envs * self._num_robots, 4),
        )
        tf_pos_to_target = tf_pos_to_target_flat.reshape(
            self.num_envs, self._num_robots, 3
        )
        return torch.linalg.norm(tf_pos_to_target[:, :, 0:2], dim=-1)

    def advance_step_state(self) -> None:
        # Divide displacement by elapsed time since the center last moved; the event skips steps.
        self._steps_since_center_change += 1
        center_displacement = (
            self._formation_center[:, 0:3] - self._formation_center_prev[:, 0:3]
        )
        center_moved = torch.linalg.norm(center_displacement, dim=-1) > 1e-8
        elapsed_s = (
            self._steps_since_center_change.to(center_displacement.dtype) * self.step_dt
        )
        new_center_vel = center_displacement / elapsed_s.clamp_min(
            self.step_dt
        ).unsqueeze(-1)
        self._formation_center_vel = torch.where(
            center_moved.unsqueeze(-1), new_center_vel, self._formation_center_vel
        )
        # Masked assignment synchronizes with the CPU; where keeps this per-step update fused.
        self._formation_center_prev = torch.where(
            center_moved.unsqueeze(-1),
            self._formation_center,
            self._formation_center_prev,
        )
        self._steps_since_center_change = torch.where(
            center_moved,
            torch.zeros_like(self._steps_since_center_change),
            self._steps_since_center_change,
        )
        if hasattr(self, "tf_buffer"):
            from rclpy.time import Duration, Time

            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    "world",
                    "formation_center",
                    Time(),
                    Duration(
                        seconds=0,
                        nanoseconds=200000000,
                    ),
                )
                logging.debug(
                    f"Got transform from 'world' to 'formation_center': {tf_stamped}"
                )
                # TF is env-local; add each origin so unstacked environments get their own pose.
                translation = tf_stamped.transform.translation
                self._formation_center[:, 0:3] = self.scene.env_origins + torch.tensor(
                    (translation.x, translation.y, translation.z),
                    dtype=self._formation_center.dtype,
                    device=self._formation_center.device,
                )
                self._formation_center[:, 3] = tf_stamped.transform.rotation.x
                self._formation_center[:, 4] = tf_stamped.transform.rotation.y
                self._formation_center[:, 5] = tf_stamped.transform.rotation.z
                self._formation_center[:, 6] = tf_stamped.transform.rotation.w
                reanchor = self._formation_center_tf_reanchor
                self._formation_center_prev[reanchor] = self._formation_center[
                    reanchor
                ].clone()
                self._steps_since_center_change[reanchor] = 0
                self._formation_center_vel[reanchor] = 0.0
                self._formation_center_tf_reanchor[reanchor] = False
            except Exception as e:
                logging.warning(
                    f"Failed to get transform from 'world' to 'formation_center': {e}"
                )

        self._update_goal_positions()

        # Advance once per step; the mirror retains the pre-advance reward reference.
        self._prev_dist2d_to_target_pre.copy_(self._prev_dist2d_to_target)
        current_dist2d_to_target = self._current_dist2d_to_target()
        self._prev_dist2d_to_target.copy_(current_dist2d_to_target.detach())

        tf_pos_robots = torch.stack(
            [robot.data.root_link_pose_w[:, 0:3] for robot in self._all_robots],
            dim=1,
        )
        tf_quat_robots = torch.stack(
            [robot.data.root_link_pose_w[:, 3:7] for robot in self._all_robots],
            dim=1,
        )
        tf_pos_flat = tf_pos_robots.reshape(self.num_envs * self._num_robots, 3)
        tf_quat_flat = tf_quat_robots.reshape(self.num_envs * self._num_robots, 4)
        tf_pos_targets_flat = self._goals[:, :, 0:3].reshape(
            self.num_envs * self._num_robots, 3
        )
        tf_quat_targets_flat = self._goals[:, :, 3:7].reshape(
            self.num_envs * self._num_robots, 4
        )
        _, tf_quat_to_target_flat = subtract_frame_transforms(
            t01=tf_pos_flat,
            q01=tf_quat_flat,
            t02=tf_pos_targets_flat,
            q02=tf_quat_targets_flat,
        )
        tf_rotmat_to_target = matrix_from_quat(tf_quat_to_target_flat).reshape(
            self.num_envs, self._num_robots, 3, 3
        )
        yaw_to_target = torch.atan2(
            tf_rotmat_to_target[:, :, 1, 0], tf_rotmat_to_target[:, :, 0, 0]
        )
        formation_pos_ok = torch.all(
            current_dist2d_to_target
            <= self.cfg.formation_success_position_tolerance_xy,
            dim=1,
        )
        yaw_ok = torch.all(
            torch.abs(yaw_to_target) <= self.cfg.formation_success_yaw_tolerance,
            dim=1,
        )
        successful_step = (
            (formation_pos_ok & yaw_ok)
            if self.cfg.require_yaw_match
            else formation_pos_ok
        )
        self._success_hold_steps = torch.where(
            successful_step,
            torch.clamp(self._success_hold_steps + 1, max=self.cfg.success_hold_steps),
            torch.zeros_like(self._success_hold_steps),
        )
        held = self._success_hold_steps >= self.cfg.success_hold_steps
        self._episode_success_latch |= held

        self._formation_center_marker.visualize(
            self._formation_center[:, 0:3], self._formation_center[:, 3:7]
        )
        self._target_marker.visualize(self._goals[:, 0, 0:3], self._goals[:, 0, 3:7])

    def extract_step_return(self) -> StepReturn:
        formation_center_vel = self._formation_center_vel

        tf_pos_robots = torch.stack(
            [r.data.root_link_pose_w[:, 0:3] for r in self._all_robots], dim=1
        )
        tf_quat_robots = torch.stack(
            [r.data.root_link_pose_w[:, 3:7] for r in self._all_robots], dim=1
        )
        tf_lin_vel_robots = torch.stack(
            [r.data.root_lin_vel_w[:, 0:3] for r in self._all_robots], dim=1
        )

        return self._compute_step_return_impl(
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            tf_pos_robots=tf_pos_robots,
            tf_quat_robots=tf_quat_robots,
            tf_lin_vel_robots=tf_lin_vel_robots,
            tf_pos_targets=self._goals[:, :, 0:3],
            tf_quat_targets=self._goals[:, :, 3:7],
            formation_center_vel=formation_center_vel,
        )

    def _compute_step_return_impl(
        self,
        *,
        act_current: torch.Tensor,
        act_previous: torch.Tensor,
        tf_pos_robots: torch.Tensor,
        tf_quat_robots: torch.Tensor,
        tf_lin_vel_robots: torch.Tensor,
        tf_pos_targets: torch.Tensor,
        tf_quat_targets: torch.Tensor,
        formation_center_vel: torch.Tensor,
    ) -> StepReturn:
        num_envs = self.num_envs
        num_robots = tf_pos_robots.size(1)
        device = self.device

        rew = self.cfg.reward

        tf_pos_flat = tf_pos_robots.reshape(num_envs * num_robots, 3)
        tf_quat_flat = tf_quat_robots.reshape(num_envs * num_robots, 4)
        tf_pos_targets_flat = tf_pos_targets.reshape(num_envs * num_robots, 3)
        tf_quat_targets_flat = tf_quat_targets.reshape(num_envs * num_robots, 4)

        tf_pos_to_target_flat, _tf_quat_to_target_flat = subtract_frame_transforms(
            t01=tf_pos_flat,
            q01=tf_quat_flat,
            t02=tf_pos_targets_flat,
            q02=tf_quat_targets_flat,
        )
        tf_rotmat_to_target_flat = matrix_from_quat(_tf_quat_to_target_flat)

        tf_pos_to_target = tf_pos_to_target_flat.reshape(num_envs, num_robots, 3)
        tf_rotmat_to_target = tf_rotmat_to_target_flat.reshape(
            num_envs, num_robots, 3, 3
        )

        tf_pos2d_to_target = tf_pos_to_target[:, :, :2]
        dist2d_to_target = torch.norm(tf_pos2d_to_target, dim=-1)
        angle_to_target_pos = torch.atan2(
            tf_pos_to_target[:, :, 1], tf_pos_to_target[:, :, 0]
        )
        yaw_to_target = torch.atan2(
            tf_rotmat_to_target[:, :, 1, 0], tf_rotmat_to_target[:, :, 0, 0]
        )

        formation_center_vel_2d = formation_center_vel[:, :2]
        robot_vel_xy = tf_lin_vel_robots[:, :, :2]

        tf_rot2dtrigyaw_to_target = torch.stack(
            (torch.sin(yaw_to_target), torch.cos(yaw_to_target)),
            dim=-1,
        )

        per_robot_pos2d = tf_pos2d_to_target.reshape(num_envs, -1)
        per_robot_yaw2d = tf_rot2dtrigyaw_to_target.reshape(num_envs, -1)
        per_robot_vel_xy = robot_vel_xy.reshape(num_envs, -1)

        _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)

        formation_tracking_metric = torch.mean(torch.square(dist2d_to_target), dim=-1)
        formation_progress_metric = compute_mean_planar_formation_progress(
            previous_dist2d=self._prev_dist2d_to_target_pre,
            current_dist2d=dist2d_to_target,
            step_dt=self.step_dt,
        )
        formation_velocity_metric = compute_mean_planar_velocity_tracking(
            robot_vel_xy=robot_vel_xy,
            target_vel_xy=formation_center_vel_2d,
            std=rew.tanh_std_formation_velocity,
        )
        point_towards_metric = torch.mean(
            1.0
            - torch.tanh(
                torch.abs(angle_to_target_pos) / rew.tanh_std_point_towards_target
            ),
            dim=-1,
        )
        _formation_precision = 1.0 - torch.tanh(
            dist2d_to_target / rew.tanh_std_formation_precision
        )
        formation_precision_metric = torch.mean(_formation_precision, dim=-1)
        _orientation_precision = _formation_precision * (
            1.0
            - torch.tanh(torch.abs(yaw_to_target) / rew.tanh_std_formation_orientation)
        )
        formation_orientation_metric = torch.mean(_orientation_precision, dim=-1)
        _mean_orientation_precision = torch.mean(_orientation_precision, dim=-1)
        action_rate_in_formation_metric = _mean_orientation_precision * (
            1.0 - torch.tanh(_action_rate / rew.tanh_std_action_rate_in_formation)
        )

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(_action_rate),
            "penalty_formation_tracking": rew.penalty_formation_tracking(
                formation_tracking_metric
            ),
            "reward_formation_progress": rew.reward_formation_progress(
                formation_progress_metric
            ),
            "reward_formation_velocity": rew.reward_formation_velocity(
                formation_velocity_metric
            ),
            "reward_point_towards_target": rew.reward_point_towards_target(
                point_towards_metric
            ),
            "reward_formation_precision": rew.reward_formation_precision(
                formation_precision_metric
            ),
            "reward_formation_orientation": rew.reward_formation_orientation(
                formation_orientation_metric
            ),
            "reward_action_rate_in_formation": rew.reward_action_rate_in_formation(
                action_rate_in_formation_metric
            ),
        }
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        formation_pos_ok = torch.all(
            dist2d_to_target <= self.cfg.formation_success_position_tolerance_xy,
            dim=1,
        )
        yaw_ok = torch.all(
            torch.abs(yaw_to_target) <= self.cfg.formation_success_yaw_tolerance,
            dim=1,
        )
        successful_step = (
            (formation_pos_ok & yaw_ok)
            if self.cfg.require_yaw_match
            else formation_pos_ok
        )
        reward["reward_in_zone"] = rew.reward_in_zone(successful_step.float())
        held = self._success_hold_steps >= self.cfg.success_hold_steps
        if self.cfg.terminate_on_success:
            termination = held
        else:
            termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        episode_success = self._episode_success_latch.clone()

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "tf_pos2d_robots_to_targets": per_robot_pos2d,
                    "tf_rot2dtrigyaw_robots_to_targets": per_robot_yaw2d,
                    "formation_center_vel_2d": formation_center_vel_2d,
                },
                ObservationGroup.STATE_DYN: {
                    "robot_vel_xy": per_robot_vel_xy,
                },
            },
            reward,
            termination,
            truncation,
            info=episode_outcome_info(
                success=episode_success,
                metrics={
                    "formation_distance_error": torch.mean(dist2d_to_target, dim=1),
                    "formation_orientation_error": torch.mean(
                        torch.abs(yaw_to_target), dim=1
                    ),
                    "success_hold_steps": self._success_hold_steps.float(),
                    "formation_success_position_tolerance_xy": torch.full(
                        (num_envs,),
                        self.cfg.formation_success_position_tolerance_xy,
                        device=device,
                    ),
                    "formation_success_yaw_tolerance": torch.full(
                        (num_envs,),
                        self.cfg.formation_success_yaw_tolerance,
                        device=device,
                    ),
                },
            ),
        )


class Task(_FormationTaskMixin, SrbEnv, GroundEnv):
    """Multi-robot formation following on the owned SRB lifecycle."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)
