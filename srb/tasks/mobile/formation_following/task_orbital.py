from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.action import ThrustActionGroup
from srb.core.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
    ViewerCfg,
)
from srb.core.env.mobile import OrbitalMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import (
    apply_external_force_torque,
    offset_pose_natural,
    reset_pose_natural,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import LinearRewardTerm
from srb.core.sim import ArrowCfg, PreviewSurfaceCfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    quat_apply,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)


class RewardCfg(OrbitalMobileRewardCfg):
    """Orbital slot tracking and station-keeping rewards.

    Fuel consumption is disabled because station-keeping requires sustained
    thrust. Precision and orientation terms activate as the formation is reached.
    """

    penalty_fuel_consumption: LinearRewardTerm = LinearRewardTerm(weight=0.0, limit=8.0)

    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-8.0, limit=32.0)

    penalty_action_magnitude: LinearRewardTerm = LinearRewardTerm(
        weight=-8.0, limit=32.0
    )

    penalty_formation_tracking: LinearRewardTerm = LinearRewardTerm(
        weight=-2.0, limit=16.0
    )

    reward_formation_precision: LinearRewardTerm = LinearRewardTerm(
        weight=16.0, limit=16.0
    )

    reward_formation_orientation: LinearRewardTerm = LinearRewardTerm(
        weight=48.0, limit=48.0
    )
    """Gated by formation position precision."""

    reward_action_rate_in_formation: LinearRewardTerm = LinearRewardTerm(
        weight=48.0, limit=48.0
    )

    tanh_std_formation_precision: float = 0.25
    tanh_std_formation_orientation: float = 0.25
    tanh_std_action_rate_in_formation: float = 0.2


@configclass
class SceneCfg(OrbitalSceneCfg):
    pass


@configclass
class EventCfg(OrbitalEventCfg):
    formation_center_evolution: SrbEventTermCfg = SrbEventTermCfg(
        func=offset_pose_natural,
        mode="interval",
        interval_range_s=(0.2, 0.2),
        is_global_time=True,
        category=RandomizationCategory.TASK,
        params={
            "env_attr_name": "_formation_center",
            "pos_axes": ("x", "y", "z"),
            "pos_step_range": (0.01, 0.04),
            "pos_smoothness": 0.97,
            "pos_step_smoothness": 0.9,
            "pos_bounds": {
                "x": (-24.0, 24.0),
                "y": (-24.0, 24.0),
                "z": (-24.0, 24.0),
            },
            "orient_yaw_only": False,
            "orient_smoothness": 0.92,
        },
    )
    push_robot: SrbEventTermCfg = SrbEventTermCfg(
        func=apply_external_force_torque,
        mode="interval",
        interval_range_s=(5.0, 20.0),
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "force_range": (-1.5, 1.5),
            "torque_range": (-0.05, 0.05),
        },
    )


@configclass
class TaskCfg(OrbitalEnvCfg):
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    events: EventCfg = EventCfg()

    reward: RewardCfg = RewardCfg()

    env_rate: float = 1.0 / 40.0
    agent_rate: float = 1.0 / 10.0
    episode_length_s: float = 60.0
    is_finite_horizon: bool = False

    # Local-frame offsets from the formation center, in meters.
    formation_offsets: tuple[tuple[float, float, float], ...] = (
        (3.0, 0.0, 0.0),  # Front position
        (-1.5, 2.6, 0.0),  # Rear-left position
        (-1.5, -2.6, 0.0),  # Rear-right position
        (-1.5, 0.0, 2.6),  # Top position
    )
    randomize_formation_assignment: bool = True
    formation_scale: float = 1.0

    formation_center_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/formation_center",
        markers={
            "center": ArrowCfg(
                tail_radius=0.03,
                tail_length=0.4,
                head_radius=0.12,
                head_length=0.2,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.8, 0.2, 0.2)),
            )
        },
    )
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
        eye=(3.0, -3.0, 3.0),
        lookat=(0.0, 0.0, 0.0),
        origin_type="asset_root",
        asset_name="robot",
    )

    def __post_init__(self):
        # Disable fuel consumption
        if isinstance(self.robot.action_group, ThrustActionGroup):
            self.robot.action_group.thrust.fuel_consumption_rate = 0.0

        super().__post_init__()


############
### Task ###
############


class Task(SrbEnv, OrbitalEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._formation_center_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.formation_center_marker_cfg
        )
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._num_formation_positions = len(self.cfg.formation_offsets)
        self._formation_offsets_base = torch.tensor(
            self.cfg.formation_offsets, device=self.device, dtype=torch.float32
        )
        self._formation_offsets_base *= self.cfg.formation_scale

        self._formation_center = torch.zeros(self.num_envs, 7, device=self.device)
        self._formation_center[:, 0:3] = self.scene.env_origins
        self._formation_center[:, 6] = 1.0

        self._formation_center_prev = self._formation_center.clone()
        self._steps_since_center_change = torch.zeros(
            self.num_envs, dtype=torch.int32, device=self.device
        )
        self._formation_center_vel = torch.zeros(self.num_envs, 3, device=self.device)
        self._formation_center_tf_initialized = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )

        self._formation_position_idx = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )

        self._goal = torch.zeros(self.num_envs, 7, device=self.device)
        self._goal[:, 6] = 1.0

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
        self._formation_center_tf_initialized[env_ids] = False
        # Prevent natural-motion state from leaking across episode resets.
        reset_pose_natural(self, env_ids, "_formation_center")

        if self.cfg.randomize_formation_assignment:
            self._formation_position_idx[env_ids] = torch.randint(
                0,
                self._num_formation_positions,
                (num_reset_envs,),
                device=self.device,
            )
        else:
            # Use environment IDs so partial resets preserve slot assignments.
            env_ids_tensor = torch.as_tensor(
                env_ids, dtype=torch.long, device=self.device
            )
            self._formation_position_idx[env_ids] = (
                env_ids_tensor % self._num_formation_positions
            )

        self._update_goal_positions(env_ids)

        # Reuse one listener; rebuilding it leaks /tf callbacks.
        if hasattr(self.unwrapped, "ros_node") and not hasattr(self, "tf_buffer"):
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer,
                self.unwrapped.ros_node,  # type: ignore[misc]
            )
            logging.info("TF listener initialized for formation center.")
            self.cfg.events.formation_center_evolution.interval_range_s = (
                1.0e9,
                1.0e9,
            )
            self.event_manager.set_term_cfg(
                "formation_center_evolution",
                self.cfg.events.formation_center_evolution,
            )

    def _update_goal_positions(self, env_ids: Sequence[int] | None = None):
        if env_ids is None:
            env_ids_tensor = torch.arange(self.num_envs, device=self.device)
        else:
            env_ids_tensor = torch.as_tensor(
                env_ids,
                dtype=torch.long,
                device=self.device,
            )

        offsets_3d = self._formation_offsets_base[
            self._formation_position_idx[env_ids_tensor]
        ]

        center_quat = self._formation_center[env_ids_tensor, 3:7]
        rotated_offsets = quat_apply(center_quat, offsets_3d)

        self._goal[env_ids_tensor, 0:3] = (
            self._formation_center[env_ids_tensor, 0:3] + rotated_offsets
        )
        self._goal[env_ids_tensor, 3:7] = center_quat

    def advance_step_state(self) -> None:
        # The center moves on interval events, so divide displacement by elapsed
        # time since its last move and hold the estimate between events.
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
        # Boolean-mask writes call nonzero(), synchronizing CPU and GPU each step.
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
                # TF is env-local; add each origin for unstacked environments.
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
                first_pose = ~self._formation_center_tf_initialized
                self._formation_center_prev[first_pose] = self._formation_center[
                    first_pose
                ]
                self._formation_center_vel[first_pose] = 0.0
                self._steps_since_center_change[first_pose] = 0
                self._formation_center_tf_initialized[:] = True
            except Exception as e:
                logging.warning(
                    f"Failed to get transform from 'world' to 'formation_center': {e}"
                )

        self._update_goal_positions()

        self._formation_center_marker.visualize(
            self._formation_center[:, 0:3], self._formation_center[:, 3:7]
        )
        self._target_marker.visualize(self._goal[:, 0:3], self._goal[:, 3:7])

    def extract_step_return(self) -> StepReturn:
        formation_center_vel = self._formation_center_vel

        rew = self.cfg.reward

        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        tf_pos_robot, tf_quat_robot, vel_lin_robot, vel_ang_robot = (
            self._robot_base_state()
        )

        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        tf_pos_target = self._goal[:, 0:3]
        tf_quat_target = self._goal[:, 3:7]

        tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
        tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

        tf_pos_robot_to_target, tf_quat_robot_to_target = subtract_frame_transforms(
            t01=tf_pos_robot,
            q01=tf_quat_robot,
            t02=tf_pos_target,
            q02=tf_quat_target,
        )
        tf_rotmat_robot_to_target = matrix_from_quat(tf_quat_robot_to_target)
        tf_rot6d_robot_to_target = rotmat_to_rot6d(tf_rotmat_robot_to_target)
        dist_robot_to_target = torch.norm(tf_pos_robot_to_target, dim=-1)

        _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)
        _action_magnitude = torch.mean(torch.square(act_current), dim=1)
        formation_tracking_metric = torch.square(dist_robot_to_target)
        _formation_precision = 1.0 - torch.tanh(
            dist_robot_to_target / rew.tanh_std_formation_precision
        )
        orientation_error = torch.linalg.matrix_norm(
            tf_rotmat_robot_to_target
            - torch.eye(3, device=self.device)
            .unsqueeze(0)
            .expand_as(tf_rotmat_robot_to_target),
            ord="fro",
        )
        _orientation_precision = _formation_precision * (
            1.0 - torch.tanh(orientation_error / rew.tanh_std_formation_orientation)
        )
        action_rate_in_formation_metric = _orientation_precision * (
            1.0 - torch.tanh(_action_rate / rew.tanh_std_action_rate_in_formation)
        )

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(_action_rate),
            "penalty_fuel_consumption": rew.penalty_fuel_consumption(
                torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
            ),
            "penalty_action_magnitude": rew.penalty_action_magnitude(_action_magnitude),
            "penalty_formation_tracking": rew.penalty_formation_tracking(
                formation_tracking_metric
            ),
            "reward_formation_precision": rew.reward_formation_precision(
                _formation_precision
            ),
            "reward_formation_orientation": rew.reward_formation_orientation(
                _orientation_precision
            ),
            "reward_action_rate_in_formation": rew.reward_action_rate_in_formation(
                action_rate_in_formation_metric
            ),
        }

        termination = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "act_previous": act_previous,
                    "tf_rot6d_robot": tf_rot6d_robot,
                    "vel_lin_robot": vel_lin_robot,
                    "vel_ang_robot": vel_ang_robot,
                    "tf_pos_robot_to_target": tf_pos_robot_to_target,
                    "tf_rot6d_robot_to_target": tf_rot6d_robot_to_target,
                    "formation_center_vel": formation_center_vel,
                },
                ObservationGroup.PROPRIO: {
                    "imu_lin_acc": imu_lin_acc,
                    "imu_ang_vel": imu_ang_vel,
                },
            },
            reward,
            termination,
            truncation,
        )
