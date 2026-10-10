from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.action import ThrustActionGroup
from srb.core.asset import (
    AssetVariant,
    ExtravehicularScenery,
    IntravehicularScenery,
    OrbitalRobot,
)
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
    reset_root_state_uniform,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    HyperbolicRewardTerm,
    LinearRewardTerm,
    LogCoshRewardTerm,
    ScaledRewardTerm,
)
from srb.core.sim import ArrowCfg, PreviewSurfaceCfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    radians,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)


class RewardCfg(OrbitalMobileRewardCfg):
    """Position shaping for translation-only waypoint control."""

    penalty_fuel_consumption: LinearRewardTerm = LinearRewardTerm(weight=0.0, limit=8.0)
    """Disabled because waypoint tracking requires sustained thrust."""

    penalty_action_rate: LogCoshRewardTerm = LogCoshRewardTerm(weight=-4.0, limit=8.0)
    """Smooths small corrections while saturating for large action changes."""

    penalty_action_magnitude: LogCoshRewardTerm = LogCoshRewardTerm(
        weight=-1.0, limit=4.0
    )
    """Encourages fuel-efficient low-thrust operation."""

    penalty_angular_velocity: LogCoshRewardTerm = LogCoshRewardTerm(
        weight=-2.0, limit=8.0
    )
    """Suppresses rotational drift from external perturbations."""

    penalty_position_distance: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=8.0
    )
    """Provides a coarse gradient toward the target."""

    penalty_velocity_at_target: LogCoshRewardTerm = LogCoshRewardTerm(
        weight=-4.0, limit=8.0
    )
    """Encourages braking near the target without penalizing transit."""

    reward_position_approach: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=4.0, scale=2.0
    )
    """Provides dense shaping at medium range."""

    reward_position_precision: ScaledRewardTerm = ScaledRewardTerm(
        weight=8.0, limit=8.0
    )
    """Flat-top gives zero gradient at the waypoint for stable settling."""

    reward_smooth_station_keeping: ScaledRewardTerm = ScaledRewardTerm(
        weight=8.0, limit=8.0
    )
    """Rewards low action rate near the target without pushing at rest."""

    gaussian_std_position_precision: float = 0.5
    """Position precision width in metres."""
    gaussian_std_station_keeping_action_rate: float = 0.3
    """Action-rate width for the station-keeping reward."""


@configclass
class SceneCfg(OrbitalSceneCfg):
    pass


@configclass
class EventCfg(OrbitalEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (-0.1, 0.1),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.05, 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
                "roll": (-radians(5.0), radians(5.0)),
                "pitch": (-radians(5.0), radians(5.0)),
                "yaw": (-radians(5.0), radians(5.0)),
            },
        },
    )
    # Keep drift trackable at low thrust; attitude tracking is not required.
    target_pose_evolution: SrbEventTermCfg = SrbEventTermCfg(
        func=offset_pose_natural,
        mode="interval",
        interval_range_s=(0.2, 0.2),
        is_global_time=True,
        category=RandomizationCategory.TASK,
        params={
            "env_attr_name": "_goal",
            "pos_axes": ("x", "y", "z"),
            "pos_step_range": (0.005, 0.02),
            "pos_smoothness": 0.98,
            "pos_step_smoothness": 0.95,
            "pos_bounds": {
                "x": (-4.0, 4.0),
                "y": (-4.0, 4.0),
                "z": (-4.0, 4.0),
            },
            "orient_yaw_only": False,
            "orient_smoothness": 0.9,
        },
    )
    # Small torque avoids spins the spacecraft cannot correct.
    push_robot: SrbEventTermCfg = SrbEventTermCfg(
        func=apply_external_force_torque,
        mode="interval",
        interval_range_s=(5.0, 20.0),
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "force_range": (-0.5, 0.5),
            "torque_range": (-0.005, 0.005),
        },
    )


@configclass
class TaskCfg(OrbitalEnvCfg):
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    robot: OrbitalRobot | AssetVariant = assets.SatelliteMockup()
    scenery: ExtravehicularScenery | IntravehicularScenery | AssetVariant | None = None
    events: EventCfg = EventCfg()

    reward: RewardCfg = RewardCfg()

    env_rate: float = 1.0 / 40.0
    agent_rate: float = 1.0 / 10.0
    episode_length_s: float = 30.0
    is_finite_horizon: bool = False

    target_pos_range_ratio: float = 0.9
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


class Task(SrbEnv, OrbitalEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._goal = torch.zeros(self.num_envs, 7, device=self.device)
        self._goal[:, 0:3] = self.scene.env_origins
        self._goal[:, 6] = 1.0  # xyzw identity (w at index 6)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        self._goal[env_ids, 0:3] = self.scene.env_origins[env_ids]
        self._goal[env_ids, 3:7] = torch.tensor(
            [0.0, 0.0, 0.0, 1.0],
            device=self.device,  # xyzw identity
        )
        # Reset drift state at each episode boundary.
        reset_pose_natural(self, env_ids, "_goal")

        # Recreating the listener would leak its persistent /tf callback.
        if hasattr(self.unwrapped, "ros_node") and not hasattr(self, "tf_buffer"):
            from tf2_ros import Buffer, TransformListener

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer,
                self.unwrapped.ros_node,  # type: ignore[misc]
            )
            logging.info("TF listener initialized.")
            self.cfg.events.target_pose_evolution.interval_range_s = (1.0e9, 1.0e9)
            self.event_manager.set_term_cfg(
                "target_pose_evolution", self.cfg.events.target_pose_evolution
            )

    def advance_step_state(self) -> None:
        if hasattr(self, "tf_buffer"):
            from rclpy.time import Duration, Time

            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    "world",
                    "target",
                    Time(),
                    Duration(
                        seconds=0,
                        nanoseconds=200000000,
                    ),
                )
                logging.debug(f"Got transform from 'world' to 'target': {tf_stamped}")
                # The TF pose is env-local: offset it by each env origin so every
                # env (distinct origins under `env.stack=false`) gets it in its
                # own cell rather than env_0's world position.
                translation = tf_stamped.transform.translation
                self._goal[:, 0:3] = self.scene.env_origins + torch.tensor(
                    (translation.x, translation.y, translation.z),
                    dtype=self._goal.dtype,
                    device=self._goal.device,
                )
                self._goal[:, 3] = tf_stamped.transform.rotation.x
                self._goal[:, 4] = tf_stamped.transform.rotation.y
                self._goal[:, 5] = tf_stamped.transform.rotation.z
                self._goal[:, 6] = tf_stamped.transform.rotation.w
            except Exception as e:
                logging.warning(
                    f"Failed to get transform from 'world' to 'target': {e}"
                )

        self._target_marker.visualize(self._goal[:, 0:3], self._goal[:, 3:7])

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward
        num_envs = self.num_envs
        device = self.device

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

        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        action_magnitude_metric = torch.mean(torch.square(act_current), dim=1)
        angular_velocity_metric = torch.norm(vel_ang_robot, dim=-1)
        position_distance_metric = torch.square(dist_robot_to_target)

        position_precision = torch.exp(
            -torch.square(dist_robot_to_target)
            / (2.0 * rew.gaussian_std_position_precision**2)
        )

        velocity_at_target_metric = position_precision * torch.norm(
            vel_lin_robot, dim=-1
        )

        station_keeping_metric = position_precision * torch.exp(
            -action_rate_metric
            / (2.0 * rew.gaussian_std_station_keeping_action_rate**2)
        )

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_action_magnitude": rew.penalty_action_magnitude(
                action_magnitude_metric
            ),
            "penalty_angular_velocity": rew.penalty_angular_velocity(
                angular_velocity_metric
            ),
            "penalty_position_distance": rew.penalty_position_distance(
                position_distance_metric
            ),
            "penalty_velocity_at_target": rew.penalty_velocity_at_target(
                velocity_at_target_metric
            ),
            "reward_position_approach": rew.reward_position_approach(
                dist_robot_to_target
            ),
            "reward_position_precision": rew.reward_position_precision(
                position_precision
            ),
            "reward_smooth_station_keeping": rew.reward_smooth_station_keeping(
                station_keeping_metric
            ),
        }

        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
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
