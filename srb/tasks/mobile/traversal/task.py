from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import AssetVariant, GroundRobot, Terrain
from srb.core.domain import Domain
from srb.core.env import (
    GroundEnv,
    GroundEnvCfg,
    GroundEventCfg,
    GroundSceneCfg,
    ViewerCfg,
)
from srb.core.env.mobile import GroundMobileRewardCfg
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import push_by_setting_velocity, reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import LinearRewardTerm
from srb.core.sim import PreviewSurfaceCfg
from srb.core.sim.spawners.shapes.extras.cfg import PinnedArrowCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    euler_xyz_from_quat,
    matrix_from_quat,
    quat_inv,
    quat_mul,
    radians,
    rotmat_to_rot6d,
)

##############
### Config ###
##############


class RewardCfg(GroundMobileRewardCfg):
    """Traversal rewards for forward progress, smooth actions, and recovery.

    Action-rate weight is reduced to -0.05 for rough terrain. Progress targets
    0.15 m/s; smooth-action reward is gated on forward progress. Rollover is
    penalized rather than terminal. Inherits other base mobile rewards.
    """

    ## Override: lighter action rate penalty
    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-0.05, limit=4.0)
    """Reduced from the inherited -0.1 for rough-terrain corrections."""

    ## Task-specific terms
    penalty_lateral_pos_deviation: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=4.0
    )
    """Penalises lateral deviation from the straight-line path."""

    reward_vel_progress: LinearRewardTerm = LinearRewardTerm(weight=8.0, limit=8.0)
    """Rewards forward velocity progress along the initial heading."""

    reward_action_rate_at_target: LinearRewardTerm = LinearRewardTerm(
        weight=32.0, limit=32.0
    )
    """Rewards smooth actions when forward velocity exceeds the gate."""

    penalty_rollover: LinearRewardTerm = LinearRewardTerm(weight=-16.0, limit=16.0)
    """Penalizes tilt beyond the threshold while allowing recovery."""

    ## Shaping parameters (transparent to BaseRewardCfg)
    exp_std_vel_progress: float = 0.05
    target_velocity: float = 0.15
    tanh_std_action_rate_at_target: float = 0.2


@configclass
class SceneCfg(GroundSceneCfg):
    env_spacing: float = 100.0


@configclass
class EventCfg(GroundEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "z": (1.0, 1.0),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {},
        },
    )

    push_robot: SrbEventTermCfg = SrbEventTermCfg(
        func=push_by_setting_velocity,
        mode="interval",
        interval_range_s=(30.0, 60.0),
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "velocity_range": {
                "x": (-0.25, 0.25),
                "y": (-0.25, 0.25),
                "z": (-0.05, 0.05),
                "roll": (-radians(5.0), radians(5.0)),
                "pitch": (-radians(5.0), radians(5.0)),
                "yaw": (-radians(10.0), radians(10.0)),
            },
        },
    )


@configclass
class TaskCfg(GroundEnvCfg):
    """Configuration for the terrain traversal task.

    A ground rover drives forward along a straight-line path at a target
    velocity over procedural terrain.  Excessive tilt receives the rollover
    penalty; the episode remains recoverable until time-limit truncation.
    """

    ## Scenario
    domain: Domain = Domain.MARS

    ## Assets
    scenery: Terrain | AssetVariant | None = assets.TerrainGenerator()
    robot: GroundRobot | AssetVariant = assets.Nifty()

    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 120.0
    is_finite_horizon: bool = False

    ## Penalty thresholds
    max_tilt_rad: float = 1.5708
    """Rollover penalty threshold (default: 90 deg); does not terminate episodes."""

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(5.0 + 20.0, 5.0 - 20.0, 0.0 + 30.0),
        lookat=(5.0, 5.0, 2.5),
        origin_type="env",
    )

    ## Target
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": PinnedArrowCfg(
                pin_radius=0.01,
                pin_length=1.0,
                tail_radius=0.01,
                tail_length=0.2,
                head_radius=0.05,
                head_length=0.1,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()


############
### Task ###
############


class _TraversalTaskMixin:
    """Target-state and extraction logic shared by owned and legacy arms.

    Observations cover the normalized lateral distance and rotation to the
    target, and IMU. Actions are linear and angular velocity commands; rewards
    shape velocity progress and smooth actions, and penalize lateral deviation
    and rollover.
    """

    cfg: TaskCfg

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        _robot_root_pose = self._robot.data.root_pose_w[env_ids]
        self._tf_pos2_init[env_ids] = _robot_root_pose[:, :2]
        self._tf_yaw_init[env_ids] = euler_xyz_from_quat(_robot_root_pose[:, 3:])[2]
        self._tf_dir2d_init[env_ids] = torch.stack(
            (
                torch.cos(self._tf_yaw_init[env_ids]),
                torch.sin(self._tf_yaw_init[env_ids]),
            ),
            dim=-1,
        )
        self._tf_quat_target[env_ids, 2] = torch.sin(self._tf_yaw_init[env_ids] / 2.0)
        self._tf_quat_target[env_ids, 3] = torch.cos(self._tf_yaw_init[env_ids] / 2.0)

    def _current_target_pos(self) -> torch.Tensor:
        """Target position: 1 m ahead of the robot along its initial heading."""
        root_pos = self._robot.data.root_pos_w
        target = torch.empty_like(self._tf_pos_target)
        target[:, :2] = self._tf_pos2_init + self._tf_dir2d_init * (
            torch.sum(
                (root_pos[:, :2] - self._tf_pos2_init) * self._tf_dir2d_init,
                dim=1,
            )
            + 1.0
        ).unsqueeze(-1)
        target[:, 2] = root_pos[:, 2]
        return target

    def advance_step_state(self) -> None:
        self._tf_pos_target[:] = self._current_target_pos()
        self._target_marker.visualize(self._tf_pos_target, self._tf_quat_target)

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
        device = self.device

        rew = self.cfg.reward

        tf_pos_target = self._current_target_pos()

        ################
        ## Raw states ##
        ################
        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        tf_pos_robot = self._robot.data.root_pos_w
        tf_quat_robot = self._robot.data.root_quat_w
        vel_lin_robot = self._robot.data.root_lin_vel_w

        imu_lin_acc = self._imu_robot.data.lin_acc_b
        imu_ang_vel = self._imu_robot.data.ang_vel_b

        tf_pos2d_robot_to_target = tf_pos_target[:, :2] - tf_pos_robot[:, :2]
        tf_rot6d_robot_to_target = rotmat_to_rot6d(
            matrix_from_quat(quat_mul(quat_inv(tf_quat_robot), self._tf_quat_target))
        )
        lateral_dist_robot_to_target = torch.sum(
            tf_pos2d_robot_to_target
            * torch.stack(
                (-self._tf_dir2d_init[:, 1], self._tf_dir2d_init[:, 0]), dim=-1
            ),
            dim=1,
        )
        lateral_dist_robot_to_target_normalized = torch.clip(
            lateral_dist_robot_to_target / 2.0, -1.0, 1.0
        ).unsqueeze(-1)

        #####################
        ## Reward  metrics ##
        #####################
        _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)
        _lateral_pos_precision = 1.0 / torch.exp(
            torch.abs(lateral_dist_robot_to_target)
        )
        _vel_progress = _lateral_pos_precision * torch.exp(
            -torch.square(
                torch.clip(
                    torch.sum(vel_lin_robot[:, :2] * self._tf_dir2d_init, dim=1)
                    - rew.target_velocity,
                    max=0.0,
                )
            )
            / rew.exp_std_vel_progress
        )

        action_rate_metric = _action_rate
        lateral_deviation_metric = 1.0 - _lateral_pos_precision
        vel_progress_metric = _vel_progress
        action_rate_at_target_metric = _vel_progress * (
            1.0 - torch.tanh(_action_rate / rew.tanh_std_action_rate_at_target)
        )

        #############
        ## Rewards ##
        #############
        _roll, _pitch, _ = euler_xyz_from_quat(tf_quat_robot)
        rollover_metric = (
            (torch.abs(_roll) > self.cfg.max_tilt_rad)
            | (torch.abs(_pitch) > self.cfg.max_tilt_rad)
        ).float()

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_lateral_pos_deviation": rew.penalty_lateral_pos_deviation(
                lateral_deviation_metric
            ),
            "reward_vel_progress": rew.reward_vel_progress(vel_progress_metric),
            "reward_action_rate_at_target": rew.reward_action_rate_at_target(
                action_rate_at_target_metric
            ),
            "penalty_rollover": rew.penalty_rollover(rollover_metric),
        }

        ##################
        ## Terminations ##
        ##################
        # Keep episodes recoverable after rollover; truncate at the time limit.
        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "lateral_dist_robot_to_target_normalized": lateral_dist_robot_to_target_normalized,
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


class Task(_TraversalTaskMixin, SrbEnv, GroundEnv):
    """Drive forward along a straight-line path at a target velocity."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, runtime_mode="isaac", **kwargs)

        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._tf_pos2_init = torch.zeros(
            self.num_envs, 2, device=self.device, dtype=torch.float
        )
        self._tf_yaw_init = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.float
        )
        self._tf_dir2d_init = torch.zeros(
            self.num_envs, 2, device=self.device, dtype=torch.float
        )
        self._tf_pos_target = torch.zeros(
            self.num_envs, 3, device=self.device, dtype=torch.float
        )
        self._tf_quat_target = torch.zeros(
            self.num_envs, 4, device=self.device, dtype=torch.float
        )
        self._tf_quat_target[:, 3] = 1.0
