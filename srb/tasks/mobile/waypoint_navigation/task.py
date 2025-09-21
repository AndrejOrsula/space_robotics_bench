from dataclasses import MISSING
from typing import Sequence, Tuple

import torch

from srb._typing import StepReturn
from srb.core.env import GroundEnv, GroundEnvCfg, GroundEventCfg, GroundSceneCfg
from srb.core.manager import EventTermCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import offset_pose_natural
from srb.core.sim import PreviewSurfaceCfg
from srb.core.sim.spawners.shapes.extras.cfg import PinnedArrowCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, subtract_frame_transforms

##############
### Config ###
##############


@configclass
class SceneCfg(GroundSceneCfg):
    pass


@configclass
class EventCfg(GroundEventCfg):
    target_pose_evolution: EventTermCfg = EventTermCfg(
        func=offset_pose_natural,
        mode="interval",
        interval_range_s=(0.05, 0.05),
        is_global_time=True,
        params={
            "env_attr_name": "_goal",
            "pos_axes": ("x", "y"),
            "pos_step_range": (0.005, 0.02),
            "pos_smoothness": 0.99,
            "pos_step_smoothness": 0.8,
            "pos_bounds": {
                "x": MISSING,
                "y": MISSING,
            },
            "orient_yaw_only": True,
            "orient_smoothness": 0.8,
        },
    )


@configclass
class TaskCfg(GroundEnvCfg):
    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 60.0
    is_finite_horizon: bool = False

    ## Target
    target_pos_range_ratio: float = 0.9
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

    ## Action/observation delays
    action_delay_steps: int | Tuple[int, int] = (0, 3)
    observation_delay_steps: int | Tuple[int, int] = (0, 1)

    def __post_init__(self):
        super().__post_init__()

        # Event: Waypoint target
        if (
            "hardcoded"
            not in self.events.target_pose_evolution.params["pos_bounds"].keys()  # type: ignore
        ):
            assert self.spacing is not None
            for dim in ("x", "y"):
                self.events.target_pose_evolution.params["pos_bounds"][dim] = (  # type: ignore
                    -0.5 * self.target_pos_range_ratio * self.spacing,
                    0.5 * self.target_pos_range_ratio * self.spacing,
                )


############
### Task ###
############


class Task(GroundEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        ## Initialize buffers
        self._goal = torch.zeros(self.num_envs, 7, device=self.device)
        self._goal[:, 0:3] = self.scene.env_origins
        self._goal[:, 3] = 1.0
        self._episodic_noise_tf_pos2d = torch.zeros(
            self.num_envs, 2, device=self.device
        )
        self._episodic_noise_tf_yaw = torch.zeros(self.num_envs, device=self.device)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        ## Reset goal position
        self._goal[env_ids, 0:3] = self.scene.env_origins[env_ids]
        self._goal[env_ids, 3:7] = torch.tensor(
            [1.0, 0.0, 0.0, 0.0], device=self.device
        )

        ## Randomize episodic noise
        num_reset_envs = len(env_ids)
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

    def extract_step_return(self) -> StepReturn:
        ## Visualize target
        self._target_marker.visualize(self._goal[:, 0:3], self._goal[:, 3:7])

        _robot_pose = self._robot.data.root_link_pose_w
        return _compute_step_return(
            ## Time
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            ## Actions
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            ## States
            # Root
            tf_pos_robot=_robot_pose[:, 0:3],
            tf_quat_robot=_robot_pose[:, 3:7],
            # Transforms (world frame)
            tf_pos_target=self._goal[:, 0:3],
            tf_quat_target=self._goal[:, 3:7],
            # # IMU
            # imu_lin_acc=self._imu_robot.data.lin_acc_b,
            # imu_ang_vel=self._imu_robot.data.ang_vel_b,
            ## Randomization
            episodic_noise_tf_pos2d=self._episodic_noise_tf_pos2d,
            episodic_noise_tf_yaw=self._episodic_noise_tf_yaw,
        )


@torch.jit.script
def _compute_step_return(
    *,
    ## Time
    episode_length: torch.Tensor,
    max_episode_length: int,
    truncate_episodes: bool,
    ## Actions
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    ## States
    # Root
    tf_pos_robot: torch.Tensor,
    tf_quat_robot: torch.Tensor,
    # Transforms (world frame)
    tf_pos_target: torch.Tensor,
    tf_quat_target: torch.Tensor,
    # # IMU
    # imu_lin_acc: torch.Tensor,
    # imu_ang_vel: torch.Tensor,
    ## Randomization
    episodic_noise_tf_pos2d: torch.Tensor,
    episodic_noise_tf_yaw: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
    # dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Transforms (world frame)
    # Robot -> Target
    tf_pos_robot_to_target, _tf_quat_robot_to_target = subtract_frame_transforms(
        t01=tf_pos_robot, q01=tf_quat_robot, t02=tf_pos_target, q02=tf_quat_target
    )
    tf_rotmat_robot_to_target = matrix_from_quat(_tf_quat_robot_to_target)

    # Derived states
    tf_pos2d_robot_to_target = tf_pos_robot_to_target[:, :2]
    dist2d_robot_to_target = torch.norm(tf_pos2d_robot_to_target, dim=-1)
    angle_robot_to_target_pos = torch.atan2(
        tf_pos_robot_to_target[..., 1], tf_pos_robot_to_target[..., 0]
    )
    yaw_robot_to_target = torch.atan2(
        tf_rotmat_robot_to_target[..., 1, 0], tf_rotmat_robot_to_target[..., 0, 0]
    )

    # Randomized states
    tf_pos2d_robot_to_target_with_noise = (
        tf_pos2d_robot_to_target
        + episodic_noise_tf_pos2d
        + torch.normal(mean=0.0, std=0.0025, size=(num_envs, 2), device=device)
    )
    yaw_robot_to_target_with_noise = (
        yaw_robot_to_target
        + episodic_noise_tf_yaw
        + torch.normal(
            mean=0.0,
            std=0.0087266463,  # 0.5 deg
            size=(num_envs,),
            device=device,
        )
    )
    tf_rot2dtrigyaw_robot_to_target_with_noise = torch.stack(
        (
            torch.sin(yaw_robot_to_target_with_noise),
            torch.cos(yaw_robot_to_target_with_noise),
        ),
        dim=-1,
    )

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.5
    _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)
    penalty_action_rate = WEIGHT_ACTION_RATE * _action_rate

    # Penalty: Position tracking | Robot <--> Target
    WEIGHT_POSITION_TRACKING = -1.0
    penalty_position_tracking = WEIGHT_POSITION_TRACKING * torch.square(
        dist2d_robot_to_target
    )

    # Reward: Point towards target | Robot <--> Target
    WEIGHT_POINT_TOWARDS_TARGET = 1.0
    TANH_STD_POINT_TOWARDS_TARGET = 0.7854  # 45 deg
    reward_point_towards_target = WEIGHT_POINT_TOWARDS_TARGET * (
        1.0
        - torch.tanh(
            torch.abs(angle_robot_to_target_pos) / TANH_STD_POINT_TOWARDS_TARGET
        )
    )

    # Reward: Position tracking | Robot <--> Target (precision)
    WEIGHT_POSITION_TRACKING_PRECISION = 4.0
    TANH_STD_POSITION_TRACKING_PRECISION = 0.05
    _position_tracking_precision = 1.0 - torch.tanh(
        dist2d_robot_to_target / TANH_STD_POSITION_TRACKING_PRECISION
    )
    reward_position_tracking_precision = (
        WEIGHT_POSITION_TRACKING_PRECISION * _position_tracking_precision
    )

    # Reward: Target orientation tracking once position is reached | Robot <--> Target
    WEIGHT_ORIENTATION_TRACKING = 8.0
    TANH_STD_ORIENTATION_TRACKING = 0.2618  # 15 deg
    _orientation_tracking_precision = _position_tracking_precision * (
        1.0 - torch.tanh(torch.abs(yaw_robot_to_target) / TANH_STD_ORIENTATION_TRACKING)
    )
    reward_orientation_tracking = (
        WEIGHT_ORIENTATION_TRACKING * _orientation_tracking_precision
    )

    # Reward: Action rate at target
    WEIGHT_ACTION_RATE_AT_TARGET = 32.0
    TANH_STD_ACTION_RATE_AT_TARGET = 0.1
    reward_action_rate_at_target = (
        WEIGHT_ACTION_RATE_AT_TARGET
        * _orientation_tracking_precision
        * (1.0 - torch.tanh(_action_rate / TANH_STD_ACTION_RATE_AT_TARGET))
    )

    ##################
    ## Terminations ##
    ##################
    # No termination condition
    termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
    # Truncation
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        {
            "state": {
                "tf_pos2d_robot_to_target": tf_pos2d_robot_to_target_with_noise,
                "tf_rot2dtrigyaw_robot_to_target": tf_rot2dtrigyaw_robot_to_target_with_noise,
            },
            # "proprio": {
            #     "imu_lin_acc": imu_lin_acc,
            #     "imu_ang_vel": imu_ang_vel,
            # },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_position_tracking": penalty_position_tracking,
            "reward_point_towards_target": reward_point_towards_target,
            "reward_position_tracking_precision": reward_position_tracking_precision,
            "reward_orientation_tracking": reward_orientation_tracking,
            "reward_action_rate_at_target": reward_action_rate_at_target,
        },
        termination,
        truncation,
    )
