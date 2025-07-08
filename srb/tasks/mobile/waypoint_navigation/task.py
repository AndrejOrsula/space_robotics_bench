from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing import StepReturn
from srb.core.env import GroundEnv, GroundEnvCfg, GroundEventCfg, GroundSceneCfg
from srb.core.manager import EventTermCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import offset_pose_natural
from srb.core.sim import PreviewSurfaceCfg
from srb.core.sim.spawners.shapes.extras.cfg import PinnedArrowCfg
from srb.utils.cfg import configclass
from srb.utils.math import quat_to_rot6d, subtract_frame_transforms

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
        interval_range_s=(0.2, 0.4),
        is_global_time=True,
        params={
            "env_attr_name": "_goal",
            "pos_axes": ("x", "y"),
            "pos_step_range": (0.05, 0.5),
            "pos_smoothness": 0.9,
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
                pin_length=1.0,
                tail_radius=0.01,
                tail_length=0.25,
                head_radius=0.025,
                head_length=0.1,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

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
        self._goal[:, 4] = 1.0

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        ## Reset goal position
        self._goal[env_ids, 0:3] = self.scene.env_origins[env_ids]
        self._goal[env_ids, 3:7] = torch.tensor(
            [1.0, 0.0, 0.0, 0.0], device=self.device
        )

    def extract_step_return(self) -> StepReturn:
        ## Visualize target
        self._target_marker.visualize(self._goal[:, :3], self._goal[:, 3:])

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
            tf_pos_robot=self._robot.data.root_pos_w,
            tf_quat_robot=self._robot.data.root_quat_w,
            vel_lin_robot=self._robot.data.root_lin_vel_b,
            vel_ang_robot=self._robot.data.root_ang_vel_b,
            # Transforms (world frame)
            tf_pos_target=self._goal[:, 0:3],
            tf_quat_target=self._goal[:, 3:7],
            # IMU
            imu_lin_acc=self._imu_robot.data.lin_acc_b,
            imu_ang_vel=self._imu_robot.data.ang_vel_b,
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
    vel_lin_robot: torch.Tensor,
    vel_ang_robot: torch.Tensor,
    # Transforms (world frame)
    tf_pos_target: torch.Tensor,
    tf_quat_target: torch.Tensor,
    # IMU
    imu_lin_acc: torch.Tensor,
    imu_ang_vel: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
    # dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Transforms (world frame)
    # World -> Robot
    tf_rot6d_world_to_robot = quat_to_rot6d(tf_quat_robot)
    # Robot -> Target
    tf_pos_robot_to_target, _tf_quat_robot_to_target = subtract_frame_transforms(
        t01=tf_pos_robot, q01=tf_quat_robot, t02=tf_pos_target, q02=tf_quat_target
    )
    tf_pos2d_robot_to_target = tf_pos_robot_to_target[:, :2]
    dist2d_robot_to_target = torch.norm(tf_pos2d_robot_to_target, dim=-1)
    # tf_rot6d_robot_to_target = quat_to_rot6d(_tf_quat_robot_to_target)

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Distance | Robot <--> Target
    WEIGHT_DISTANCE_ROBOT_TO_TARGET = -8.0
    penalty_distance_robot_to_target = (
        WEIGHT_DISTANCE_ROBOT_TO_TARGET * dist2d_robot_to_target
    )

    # Reward: Distance | Robot <--> Target (precision)
    WEIGHT_DISTANCE_ROBOT_TO_TARGET_PRECISION = 64.0
    TANH_STD_DISTANCE_ROBOT_TO_TARGET_PRECISION = 0.05
    reward_distance_robot_to_target_precision = (
        WEIGHT_DISTANCE_ROBOT_TO_TARGET_PRECISION
        * (
            1.0
            - torch.tanh(
                dist2d_robot_to_target / TANH_STD_DISTANCE_ROBOT_TO_TARGET_PRECISION
            )
        )
    )

    # Reward: Stopping at target
    WEIGHT_STOPPING_AT_TARGET = 32.0
    TANH_STD_STOPPING_AT_TARGET_DISTANCE = 0.05
    TANH_STD_STOPPING_AT_TARGET_VELOCITY_LINEAR = 0.025
    TANH_STD_STOPPING_AT_TARGET_VELOCITY_ANGULAR = 0.06283
    reward_stop_at_target = (
        WEIGHT_STOPPING_AT_TARGET
        * (
            1.0
            - torch.tanh(dist2d_robot_to_target / TANH_STD_STOPPING_AT_TARGET_DISTANCE)
        )
        * (
            1.0
            - torch.tanh(
                torch.norm(vel_lin_robot[:, :2], dim=-1)
                / TANH_STD_STOPPING_AT_TARGET_VELOCITY_LINEAR
            )
        )
        * (
            1.0
            - torch.tanh(
                torch.norm(vel_ang_robot[:, :2], dim=-1)
                / TANH_STD_STOPPING_AT_TARGET_VELOCITY_ANGULAR
            )
        )
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
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
                "tf_pos2d_robot_to_target": tf_pos2d_robot_to_target,
                "tf_rot6d_world_to_robot": tf_rot6d_world_to_robot,
                # "tf_rot6d_robot_to_target": tf_rot6d_robot_to_target,
            },
            "proprio": {
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_distance_robot_to_target": penalty_distance_robot_to_target,
            "reward_distance_robot_to_target_precision": reward_distance_robot_to_target_precision,
            "reward_stop_at_target": reward_stop_at_target,
        },
        termination,
        truncation,
    )
