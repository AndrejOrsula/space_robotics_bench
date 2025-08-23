from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.action import ThrustAction  # noqa: F401
from srb.core.asset import AssetVariant, ExtravehicularScenery, MobileRobot
from srb.core.env import OrbitalEnv, OrbitalEnvCfg, OrbitalEventCfg, OrbitalSceneCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg  # noqa: F401
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import apply_external_force_torque, offset_pose_natural  # noqa: F401
from srb.core.sim import ArrowCfg, PreviewSurfaceCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d, subtract_frame_transforms

##############
### Config ###
##############


@configclass
class SceneCfg(OrbitalSceneCfg):
    pass


@configclass
class EventCfg(OrbitalEventCfg):
    target_pose_evolution: EventTermCfg = EventTermCfg(
        func=offset_pose_natural,
        mode="interval",
        interval_range_s=(0.05, 0.05),
        is_global_time=True,
        params={
            "env_attr_name": "_goal",
            "pos_axes": ("x", "y", "z"),
            "pos_step_range": (0.005, 0.01),
            "pos_smoothness": 0.99,
            "pos_bounds": {
                "x": MISSING,
                "y": MISSING,
                "z": MISSING,
            },
            "orient_yaw_only": False,
            "orient_smoothness": 0.8,
        },
    )
    # push_robot: EventTermCfg = EventTermCfg(  # temporarily remove disturbances
    #     func=apply_external_force_torque,
    #     mode="interval",
    #     interval_range_s=(5.0, 20.0),
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "force_range": (-4.0, 4.0),
    #         "torque_range": (-0.19635, 0.19635),
    #     },
    # )


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Assets
    scenery: ExtravehicularScenery | MobileRobot | AssetVariant | None = (
        assets.Gateway()
    )
    scenery.asset_cfg.init_state.pos = (0.0, 0.0, -5.0)
    # TODO: Re-enable collisions with the scenery
    scenery.asset_cfg.spawn.collision_props.collision_enabled = False  # type: ignore
    scenery.asset_cfg.spawn.mesh_collision_props.mesh_approximation = None  # type: ignore
    _scenery: ExtravehicularScenery | None = MISSING  # type: ignore

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 60.0
    is_finite_horizon: bool = True

    ## Target
    target_pos_range_ratio: float = 0.9
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": ArrowCfg(
                tail_radius=0.025,
                tail_length=0.5,
                head_radius=0.1,
                head_length=0.2,
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
            for dim in ("x", "y", "z"):
                self.events.target_pose_evolution.params["pos_bounds"][dim] = (  # type: ignore
                    -50.0,
                    50.0,
                )


############
### Task ###
############


class Task(OrbitalEnv):
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

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        ## Reset goal position
        self._goal[env_ids, 0:3] = self.scene.env_origins[env_ids]
        self._goal[env_ids, 3:7] = torch.tensor(
            [1.0, 0.0, 0.0, 0.0], device=self.device
        )

    def extract_step_return(self) -> StepReturn:
        ## Visualize target
        self._target_marker.visualize(self._goal[:, 0:3], self._goal[:, 3:7])

        # ## Get remaining fuel (if applicable)
        # if self._thrust_action_term_key:
        #     thrust_action_term: ThrustAction = self.action_manager._terms[  # type: ignore
        #         self._thrust_action_term_key
        #     ]
        #     remaining_fuel = (
        #         thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
        #     ).unsqueeze(-1)
        # else:
        #     remaining_fuel = None

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
            # # IMU
            # imu_lin_acc=self._imu_robot.data.lin_acc_b,
            # imu_ang_vel=self._imu_robot.data.ang_vel_b,
            # Fuel
            # remaining_fuel=remaining_fuel,
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
    # # IMU
    # imu_lin_acc: torch.Tensor,
    # imu_ang_vel: torch.Tensor,
    # Fuel
    # remaining_fuel: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    # dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Root
    tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
    tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

    ## Transforms (world frame)
    # Robot -> Target
    tf_pos_robot_to_target, tf_quat_robot_to_target = subtract_frame_transforms(
        t01=tf_pos_robot,
        q01=tf_quat_robot,
        t02=tf_pos_target,
        q02=tf_quat_target,
    )
    tf_rotmat_robot_to_target = matrix_from_quat(tf_quat_robot_to_target)
    tf_rot6d_robot_to_target = rotmat_to_rot6d(tf_rotmat_robot_to_target)
    dist_robot_to_target = torch.norm(tf_pos_robot_to_target, dim=-1)

    # # Angle between the robot's forward direction and the target's position
    # angle_robot_to_target_pos = torch.acos(
    #     torch.clamp(
    #         tf_pos_robot_to_target[..., 0] / (dist_robot_to_target + 1.0e-6), -1.0, 1.0
    #     )
    # )
    # Angle of the relative orientation between the robot and the target
    trace = torch.einsum("...ii->...", tf_rotmat_robot_to_target)
    angle_robot_to_target_orient = torch.acos(
        torch.clamp((trace - 1.0) / 2.0, -1.0, 1.0)
    )

    # ## Fuel
    # remaining_fuel = (
    #     remaining_fuel
    #     if remaining_fuel is not None
    #     else torch.ones((num_envs, 1), dtype=dtype, device=device)
    # )

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -1.0
    _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)
    penalty_action_rate = WEIGHT_ACTION_RATE * _action_rate

    # # Penalty: Fuel consumption
    # WEIGHT_FUEL_CONSUMPTION = -8.0
    # penalty_fuel_consumption = WEIGHT_FUEL_CONSUMPTION * torch.square(
    #     1.0 - remaining_fuel.squeeze(-1)
    # )

    # Penalty: Position tracking | Robot <--> Target
    WEIGHT_POSITION_TRACKING = -2.0
    MAX_POSITION_TRACKING_PENALTY = -16.0
    penalty_position_tracking = torch.clamp_min(
        WEIGHT_POSITION_TRACKING * torch.square(dist_robot_to_target),
        min=MAX_POSITION_TRACKING_PENALTY,
    )

    # # Reward: Point towards target | Robot <--> Target
    # WEIGHT_POINT_TOWARDS_TARGET = 1.0
    # TANH_STD_POINT_TOWARDS_TARGET = 0.7854  # 45 deg
    # reward_point_towards_target = WEIGHT_POINT_TOWARDS_TARGET * (
    #     1.0
    #     - torch.tanh(
    #         torch.abs(angle_robot_to_target_pos) / TANH_STD_POINT_TOWARDS_TARGET
    #     )
    # )

    # Reward: Position tracking | Robot <--> Target (precision)
    WEIGHT_POSITION_TRACKING_PRECISION = 16.0
    TANH_STD_POSITION_TRACKING_PRECISION = 0.5
    _position_tracking_precision = 1.0 - torch.tanh(
        dist_robot_to_target / TANH_STD_POSITION_TRACKING_PRECISION
    )
    reward_position_tracking_precision = (
        WEIGHT_POSITION_TRACKING_PRECISION * _position_tracking_precision
    )

    # Reward: Target orientation tracking once position is reached | Robot <--> Target
    WEIGHT_ORIENTATION_TRACKING = 64.0
    TANH_STD_ORIENTATION_TRACKING = 0.5236  # 30 deg
    _orientation_tracking_precision = _position_tracking_precision * (
        1.0
        - torch.tanh(
            torch.abs(angle_robot_to_target_orient) / TANH_STD_ORIENTATION_TRACKING
        )
    )
    reward_orientation_tracking = (
        WEIGHT_ORIENTATION_TRACKING * _orientation_tracking_precision
    )

    # Reward: Action rate at target
    WEIGHT_ACTION_RATE_AT_TARGET = 128.0
    TANH_STD_ACTION_RATE_AT_TARGET = 0.25
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
                "tf_rot6d_robot": tf_rot6d_robot,
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
                "tf_pos_robot_to_target": tf_pos_robot_to_target,
                "tf_rot6d_robot_to_target": tf_rot6d_robot_to_target,
            },
            # "proprio": {
            #     # "imu_lin_acc": imu_lin_acc,
            #     # "imu_ang_vel": imu_ang_vel,
            #     "remaining_fuel": remaining_fuel,
            # },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            # "penalty_fuel_consumption": penalty_fuel_consumption,
            "penalty_position_tracking": penalty_position_tracking,
            # "reward_point_towards_target": reward_point_towards_target,
            "reward_position_tracking_precision": reward_position_tracking_precision,
            "reward_orientation_tracking": reward_orientation_tracking,
            "reward_action_rate_at_target": reward_action_rate_at_target,
        },
        termination,
        truncation,
    )
