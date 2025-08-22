from dataclasses import MISSING
from typing import Sequence, Tuple

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.action import ThrustAction
from srb.core.asset import AssetVariant, MobileRobot, OrbitalRobot, Scenery
from srb.core.domain import Domain
from srb.core.env import OrbitalEnv, OrbitalEnvCfg, OrbitalEventCfg, OrbitalSceneCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import push_by_setting_velocity  # noqa: F401
from srb.core.mdp import reset_root_state_uniform
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import PreviewSurfaceCfg, SphereCfg
from srb.utils.cfg import configclass
from srb.utils.math import deg_to_rad, matrix_from_quat, rotmat_to_rot6d

##############
### Config ###
##############


@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing: float = 256.0

    ## Sensors
    contacts_robot: ContactSensorCfg = ContactSensorCfg(
        prim_path=MISSING,  # type: ignore
        track_air_time=True,
    )


@configclass
class EventCfg(OrbitalEventCfg):
    randomize_robot_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-50.0, 50.0),
                "y": (-50.0, 50.0),
                "z": (100.0, 200.0),
                "roll": (-deg_to_rad(30.0), deg_to_rad(30.0)),
                "pitch": (-deg_to_rad(30.0), deg_to_rad(30.0)),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-10.0, 10.0),
                "y": (-10.0, 10.0),
                "z": (-20.0, -10.0),
                "roll": (-deg_to_rad(15.0), deg_to_rad(15.0)),
                "pitch": (-deg_to_rad(15.0), deg_to_rad(15.0)),
                "yaw": (-deg_to_rad(15.0), deg_to_rad(15.0)),
            },
        },
    )
    # push_robot: EventTermCfg = EventTermCfg(
    #     func=push_by_setting_velocity,
    #     mode="interval",
    #     interval_range_s=(2.5, 10.0),
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "velocity_range": {
    #             "x": (-1.0, 1.0),
    #             "y": (-1.0, 1.0),
    #             "z": (-1.0, 1.0),
    #             "roll": (-deg_to_rad(10.0), deg_to_rad(10.0)),
    #             "pitch": (-deg_to_rad(10.0), deg_to_rad(10.0)),
    #             "yaw": (-deg_to_rad(10.0), deg_to_rad(10.0)),
    #         },
    #     },
    # )


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scenario
    domain: Domain = Domain.MOON

    ## Assets
    robot: OrbitalRobot | AssetVariant = assets.PeregrineLander()
    scenery: Scenery | MobileRobot | AssetVariant | None = AssetVariant.PROCEDURAL

    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 40.0
    is_finite_horizon: bool = True

    ## Target
    tf_pos_target: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    tf_quat_target: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": SphereCfg(
                radius=0.5,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()

        # Sensor: Robot contacts
        self.scene.contacts_robot.prim_path = f"{self.scene.robot.prim_path}/.*"


############
### Task ###
############


class Task(OrbitalEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._contacts_robot: ContactSensor = self.scene["contacts_robot"]
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        ## Initialize buffers
        self._tf_pos_target = self.scene.env_origins + torch.tensor(
            self.cfg.tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

        ## Visualize target
        self._target_marker.visualize(self._tf_pos_target, self._tf_quat_target)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        ## Get remaining fuel (if applicable)
        if self._thrust_action_term_key:
            thrust_action_term: ThrustAction = self.action_manager._terms[  # type: ignore
                self._thrust_action_term_key
            ]
            remaining_fuel = (
                thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
            ).unsqueeze(-1)
        else:
            remaining_fuel = None

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
            projected_gravity_robot=self._robot.data.projected_gravity_b,
            # Transforms (world frame)
            tf_pos_target=self._tf_pos_target,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_robot=self._contacts_robot.compute_first_contact(self.step_dt),
            # IMU
            imu_lin_acc=self._imu_robot.data.lin_acc_b,
            imu_ang_vel=self._imu_robot.data.ang_vel_b,
            # Fuel
            remaining_fuel=remaining_fuel,
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
    projected_gravity_robot: torch.Tensor,
    # Transforms (world frame)
    tf_pos_target: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_robot: torch.Tensor,
    # IMU
    imu_lin_acc: torch.Tensor,
    imu_ang_vel: torch.Tensor,
    # Fuel
    remaining_fuel: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Root
    tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
    tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

    ## Transforms (world frame)
    # Robot -> Target
    tf_pos_robot_to_target = tf_pos_target - tf_pos_robot
    vertical_distance_to_target = torch.clamp_min(-tf_pos_robot_to_target[:, 2], 0.001)

    ## Contacts
    MAX_TOUCHDOWN_LINEAR_VELOCITY = 2.0
    MAX_TOUCHDOWN_ANGLE = 0.174533  # 10 degrees
    touchdown = contact_robot.any(dim=1)
    crash = touchdown & (
        (torch.norm(vel_lin_robot, dim=1) > MAX_TOUCHDOWN_LINEAR_VELOCITY)
        | (torch.acos(-projected_gravity_robot[:, 2]) > MAX_TOUCHDOWN_ANGLE)
    )
    landed = touchdown | (torch.norm(contact_forces_robot.mean(dim=1), dim=1) > 1.0)

    ## Fuel
    remaining_fuel = (
        remaining_fuel
        if remaining_fuel is not None
        else torch.ones((num_envs, 1), dtype=dtype, device=device)
    )

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.025
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Fuel consumption
    WEIGHT_FUEL_CONSUMPTION = -2.0
    penalty_fuel_consumption = WEIGHT_FUEL_CONSUMPTION * torch.square(
        1.0 - remaining_fuel.squeeze(-1)
    )

    # Penalty: Angular velocity
    WEIGHT_ANGULAR_VELOCITY = -0.25
    MAX_ANGULAR_VELOCITY_PENALTY = -5.0
    penalty_angular_velocity = torch.clamp_min(
        WEIGHT_ANGULAR_VELOCITY * torch.sum(torch.square(vel_ang_robot), dim=1),
        min=MAX_ANGULAR_VELOCITY_PENALTY,
    )

    # Penalty: Alignment with gravity
    WEIGHT_GRAVITY_ROTATION_ALIGNMENT = -1.0
    penalty_gravity_rotation_alignment = WEIGHT_GRAVITY_ROTATION_ALIGNMENT * (
        torch.sum(torch.square(projected_gravity_robot[:, :2]), dim=1)
        + torch.square(projected_gravity_robot[:, 2] + 1.0)
    )

    # Penalty: Horizontal linear velocity
    WEIGHT_HORIZONTAL_LINEAR_VELOCITY = -0.1
    penalty_horizontal_linear_velocity = WEIGHT_HORIZONTAL_LINEAR_VELOCITY * torch.sum(
        torch.abs(vel_lin_robot[:, :2]), dim=1
    )

    # Reward: Vertical linear velocity
    WEIGHT_VERTICAL_LINEAR_VELOCITY = 8.0
    TANH_STD_VERTICAL_LINEAR_VELOCITY = 1.0
    reward_vertical_linear_velocity = WEIGHT_VERTICAL_LINEAR_VELOCITY * (
        1.0
        - torch.tanh(
            torch.clamp_min(
                torch.square(vel_lin_robot[:, 2]) - vertical_distance_to_target, 0.0
            )
            / TANH_STD_VERTICAL_LINEAR_VELOCITY
        )
    )

    # Reward: Horizontal distance to target
    WEIGHT_HORIZONTAL_DISTANCE = 4.0
    TANH_STD_HORIZONTAL_DISTANCE = 10.0
    reward_horizontal_distance = WEIGHT_HORIZONTAL_DISTANCE * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_robot_to_target[:, :2], dim=-1)
            / TANH_STD_HORIZONTAL_DISTANCE
        )
    )

    # Reward: Distance to target (precision landing)
    WEIGHT_LANDING_DISTANCE = 12.0
    TANH_STD_LANDING_DISTANCE = 0.5
    reward_landing_distance = (
        touchdown.float()
        * WEIGHT_LANDING_DISTANCE
        * (
            1.0
            - torch.tanh(
                torch.norm(tf_pos_robot_to_target, dim=-1) / TANH_STD_LANDING_DISTANCE
            )
        )
    )

    # Reward: Alignment with gravity (precision landing)
    WEIGHT_LANDING_ANGLE = 8.0
    reward_landing_angle = (
        landed.float()
        * WEIGHT_LANDING_ANGLE
        * (1.0 - torch.tanh(torch.acos(-projected_gravity_robot[:, 2])))
    )

    # Reward: Velocity (precision landing)
    WEIGHT_LANDING_VELOCITY = 16.0
    TANH_STD_LANDING_VELOCITY = 0.01
    reward_landing_velocity = (
        landed.float()
        * WEIGHT_LANDING_VELOCITY
        * (
            1.0
            - torch.tanh(
                (torch.norm(vel_lin_robot, dim=-1) + torch.norm(vel_ang_robot, dim=-1))
                / TANH_STD_LANDING_VELOCITY
            )
        )
    )

    # Reward: Remaining fuel (upon landing)
    WEIGHT_LANDING_FUEL = 4.0
    reward_landing_fuel = (
        landed.float() * WEIGHT_LANDING_FUEL * remaining_fuel.squeeze(-1)
    )

    # Penalty: Crash
    WEIGHT_CRASH = -64.0
    penalty_crash = WEIGHT_CRASH * crash.float()

    ##################
    ## Terminations ##
    ##################
    # Termination: Crash
    termination_crash = crash
    # Termination: Robot far below the target
    termination_below_target = tf_pos_robot_to_target[:, 2] > 10.0
    # Termination
    termination = termination_crash | termination_below_target
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
                "projected_gravity_robot": projected_gravity_robot,
                "tf_pos_robot_to_target": tf_pos_robot_to_target,
            },
            "proprio": {
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
                "remaining_fuel": remaining_fuel,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_fuel_consumption": penalty_fuel_consumption,
            "penalty_angular_velocity": penalty_angular_velocity,
            "penalty_gravity_rotation_alignment": penalty_gravity_rotation_alignment,
            "penalty_horizontal_linear_velocity": penalty_horizontal_linear_velocity,
            "reward_vertical_linear_velocity": reward_vertical_linear_velocity,
            "reward_horizontal_distance": reward_horizontal_distance,
            "reward_landing_distance": reward_landing_distance,
            "reward_landing_angle": reward_landing_angle,
            "reward_landing_velocity": reward_landing_velocity,
            "reward_landing_fuel": reward_landing_fuel,
            "penalty_crash": penalty_crash,
        },
        termination,
        truncation,
    )
