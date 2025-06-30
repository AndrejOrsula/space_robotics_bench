from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.action import ThrustAction
from srb.core.asset import (
    Articulation,
    AssetVariant,
    Object,
    OrbitalManipulator,
    RigidObject,
    RigidObjectCfg,
)
from srb.core.env import (
    OrbitalManipulationEnv,
    OrbitalManipulationEnvCfg,
    OrbitalManipulationEventCfg,
    OrbitalManipulationSceneCfg,
    ViewerCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.sensor import ContactSensor
from srb.core.sim import SimforgeAssetCfg
from srb.tasks.manipulation.debris_capture.asset import select_debris
from srb.utils.cfg import configclass
from srb.utils.math import (
    deg_to_rad,
    matrix_from_quat,
    rotmat_to_rot6d,
    scale_transform,
    subtract_frame_transforms,
)

##############
### Config ###
##############


@configclass
class SceneCfg(OrbitalManipulationSceneCfg):
    env_spacing: float = 8.0
    debris: RigidObjectCfg = MISSING  # type: ignore


@configclass
class EventCfg(OrbitalManipulationEventCfg):
    randomize_obj_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("debris"),
            "pose_range": {
                "x": (2.0, 4.0),
                "y": (-1.0, 1.0),
                "z": (-1.0, 1.0),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-2.0, -0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "pitch": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "yaw": (-deg_to_rad(10.0), deg_to_rad(10.0)),
            },
        },
    )


@configclass
class TaskCfg(OrbitalManipulationEnvCfg):
    ## Assets
    robot: OrbitalManipulator | AssetVariant = assets.GenericOrbitalManipulator(
        mobile_base=assets.VenusExpress(), manipulator=assets.Franka()
    )
    debris: Object | AssetVariant | None = AssetVariant.PROCEDURAL

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 60.0
    is_finite_horizon: bool = True

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(8.0, -8.0, 4.0), lookat=(2.0, -2.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()

        # Scene: Debris
        self.scene.debris = select_debris(
            self,  # type: ignore
            prim_path="{ENV_REGEX_NS}/debris",
            init_state=RigidObjectCfg.InitialStateCfg(pos=(5.0, 0.0, 0.0)),
            activate_contact_sensors=True,
        )

        if isinstance(self.scene.debris.spawn, SimforgeAssetCfg):
            self.scene.debris.spawn.seed = self.seed + self.scene.num_envs

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############


class Task(OrbitalManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._obj: RigidObject = self.scene["debris"]
        self._contacts_end_effector: ContactSensor = self.scene["contacts_end_effector"]

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
            # Joints
            joint_pos_robot=self._manipulator.data.joint_pos,
            joint_pos_limits_robot=(
                self._manipulator.data.soft_joint_pos_limits
                if torch.all(
                    torch.isfinite(self._manipulator.data.soft_joint_pos_limits)
                )
                else None
            ),
            joint_pos_end_effector=self._end_effector.data.joint_pos
            if isinstance(self._end_effector, Articulation)
            else None,
            joint_pos_limits_end_effector=(
                self._end_effector.data.soft_joint_pos_limits
                if isinstance(self._end_effector, Articulation)
                and torch.all(
                    torch.isfinite(self._end_effector.data.soft_joint_pos_limits)
                )
                else None
            ),
            joint_acc_robot=self._manipulator.data.joint_acc,
            joint_applied_torque_robot=self._manipulator.data.applied_torque,
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Transforms (world frame)
            tf_pos_end_effector=self._tf_end_effector.data.target_pos_w[:, 0, :],
            tf_quat_end_effector=self._tf_end_effector.data.target_quat_w[:, 0, :],
            tf_pos_obj=self._obj.data.root_pos_w,
            tf_quat_obj=self._obj.data.root_quat_w,
            # Velocities
            vel_obj=self._obj.data.root_com_vel_w,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            contact_force_matrix_end_effector=self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
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
    # Joints
    joint_pos_robot: torch.Tensor,
    joint_pos_limits_robot: torch.Tensor | None,
    joint_pos_end_effector: torch.Tensor | None,
    joint_pos_limits_end_effector: torch.Tensor | None,
    joint_acc_robot: torch.Tensor,
    joint_applied_torque_robot: torch.Tensor,
    # Kinematics
    fk_pos_end_effector: torch.Tensor,
    fk_quat_end_effector: torch.Tensor,
    # Transforms (world frame)
    tf_pos_end_effector: torch.Tensor,
    tf_quat_end_effector: torch.Tensor,
    tf_pos_obj: torch.Tensor,
    tf_quat_obj: torch.Tensor,
    # Velocities
    vel_obj: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    contact_force_matrix_end_effector: torch.Tensor | None,
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

    ## Robot -> Object
    tf_pos_robot_to_obj = tf_pos_obj - tf_pos_robot
    vel_lin_robot_to_obj = vel_obj[:, :3] - vel_lin_robot
    vel_ang_robot_to_obj = vel_obj[:, 3:] - vel_ang_robot

    ## Joints
    # Robot joints
    joint_pos_robot_normalized = (
        scale_transform(
            joint_pos_robot,
            joint_pos_limits_robot[:, :, 0],
            joint_pos_limits_robot[:, :, 1],
        )
        if joint_pos_limits_robot is not None
        else joint_pos_robot
    )
    # End-effector joints
    joint_pos_end_effector_normalized = (
        scale_transform(
            joint_pos_end_effector,
            joint_pos_limits_end_effector[:, :, 0],
            joint_pos_limits_end_effector[:, :, 1],
        )
        if joint_pos_end_effector is not None
        and joint_pos_limits_end_effector is not None
        else (
            joint_pos_end_effector
            if joint_pos_end_effector is not None
            else torch.empty((num_envs, 0), dtype=dtype, device=device)
        )
    )

    ## Kinematics
    fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
    fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

    ## Transforms (world frame)
    # End-effector -> Object
    tf_pos_end_effector_to_obj, tf_quat_end_effector_to_obj = subtract_frame_transforms(
        t01=tf_pos_end_effector,
        q01=tf_quat_end_effector,
        t02=tf_pos_obj,
        q02=tf_quat_obj,
    )
    tf_rotmat_end_effector_to_obj = matrix_from_quat(tf_quat_end_effector_to_obj)
    tf_rot6d_end_effector_to_obj = rotmat_to_rot6d(tf_rotmat_end_effector_to_obj)

    ## Contacts
    contact_forces_mean_robot = contact_forces_robot.mean(dim=1)
    contact_forces_mean_end_effector = (
        contact_forces_end_effector.mean(dim=1)
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )
    contact_forces_end_effector = (
        contact_forces_end_effector
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )

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
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Fuel consumption
    WEIGHT_FUEL_CONSUMPTION = -2.0
    penalty_fuel_consumption = WEIGHT_FUEL_CONSUMPTION * torch.square(
        1.0 - remaining_fuel.squeeze(-1)
    )

    # Penalty: Angular velocity (robot)
    WEIGHT_ANGULAR_VELOCITY = -0.05
    MAX_ANGULAR_VELOCITY_PENALTY = -5.0
    penalty_angular_velocity = torch.clamp_min(
        WEIGHT_ANGULAR_VELOCITY * torch.sum(torch.square(vel_ang_robot), dim=1),
        min=MAX_ANGULAR_VELOCITY_PENALTY,
    )

    # Penalty: Joint torque
    WEIGHT_JOINT_TORQUE = -0.000025
    MAX_JOINT_TORQUE_PENALTY = -4.0
    penalty_joint_torque = torch.clamp_min(
        WEIGHT_JOINT_TORQUE
        * torch.sum(torch.square(joint_applied_torque_robot), dim=1),
        min=MAX_JOINT_TORQUE_PENALTY,
    )

    # Penalty: Joint acceleration
    WEIGHT_JOINT_ACCELERATION = -0.0005
    MAX_JOINT_ACCELERATION_PENALTY = -4.0
    penalty_joint_acceleration = torch.clamp_min(
        WEIGHT_JOINT_ACCELERATION * torch.sum(torch.square(joint_acc_robot), dim=1),
        min=MAX_JOINT_ACCELERATION_PENALTY,
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ = 16.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ = 0.2
    reward_distance_end_effector_to_obj = WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_end_effector_to_obj, dim=-1)
            / TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ
        )
    )

    # Reward: Grasp object
    WEIGHT_GRASP = 32.0
    THRESHOLD_GRASP = 5.0
    reward_grasp = (
        WEIGHT_GRASP
        * (
            torch.mean(
                torch.max(
                    torch.norm(contact_force_matrix_end_effector, dim=-1), dim=-1
                )[0],
                dim=1,
            )
            > THRESHOLD_GRASP
        )
        if contact_force_matrix_end_effector is not None
        else torch.zeros(num_envs, dtype=dtype, device=device)
    )

    # Reward: Minimize relative linear velocity
    WEIGHT_REL_LIN_VEL = 12.0
    TANH_STD_REL_LIN_VEL = 0.01
    reward_minimize_rel_lin_vel = WEIGHT_REL_LIN_VEL * (
        1.0
        - torch.tanh(torch.norm(vel_lin_robot_to_obj, dim=-1) / TANH_STD_REL_LIN_VEL)
    )

    # Reward: Minimize relative angular velocity
    WEIGHT_REL_ANG_VEL = 24.0
    TANH_STD_REL_ANG_VEL = 0.03
    reward_minimize_rel_ang_vel = WEIGHT_REL_ANG_VEL * (
        1.0
        - torch.tanh(torch.norm(vel_ang_robot_to_obj, dim=-1) / TANH_STD_REL_ANG_VEL)
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
                "rot6d_robot": tf_rot6d_robot,
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
                "tf_pos_robot_to_obj": tf_pos_robot_to_obj,
                "tf_pos_end_effector_to_obj": tf_pos_end_effector_to_obj,
                "tf_rot6d_end_effector_to_obj": tf_rot6d_end_effector_to_obj,
                "vel_obj": vel_obj,
                "vel_lin_robot_to_obj": vel_lin_robot_to_obj,
                "vel_ang_robot_to_obj": vel_ang_robot_to_obj,
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
            },
            "state_dyn": {
                "contact_forces_robot": contact_forces_robot,
                "contact_forces_end_effector": contact_forces_end_effector,
            },
            "proprio": {
                "fk_pos_end_effector": fk_pos_end_effector,
                "fk_rot6d_end_effector": fk_rot6d_end_effector,
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
                "remaining_fuel": remaining_fuel,
            },
            "proprio_dyn": {
                "joint_pos_robot_normalized": joint_pos_robot_normalized,
                "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
                "joint_acc_robot": joint_acc_robot,
                "joint_applied_torque_robot": joint_applied_torque_robot,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_fuel_consumption": penalty_fuel_consumption,
            "penalty_angular_velocity": penalty_angular_velocity,
            "penalty_joint_torque": penalty_joint_torque,
            "penalty_joint_acceleration": penalty_joint_acceleration,
            "reward_distance_end_effector_to_obj": reward_distance_end_effector_to_obj,
            "reward_grasp": reward_grasp,
            "reward_minimize_rel_lin_vel": reward_minimize_rel_lin_vel,
            "reward_minimize_rel_ang_vel": reward_minimize_rel_ang_vel,
        },
        termination,
        truncation,
    )
