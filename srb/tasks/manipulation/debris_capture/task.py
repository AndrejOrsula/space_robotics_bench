from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import (
    Articulation,
    AssetVariant,
    ExtravehicularScenery,
    MobileRobot,
    Object,
    RigidObject,
    RigidObjectCfg,
)
from srb.core.domain import Domain
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    deg_to_rad,
    matrix_from_quat,
    rotmat_to_rot6d,
    rpy_to_quat,
    scale_transform,
    subtract_frame_transforms,
)

from .asset import select_debris

##############
### Config ###
##############


@configclass
class SceneCfg(ManipulationSceneCfg):
    debris: RigidObjectCfg = MISSING  # type: ignore


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_obj_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("debris"),
            "pose_range": {
                "x": (-0.25, 0.25),
                "y": (-0.25, 0.25),
                "z": (-0.25, 0.25),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.2 - 0.05, -0.2 + 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
                "roll": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "pitch": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "yaw": (-deg_to_rad(10.0), deg_to_rad(10.0)),
            },
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    ## Scenario
    domain: Domain = Domain.ORBIT

    ## Assets
    scenery: ExtravehicularScenery | MobileRobot | AssetVariant | None = (
        assets.VenusExpress()
    )
    scenery.asset_cfg.init_state.pos = (-0.55, 0.0, -0.35)
    scenery.asset_cfg.init_state.rot = rpy_to_quat(0.0, 0.0, 90.0)

    _scenery: ExtravehicularScenery = MISSING  # type: ignore
    pedestal: Object | AssetVariant | None = None
    debris: Object | AssetVariant | None = AssetVariant.DATASET

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = True

    def __post_init__(self):
        super().__post_init__()

        # Scene: Debris
        self.scene.debris = select_debris(
            self,
            prim_path="{ENV_REGEX_NS}/debris",
            init_state=RigidObjectCfg.InitialStateCfg(pos=(1.5, 0.0, 0.5)),
            activate_contact_sensors=True,
        )

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.debris.prim_path
            ]

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############


class Task(ManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._obj: RigidObject = self.scene["debris"]

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        return _compute_step_return(
            ## Time
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            ## Actions
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            ## States
            # Joints
            joint_pos_robot=self._robot.data.joint_pos,
            joint_pos_limits_robot=(
                self._robot.data.soft_joint_pos_limits
                if torch.all(torch.isfinite(self._robot.data.soft_joint_pos_limits))
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
            joint_acc_robot=self._robot.data.joint_acc,
            joint_applied_torque_robot=self._robot.data.applied_torque,
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Transforms (world frame)
            tf_pos_end_effector=self._tf_end_effector.data.target_pos_w[:, 0, :],
            tf_quat_end_effector=self._tf_end_effector.data.target_quat_w[:, 0, :],
            tf_pos_obj=self._obj.data.root_com_pos_w,
            tf_quat_obj=self._obj.data.root_com_quat_w,
            # Object velocity
            vel_lin_obj=self._obj.data.root_com_lin_vel_w,
            vel_ang_obj=self._obj.data.root_com_ang_vel_w,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            contact_force_matrix_end_effector=self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
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
    # Object velocity
    vel_lin_obj: torch.Tensor,
    vel_ang_obj: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    contact_force_matrix_end_effector: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
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

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
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
    TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ = 0.05
    reward_distance_end_effector_to_obj = WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_end_effector_to_obj, dim=-1)
            / TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ
        )
    )

    # Reward: Grasp object
    WEIGHT_GRASP = 16.0
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

    # Penalty: Debris velocity
    WEIGHT_DEBRIS_VELOCITY_LIN = -3.0
    WEIGHT_DEBRIS_VELOCITY_ANG = -1.0
    penalty_debris_velocity = WEIGHT_DEBRIS_VELOCITY_LIN * torch.norm(
        vel_lin_obj, dim=-1
    ) + WEIGHT_DEBRIS_VELOCITY_ANG * torch.norm(vel_ang_obj, dim=-1)

    ##################
    ## Terminations ##
    ##################
    # Termination: Debris too far from the end-effector
    termination_debris_too_far = torch.norm(tf_pos_end_effector_to_obj, dim=-1) > 10.0
    # Termination
    termination = termination_debris_too_far
    # Truncation
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        {
            "state": {
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                "tf_pos_end_effector_to_obj": tf_pos_end_effector_to_obj,
                "tf_rot6d_end_effector_to_obj": tf_rot6d_end_effector_to_obj,
                "vel_lin_obj": vel_lin_obj,
                "vel_ang_obj": vel_ang_obj,
            },
            "state_dyn": {
                "contact_forces_robot": contact_forces_robot,
                "contact_forces_end_effector": contact_forces_end_effector,
            },
            "proprio": {
                "fk_pos_end_effector": fk_pos_end_effector,
                "fk_rot6d_end_effector": fk_rot6d_end_effector,
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
            "penalty_joint_torque": penalty_joint_torque,
            "penalty_joint_acceleration": penalty_joint_acceleration,
            "reward_distance_end_effector_to_obj": reward_distance_end_effector_to_obj,
            "reward_grasp": reward_grasp,
            "penalty_debris_velocity": penalty_debris_velocity,
        },
        termination,
        truncation,
    )
