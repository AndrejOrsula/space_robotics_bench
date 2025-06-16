from dataclasses import MISSING
from typing import List, Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import AssetVariant, Humanoid, LeggedRobot
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import push_by_setting_velocity  # noqa: F401
from srb.core.mdp import reset_joints_by_scale
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d, scale_transform

from .task import EventCfg, SceneCfg, Task, TaskCfg

##############
### Config ###
##############


@configclass
class LocomotionSceneCfg(SceneCfg):
    contacts_robot: ContactSensorCfg = ContactSensorCfg(
        prim_path=MISSING,  # type: ignore
        update_period=0.0,
        history_length=3,
        track_air_time=True,
    )


@configclass
class LocomotionEventCfg(EventCfg):
    randomize_robot_joints: EventTermCfg = EventTermCfg(
        func=reset_joints_by_scale,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )
    # push_robot: EventTermCfg = EventTermCfg(
    #     func=push_by_setting_velocity,
    #     mode="interval",
    #     interval_range_s=(10.0, 15.0),
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "velocity_range": {
    #             "x": (-0.5, 0.5),
    #             "y": (-0.5, 0.5),
    #         },
    #     },
    # )


@configclass
class LocomotionTaskCfg(TaskCfg):
    ## Assets
    robot: LeggedRobot | Humanoid | AssetVariant = assets.Spot()
    _robot: LeggedRobot = MISSING  # type: ignore

    ## Scene
    scene: LocomotionSceneCfg = LocomotionSceneCfg()

    ## Events
    events: LocomotionEventCfg = LocomotionEventCfg()

    ## Time
    env_rate: float = 1.0 / 250.0

    def __post_init__(self):
        super().__post_init__()

        # Sensor: Robot contacts
        self.scene.contacts_robot.prim_path = f"{self.scene.robot.prim_path}/.*"


############
### Task ###
############


class LocomotionTask(Task):
    cfg: LocomotionTaskCfg

    def __init__(self, cfg: LocomotionTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._contacts_robot: ContactSensor = self.scene["contacts_robot"]

        ## Cache metrics
        self._feet_indices, _ = self._robot.find_bodies(
            self.cfg._robot.regex_feet_links
        )
        _all_body_indices, _ = self._robot.find_bodies(".*")
        self._undesired_contact_body_indices = [
            idx for idx in _all_body_indices if idx not in self._feet_indices
        ]

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        if self.cfg.command_vis or self.cfg.debug_vis:
            self._update_visualization_markers()

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
            tf_quat_robot=self._robot.data.root_quat_w,
            vel_lin_robot=self._robot.data.root_lin_vel_b,
            vel_ang_robot=self._robot.data.root_ang_vel_b,
            projected_gravity_robot=self._robot.data.projected_gravity_b,
            # Joints
            joint_pos_robot=self._robot.data.joint_pos,
            joint_pos_limits_robot=(
                self._robot.data.soft_joint_pos_limits
                if torch.all(torch.isfinite(self._robot.data.soft_joint_pos_limits))
                else None
            ),
            joint_acc_robot=self._robot.data.joint_acc,
            joint_applied_torque_robot=self._robot.data.applied_torque,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_robot=self._contacts_robot.compute_first_contact(self.step_dt),
            contact_last_air_time=self._contacts_robot.data.last_air_time,  # type: ignore
            # IMU
            imu_lin_acc=self._imu_robot.data.lin_acc_b,
            imu_ang_vel=self._imu_robot.data.ang_vel_b,
            ## Robot descriptors
            robot_feet_indices=self._feet_indices,
            robot_undesired_contact_body_indices=self._undesired_contact_body_indices,
            ## Command
            command=self._command,
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
    tf_quat_robot: torch.Tensor,
    vel_lin_robot: torch.Tensor,
    vel_ang_robot: torch.Tensor,
    projected_gravity_robot: torch.Tensor,
    # Joints
    joint_pos_robot: torch.Tensor,
    joint_pos_limits_robot: torch.Tensor | None,
    joint_acc_robot: torch.Tensor,
    joint_applied_torque_robot: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_robot: torch.Tensor,
    contact_last_air_time: torch.Tensor,
    # IMU
    imu_lin_acc: torch.Tensor,
    imu_ang_vel: torch.Tensor,
    ## Robot descriptors
    robot_feet_indices: List[int],
    robot_undesired_contact_body_indices: List[int],
    ## Command
    command: torch.Tensor,
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

    ## Joints
    joint_pos_robot_normalized = (
        scale_transform(
            joint_pos_robot,
            joint_pos_limits_robot[:, :, 0],
            joint_pos_limits_robot[:, :, 1],
        )
        if joint_pos_limits_robot is not None
        else joint_pos_robot
    )

    ## Contacts
    contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

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
    WEIGHT_JOINT_ACCELERATION = -0.00000025
    MAX_JOINT_ACCELERATION_PENALTY = -2.0
    penalty_joint_acceleration = torch.clamp_min(
        WEIGHT_JOINT_ACCELERATION * torch.sum(torch.square(joint_acc_robot), dim=1),
        min=MAX_JOINT_ACCELERATION_PENALTY,
    )

    # Penalty: Undesired robot contacts
    WEIGHT_UNDESIRED_ROBOT_CONTACTS = -2.0
    THRESHOLD_UNDESIRED_ROBOT_CONTACTS = 1.0
    penalty_undesired_robot_contacts = WEIGHT_UNDESIRED_ROBOT_CONTACTS * (
        torch.max(
            torch.norm(
                contact_forces_robot[:, robot_undesired_contact_body_indices, :],
                dim=-1,
            ),
            dim=1,
        )[0]
        > THRESHOLD_UNDESIRED_ROBOT_CONTACTS
    )

    # Reward: Command tracking (linear)
    WEIGHT_CMD_LIN_VEL_XY = 3.0
    EXP_STD_CMD_LIN_VEL_XY = 0.5
    reward_cmd_lin_vel_xy = WEIGHT_CMD_LIN_VEL_XY * torch.exp(
        -torch.sum(torch.square(command[:, :2] - vel_lin_robot[:, :2]), dim=1)
        / EXP_STD_CMD_LIN_VEL_XY
    )

    # Reward: Command tracking (angular)
    WEIGHT_CMD_ANG_VEL_Z = 1.5
    EXP_STD_CMD_ANG_VEL_Z = 0.25
    reward_cmd_ang_vel_z = WEIGHT_CMD_ANG_VEL_Z * torch.exp(
        -torch.square(command[:, 2] - vel_ang_robot[:, 2]) / EXP_STD_CMD_ANG_VEL_Z
    )

    # Reward: Feet air time
    WEIGHT_FEET_AIR_TIME = 0.5
    THRESHOLD_FEET_AIR_TIME = 0.1
    reward_feet_air_time = (
        WEIGHT_FEET_AIR_TIME
        * (torch.norm(command[:, :2], dim=1) > THRESHOLD_FEET_AIR_TIME)
        * torch.sum(
            (contact_last_air_time[:, robot_feet_indices] - 0.5)
            * contact_robot[:, robot_feet_indices],
            dim=1,
        )
    )

    # Penalty: Minimize non-command motion (linear)
    WEIGHT_UNDESIRED_LIN_VEL_Z = -0.5
    penalty_undesired_lin_vel_z = WEIGHT_UNDESIRED_LIN_VEL_Z * torch.square(
        vel_lin_robot[:, 2]
    )

    # Penalty: Minimize non-command motion (angular)
    WEIGHT_UNDESIRED_ANG_VEL_XY = -0.1
    penalty_undesired_ang_vel_xy = WEIGHT_UNDESIRED_ANG_VEL_XY * torch.sum(
        torch.square(vel_ang_robot[:, :2]), dim=-1
    )

    # Penalty: Minimize rotation with the gravity direction
    WEIGHT_GRAVITY_ROTATION_ALIGNMENT = -2.0
    penalty_gravity_rotation_alignment = WEIGHT_GRAVITY_ROTATION_ALIGNMENT * (
        torch.sum(torch.square(projected_gravity_robot[:, :2]), dim=1)
        + torch.square(projected_gravity_robot[:, 2] + 1.0)
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
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "tf_rot6d_robot": tf_rot6d_robot,
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
                "projected_gravity_robot": projected_gravity_robot,
            },
            "state_dyn": {
                "contact_forces_robot": contact_forces_robot,
            },
            "proprio": {
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
            },
            "proprio_dyn": {
                "joint_pos_robot_normalized": joint_pos_robot_normalized,
                "joint_acc_robot": joint_acc_robot,
                "joint_applied_torque_robot": joint_applied_torque_robot,
            },
            "command": {
                "cmd_vel": command,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_joint_torque": penalty_joint_torque,
            "penalty_joint_acceleration": penalty_joint_acceleration,
            "penalty_undesired_robot_contacts": penalty_undesired_robot_contacts,
            "reward_cmd_lin_vel_xy": reward_cmd_lin_vel_xy,
            "reward_cmd_ang_vel_z": reward_cmd_ang_vel_z,
            "reward_feet_air_time": reward_feet_air_time,
            "penalty_undesired_lin_vel_z": penalty_undesired_lin_vel_z,
            "penalty_undesired_ang_vel_xy": penalty_undesired_ang_vel_xy,
            "penalty_gravity_rotation_alignment": penalty_gravity_rotation_alignment,
        },
        termination,
        truncation,
    )
