"""Shared observation, reward, and termination logic for template tasks."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import Articulation
from srb.core.sensor import ContactSensor
from srb.utils.math import matrix_from_quat, normalize_joint_pos, rotmat_to_rot6d

if TYPE_CHECKING:
    from srb.core.env.mobile_manipulation.aerial_manipulation.env import (
        AerialManipulationEnv,
    )
    from srb.core.env.mobile_manipulation.ground_manipulation.env import (
        GroundManipulationEnv,
    )
    from srb.core.env.mobile_manipulation.orbital_manipulation.env import (
        OrbitalManipulationEnv,
    )

    TemplateEnv = AerialManipulationEnv | GroundManipulationEnv | OrbitalManipulationEnv


def template_step_return(env: "TemplateEnv") -> StepReturn:
    """Build template observations, shared penalties, and episode truncation."""
    rew = env.cfg.reward
    num_envs = env.num_envs

    ############
    ## States ##
    ############
    tf_pos_robot, tf_quat_robot, vel_lin_robot, vel_ang_robot = env._robot_base_state()

    tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
    tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

    # IMU
    imu_lin_acc = env._imu_robot.data.lin_acc_b
    imu_ang_vel = env._imu_robot.data.ang_vel_b

    # Joints
    joint_pos_robot = env._manipulator.data.joint_pos
    joint_pos_limits_robot = env._manipulator.data.soft_joint_pos_limits
    joint_pos_robot_normalized = normalize_joint_pos(
        joint_pos_robot, joint_pos_limits_robot
    )

    joint_pos_end_effector = (
        env._end_effector.data.joint_pos
        if isinstance(env._end_effector, Articulation)
        else None
    )
    joint_pos_end_effector_normalized = (
        normalize_joint_pos(
            joint_pos_end_effector,
            env._end_effector.data.soft_joint_pos_limits,
        )
        if joint_pos_end_effector is not None
        and isinstance(env._end_effector, Articulation)
        else (
            joint_pos_end_effector
            if joint_pos_end_effector is not None
            else torch.empty((num_envs, 0), dtype=torch.float32, device=env.device)
        )
    )

    # Kinematics
    fk_pos_end_effector = env._tf_end_effector.data.target_pos_source[:, 0, :]
    fk_quat_end_effector = env._tf_end_effector.data.target_quat_source[:, 0, :]
    fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
    fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

    # Contacts
    contact_forces_robot = env._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
    contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

    contact_forces_end_effector = (
        env._contacts_end_effector.data.net_forces_w
        if isinstance(env._contacts_end_effector, ContactSensor)
        else None
    )
    contact_forces_mean_end_effector = (
        contact_forces_end_effector.mean(dim=1)
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=torch.float32, device=env.device)
    )
    contact_forces_end_effector_obs = (
        contact_forces_end_effector
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=torch.float32, device=env.device)
    )

    #############
    ## Rewards ##
    #############
    reward: dict[str, torch.Tensor] = {}

    # Penalty: Action rate
    _action_rate = torch.mean(
        torch.square(env.action_manager.action - env.action_manager.prev_action),
        dim=1,
    )
    reward["penalty_action_rate"] = rew.penalty_action_rate(_action_rate)

    # Penalty: Undesired robot contacts
    _max_contact = torch.max(torch.norm(contact_forces_robot, dim=-1), dim=1)[0]
    reward["penalty_undesired_robot_contacts"] = rew.penalty_undesired_robot_contacts(
        _max_contact
    )

    ##################
    ## Terminations ##
    ##################
    termination = torch.zeros(num_envs, dtype=torch.bool, device=env.device)
    truncation = (
        env.episode_length_buf >= env.max_episode_length
        if env.cfg.truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=env.device)
    )

    return StepReturn(
        {
            ObservationGroup.STATE: {
                # Env-local, so replicas at different origins observe alike.
                "tf_pos_robot": tf_pos_robot - env.scene.env_origins,
                "tf_rot6d_robot": tf_rot6d_robot,
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
            },
            ObservationGroup.STATE_DYN: {
                "contact_forces_robot": contact_forces_robot,
                "contact_forces_end_effector": contact_forces_end_effector_obs,
                "fk_pos_end_effector": fk_pos_end_effector,
                "fk_rot6d_end_effector": fk_rot6d_end_effector,
            },
            ObservationGroup.PROPRIO: {
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
            },
            ObservationGroup.PROPRIO_DYN: {
                "joint_pos_robot_normalized": joint_pos_robot_normalized,
                "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
            },
        },
        reward,
        termination,
        truncation,
    )
