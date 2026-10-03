"""Shared state and observation helpers for fixed-base manipulation tasks."""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import torch

from srb._typing import ObservationGroup
from srb.core.asset import Articulation
from srb.utils.math import matrix_from_quat, normalize_joint_pos, rotmat_to_rot6d

if TYPE_CHECKING:
    from srb.core.env.manipulation.env import ManipulationEnv


class ManipulatorStepState(NamedTuple):
    """The manipulator state shared by every fixed-base task observation."""

    num_envs: int
    device: torch.device | str
    dtype: torch.dtype
    act_current: torch.Tensor
    act_previous: torch.Tensor
    joint_acc_robot: torch.Tensor
    joint_applied_torque_robot: torch.Tensor
    joint_pos_robot_normalized: torch.Tensor
    joint_pos_ee_normalized: torch.Tensor
    fk_pos_ee: torch.Tensor
    fk_rotmat_ee: torch.Tensor
    fk_rot6d_ee: torch.Tensor
    tf_pos_ee: torch.Tensor
    tf_quat_ee: torch.Tensor


def manipulator_step_state(env: "ManipulationEnv") -> ManipulatorStepState:
    """Read shared task state; normalize finite joint limits and keep empty EE blocks."""
    num_envs = env.num_envs
    device = env.device
    dtype = torch.float32

    act_current = env.action_manager.action
    act_previous = env.action_manager.prev_action

    # Joints
    joint_pos_robot = env._robot.data.joint_pos
    joint_pos_limits = env._robot.data.soft_joint_pos_limits
    joint_acc_robot = env._robot.data.joint_acc
    joint_applied_torque_robot = env._robot.data.applied_torque

    joint_pos_robot_normalized = normalize_joint_pos(joint_pos_robot, joint_pos_limits)

    # End-effector joints
    if isinstance(env._end_effector, Articulation):
        joint_pos_ee = env._end_effector.data.joint_pos
        ee_limits = env._end_effector.data.soft_joint_pos_limits
        joint_pos_ee_normalized = normalize_joint_pos(joint_pos_ee, ee_limits)
    else:
        joint_pos_ee_normalized = torch.empty((num_envs, 0), dtype=dtype, device=device)

    # Kinematics
    fk_pos_ee = env._tf_end_effector.data.target_pos_source[:, 0, :]
    fk_quat_ee = env._tf_end_effector.data.target_quat_source[:, 0, :]
    fk_rotmat_ee = matrix_from_quat(fk_quat_ee)
    fk_rot6d_ee = rotmat_to_rot6d(fk_rotmat_ee)

    # Transforms (world frame)
    tf_pos_ee = env._tf_end_effector.data.target_pos_w[:, 0, :]
    tf_quat_ee = env._tf_end_effector.data.target_quat_w[:, 0, :]

    return ManipulatorStepState(
        num_envs=num_envs,
        device=device,
        dtype=dtype,
        act_current=act_current,
        act_previous=act_previous,
        joint_acc_robot=joint_acc_robot,
        joint_applied_torque_robot=joint_applied_torque_robot,
        joint_pos_robot_normalized=joint_pos_robot_normalized,
        joint_pos_ee_normalized=joint_pos_ee_normalized,
        fk_pos_ee=fk_pos_ee,
        fk_rotmat_ee=fk_rotmat_ee,
        fk_rot6d_ee=fk_rot6d_ee,
        tf_pos_ee=tf_pos_ee,
        tf_quat_ee=tf_quat_ee,
    )


def manipulator_observation_tail(
    state: ManipulatorStepState,
    contact_forces_robot: torch.Tensor,
    contact_forces_ee: torch.Tensor,
) -> dict[ObservationGroup, dict[str, torch.Tensor]]:
    """Build shared contact and proprioceptive groups from task state."""
    return {
        ObservationGroup.STATE_DYN: {
            "contact_forces_robot": contact_forces_robot,
            "contact_forces_end_effector": contact_forces_ee,
        },
        ObservationGroup.PROPRIO: {
            "fk_pos_end_effector": state.fk_pos_ee,
            "fk_rot6d_end_effector": state.fk_rot6d_ee,
        },
        ObservationGroup.PROPRIO_DYN: {
            "joint_pos_robot_normalized": state.joint_pos_robot_normalized,
            "joint_pos_end_effector_normalized": state.joint_pos_ee_normalized,
            "joint_acc_robot": state.joint_acc_robot,
            "joint_applied_torque_robot": state.joint_applied_torque_robot,
        },
    }
