from typing import TYPE_CHECKING, Type

import isaaclab.utils.math as math_utils
import torch
from isaaclab.envs.mdp.actions.actions_cfg import (
    OperationalSpaceControllerActionCfg as __OperationalSpaceControllerActionCfg,
)
from isaaclab.envs.mdp.actions.task_space_actions import (
    OperationalSpaceControllerAction as __OperationalSpaceControllerAction,
)
from isaaclab.managers.action_manager import ActionTerm
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation


class OperationalSpaceControllerAction(__OperationalSpaceControllerAction):
    cfg: "OperationalSpaceControllerActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation"

    def __init__(self, cfg: "OperationalSpaceControllerActionCfg", env: "AnyEnv"):
        super().__init__(
            cfg,
            env,  # type: ignore
        )

        if self.cfg.base_name:
            base_ids, base_names = self._asset.find_bodies(self.cfg.base_name)
            if len(base_ids) != 1:
                raise ValueError(
                    f"Expected one match for the base name: {self.cfg.base_name}. Found {len(base_ids)}: {base_names}."
                )
            self._base_idx = base_ids[0]
        else:
            self._base_idx = None

        self._asset.write_joint_stiffness_to_sim(0.0, joint_ids=self._joint_ids)
        self._asset.write_joint_damping_to_sim(0.0, joint_ids=self._joint_ids)

    @property
    def jacobian_b(self) -> torch.Tensor:
        jacobian = self.jacobian_w

        if self._base_idx:
            base_quat_w = self._asset.data.body_quat_w[:, self._base_idx]
        else:
            base_quat_w = self._asset.data.root_quat_w

        base_rot_matrix = math_utils.matrix_from_quat(math_utils.quat_inv(base_quat_w))
        jacobian[:, :3, :] = torch.bmm(base_rot_matrix, jacobian[:, :3, :])
        jacobian[:, 3:, :] = torch.bmm(base_rot_matrix, jacobian[:, 3:, :])
        return jacobian

    def _compute_ee_pose(self):
        self._ee_pose_w[:, 0:3] = self._asset.data.body_pos_w[:, self._ee_body_idx]
        self._ee_pose_w[:, 3:7] = self._asset.data.body_quat_w[:, self._ee_body_idx]

        if self._base_idx is not None:
            base_pos_w = self._asset.data.body_pos_w[:, self._base_idx]
            base_quat_w = self._asset.data.body_quat_w[:, self._base_idx]
        else:
            base_pos_w = self._asset.data.root_pos_w
            base_quat_w = self._asset.data.root_quat_w

        self._ee_pose_b_no_offset[:, 0:3], self._ee_pose_b_no_offset[:, 3:7] = (
            math_utils.subtract_frame_transforms(
                base_pos_w,
                base_quat_w,
                self._ee_pose_w[:, 0:3],
                self._ee_pose_w[:, 3:7],
            )
        )

        if self.cfg.body_offset is not None:
            self._ee_pose_b[:, 0:3], self._ee_pose_b[:, 3:7] = (
                math_utils.combine_frame_transforms(
                    self._ee_pose_b_no_offset[:, 0:3],
                    self._ee_pose_b_no_offset[:, 3:7],
                    self._offset_pos,
                    self._offset_rot,
                )
            )
        else:
            self._ee_pose_b[:] = self._ee_pose_b_no_offset

    def _compute_ee_velocity(self):
        self._ee_vel_w[:] = self._asset.data.body_vel_w[:, self._ee_body_idx, :]

        if self._base_idx is not None:
            base_vel_w = self._asset.data.body_vel_w[:, self._base_idx]
        else:
            base_vel_w = self._asset.data.root_vel_w
        relative_vel_w = self._ee_vel_w - base_vel_w

        self._ee_vel_b[:, 0:3] = math_utils.quat_rotate_inverse(
            self._asset.data.root_quat_w, relative_vel_w[:, 0:3]
        )
        self._ee_vel_b[:, 3:6] = math_utils.quat_rotate_inverse(
            self._asset.data.root_quat_w, relative_vel_w[:, 3:6]
        )

        # Account for the offset
        if self.cfg.body_offset is not None:
            # Compute offset vector in root frame
            r_offset_b = math_utils.quat_rotate(
                self._ee_pose_b_no_offset[:, 3:7], self._offset_pos
            )
            # Adjust the linear velocity to account for the offset
            self._ee_vel_b[:, :3] += torch.cross(
                self._ee_vel_b[:, 3:], r_offset_b, dim=-1
            )
            # Angular velocity is not affected by the offset

    def _compute_ee_jacobian(self):
        self._jacobian_b[:] = self.jacobian_b
        if self.cfg.body_offset is not None:
            self._jacobian_b[:, 0:3, :] += torch.bmm(
                math_utils.skew_symmetric_matrix(self._offset_pos),
                self._jacobian_b[:, 3:, :],
            )
            self._jacobian_b[:, 3:, :] = torch.bmm(
                math_utils.matrix_from_quat(self._offset_rot),
                self._jacobian_b[:, 3:, :],
            )


@configclass
class OperationalSpaceControllerActionCfg(__OperationalSpaceControllerActionCfg):
    class_type: Type[ActionTerm] = OperationalSpaceControllerAction
    base_name: str = ""
