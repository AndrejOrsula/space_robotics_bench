from typing import TYPE_CHECKING, Type

import isaaclab.utils.math as math_utils
import torch
from isaaclab.envs.mdp.actions.actions_cfg import (
    DifferentialInverseKinematicsActionCfg as __DifferentialInverseKinematicsActionCfg,
)
from isaaclab.envs.mdp.actions.task_space_actions import (
    DifferentialInverseKinematicsAction as __DifferentialInverseKinematicsAction,
)
from isaaclab.managers.action_manager import ActionTerm
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation


class DifferentialInverseKinematicsAction(__DifferentialInverseKinematicsAction):
    cfg: "DifferentialInverseKinematicsActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation"

    def __init__(self, cfg: "DifferentialInverseKinematicsActionCfg", env: "AnyEnv"):
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

    def _compute_frame_pose(self) -> tuple[torch.Tensor, torch.Tensor]:
        ee_pos_w = self._asset.data.body_pos_w[:, self._body_idx]
        ee_quat_w = self._asset.data.body_quat_w[:, self._body_idx]

        if self._base_idx is not None:
            base_pos_w = self._asset.data.body_pos_w[:, self._base_idx]
            base_quat_w = self._asset.data.body_quat_w[:, self._base_idx]
        else:
            base_pos_w = self._asset.data.root_pos_w
            base_quat_w = self._asset.data.root_quat_w

        ee_pose_b, ee_quat_b = math_utils.subtract_frame_transforms(
            base_pos_w, base_quat_w, ee_pos_w, ee_quat_w
        )

        if self.cfg.body_offset is not None:
            ee_pose_b, ee_quat_b = math_utils.combine_frame_transforms(
                ee_pose_b, ee_quat_b, self._offset_pos, self._offset_rot
            )

        return ee_pose_b, ee_quat_b

    def _compute_frame_jacobian(self) -> torch.Tensor:
        jacobian = self.jacobian_b
        if self.cfg.body_offset is not None:
            jacobian[:, :3, :] += torch.bmm(
                math_utils.skew_symmetric_matrix(self._offset_pos), jacobian[:, 3:, :]
            )
            jacobian[:, 3:, :] = torch.bmm(
                math_utils.matrix_from_quat(self._offset_rot), jacobian[:, 3:, :]
            )
        return jacobian


@configclass
class DifferentialInverseKinematicsActionCfg(__DifferentialInverseKinematicsActionCfg):
    class_type: Type[ActionTerm] = DifferentialInverseKinematicsAction
    base_name: str = ""
