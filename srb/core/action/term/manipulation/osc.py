from typing import TYPE_CHECKING

import torch
from isaaclab.envs.mdp.actions.actions_cfg import (
    OperationalSpaceControllerActionCfg as __OperationalSpaceControllerActionCfg,
)
from isaaclab.envs.mdp.actions.task_space_actions import (
    OperationalSpaceControllerAction as __OperationalSpaceControllerAction,
)
from isaaclab.managers.action_manager import ActionTerm
from isaaclab.utils.math import quat_apply_inverse

from srb.core.action.term.manipulation._frame_jacobian import (
    resolve_jacobian_base_body,
)
from srb.utils.cfg import configclass

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
            env,  # type: ignore[misc]
        )

        # Root body only: upstream's ``jacobian_b`` and the offset lever, pose
        # and velocity it is combined with are all root-frame.
        self._base_idx = resolve_jacobian_base_body(self._asset, self.cfg.base_name)

        self._asset.write_joint_stiffness_to_sim(0.0, joint_ids=self._joint_ids)
        self._asset.write_joint_damping_to_sim(0.0, joint_ids=self._joint_ids)

    def _compute_ee_velocity(self):
        super()._compute_ee_velocity()
        # Upstream subtracts only the root linear velocity; on a rotating floating
        # base, remove the lever-arm term so damping sees only arm motion.
        data = self._asset.data
        lever_w = torch.cross(
            data.root_link_vel_w.torch[:, 3:6],
            data.body_link_pos_w.torch[:, self._ee_body_idx]
            - data.root_link_pos_w.torch,
            dim=-1,
        )
        self._ee_vel_b[:, 0:3] -= quat_apply_inverse(data.root_quat_w.torch, lever_w)


@configclass
class OperationalSpaceControllerActionCfg(__OperationalSpaceControllerActionCfg):
    class_type: type[ActionTerm] = OperationalSpaceControllerAction
    base_name: str = ""
