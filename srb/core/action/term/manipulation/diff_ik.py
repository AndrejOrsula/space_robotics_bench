from typing import TYPE_CHECKING

from isaaclab.envs.mdp.actions.actions_cfg import (
    DifferentialInverseKinematicsActionCfg as __DifferentialInverseKinematicsActionCfg,
)
from isaaclab.envs.mdp.actions.task_space_actions import (
    DifferentialInverseKinematicsAction as __DifferentialInverseKinematicsAction,
)
from isaaclab.managers.action_manager import ActionTerm

from srb.core.action.term.manipulation._frame_jacobian import (
    resolve_jacobian_base_body,
)
from srb.utils.cfg import configclass

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
            env,  # type: ignore[misc]
        )

        # Root body only: upstream's ``jacobian_b`` and the offset lever, pose
        # and velocity it is combined with are all root-frame.
        self._base_idx = resolve_jacobian_base_body(self._asset, self.cfg.base_name)


@configclass
class DifferentialInverseKinematicsActionCfg(__DifferentialInverseKinematicsActionCfg):
    class_type: type[ActionTerm] = DifferentialInverseKinematicsAction
    base_name: str = ""
