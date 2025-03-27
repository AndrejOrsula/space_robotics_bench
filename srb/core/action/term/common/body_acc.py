from typing import TYPE_CHECKING, Type

import torch

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation, RigidObject


class BodyAccelerationAction(ActionTerm):
    cfg: "BodyAccelerationActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation | RigidObject"

    @property
    def action_dim(self) -> int:
        return 6

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = self.raw_actions * self.cfg.scale

    def apply_actions(self):
        applied_velocities = (
            self._asset._data.body_vel_w[:, 0].squeeze(1) + self.processed_actions
        )
        if self.cfg.relative:
            applied_velocities += self._asset._data.body_acc_w[:, 0].squeeze(1)

        self._asset.write_root_velocity_to_sim(applied_velocities)


@configclass
class BodyAccelerationActionCfg(ActionTermCfg):
    class_type: Type = BodyAccelerationAction

    relative: bool = False
    scale: float = 1.0
