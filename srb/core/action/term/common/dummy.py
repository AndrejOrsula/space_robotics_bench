from typing import TYPE_CHECKING, Type

import torch

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation, RigidObject


class DummyAction(ActionTerm):
    cfg: "DummyActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation | RigidObject"

    @property
    def action_dim(self) -> int:
        return self.cfg.dim

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        self._processed_actions = self._raw_actions

    def apply_actions(self):
        pass


@configclass
class DummyActionCfg(ActionTermCfg):
    class_type: Type = DummyAction
    asset_name: str = "robot"
    dim: int = 1
