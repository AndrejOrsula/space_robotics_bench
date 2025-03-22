from dataclasses import MISSING

import torch

from srb.core.action import (
    DifferentialInverseKinematicsActionCfg,
    OperationalSpaceControllerActionCfg,
)
from srb.core.action.action_group import ActionGroup
from srb.utils.cfg import configclass


@configclass
class InverseKinematicsActionGroup(ActionGroup):
    delta_twist: DifferentialInverseKinematicsActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        if self.delta_twist.controller.command_type == "pose":
            assert self.delta_twist.controller.use_relative_mode
            return twist
        else:
            return twist[:3]


@configclass
class OperationalSpaceControlActionGroup(ActionGroup):
    delta_twist: OperationalSpaceControllerActionCfg = MISSING  # type: ignore
