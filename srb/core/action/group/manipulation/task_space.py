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
            assert self.delta_twist.controller.use_relative_mode, (
                "Only relative mode is supported"
            )
            return twist
        else:
            return twist[:3]


@configclass
class OperationalSpaceControlActionGroup(ActionGroup):
    delta_twist: OperationalSpaceControllerActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        assert len(self.delta_twist.controller_cfg.target_types) == 1, (
            "Only one target type is supported"
        )
        assert "pose_rel" in self.delta_twist.controller_cfg.target_types, (
            "Only pose_rel is supported"
        )
        assert self.delta_twist.controller_cfg.impedance_mode == "fixed", (
            "Only fixed impedance mode is supported"
        )
        return twist
