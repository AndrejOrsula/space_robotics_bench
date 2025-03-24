from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import MulticopterBodyAccelerationActionCfg
from srb.utils.cfg import configclass


@configclass
class MulticopterBodyAccelerationActionGroup(ActionGroup):
    cmd_vel: MulticopterBodyAccelerationActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return torch.concat(
            (
                twist[:3],
                twist[5].unsqueeze(0),
            ),
        )
