from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import MulticopterBodyAccelerationActionCfg
from srb.core.command_mapping import CommandDestination, CommandMapping, Select
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass

_MULTICOPTER_COMMAND_MAPPING = CommandMapping(
    operation=Select(indices=(0, 1, 2, 5)),
    destinations=(CommandDestination(name="cmd_vel", start=0, stop=4),),
)


@spec(
    kind="action",
    schema_version=2,
    axes=["linear_x", "linear_y", "linear_z", "angular_z"],
    normalized=True,
    scale="MulticopterBodyAccelerationActionCfg.scale",
    fields=[
        TermSpec(
            name="cmd_vel",
            shape=(4,),
            dtype="float32",
            units="m/s,m/s,m/s,rad/s",
            frame="body",
            min=-1.0,
            max=1.0,
            semantic_label="multicopter.body_velocity",
        ),
    ],
)
@configclass
class MulticopterBodyAccelerationActionGroup(ActionGroup):
    cmd_vel: MulticopterBodyAccelerationActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_MULTICOPTER_COMMAND_MAPPING)
