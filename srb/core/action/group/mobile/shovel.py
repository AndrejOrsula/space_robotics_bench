"""Action group for a single-DoF linear shovel payload."""

from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term.common.shovel import CoupledShovelVelocityActionCfg
from srb.core.command_mapping import CommandDestination, CommandMapping, OptionalThird
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass

_SHOVEL_VELOCITY_COMMAND_MAPPING = CommandMapping(
    operation=OptionalThird(),
    destinations=(CommandDestination(name="joint_vel", start=0, stop=1),),
)


@spec(
    kind="action",
    fields=[
        TermSpec(
            name="velocity",
            shape=(1,),
            dtype="float32",
            units="m/s",
            semantic_label="shovel.linear_velocity",
        ),
    ],
)
@configclass
class ShovelJointVelocityActionGroup(ActionGroup):
    """One lift-velocity command, with optional measured-state bucket curl.

    Policy actions use the normalised ``[-1, 1]`` input consumed by
    :class:`JointVelocityActionCfg`. Scripted/teleop commands reuse the third
    linear-twist channel, matching the existing blade payload convention.
    """

    joint_vel: CoupledShovelVelocityActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_SHOVEL_VELOCITY_COMMAND_MAPPING)
