"""Action group for filtered wheeled-drive control."""

from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term.mobile.smooth_wheeled import FilteredWheeledDriveActionCfg
from srb.core.command_mapping import CommandDestination, CommandMapping, Select
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass

_FILTERED_WHEELED_DRIVE_COMMAND_MAPPING = CommandMapping(
    operation=Select(indices=(0, 5)),
    destinations=(CommandDestination(name="cmd_vel", start=0, stop=2),),
)


@spec(
    kind="action",
    fields=[
        TermSpec(
            name="lin",
            shape=(1,),
            dtype="float32",
            units="m/s",
            semantic_label="base.linear_velocity_x",
            min=-2.0,
            max=2.0,
        ),
        TermSpec(
            name="ang",
            shape=(1,),
            dtype="float32",
            units="rad/s",
            semantic_label="base.angular_velocity_z",
            min=-1.0,
            max=1.0,
        ),
    ],
)
@configclass
class FilteredWheeledDriveActionGroup(ActionGroup):
    """Expose the filtered drive term and map twists to forward and yaw rates."""

    cmd_vel: FilteredWheeledDriveActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        # twist is the 6-DoF SE(3) command (vx, vy, vz, wx, wy, wz). The term
        # consumes [v_lin, v_ang] = [vx, wz], so take index 0 and index 5 (yaw
        # rate) — not twist[:2], which would route lateral vy into the yaw slot.
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_FILTERED_WHEELED_DRIVE_COMMAND_MAPPING)
