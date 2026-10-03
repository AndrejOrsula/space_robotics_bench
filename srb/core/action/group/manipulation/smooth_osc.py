"""Action group for smooth operational-space control and teleoperation."""

from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.group.manipulation._task_space_spec import (
    OSC_DELTA_TWIST_LAYOUT,
    OSC_DELTA_TWIST_TERM,
)
from srb.core.action.term.manipulation.smooth_osc import (
    SmoothOperationalSpaceControllerActionCfg,
)
from srb.core.command_mapping import (
    AppendOnes,
    CommandDestination,
    CommandMapping,
    Concat,
    Identity,
)
from srb.core.spec import spec
from srb.utils.cfg import configclass


@spec(
    kind="action",
    schema_version=2,
    layout=list(OSC_DELTA_TWIST_LAYOUT),
    fields=[OSC_DELTA_TWIST_TERM],
)
@configclass
class SmoothOSCActionGroup(ActionGroup):
    """Expose a filtered OSC term and map twists for supported impedance modes."""

    delta_twist: SmoothOperationalSpaceControllerActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        """Map a 6D twist to the configured OSC vector; require ``pose_rel``."""
        if not (
            len(self.delta_twist.controller_cfg.target_types) == 1
            and "pose_rel" in self.delta_twist.controller_cfg.target_types
        ):
            raise ValueError(
                "Only pose_rel OSC target type is supported for teleoperation"
            )
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        if not (
            len(self.delta_twist.controller_cfg.target_types) == 1
            and "pose_rel" in self.delta_twist.controller_cfg.target_types
        ):
            return None
        match self.delta_twist.controller_cfg.impedance_mode:
            case "fixed":
                operation = Identity()
                width = 6
            case "variable_kp":
                operation = Concat(parts=(Identity(), AppendOnes(count=6)))
                width = 12
            case "variable":
                operation = Concat(parts=(Identity(), AppendOnes(count=12)))
                width = 18
            case _:
                raise ValueError(
                    f"Unknown impedance mode: "
                    f"{self.delta_twist.controller_cfg.impedance_mode!r}; set it to "
                    "'fixed', 'variable_kp', or 'variable'"
                )
        return self._declared_command_mapping(
            CommandMapping(
                operation=operation,
                destinations=(
                    CommandDestination(name="delta_twist", start=0, stop=width),
                ),
            )
        )
