"""RL-only action group for filtered joint-position control.

The filtered term lives in ``term.common.smooth_joint``. Policies send joint
deltas directly to it; scripted teleoperation is unsupported. Use
``SmoothOSCActionGroup`` for teleoperation.
"""

from dataclasses import MISSING

import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term.common.smooth_joint import FilteredJointPositionActionCfg
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass


@spec(
    kind="action",
    fields=[
        TermSpec(
            name="joint_pos",
            shape=(1,),
            dtype="float32",
            units="rad",
            semantic_label="articulation.joint_targets",
        ),
    ],
)
@configclass
class FilteredJointPositionActionGroup(ActionGroup):
    """Expose the filtered joint-position term through the action manager."""

    joint_pos: FilteredJointPositionActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        raise NotImplementedError(
            "FilteredJointPositionActionGroup does not support teleop-style "
            "map_cmd_to_action from a task-space twist. Use SmoothOSCActionGroup "
            "for teleoperation. This group is RL-only; the policy outputs joint "
            "deltas directly to process_actions."
        )
