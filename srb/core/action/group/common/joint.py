from dataclasses import MISSING

import torch

from srb.core.action import (
    BinaryJointPositionActionCfg,
    BinaryJointVelocityActionCfg,
    EMAJointPositionToLimitsActionCfg,
    JointEffortActionCfg,
    JointPositionActionCfg,
    JointPositionToLimitsActionCfg,
    JointVelocityActionCfg,
    RelativeJointPositionActionCfg,
)
from srb.core.action.action_group import ActionGroup
from srb.core.command_mapping import CommandDestination, CommandMapping, EventSign
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass

_BINARY_JOINT_POSITION_COMMAND_MAPPING = CommandMapping(
    operation=EventSign(),
    destinations=(CommandDestination(name="joint_pos", start=0, stop=1),),
)
_BINARY_JOINT_VELOCITY_COMMAND_MAPPING = CommandMapping(
    operation=EventSign(),
    destinations=(CommandDestination(name="joint_vel", start=0, stop=1),),
)


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
class JointPositionActionGroup(ActionGroup):
    joint_pos: JointPositionActionCfg = JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


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
class JointPositionOffsetActionGroup(ActionGroup):
    joint_pos: JointPositionActionCfg = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.2,
        use_default_offset=True,
    )


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
class JointPositionRelativeActionGroup(ActionGroup):
    joint_pos: RelativeJointPositionActionCfg = RelativeJointPositionActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


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
class JointPositionBoundedActionGroup(ActionGroup):
    joint_pos: JointPositionToLimitsActionCfg = JointPositionToLimitsActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


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
class JointPositionBoundedEMAActionGroup(ActionGroup):
    joint_pos: EMAJointPositionToLimitsActionCfg = EMAJointPositionToLimitsActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


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
class JointPositionBinaryActionGroup(ActionGroup):
    joint_pos: BinaryJointPositionActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_BINARY_JOINT_POSITION_COMMAND_MAPPING)


@spec(
    kind="action",
    fields=[
        TermSpec(
            name="joint_vel",
            shape=(1,),
            dtype="float32",
            units="rad/s",
            semantic_label="articulation.joint_targets",
        ),
    ],
)
@configclass
class JointVelocityActionGroup(ActionGroup):
    joint_vel: JointVelocityActionCfg = JointVelocityActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


@spec(
    kind="action",
    fields=[
        TermSpec(
            name="joint_vel",
            shape=(1,),
            dtype="float32",
            units="rad/s",
            semantic_label="articulation.joint_targets",
        ),
    ],
)
@configclass
class JointVelocityBinaryActionGroup(ActionGroup):
    joint_vel: BinaryJointVelocityActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_BINARY_JOINT_VELOCITY_COMMAND_MAPPING)


@spec(
    kind="action",
    fields=[
        TermSpec(
            name="joint_eff",
            shape=(1,),
            dtype="float32",
            units="Nm",
            semantic_label="articulation.joint_targets",
        ),
    ],
)
@configclass
class JointEffortActionGroup(ActionGroup):
    joint_eff: JointEffortActionCfg = JointEffortActionCfg(
        asset_name="robot", joint_names=[".*"]
    )
