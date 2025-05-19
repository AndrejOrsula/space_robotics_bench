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
from srb.utils.cfg import configclass


@configclass
class JointPositionActionGroup(ActionGroup):
    joint_pos: JointPositionActionCfg = JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


@configclass
class JointPositionRelativeActionGroup(ActionGroup):
    joint_pos: RelativeJointPositionActionCfg = RelativeJointPositionActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


@configclass
class JointPositionBoundedActionGroup(ActionGroup):
    joint_pos: JointPositionToLimitsActionCfg = JointPositionToLimitsActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


@configclass
class JointPositionBoundedEMAActionGroup(ActionGroup):
    joint_pos: EMAJointPositionToLimitsActionCfg = EMAJointPositionToLimitsActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


@configclass
class JointPositionBinaryActionGroup(ActionGroup):
    joint_pos: BinaryJointPositionActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return torch.Tensor((-1.0 if event else 1.0,)).to(device=twist.device)


@configclass
class JointVelocityActionGroup(ActionGroup):
    joint_vel: JointVelocityActionCfg = JointVelocityActionCfg(
        asset_name="robot", joint_names=[".*"]
    )


@configclass
class JointVelocityBinaryActionGroup(ActionGroup):
    joint_vel: BinaryJointVelocityActionCfg = MISSING  # type: ignore

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return torch.Tensor((-1.0 if event else 1.0,)).to(device=twist.device)


@configclass
class JointEffortActionGroup(ActionGroup):
    joint_eff: JointEffortActionCfg = JointEffortActionCfg(
        asset_name="robot", joint_names=[".*"]
    )
