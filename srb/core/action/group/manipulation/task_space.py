from dataclasses import MISSING

import torch

from srb.core.action import (
    DifferentialInverseKinematicsActionCfg,
    OperationalSpaceControllerActionCfg,
)
from srb.core.action.action_group import ActionGroup
from srb.core.action.group.manipulation._task_space_spec import (
    OSC_DELTA_TWIST_LAYOUT,
    OSC_DELTA_TWIST_TERM,
    POSE_DELTA_LAYOUT,
    POSE_DELTA_UNITS,
)
from srb.core.command_mapping import (
    AppendOnes,
    CommandDestination,
    CommandMapping,
    Concat,
    Identity,
    SliceFirst,
    apply_command_mapping,
)
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass


@spec(
    kind="action",
    schema_version=2,
    layout=list(POSE_DELTA_LAYOUT),
    position_only_layout=list(POSE_DELTA_LAYOUT[:3]),
    scale="DifferentialInverseKinematicsActionCfg.scale",
    fields=[
        # One runtime term. (1,) is the placeholder the builder resolves to the
        # action manager's width: 6 for a relative pose, 3 for position only.
        TermSpec(
            name="delta_twist",
            shape=(1,),
            dtype="float32",
            units=POSE_DELTA_UNITS,
            frame="root",
            semantic_label="end_effector.delta_pose",
        ),
    ],
)
@configclass
class InverseKinematicsActionGroup(ActionGroup):
    delta_twist: DifferentialInverseKinematicsActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        if (
            self.delta_twist.controller.command_type == "pose"
            and not self.delta_twist.controller.use_relative_mode
        ):
            raise ValueError("Only relative mode is supported")
        mapping = self.get_command_mapping()
        if mapping is None:
            raise NotImplementedError()
        return apply_command_mapping(mapping, twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        if self.delta_twist.controller.command_type == "pose":
            if not (self.delta_twist.controller.use_relative_mode):
                return None
            operation = Identity()
        else:
            operation = SliceFirst(count=3)
        return self._declared_command_mapping(
            CommandMapping(
                operation=operation,
                destinations=(
                    CommandDestination(
                        name="delta_twist",
                        start=0,
                        stop=6 if isinstance(operation, Identity) else 3,
                    ),
                ),
            )
        )


@spec(
    kind="action",
    schema_version=2,
    layout=list(OSC_DELTA_TWIST_LAYOUT),
    fields=[OSC_DELTA_TWIST_TERM],
)
@configclass
class OperationalSpaceControlActionGroup(ActionGroup):
    delta_twist: OperationalSpaceControllerActionCfg = MISSING  # type: ignore[assignment]

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
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
