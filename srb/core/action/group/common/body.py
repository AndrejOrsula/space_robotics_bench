import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import BodyAccelerationActionCfg
from srb.core.command_mapping import CommandDestination, CommandMapping, Identity
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass

_BODY_VELOCITY_COMMAND_MAPPING = CommandMapping(
    operation=Identity(),
    destinations=(CommandDestination(name="body_vel", start=0, stop=6),),
)


#: Channel order of the single runtime ``body_vel`` term.
_BODY_ACCELERATION_AXES = [
    "linear_x",
    "linear_y",
    "linear_z",
    "angular_x",
    "angular_y",
    "angular_z",
]


def _body_acceleration_term(frame: str) -> TermSpec:
    """Schema-v2 descriptor of the runtime ``body_vel`` term (6 channels).

    ``BodyAccelerationAction`` clamps each channel to [-1, 1], multiplies it by
    ``BodyAccelerationActionCfg.scale`` and integrates the result into the root
    velocity as ``acceleration * physics_dt`` on every physics substep, so the
    scaled command is an acceleration. Schema version 1 labelled the two
    halves ``lin``/``ang`` as velocities (m/s, rad/s).
    """
    return TermSpec(
        name="body_vel",
        shape=(6,),
        dtype="float32",
        units="m/s^2,m/s^2,m/s^2,rad/s^2,rad/s^2,rad/s^2",
        frame=frame,
        min=-1.0,
        max=1.0,
        semantic_label="body.acceleration",
    )


@spec(
    kind="action",
    schema_version=2,
    axes=_BODY_ACCELERATION_AXES,
    normalized=True,
    scale="BodyAccelerationActionCfg.scale",
    fields=[_body_acceleration_term("world")],
)
@configclass
class BodyAccelerationActionGroup(ActionGroup):
    body_vel: BodyAccelerationActionCfg = BodyAccelerationActionCfg(asset_name="robot")

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_BODY_VELOCITY_COMMAND_MAPPING)


@spec(
    kind="action",
    schema_version=2,
    axes=_BODY_ACCELERATION_AXES,
    normalized=True,
    scale="BodyAccelerationActionCfg.scale",
    fields=[_body_acceleration_term("body")],
)
@configclass
class BodyAccelerationRelativeActionGroup(ActionGroup):
    body_vel: BodyAccelerationActionCfg = BodyAccelerationActionCfg(
        asset_name="robot", relative=True
    )

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_BODY_VELOCITY_COMMAND_MAPPING)
