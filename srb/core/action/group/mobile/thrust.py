import torch

from srb.core.action.action_group import ActionGroup
from srb.core.action.term import ThrustActionCfg
from srb.core.command_mapping import CommandDestination, CommandMapping, Thrust
from srb.core.spec import TermSpec, spec
from srb.utils.cfg import configclass


@spec(
    kind="action",
    fields=[
        TermSpec(
            name="thrust",
            shape=(1,),  # per-thruster placeholder; builder resolves the real dim
            dtype="float32",
            units="normalised",
            semantic_label="body.thruster_throttles",
        ),
    ],
)
@configclass
class ThrustActionGroup(ActionGroup):
    thrust: ThrustActionCfg = ThrustActionCfg(asset_name="robot")

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        """Map a Cartesian teleop command onto directional thruster firings."""
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        directions = tuple(
            tuple(thruster.direction) for thruster in self.thrust.thrusters
        )
        gimbal = tuple(
            thruster.gimbal_limits not in (None, (0.0, 0.0))
            and any(component != 0 for component in thruster.direction)
            for thruster in self.thrust.thrusters
        )
        width = len(directions) + 2 * sum(gimbal)
        # Torque of each thruster's unit force about the body origin, scaled so
        # the largest moment arm saturates at a unit rotation command.
        moments: tuple[tuple[float, ...], ...] = ()
        if directions:
            forces = -torch.nn.functional.normalize(
                torch.tensor(directions, dtype=torch.float64), dim=-1
            )
            offsets = torch.tensor(
                tuple(tuple(thruster.offset) for thruster in self.thrust.thrusters),
                dtype=torch.float64,
            )
            arms = torch.linalg.cross(offsets, forces)
            scale = float(torch.linalg.vector_norm(arms, dim=-1).max())
            if scale > 0.0:
                moments = tuple(map(tuple, (arms / scale).tolist()))
        return self._declared_command_mapping(
            CommandMapping(
                operation=Thrust(
                    directions=directions,
                    gimbal=gimbal,
                    moments=moments,
                ),
                destinations=(CommandDestination(name="thrust", start=0, stop=width),),
            )
        )
