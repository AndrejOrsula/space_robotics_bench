from typing import ClassVar

import torch

from srb.core.action import (
    ActionGroup,
    JointPositionToLimitsActionCfg,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import (
    ArticulationCfg,
    Frame,
    MultiManipulator,
)
from srb.core.command_mapping import CommandDestination, CommandMapping, LevionTail2
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.cfg import configclass
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT

_LEVION_ARMS_COMMAND_MAPPING = CommandMapping(
    operation=LevionTail2(),
    destinations=(CommandDestination(name="joint_positions", start=0, stop=4),),
)


@configclass
class LevionArmsActionGroup(ActionGroup):
    joint_positions: JointPositionToLimitsActionCfg = JointPositionToLimitsActionCfg(
        asset_name="robot",
        joint_names=[
            "left_shoulder_joint",
            "left_elbow_joint",
            "right_shoulder_joint",
            "right_elbow_joint",
        ],
        preserve_order=True,
    )

    def map_cmd_to_action(self, twist: torch.Tensor, event: bool) -> torch.Tensor:
        return self._apply_declared_command_mapping(twist, event)

    def get_command_mapping(self) -> CommandMapping | None:
        return self._declared_command_mapping(_LEVION_ARMS_COMMAND_MAPPING)


class LevionArms(MultiManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/levion_arms",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("manipulator")
            .joinpath("levion_arms.usdc")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                rigid_body_enabled=True,
                max_depenetration_velocity=5.0,
                enable_gyroscopic_forces=True,
                # Cap free-floating base velocity to prevent extreme joint commands
                # from corrupting the GPU articulation view. Add no damping in zero-g.
                max_linear_velocity=100.0,
                max_angular_velocity=100.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                # Disable arm self-collision: interpenetration under extreme joint
                # commands creates explosive PhysX forces and corrupts the GPU view.
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                ".*shoulder_joint": 0.0,
                "right_elbow_joint": 0.4,
                "left_elbow_joint": -0.4,
            },
        ),
        actuators={
            "left_arm": ImplicitActuatorCfg(
                joint_names_expr=["left_shoulder_joint", "left_elbow_joint"],
                effort_limit_sim=15.0,
                stiffness=400.0,
                damping=80.0,
            ),
            "right_arm": ImplicitActuatorCfg(
                joint_names_expr=["right_shoulder_joint", "right_elbow_joint"],
                effort_limit_sim=15.0,
                stiffness=400.0,
                damping=80.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "dual_arm_joint_pos_abs": LevionArmsActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
