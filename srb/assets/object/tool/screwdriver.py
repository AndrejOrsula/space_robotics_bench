from math import pi
from typing import ClassVar

from srb.core.action import (
    ActionGroup,
    BinaryJointVelocityActionCfg,
    JointVelocityBinaryActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import (
    ActiveTool,
    ArticulationCfg,
    Frame,
    RigidObjectCfg,
    Tool,
    Transform,
)
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT, SRB_ASSETS_DIR_SRB_ROBOT


class ManualScrewdriverM3(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/screwdriver",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("screwdriver_hex_m3.usdz").as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1500.0),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="screwdriver")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.075)))


class ManualScrewdriverM5(Tool):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/screwdriver",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("screwdriver_hex_m5.usdz").as_posix()
            ),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1500.0),
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="screwdriver")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.075)))


class ElectricScrewdriverM3(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/screwdriver",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("screwdriver")
            .joinpath("electric_screwdriver_hex_m3.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        actuators={
            "driver": ImplicitActuatorCfg(
                joint_names_expr=["driver_joint", "counterweight_joint"],
                velocity_limit_sim=4.0 * pi,
                effort_limit_sim=100.0,
                stiffness=0.0,
                damping=5000.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_vel_bin": JointVelocityBinaryActionGroup(
            BinaryJointVelocityActionCfg(
                asset_name="screwdriver",
                joint_names=["driver_joint", "counterweight_joint"],
                open_command_expr={"driver_joint": 0.0, "counterweight_joint": 0.0},
                close_command_expr={
                    "driver_joint": 4.0 * pi,
                    "counterweight_joint": -4.0 * pi,
                },
            )
        ),
    }

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="body")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.075)))


class ElectricScrewdriverM5(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/screwdriver",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("screwdriver")
            .joinpath("electric_screwdriver_hex_m5.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        actuators={
            "driver": ImplicitActuatorCfg(
                joint_names_expr=["driver_joint", "counterweight_joint"],
                velocity_limit_sim=4.0 * pi,
                effort_limit_sim=100.0,
                stiffness=0.0,
                damping=5000.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_vel_bin": JointVelocityBinaryActionGroup(
            BinaryJointVelocityActionCfg(
                asset_name="screwdriver",
                joint_names=["driver_joint", "counterweight_joint"],
                open_command_expr={"driver_joint": 0.0, "counterweight_joint": 0.0},
                close_command_expr={
                    "driver_joint": 4.0 * pi,
                    "counterweight_joint": -4.0 * pi,
                },
            )
        ),
    }

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="body")
    frame_tool_centre_point: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.075)))
