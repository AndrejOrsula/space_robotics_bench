from srb.core.action import (
    ActionGroup,
    BinaryJointPositionActionCfg,
    JointPositionBinaryActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ActiveTool, ArticulationCfg, Frame, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class SOArm100Gripper(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/so_arm100_gripper",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("gripper")
            .joinpath("so_arm100_5dof_gripper.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
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
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={"Gripper": 0.0},
        ),
        actuators={
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["Gripper"],
                velocity_limit=1.5,
                effort_limit=2.0,
                stiffness=10.0,
                damping=1.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = JointPositionBinaryActionGroup(
        BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["Gripper"],
            close_command_expr={"Gripper": 0.0},
            open_command_expr={"Gripper": deg_to_rad(60.0)},
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="Fixed_Gripper")
    frame_tool_centre_point: Frame = Frame(
        prim_relpath="Fixed_Gripper",
        offset=Transform(pos=(0.0, 0.0, 0.095)),
    )
