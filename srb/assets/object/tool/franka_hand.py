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
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class FrankaHand(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/franka_hand",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("gripper")
            .joinpath("franka_hand.usdz")
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
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={"panda_finger_joint.*": 0.04},
        ),
        actuators={
            "panda_hand": ImplicitActuatorCfg(
                joint_names_expr=["panda_finger_joint.*"],
                effort_limit=200.0,
                velocity_limit=0.2,
                stiffness=2000.0,
                damping=250.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = JointPositionBinaryActionGroup(
        BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["panda_finger_joint.*"],
            close_command_expr={"panda_finger_joint.*": 0.0},
            open_command_expr={"panda_finger_joint.*": 0.04},
        ),
    )

    ## Frames
    frame_mount: Frame = Frame(prim_relpath="panda_hand")
    frame_tool_centre_point: Frame = Frame(
        prim_relpath="panda_hand", offset=Transform(pos=(0.0, 0.0, 0.1034))
    )
