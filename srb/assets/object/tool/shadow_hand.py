from srb.core.action import ActionGroup, JointPositionActionGroup
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ActiveTool, ArticulationCfg, Frame, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    FixedTendonPropertiesCfg,
    JointDrivePropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.nucleus import ISAAC_NUCLEUS_DIR


class ShadowHand(ActiveTool):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/shadow_hand",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/ShadowHand/shadow_hand_instanceable.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=True,
                max_depenetration_velocity=1000.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
            joint_drive_props=JointDrivePropertiesCfg(drive_type="force"),
            fixed_tendons_props=FixedTendonPropertiesCfg(
                limit_stiffness=30.0, damping=0.1
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.5),
            rot=(0.0, 0.0, -0.7071, 0.7071),
            joint_pos={".*": 0.0},
        ),
        actuators={
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=[
                    "robot0_WR.*",
                    "robot0_(FF|MF|RF|LF|TH)J(3|2|1)",
                    "robot0_(LF|TH)J4",
                    "robot0_THJ0",
                ],
                effort_limit={
                    "robot0_WRJ1": 4.785,
                    "robot0_WRJ0": 2.175,
                    "robot0_(FF|MF|RF|LF)J1": 0.7245,
                    "robot0_FFJ(3|2)": 0.9,
                    "robot0_MFJ(3|2)": 0.9,
                    "robot0_RFJ(3|2)": 0.9,
                    "robot0_LFJ(4|3|2)": 0.9,
                    "robot0_THJ4": 2.3722,
                    "robot0_THJ3": 1.45,
                    "robot0_THJ(2|1)": 0.99,
                    "robot0_THJ0": 0.81,
                },
                stiffness={
                    "robot0_WRJ.*": 5.0,
                    "robot0_(FF|MF|RF|LF|TH)J(3|2|1)": 1.0,
                    "robot0_(LF|TH)J4": 1.0,
                    "robot0_THJ0": 1.0,
                },
                damping={
                    "robot0_WRJ.*": 0.5,
                    "robot0_(FF|MF|RF|LF|TH)J(3|2|1)": 0.1,
                    "robot0_(LF|TH)J4": 0.1,
                    "robot0_THJ0": 0.1,
                },
            ),
        },
    )

    ## Actions
    actions: ActionGroup = JointPositionActionGroup()

    ## Frames
    frame_mount: Frame = Frame(
        prim_relpath="robot0_forearm", offset=Transform(pos=(0.0, 0.0, -0.04))
    )
    frame_tool_centre_point: Frame = Frame(
        prim_relpath="robot0_forearm", offset=Transform(pos=(0.0, -0.025, 0.3))
    )
