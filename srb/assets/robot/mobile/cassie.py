from srb.core.action import (
    ActionGroup,
    JointPositionActionCfg,
    JointPositionActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, LeggedRobot, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.nucleus import ISAACLAB_NUCLEUS_DIR


class Cassie(LeggedRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/cassie",
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Agility/Cassie/cassie.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.9),
            joint_pos={
                "hip_abduction_left": 0.1,
                "hip_rotation_left": 0.0,
                "hip_flexion_left": 1.0,
                "thigh_joint_left": -1.8,
                "ankle_joint_left": 1.57,
                "toe_joint_left": -1.57,
                "hip_abduction_right": -0.1,
                "hip_rotation_right": 0.0,
                "hip_flexion_right": 1.0,
                "thigh_joint_right": -1.8,
                "ankle_joint_right": 1.57,
                "toe_joint_right": -1.57,
            },
            joint_vel={".*": 0.0},
        ),
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=["hip_.*", "thigh_.*", "ankle_.*"],
                effort_limit=200.0,
                velocity_limit=10.0,
                stiffness={
                    "hip_abduction.*": 100.0,
                    "hip_rotation.*": 100.0,
                    "hip_flexion.*": 200.0,
                    "thigh_joint.*": 200.0,
                    "ankle_joint.*": 200.0,
                },
                damping={
                    "hip_abduction.*": 3.0,
                    "hip_rotation.*": 3.0,
                    "hip_flexion.*": 6.0,
                    "thigh_joint.*": 6.0,
                    "ankle_joint.*": 6.0,
                },
            ),
            "toes": ImplicitActuatorCfg(
                joint_names_expr=["toe_.*"],
                effort_limit=20.0,
                velocity_limit=10.0,
                stiffness={
                    "toe_joint.*": 20.0,
                },
                damping={
                    "toe_joint.*": 1.0,
                },
            ),
        },
    )

    ## Actions
    actions: ActionGroup = JointPositionActionGroup(
        JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5)
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="pelvis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="pelvis",
        offset=Transform(
            pos=(-0.125, 0.0, 0.1),
            rot=rpy_to_quat(0.0, -30.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="pelvis",
        offset=Transform(
            pos=(0.1, 0.0, 0.125),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="pelvis/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )

    ## Links
    regex_feet_links: str = "(left|right)_toe"
