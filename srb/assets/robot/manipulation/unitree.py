from srb.core.action import (
    ActionGroup,
    DifferentialIKControllerCfg,
    DifferentialInverseKinematicsActionCfg,
    InverseKinematicsActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, SerialManipulator, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
from srb.utils.nucleus import ISAACLAB_NUCLEUS_DIR


class UnitreeZ1(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/unitree_z1",
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Z1/z1.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "joint1": 0.0,
                "joint2": deg_to_rad(90.0),
                "joint3": deg_to_rad(-45.0),
                "joint4": deg_to_rad(45.0),
                "joint5": deg_to_rad(0.0),
                "joint6": deg_to_rad(0.0),
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                velocity_limit=100.0,
                effort_limit=87.0,
                stiffness=800.0,
                damping=40.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["joint.*"],
            base_name="link00",
            body_name="link06",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.05,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="link00")
    frame_flange: Frame = Frame(
        prim_relpath="link06",
        offset=Transform(pos=(0.05, 0.0, 0.0), rot=rpy_to_quat(0.0, 90.0, 0.0)),
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="link00/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="link06/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )
