from srb.assets.object.tool import SOArm100Gripper
from srb.core.action import (  # noqa: F401
    ActionGroup,
    DifferentialIKControllerCfg,
    DifferentialInverseKinematicsActionCfg,
    InverseKinematicsActionGroup,
    OperationalSpaceControlActionGroup,
    OperationalSpaceControllerActionCfg,
    OperationalSpaceControllerCfg,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, SerialManipulator, Tool, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class SOArm100D5(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/so_arm100",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("manipulator")
            .joinpath("so_arm100_5dof.usdz")
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
                enabled_self_collisions=True,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "Shoulder_Rotation": 0.0,
                "Shoulder_Pitch": deg_to_rad(-15.0),
                "Elbow": deg_to_rad(20.0),
                "Wrist_Pitch": deg_to_rad(5.0),
                "Wrist_Roll": 0.0,
            },
            pos=(0.0, 0.0, 0.015),
            rot=rpy_to_quat(0.0, 0.0, 90.0),
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                velocity_limit=10.0,
                effort_limit=10.0,
                stiffness=100.0,
                damping=20.0,
            ),
        },
    )
    end_effector: Tool | None = SOArm100Gripper()

    ## Actions
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            base_name="Base",
            body_name="Fixed_Gripper",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="pinv",
            ),
            scale=0.05,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*"],
    #         body_name="Fixed_Gripper",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.05,
    #         orientation_scale=0.05,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="Base")
    frame_flange: Frame = Frame(
        prim_relpath="Fixed_Gripper",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="Base/camera_base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="Fixed_Gripper/camera_wrist",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )


class SOArm100D7(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/so_arm100",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("manipulator")
            .joinpath("so_arm100_7dof.usdz")
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
                enabled_self_collisions=True,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "Shoulder_Pitch": deg_to_rad(120.0),
                "Shoulder_Yaw": 0.0,
                "Humeral_Rotation": 0.0,
                "Elbow": deg_to_rad(-90.0),
                "Wrist_Roll": 0.0,
                "Wrist_Yaw": 0.0,
                "Wrist_Pitch": deg_to_rad(-90.0),
            },
            rot=rpy_to_quat(180.0, 0.0, 0.0),
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                velocity_limit=10.0,
                effort_limit=10.0,
                stiffness=100.0,
                damping=20.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[".*"],
            base_name="Base",
            body_name="End_Servo",
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
    frame_base: Frame = Frame(prim_relpath="Base")
    frame_flange: Frame = Frame(
        prim_relpath="End_Servo",
        offset=Transform(
            pos=(0.0, 0.0, -0.05),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="Base/camera_base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="End_Servo/camera_wrist",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
