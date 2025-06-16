from srb.core.action import (  # noqa: F401
    ActionGroup,
    DifferentialIKControllerCfg,
    DifferentialInverseKinematicsActionCfg,
    InverseKinematicsActionGroup,
    JointEffortActionCfg,
    JointEffortActionGroup,
    JointPositionRelativeActionGroup,
    OperationalSpaceControlActionGroup,
    OperationalSpaceControllerActionCfg,
    OperationalSpaceControllerCfg,
    RelativeJointPositionActionCfg,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, SerialManipulator, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
from srb.utils.nucleus import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR


class UR3(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur3",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur3/ur3.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR3e(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur3e",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur3e/ur3e.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR5(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur5",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur5/ur5.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR5e(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur5e",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur5e/ur5e.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="fixed",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_task=100.0,
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_limits_task=(0.5, 2.5),
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         damping_ratio_scale=1.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR10(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur10",
        spawn=UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/UniversalRobots/UR10/ur10_instanceable.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="ee_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="ee_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(
        prim_relpath="ee_link",
        offset=Transform(rot=rpy_to_quat(180.0, -90.0, 0.0)),
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="ee_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR10e(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur10e",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur10e/ur10e.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR16e(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur16e",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur16e/ur16e.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR20(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur20",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur20/ur20.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class UR30(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ur30",
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur30/ur30.usd",
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": deg_to_rad(-90.0),
                "elbow_joint": deg_to_rad(90.0),
                "wrist_1_joint": deg_to_rad(-90.0),
                "wrist_2_joint": deg_to_rad(-90.0),
                "wrist_3_joint": deg_to_rad(-90.0),
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
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )


class RandomURManipulator(SerialManipulator):
    asset_cfg: ArticulationCfg = UR30().asset_cfg.copy()  # type: ignore
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal"
    asset_cfg.spawn = MultiAssetSpawnerCfg(
        random_choice=False,
        assets_cfg=(
            UR3e().asset_cfg.spawn,  # type: ignore
            UR5().asset_cfg.spawn,  # type: ignore
            UR10e().asset_cfg.spawn,  # type: ignore
            UR16e().asset_cfg.spawn,  # type: ignore
            UR20().asset_cfg.spawn,  # type: ignore
            UR30().asset_cfg.spawn,  # type: ignore
        ),
        activate_contact_sensors=True,
    )

    ## Actions
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[".*_joint"],
            base_name="base_link",
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=[".*_joint"],
    #         body_name="wrist_3_link",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             inertial_dynamics_decoupling=True,
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #         ),
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    frame_base_camera: Frame = Frame(
        prim_relpath="base_link/camera_base",
        offset=Transform(
            pos=(0.06, 0.0, 0.15),
            rot=rpy_to_quat(0.0, -10.0, 0.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="wrist_3_link/camera_wrist",
        offset=Transform(
            pos=(0.07, 0.0, 0.05),
            rot=rpy_to_quat(0.0, -60.0, 180.0),
        ),
    )
