from srb.assets.object.tool import FrankaHand
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
from srb.core.asset import ArticulationCfg, Frame, SerialManipulator, Tool, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


### ANCHOR: example_p1 (docs)
class Franka(SerialManipulator):
    ## Model - Articulation with several links connected by joints
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/franka",
        ## Spawner loads a static USD file
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("manipulator")
            .joinpath("franka_arm.usdz")
            .as_posix(),
            ### ANCHOR_END: example_p1 (docs)
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
            ### ANCHOR: example_p2 (docs)
        ),
        ## Initial joint configuration of the robot
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "panda_joint1": 0.0,
                "panda_joint2": 0.0,
                "panda_joint3": 0.0,
                "panda_joint4": deg_to_rad(-90.0),
                "panda_joint5": 0.0,
                "panda_joint6": deg_to_rad(90.0),
                "panda_joint7": deg_to_rad(45.0),
            },
        ),
        ### ANCHOR_END: example_p2 (docs)
        actuators={
            "panda_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[1-4]"],
                effort_limit=87.0,
                velocity_limit=2.175,
                stiffness=4000.0,
                damping=800.0,
            ),
            "panda_forearm": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[5-7]"],
                effort_limit=12.0,
                velocity_limit=2.61,
                stiffness=4000.0,
                damping=800.0,
            ),
        },
        ### ANCHOR: example_p3 (docs)
    )
    ## End effector - The default hand is separate to allow for easy replacement
    end_effector: Tool | None = FrankaHand()

    ## Actions - Inverse Kinematics action group that drives all joints
    actions: ActionGroup = InverseKinematicsActionGroup(
        DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint[1-7]"],
            base_name="panda_link0",
            body_name="panda_link7",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
            ),
            scale=0.1,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
        ),
    )
    ### ANCHOR_END: example_p3 (docs)
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=["panda_joint[1-7]"],
    #         body_name="panda_link7",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="fixed",
    #             motion_stiffness_task=100.0,
    #             motion_damping_ratio_task=1.0,
    #             # motion_stiffness_task=250.0,
    #             # motion_damping_ratio_task=1.5,
    #             nullspace_control="position",
    #         ),
    #         nullspace_joint_pos_target="center",
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=["panda_joint[1-7]"],
    #         body_name="panda_link7",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable_kp",
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_task=1.0,
    #             # motion_damping_ratio_task=1.5,
    #             nullspace_control="position",
    #         ),
    #         nullspace_joint_pos_target="center",
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )
    # actions: ActionGroup = OperationalSpaceControlActionGroup(
    #     OperationalSpaceControllerActionCfg(
    #         asset_name="robot",
    #         joint_names=["panda_joint[1-7]"],
    #         body_name="panda_link7",
    #         controller_cfg=OperationalSpaceControllerCfg(
    #             target_types=["pose_rel"],
    #             impedance_mode="variable",
    #             motion_stiffness_limits_task=(10.0, 250.0),
    #             motion_damping_ratio_limits_task=(0.5, 2.5),
    #             nullspace_control="position",
    #         ),
    #         nullspace_joint_pos_target="center",
    #         position_scale=0.1,
    #         orientation_scale=0.1,
    #         stiffness_scale=120.0,
    #         damping_ratio_scale=1.0,
    #         body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
    #     )
    # )

    ### ANCHOR: example_p4 (docs)
    ## Frames - Relevant frames for attaching the robot and mounting tool/sensors
    frame_base: Frame = Frame(prim_relpath="panda_link0")
    frame_flange: Frame = Frame(
        prim_relpath="panda_link7",
        offset=Transform(
            pos=(0.0, 0.0, 0.107),
            rot=rpy_to_quat(0.0, 0.0, -45.0),
        ),
    )
    frame_wrist_camera: Frame = Frame(
        prim_relpath="panda_link7/camera_wrist",
        offset=Transform(
            pos=(0.075, -0.075, 0.1),
            rot=rpy_to_quat(0.0, -80.0, 135.0),
        ),
    )
    frame_base_camera: Frame = Frame(
        prim_relpath="panda_link0/camera_base",
        offset=Transform(
            pos=(0.15, 0.0, 0.1),
            rot=rpy_to_quat(0.0, 45.0, 0.0),
        ),
    )
    ### ANCHOR_END: example_p4 (docs)
