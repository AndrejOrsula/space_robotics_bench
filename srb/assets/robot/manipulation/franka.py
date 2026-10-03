from typing import ClassVar

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
from srb.core.sensor import (
    CameraCfg,
    ContactSensorCfg,
    PinholeCameraCfg,
    SensorBaseCfg,
)
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import radians, rpy_to_quat
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
                enabled_self_collisions=False,
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
                "panda_joint4": radians(-90.0),
                "panda_joint5": 0.0,
                "panda_joint6": radians(90.0),
                "panda_joint7": radians(45.0),
            },
        ),
        ### ANCHOR_END: example_p2 (docs)
        actuators={
            "panda_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[1-4]"],
                effort_limit_sim=87.0,
                velocity_limit_sim=2.175,
                stiffness=4000.0,
                damping=800.0,
            ),
            "panda_forearm": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[5-7]"],
                effort_limit_sim=12.0,
                velocity_limit_sim=2.61,
                stiffness=4000.0,
                damping=800.0,
            ),
        },
        ### ANCHOR: example_p3 (docs)
    )
    ## End effector - The default hand is separate to allow for easy replacement
    end_effector: Tool | None = FrankaHand()

    ## Action modes — coded-in alternatives selectable via
    ## `env.robot.action_mode=<key>`. The first entry is the default.
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "diff_ik": InverseKinematicsActionGroup(
            DifferentialInverseKinematicsActionCfg(
                asset_name="robot",
                joint_names=["panda_joint[1-7]"],
                base_name="panda_link0",
                body_name="panda_link7",
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    use_relative_mode=True,
                    ik_method="dls",
                ),
                scale=0.1,
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
            ),
        ),
        "osc_fixed": OperationalSpaceControlActionGroup(
            OperationalSpaceControllerActionCfg(
                asset_name="robot",
                joint_names=["panda_joint[1-7]"],
                body_name="panda_link7",
                controller_cfg=OperationalSpaceControllerCfg(
                    target_types=["pose_rel"],
                    impedance_mode="fixed",
                    motion_stiffness_task=100.0,
                    motion_damping_ratio_task=1.0,
                    nullspace_control="position",
                    inertial_dynamics_decoupling=True,
                ),
                nullspace_joint_pos_target="center",
                position_scale=0.1,
                orientation_scale=0.1,
                body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
            )
        ),
        "osc_var_stiffness": OperationalSpaceControlActionGroup(
            OperationalSpaceControllerActionCfg(
                asset_name="robot",
                joint_names=["panda_joint[1-7]"],
                body_name="panda_link7",
                controller_cfg=OperationalSpaceControllerCfg(
                    target_types=["pose_rel"],
                    impedance_mode="variable_kp",
                    motion_stiffness_limits_task=(10.0, 250.0),
                    motion_damping_ratio_task=1.0,
                    nullspace_control="position",
                    inertial_dynamics_decoupling=True,
                ),
                nullspace_joint_pos_target="center",
                position_scale=0.1,
                orientation_scale=0.1,
                stiffness_scale=120.0,
                body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
            )
        ),
        "osc_var_full": OperationalSpaceControlActionGroup(
            OperationalSpaceControllerActionCfg(
                asset_name="robot",
                joint_names=["panda_joint[1-7]"],
                body_name="panda_link7",
                controller_cfg=OperationalSpaceControllerCfg(
                    target_types=["pose_rel"],
                    impedance_mode="variable",
                    motion_stiffness_limits_task=(10.0, 250.0),
                    motion_damping_ratio_limits_task=(0.5, 2.5),
                    nullspace_control="position",
                    inertial_dynamics_decoupling=True,
                ),
                nullspace_joint_pos_target="center",
                position_scale=0.1,
                orientation_scale=0.1,
                stiffness_scale=120.0,
                damping_ratio_scale=1.0,
                body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
            )
        ),
        "joint_pos_rel": JointPositionRelativeActionGroup(),
    }
    ### ANCHOR_END: example_p3 (docs)

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
    ### ANCHOR_END: example_p4 (docs)

    ## Sensors
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "contacts_robot": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/franka/.*",
        ),
        "cam_base": CameraCfg(
            prim_path="{ENV_REGEX_NS}/franka/panda_link0/camera_base",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.16, 0.0, 0.1),
                rot=rpy_to_quat(0.0, 55.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
        "cam_wrist": CameraCfg(
            prim_path="{ENV_REGEX_NS}/franka/panda_link7/camera_wrist",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.075, -0.075, 0.1),
                rot=rpy_to_quat(0.0, -80.0, 135.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }
