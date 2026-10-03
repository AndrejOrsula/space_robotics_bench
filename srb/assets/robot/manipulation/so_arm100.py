from typing import ClassVar

from srb.assets.object.tool import SOArm100Gripper
from srb.core.action import (  # noqa: F401
    ActionGroup,
    DifferentialIKControllerCfg,
    DifferentialInverseKinematicsActionCfg,
    InverseKinematicsActionGroup,
    JointPositionRelativeActionGroup,
    OperationalSpaceControlActionGroup,
    OperationalSpaceControllerActionCfg,
    OperationalSpaceControllerCfg,
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
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import radians, rpy_to_quat
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
                enabled_self_collisions=False,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "Shoulder_Rotation": 0.0,
                "Shoulder_Pitch": radians(-15.0),
                "Elbow": radians(20.0),
                "Wrist_Pitch": radians(5.0),
                "Wrist_Roll": 0.0,
            },
            pos=(0.0, 0.0, 0.015),
            rot=rpy_to_quat(0.0, 0.0, 90.0),
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                velocity_limit_sim=10.0,
                effort_limit_sim=10.0,
                stiffness=100.0,
                damping=20.0,
            ),
        },
    )
    end_effector: Tool | None = SOArm100Gripper()

    ## Action modes — coded-in alternatives selectable via
    ## `env.robot.action_mode=<key>`. The first entry is the default.
    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "diff_ik": InverseKinematicsActionGroup(
            DifferentialInverseKinematicsActionCfg(
                asset_name="robot",
                joint_names=[".*"],
                base_name="Base",
                body_name="Fixed_Gripper",
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    use_relative_mode=True,
                    ik_method="dls",
                ),
                scale=0.05,
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
            ),
        ),
        "osc_var_stiffness": OperationalSpaceControlActionGroup(
            OperationalSpaceControllerActionCfg(
                asset_name="robot",
                joint_names=[".*"],
                body_name="Fixed_Gripper",
                controller_cfg=OperationalSpaceControllerCfg(
                    target_types=["pose_rel"],
                    impedance_mode="variable_kp",
                    inertial_dynamics_decoupling=True,
                    motion_stiffness_limits_task=(10.0, 250.0),
                    motion_damping_ratio_task=1.0,
                ),
                position_scale=0.05,
                orientation_scale=0.05,
                stiffness_scale=120.0,
                body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
            )
        ),
        "joint_pos_rel": JointPositionRelativeActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="Base")
    frame_flange: Frame = Frame(
        prim_relpath="Fixed_Gripper",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    ## Sensors
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "contacts_robot": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/so_arm100/.*",
        ),
        "cam_base": CameraCfg(
            prim_path="{ENV_REGEX_NS}/so_arm100/Base/camera_base",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                rot=rpy_to_quat(0.0, 0.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
        "cam_wrist": CameraCfg(
            prim_path="{ENV_REGEX_NS}/so_arm100/Fixed_Gripper/camera_wrist",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                rot=rpy_to_quat(0.0, 0.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }


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
                enabled_self_collisions=False,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "Shoulder_Pitch": radians(120.0),
                "Shoulder_Yaw": 0.0,
                "Humeral_Rotation": 0.0,
                "Elbow": radians(-90.0),
                "Wrist_Roll": 0.0,
                "Wrist_Yaw": 0.0,
                "Wrist_Pitch": radians(-90.0),
            },
            rot=rpy_to_quat(180.0, 0.0, 0.0),
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                velocity_limit_sim=10.0,
                effort_limit_sim=10.0,
                stiffness=100.0,
                damping=20.0,
            ),
        },
    )

    ## Action modes — coded-in alternatives selectable via
    ## `env.robot.action_mode=<key>`. The first entry is the default.
    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "diff_ik": InverseKinematicsActionGroup(
            DifferentialInverseKinematicsActionCfg(
                asset_name="robot",
                joint_names=[".*"],
                base_name="Base",
                body_name="End_Servo",
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    use_relative_mode=True,
                    ik_method="dls",
                ),
                scale=0.05,
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(),
            ),
        ),
        "joint_pos_rel": JointPositionRelativeActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="Base")
    frame_flange: Frame = Frame(
        prim_relpath="End_Servo",
        offset=Transform(
            pos=(0.0, 0.0, -0.05),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    ## Sensors
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "contacts_robot": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/so_arm100/.*",
        ),
        "cam_base": CameraCfg(
            prim_path="{ENV_REGEX_NS}/so_arm100/Base/camera_base",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                rot=rpy_to_quat(0.0, 0.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
        "cam_wrist": CameraCfg(
            prim_path="{ENV_REGEX_NS}/so_arm100/End_Servo/camera_wrist",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                rot=rpy_to_quat(0.0, 0.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }
