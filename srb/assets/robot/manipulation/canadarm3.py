from typing import ClassVar

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
from srb.core.asset import ArticulationCfg, Frame, SerialManipulator, Transform
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


class Canadarm3(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/canadarm3_large",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("manipulator")
            .joinpath("canadarm3_large.usdz")
            .as_posix(),
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
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "canadarm3_large_joint_1": radians(50.0),
                "canadarm3_large_joint_2": 0.0,
                "canadarm3_large_joint_3": radians(55.0),
                "canadarm3_large_joint_4": radians(75.0),
                "canadarm3_large_joint_5": radians(-30.0),
                "canadarm3_large_joint_6": 0.0,
                "canadarm3_large_joint_7": 0.0,
            },
        ),
        actuators={
            "joints": ImplicitActuatorCfg(
                joint_names_expr=["canadarm3_large_joint_[1-7]"],
                effort_limit_sim=2500.0,
                velocity_limit_sim=5.0,
                stiffness=40000.0,
                damping=25000.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "diff_ik": InverseKinematicsActionGroup(
            DifferentialInverseKinematicsActionCfg(
                asset_name="robot",
                joint_names=["canadarm3_large_joint_[1-7]"],
                base_name="canadarm3_large_0",
                body_name="canadarm3_large_7",
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    use_relative_mode=True,
                    ik_method="dls",
                ),
                scale=0.1,
                # The environment binds this to `robot_ee` (flange plus tool
                # TCP). The former identity-rotated (0, 0, -0.45) m offset was
                # 10 mm off-frame and put the rotation pivot 0.44 m from TCP.
            ),
        ),
        "joint_pos_rel": JointPositionRelativeActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="canadarm3_large_0")
    frame_flange: Frame = Frame(
        prim_relpath="canadarm3_large_7",
        offset=Transform(
            pos=(0.0, 0.0, -0.44),
            rot=rpy_to_quat(0.0, 180.0, 0.0),
        ),
    )

    ## Sensors
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "contacts_robot": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/canadarm3_large/.*",
        ),
        "cam_base": CameraCfg(
            prim_path="{ENV_REGEX_NS}/canadarm3_large/canadarm3_large_0/camera_base",
            offset=CameraCfg.OffsetCfg(
                pos=(0.06, 0.0, 0.15),
                rot=rpy_to_quat(0.0, -10.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
        "cam_wrist": CameraCfg(
            prim_path="{ENV_REGEX_NS}/canadarm3_large/canadarm3_large_7/camera_wrist",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.0, -0.45),
                rot=rpy_to_quat(0.0, 90.0, 180.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }
