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
from srb.core.asset import ArticulationCfg, Frame, SerialManipulator
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


class Vispa(SerialManipulator):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/vispa",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("manipulator")
            .joinpath("vispa.usdz")
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
                "joint1": radians(0.0),
                "joint2": radians(-30.0),
                "joint3": radians(120.0),
                "joint4": radians(0.0),
                "joint5": radians(90.0),
                "joint6": radians(0.0),
            },
        ),
        actuators={
            "joints": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-6]"],
                effort_limit_sim=500.0,
                velocity_limit_sim=5.0,
                stiffness=2500.0,
                damping=500.0,
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
                joint_names=["joint[1-6]"],
                base_name="link0",
                body_name="link6",
                controller=DifferentialIKControllerCfg(
                    command_type="pose",
                    use_relative_mode=True,
                    ik_method="dls",
                ),
                scale=0.05,
            ),
        ),
        "osc_var_full": OperationalSpaceControlActionGroup(
            OperationalSpaceControllerActionCfg(
                asset_name="robot",
                joint_names=["joint[1-6]"],
                body_name="link6",
                controller_cfg=OperationalSpaceControllerCfg(
                    target_types=["pose_rel"],
                    impedance_mode="variable",
                    motion_stiffness_limits_task=(10.0, 250.0),
                    motion_damping_ratio_limits_task=(0.5, 2.5),
                    nullspace_control="none",
                    inertial_dynamics_decoupling=True,
                ),
                nullspace_joint_pos_target="none",
                position_scale=0.1,
                orientation_scale=0.1,
                stiffness_scale=120.0,
                damping_ratio_scale=1.0,
                body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
            )
        ),
        "joint_pos_rel": JointPositionRelativeActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="link0")
    frame_flange: Frame = Frame(prim_relpath="link6")

    ## Sensors
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "contacts_robot": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/vispa/.*",
        ),
        "cam_base": CameraCfg(
            prim_path="{ENV_REGEX_NS}/vispa/link0/camera_base",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.15, 0.0, 0.1),
                rot=rpy_to_quat(0.0, 45.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
        "cam_wrist": CameraCfg(
            prim_path="{ENV_REGEX_NS}/vispa/link6/camera_wrist",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.1, 0.0, 0.0),
                rot=rpy_to_quat(0.0, -80.0, 180.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }
