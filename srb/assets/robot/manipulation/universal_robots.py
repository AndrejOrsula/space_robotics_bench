from typing import ClassVar

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
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import radians, rpy_to_quat
from srb.utils.nucleus import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

## UR arms share tuning, home pose, action modes, and camera layout. Factories
## hold common config; each class supplies its asset and model-specific names.


## Model
def _ur_asset_cfg(name: str, usd_path: str) -> ArticulationCfg:
    return ArticulationCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        spawn=UsdFileCfg(
            usd_path=usd_path,
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
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": radians(-90.0),
                "elbow_joint": radians(90.0),
                "wrist_1_joint": radians(-90.0),
                "wrist_2_joint": radians(-90.0),
                "wrist_3_joint": radians(-90.0),
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                velocity_limit_sim=100.0,
                effort_limit_sim=87.0,
                stiffness=800.0,
                damping=40.0,
            ),
        },
    )


## Actions
def _ur_action_modes(body_name: str = "wrist_3_link") -> dict[str, ActionGroup]:
    return {
        "diff_ik": InverseKinematicsActionGroup(
            DifferentialInverseKinematicsActionCfg(
                asset_name="robot",
                joint_names=[".*_joint"],
                base_name="base_link",
                body_name=body_name,
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
                joint_names=[".*_joint"],
                body_name=body_name,
                controller_cfg=OperationalSpaceControllerCfg(
                    target_types=["pose_rel"],
                    impedance_mode="fixed",
                    motion_stiffness_task=100.0,
                    motion_damping_ratio_task=1.0,
                    nullspace_control="none",
                    inertial_dynamics_decoupling=True,
                ),
                nullspace_joint_pos_target="none",
                position_scale=0.1,
                orientation_scale=0.1,
                body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
            )
        ),
        "osc_var_stiffness": OperationalSpaceControlActionGroup(
            OperationalSpaceControllerActionCfg(
                asset_name="robot",
                joint_names=[".*_joint"],
                body_name=body_name,
                controller_cfg=OperationalSpaceControllerCfg(
                    target_types=["pose_rel"],
                    impedance_mode="variable_kp",
                    motion_stiffness_limits_task=(10.0, 250.0),
                    motion_damping_ratio_task=1.0,
                    nullspace_control="none",
                    inertial_dynamics_decoupling=True,
                ),
                nullspace_joint_pos_target="none",
                position_scale=0.1,
                orientation_scale=0.1,
                stiffness_scale=120.0,
                body_offset=OperationalSpaceControllerActionCfg.OffsetCfg(),
            )
        ),
        "osc_var_full": OperationalSpaceControlActionGroup(
            OperationalSpaceControllerActionCfg(
                asset_name="robot",
                joint_names=[".*_joint"],
                body_name=body_name,
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
def _ur_sensor_cfgs(
    name: str, wrist_body: str = "wrist_3_link"
) -> dict[str, SensorBaseCfg]:
    return {
        "contacts_robot": ContactSensorCfg(
            prim_path=f"{{ENV_REGEX_NS}}/{name}/.*",
        ),
        "cam_base": CameraCfg(
            prim_path=f"{{ENV_REGEX_NS}}/{name}/base_link/camera_base",
            offset=CameraCfg.OffsetCfg(
                pos=(0.06, 0.0, 0.15),
                rot=rpy_to_quat(0.0, -10.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
        "cam_wrist": CameraCfg(
            prim_path=f"{{ENV_REGEX_NS}}/{name}/{wrist_body}/camera_wrist",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.07, 0.0, 0.05),
                rot=rpy_to_quat(0.0, -60.0, 180.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }


class UR3(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur3", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur3/ur3.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur3")


class UR3e(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur3e", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur3e/ur3e.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur3e")


class UR5(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur5", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur5/ur5.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur5")


class UR5e(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur5e", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur5e/ur5e.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur5e")


class UR10(SerialManipulator):
    ## The older UR10 instanceable USD uses ``ee_link`` (rotated 180/-90/0) as flange.
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur10",
        f"{ISAACLAB_NUCLEUS_DIR}/Robots/UniversalRobots/UR10/ur10_instanceable.usd",
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes(
        body_name="ee_link"
    )
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(
        prim_relpath="ee_link",
        offset=Transform(rot=rpy_to_quat(180.0, -90.0, 0.0)),
    )
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs(
        "ur10", wrist_body="ee_link"
    )


class UR10e(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur10e", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur10e/ur10e.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur10e")


class UR16e(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur16e", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur16e/ur16e.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur16e")


class UR20(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur20", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur20/ur20.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur20")


class UR30(SerialManipulator):
    asset_cfg: ArticulationCfg = _ur_asset_cfg(
        "ur30", f"{ISAAC_NUCLEUS_DIR}/Robots/UniversalRobots/ur30/ur30.usd"
    )
    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("ur30")


class RandomURManipulator(SerialManipulator):
    ## Random UR arm; keeps prim name ``anymal`` for scene compatibility.
    asset_cfg: ArticulationCfg = UR30().asset_cfg.copy()  # type: ignore[misc]
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal"
    asset_cfg.spawn = MultiAssetSpawnerCfg(
        random_choice=False,
        assets_cfg=(
            UR3e().asset_cfg.spawn,  # type: ignore[union-attr]
            UR5().asset_cfg.spawn,  # type: ignore[union-attr]
            UR10e().asset_cfg.spawn,  # type: ignore[union-attr]
            UR16e().asset_cfg.spawn,  # type: ignore[union-attr]
            UR20().asset_cfg.spawn,  # type: ignore[union-attr]
            UR30().asset_cfg.spawn,  # type: ignore[union-attr]
        ),
        activate_contact_sensors=True,
    )

    action_modes: ClassVar[dict[str, ActionGroup]] = _ur_action_modes()
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_flange: Frame = Frame(prim_relpath="wrist_3_link")
    sensor_cfgs: dict[str, SensorBaseCfg] = _ur_sensor_cfgs("anymal")
