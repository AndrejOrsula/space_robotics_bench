from dataclasses import MISSING
from typing import ClassVar

from srb.core.action import (
    ActionGroup,
    JointPositionBoundedActionGroup,
)
from srb.core.actuator import ActuatorNetLSTMCfg
from srb.core.asset import ArticulationCfg, Frame, LeggedRobot, Transform
from srb.core.sensor import (
    CameraCfg,
    ContactSensorCfg,
    ImuCfg,
    PinholeCameraCfg,
    SensorBaseCfg,
)
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.nucleus import ISAACLAB_NUCLEUS_DIR

# Forward egocentric pose from Isaac Lab's ANYmal-C sensor tutorial
# (fdc650c8): (0.510, 0, 0.015) m, rpy=(-90, 0, -90) deg in ROS optical
# convention. ANYmal-D reuses it because no reference pose is available; the
# previous offset incorrectly placed the camera 2 m overhead, looking up.
_CAM_FRONT_POS = (0.510, 0.0, 0.015)
_CAM_FRONT_ROT = rpy_to_quat(-90.0, 0.0, -90.0)


class AnymalC(LeggedRobot):
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path=MISSING,  # type: ignore[arg-type]
        spawn=UsdFileCfg(
            usd_path=MISSING,  # type: ignore[arg-type]
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
            pos=(0.0, 0.0, 0.6),
            joint_pos={
                ".*HAA": 0.0,
                ".*F_HFE": 0.4,
                ".*H_HFE": -0.4,
                ".*F_KFE": -0.8,
                ".*H_KFE": 0.8,
            },
        ),
        actuators={  # type: ignore[misc]
            "legs": ActuatorNetLSTMCfg(
                joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
                network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt",
                saturation_effort=120.0,
                effort_limit=80.0,
                velocity_limit=7.5,
            ),
        },
    )
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal_c"
    asset_cfg.spawn.usd_path = (  # type: ignore[union-attr]
        f"{ISAACLAB_NUCLEUS_DIR}/Robots/ANYbotics/ANYmal-C/anymal_c.usd"
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/anymal_c/base",
        ),
        "contacts_feet": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/anymal_c/.*FOOT",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/anymal_c/base/camera_front",
            offset=CameraCfg.OffsetCfg(
                pos=_CAM_FRONT_POS,
                rot=_CAM_FRONT_ROT,
                convention="ros",
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_pos": JointPositionBoundedActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(-0.075, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.275, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*FOOT"


class AnymalD(LeggedRobot):
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path=MISSING,  # type: ignore[arg-type]
        spawn=UsdFileCfg(
            usd_path=MISSING,  # type: ignore[arg-type]
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
            pos=(0.0, 0.0, 0.6),
            joint_pos={
                ".*HAA": 0.0,
                ".*F_HFE": 0.4,
                ".*H_HFE": -0.4,
                ".*F_KFE": -0.8,
                ".*H_KFE": 0.8,
            },
        ),
        actuators={  # type: ignore[misc]
            "legs": ActuatorNetLSTMCfg(
                joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
                network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt",
                saturation_effort=120.0,
                effort_limit=80.0,
                velocity_limit=7.5,
            ),
        },
    )
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal_d"
    asset_cfg.spawn.usd_path = (  # type: ignore[union-attr]
        f"{ISAACLAB_NUCLEUS_DIR}/Robots/ANYbotics/ANYmal-D/anymal_d.usd"
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/anymal_d/base",
        ),
        "contacts_feet": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/anymal_d/.*FOOT",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/anymal_d/base/camera_front",
            offset=CameraCfg.OffsetCfg(
                pos=_CAM_FRONT_POS,
                rot=_CAM_FRONT_ROT,
                convention="ros",
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_pos": JointPositionBoundedActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(-0.075, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.275, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*FOOT"


class RandomAnymalQuadruped(LeggedRobot):
    asset_cfg: ArticulationCfg = AnymalD().asset_cfg.copy()  # type: ignore[misc]
    asset_cfg.prim_path = "{ENV_REGEX_NS}/anymal"
    asset_cfg.spawn = MultiAssetSpawnerCfg(
        random_choice=False,
        assets_cfg=(
            AnymalC().asset_cfg.spawn,  # type: ignore[union-attr]
            AnymalD().asset_cfg.spawn,  # type: ignore[union-attr]
        ),
        activate_contact_sensors=True,
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/anymal/base",
        ),
        "contacts_feet": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/anymal/.*FOOT",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/anymal/base/camera_front",
            offset=CameraCfg.OffsetCfg(
                pos=_CAM_FRONT_POS,
                rot=_CAM_FRONT_ROT,
                convention="ros",
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "joint_pos": JointPositionBoundedActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(-0.075, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.275, 0.0, 0.09),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    ## Links
    regex_feet_links: str = ".*FOOT"
