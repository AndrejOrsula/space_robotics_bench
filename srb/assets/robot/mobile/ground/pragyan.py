from typing import ClassVar

from srb.assets.robot.mobile.ground._spawn_defaults import (
    wheeled_rover_spawn_properties,
)
from srb.core.action import ActionGroup, WheeledDriveActionCfg, WheeledDriveActionGroup
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, Transform, WheeledRobot
from srb.core.sensor import (
    CameraCfg,
    ContactSensorCfg,
    ImuCfg,
    PinholeCameraCfg,
    SensorBaseCfg,
)
from srb.core.sim import (
    UsdFileCfg,
)
from srb.utils.math import radians, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Pragyan(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/pragyan/body",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/pragyan/.*wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/pragyan/body/camera_front",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.1961, -0.41564, 0.4044),
                rot=rpy_to_quat(0.0, 0.0, -90.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/pragyan",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("pragyan.usdz")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "drive_joints": ImplicitActuatorCfg(
                joint_names_expr=["wheel_drive_joint_.*"],
                velocity_limit_sim=40.0,
                effort_limit_sim=150.0,
                damping=5000.0,
                stiffness=0.0,
            ),
            "rocker_joints": ImplicitActuatorCfg(
                joint_names_expr=["rocker_joint_.*"],
                velocity_limit_sim=5.0,
                effort_limit_sim=2500.0,
                damping=400.0,
                stiffness=1000.0,
            ),
            "bogie_joints": ImplicitActuatorCfg(
                joint_names_expr=["boogie_joint_.*"],
                velocity_limit_sim=4.0,
                effort_limit_sim=500.0,
                damping=200.0,
                stiffness=250.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.8533, 0.945),
                wheel_radius=0.1125,
                drive_joint_names=[
                    "wheel_drive_joint_lf",
                    "wheel_drive_joint_rf",
                    "wheel_drive_joint_lm",
                    "wheel_drive_joint_rm",
                    "wheel_drive_joint_lr",
                    "wheel_drive_joint_rr",
                ],
                scale_linear=0.35,
                scale_angular=radians(80),
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="body")
    frame_payload_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.0, 0.285, 0.379),
            rot=rpy_to_quat(0.0, 0.0, -90.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.008, -0.31175, 0.389),
            rot=rpy_to_quat(0.0, 0.0, -90.0),
        ),
    )
