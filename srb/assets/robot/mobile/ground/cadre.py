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
from srb.utils.math import radians
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Cadre(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/cadre/chassis",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/cadre/.*wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/cadre/chassis/camera_front",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.2357, 0.0, 0.1893),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/cadre",
        spawn=UsdFileCfg(
            # NOTE: This is in unofficial model created from scratch based on reference images
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("cadre.usdz")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "wheel_joints": ImplicitActuatorCfg(
                joint_names_expr=["wheel_joint_.*"],
                effort_limit_sim=250.0,
                velocity_limit_sim=6.28,
                stiffness=0.0,
                damping=2000.0,
            ),
            "suspension_joints": ImplicitActuatorCfg(
                joint_names_expr=["suspension_joint.*"],
                velocity_limit_sim=0.25,
                effort_limit_sim=500.0,
                damping=5.0,
                stiffness=400.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.306, 0.35),
                wheel_radius=0.095,
                drive_joint_names=[
                    "wheel_joint_lf",
                    "wheel_joint_rf",
                    "wheel_joint_lr",
                    "wheel_joint_rr",
                ],
                scale_linear=0.4,
                scale_angular=radians(45.0),
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="chassis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(-0.2, 0.0, 0.2175),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.15, 0.0, 0.2175),
        ),
    )
