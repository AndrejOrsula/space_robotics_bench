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


class LeoRover(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/leo_rover/chassis",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/leo_rover/.*wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/leo_rover/chassis/camera_front",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                # Top-mounted
                pos=(0.105, 0.0, 0.22),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/leo_rover",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("leo_rover.usdc")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "drive": ImplicitActuatorCfg(
                joint_names_expr=["wheel_joint_.*"],
                effort_limit_sim=80.0,
                velocity_limit_sim=30.0,
                stiffness=0.0,
                damping=5000.0,
            ),
            "rocker": ImplicitActuatorCfg(
                joint_names_expr=["rocker_joint_.*"],
                velocity_limit_sim=2.0,
                effort_limit_sim=500.0,
                damping=0.5,
                stiffness=2.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.2975, 0.3587),
                wheel_radius=0.065,
                drive_joint_names=[
                    "wheel_joint_FL",
                    "wheel_joint_FR",
                    "wheel_joint_RL",
                    "wheel_joint_RR",
                ],
                scale_linear=0.4,
                scale_angular=radians(60),
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="chassis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(-0.0915, 0.0, 0.2),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.088, 0.0, 0.2),
        ),
    )
