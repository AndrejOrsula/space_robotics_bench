from typing import ClassVar

from srb.assets.object.payload import HuskyPayloadLunalab
from srb.assets.robot.mobile.ground._spawn_defaults import (
    wheeled_rover_spawn_properties,
)
from srb.core.action import ActionGroup, WheeledDriveActionCfg, WheeledDriveActionGroup
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import (
    ArticulationCfg,
    Frame,
    Payload,
    Transform,
    WheeledRobot,
)
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


class Husky(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/husky/chassis",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/husky/.*wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/husky/chassis/camera_front",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.04, 0.0, 0.7),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/husky",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("husky.usdc")
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
        },
    )

    ## Payload - The default payload is separate to allow for easy replacement
    payload: Payload | None = HuskyPayloadLunalab()

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        # USD wheel centres: x=+-0.2560 m (0.512 m wheelbase), y=+-0.2829 m
        # (0.5658 m track); mesh radius is 0.168 m. On the default ground
        # plane, measured rolling radius is 0.160 m (joints at 2 and 4.76
        # rad/s); use this calibrated value. Left-first order matches the
        # skid-steer term's y<0 inner-wheel slot for counter-clockwise turns.
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.512, 0.5658),
                wheel_radius=0.160,
                drive_joint_names=[
                    "wheel_joint_fl",
                    "wheel_joint_fr",
                    "wheel_joint_rl",
                    "wheel_joint_rr",
                ],
                scale_linear=0.1,
                scale_angular=radians(45.0),
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="chassis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.240072, 0.080433, 0.381129),
        ),
    )
    frame_manipulator_mount: Frame = Frame(  # Right
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.24, -0.159664, 0.383129),
        ),
    )
