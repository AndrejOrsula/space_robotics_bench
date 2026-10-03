from typing import ClassVar

from srb.assets.robot.mobile.ground._spawn_defaults import (
    wheeled_rover_spawn_properties,
)
from srb.core.action import (
    ActionGroup,
    FilteredWheeledDriveActionCfg,
    FilteredWheeledDriveActionGroup,
    WheeledDriveActionCfg,
    WheeledDriveActionGroup,
)
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


class RaphRover(WheeledRobot):
    ## Links
    regex_wheels_links: str = "wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/raph_rover/chassis",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/raph_rover/wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/raph_rover/chassis/camera_front",
            offset=CameraCfg.OffsetCfg(
                pos=(0.1315, 0.0, 0.03),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/raph_rover",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("raph_rover.usdc")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "drive": ImplicitActuatorCfg(
                joint_names_expr=["drive_joint_.*"],
                effort_limit_sim=80.0,
                velocity_limit_sim=30.0,
                stiffness=0.0,
                damping=500.0,
            ),
            "steer": ImplicitActuatorCfg(
                joint_names_expr=["steer_joint_.*"],
                velocity_limit_sim=2.0,
                effort_limit_sim=500.0,
                damping=50.0,
                stiffness=400.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        # USD geometry: radius 0.0885 m, wheelbase 0.381845 m, track 0.390 m.
        # List joints right-then-left: action slots use left=-y, assets left=+y.
        # This preserves Ackermann side mapping (as in mars_rover/perseverance).
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.381845, 0.390),
                wheel_radius=0.0885,
                drive_joint_names=[
                    "drive_joint_front_right",
                    "drive_joint_front_left",
                    "drive_joint_back_right",
                    "drive_joint_back_left",
                ],
                steering_joint_names=[
                    "steer_joint_back_right",
                    "steer_joint_back_left",
                ],
                steering_axle="rear",
                scale_linear=0.4,
                scale_angular=radians(60),
            )
        ),
        # Same USD geometry and joint order as "drive".
        "drive_filtered": FilteredWheeledDriveActionGroup(
            cmd_vel=FilteredWheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.381845, 0.390),
                wheel_radius=0.0885,
                drive_joint_names=[
                    "drive_joint_front_right",
                    "drive_joint_front_left",
                    "drive_joint_back_right",
                    "drive_joint_back_left",
                ],
                steering_joint_names=[
                    "steer_joint_back_right",
                    "steer_joint_back_left",
                ],
                steering_axle="rear",
                scale_linear=0.4,
                scale_angular=radians(60),
                v_lin_max=0.4,
                v_ang_max=1.0,
                accel_lin_max=0.8,
                accel_ang_max=2.0,
                filter_tau_s=0.15,
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="chassis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.38, 0.0, -0.1),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.0, 0.0, 0.12),
        ),
    )
