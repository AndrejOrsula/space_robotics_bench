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


class OpenSourceRover(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/open_source_rover/chassis",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/open_source_rover/.*wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/open_source_rover/chassis/camera_front",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(-0.17, 0.0, 0.075),
                rot=rpy_to_quat(0.0, 15.0, 180.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/open_source_rover",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("osr.usdz")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "joint_wheel": ImplicitActuatorCfg(
                joint_names_expr=["joint_wheel_.*"],
                velocity_limit_sim=40.0,
                effort_limit_sim=50.0,
                damping=2000.0,
                stiffness=0.0,
            ),
            "joints_steer": ImplicitActuatorCfg(
                joint_names_expr=["joint_steer_.*"],
                velocity_limit_sim=0.5,
                effort_limit_sim=500.0,
                damping=80.0,
                stiffness=100.0,
            ),
            "joints_rocker": ImplicitActuatorCfg(
                joint_names_expr=["joint_rocker_.*"],
                velocity_limit_sim=0.3,
                effort_limit_sim=1500.0,
                damping=2.0,
                stiffness=25.0,
            ),
            "joints_bogie": ImplicitActuatorCfg(
                joint_names_expr=["joint_bogie_.*"],
                velocity_limit_sim=0.6,
                effort_limit_sim=750.0,
                damping=4.0,
                stiffness=1.0,
            ),
            "joint_linkage": ImplicitActuatorCfg(
                joint_names_expr=["joint_linkage"],
                velocity_limit_sim=0.2,
                effort_limit_sim=25.0,
                damping=0.1,
                stiffness=0.05,
            ),
            # 500 / 50 tendon
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.5835, 0.39),
                wheelbase_mid=0.52,
                wheel_radius=0.072,
                # Maps joints to the rover term's front/middle/rear y<0/y>0
                # slots. This asset rolls toward -x; right steer joints are
                # CCW-positive and left ones CW-positive about root +z.
                # Drive slots are left-first (outer wheel at y<0); steer slots
                # put lf/rr in front and rf/lr in rear to match those signs.
                steering_joint_names=[
                    "joint_steer_rr",
                    "joint_steer_lf",
                    "joint_steer_rf",
                    "joint_steer_lr",
                ],
                drive_joint_names=[
                    "joint_wheel_lf",
                    "joint_wheel_rf",
                    "joint_wheel_lm",
                    "joint_wheel_rm",
                    "joint_wheel_lr",
                    "joint_wheel_rr",
                ],
                scale_linear=0.2,
                scale_angular=radians(20),
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="chassis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(-0.05, 0.0, 0.121),
            rot=rpy_to_quat(0.0, 0.0, 180.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(-0.1, 0.0, 0.121),
            rot=rpy_to_quat(0.0, 0.0, 180.0),
        ),
    )
