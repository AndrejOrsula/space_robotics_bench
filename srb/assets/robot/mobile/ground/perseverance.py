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


class Perseverance(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/perseverance/body",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/perseverance/.*wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/perseverance/body/camera_front",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                # Right Navcam
                pos=(-0.7675, -0.8537, 1.9793),
                rot=rpy_to_quat(0.0, 15.0, -90.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/perseverance",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("perseverance.usdz")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "drive_joints": ImplicitActuatorCfg(
                joint_names_expr=["drive_joint.*"],
                velocity_limit_sim=40.0,
                effort_limit_sim=150.0,
                damping=25000.0,
                stiffness=0.0,
            ),
            "steer_joints": ImplicitActuatorCfg(
                joint_names_expr=["steer_joint.*"],
                velocity_limit_sim=2.0,
                effort_limit_sim=400.0,
                damping=200.0,
                stiffness=500.0,
            ),
            "rocker_joints": ImplicitActuatorCfg(
                joint_names_expr=["suspension_joint_rocker.*"],
                velocity_limit_sim=5.0,
                effort_limit_sim=2500.0,
                damping=400.0,
                stiffness=4000.0,
            ),
            "bogie_joints": ImplicitActuatorCfg(
                joint_names_expr=["suspension_joint_bogie.*"],
                velocity_limit_sim=4.0,
                effort_limit_sim=500.0,
                damping=25.0,
                stiffness=200.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(2.26, 2.14764),
                wheelbase_mid=2.39164,
                wheel_radius=0.26268,
                steering_joint_names=[
                    "steer_joint_front_left",
                    "steer_joint_front_right",
                    "steer_joint_rear_left",
                    "steer_joint_rear_right",
                ],
                # Drive is right-first: the term's y<0 slot is the outer wheel
                # for a counter-clockwise turn on this +x-facing rover. Steering
                # is left-first to match its CCW-positive joints and inner wheel.
                drive_joint_names=[
                    "drive_joint_front_right",
                    "drive_joint_front_left",
                    "drive_joint_mid_right",
                    "drive_joint_mid_left",
                    "drive_joint_rear_right",
                    "drive_joint_rear_left",
                ],
                scale_linear=0.7,
                scale_angular=radians(75),
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="body")
    frame_payload_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.0, 0.0, 1.25),
            rot=rpy_to_quat(0.0, 0.0, -90.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.0, -0.875, 1.25),
            rot=rpy_to_quat(0.0, 0.0, -90.0),
        ),
    )
