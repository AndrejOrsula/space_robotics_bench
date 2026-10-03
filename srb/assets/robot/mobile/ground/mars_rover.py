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


class MarsRover(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*[Dd]rive.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/mars_rover/Body",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/mars_rover/.*[Dd]rive.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/mars_rover/Body/camera_front",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(-0.7675, 0.0, 1.9793),
                rot=rpy_to_quat(0.0, 15.0, -90.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/mars_rover",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("mars_rover.usdz")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "base_steering": ImplicitActuatorCfg(
                joint_names_expr=[".*Steer.*"],
                velocity_limit_sim=6.0,
                effort_limit_sim=12.0,
                stiffness=8000.0,
                damping=1000.0,
            ),
            "base_drive": ImplicitActuatorCfg(
                joint_names_expr=[".*Drive.*"],
                velocity_limit_sim=6.0,
                effort_limit_sim=12.0,
                stiffness=100.0,
                damping=4000.0,
            ),
            "passive_joints_boogie": ImplicitActuatorCfg(
                joint_names_expr=[".*RevoluteJoint"],
                velocity_limit_sim=15.0,
                effort_limit_sim=0.0,
                stiffness=0.0,
                damping=0.0,
            ),
            "passive_joints_rocker": ImplicitActuatorCfg(
                joint_names_expr=[".*Rocker.*"],
                velocity_limit_sim=15.0,
                effort_limit_sim=0.0,
                stiffness=0.0,
                damping=0.0,
            ),
            "passive_joints_differential": ImplicitActuatorCfg(
                joint_names_expr=[".*Differential.*"],
                velocity_limit_sim=15.0,
                effort_limit_sim=0.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        # USD tyre geometry gives a 0.105 m mesh radius, but measured rolling
        # radius on the default ground plane is 0.0987 m (joints at 2 and 4.76
        # rad/s); use this calibrated value. Tyre-centre geometry gives a
        # 0.8498 m wheelbase, 0.7686 m corner track, and 0.8924 m middle track.
        "drive": WheeledDriveActionGroup(
            WheeledDriveActionCfg(
                asset_name="robot",
                wheelbase=(0.8498, 0.7686),
                wheelbase_mid=0.8924,
                wheel_radius=0.0987,
                # Steering joints rotate clockwise about +z, opposite the
                # rover term's convention. Rear-then-front order negates the
                # term's front/rear signs; retain left-first within each pair.
                steering_joint_names=[
                    "RL_Steer_Revolute",
                    "RR_Steer_Revolute",
                    "FL_Steer_Revolute",
                    "FR_Steer_Revolute",
                ],
                # Right-first: the drive differential's first slot has y<0,
                # which is the right wheel in body coordinates.
                drive_joint_names=[
                    "FR_Drive_Continous",
                    "FL_Drive_Continous",
                    "CR_Drive_Continous",
                    "CL_Drive_Continous",
                    "RR_Drive_Continous",
                    "RL_Drive_Continous",
                ],
                scale_linear=0.5,
                scale_angular=radians(45),
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="Body")
    frame_payload_mount: Frame = Frame(
        prim_relpath="Body",
        offset=Transform(
            pos=(-0.22, 0.0, 0.23),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="Body",
        offset=Transform(
            pos=(0.25, 0.0, 0.1),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
