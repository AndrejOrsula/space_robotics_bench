from typing import ClassVar

from srb.assets.robot.mobile.ground._spawn_defaults import (
    wheeled_rover_spawn_properties,
)
from srb.core.action import (
    ActionGroup,
    JointVelocityActionCfg,
    RelativeJointPositionActionCfg,
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
from srb.utils.cfg import configclass
from srb.utils.math import radians
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


@configclass
class NiftyActionGroup(ActionGroup):
    joint_vel_wheel_drive: JointVelocityActionCfg = JointVelocityActionCfg(
        asset_name="robot",
        joint_names=[
            "wheel_joint_fl",
            "wheel_joint_fr",
            "wheel_joint_rl",
            "wheel_joint_rr",
        ],
        scale=radians(90.0),
    )
    joint_pos_steer: RelativeJointPositionActionCfg = RelativeJointPositionActionCfg(
        asset_name="robot",
        joint_names=[
            "steer_joint_fl",
            "steer_joint_fr",
            "steer_joint_rl",
            "steer_joint_rr",
        ],
        scale=radians(45.0),
    )
    joint_pos_suspension_yaw: RelativeJointPositionActionCfg = (
        RelativeJointPositionActionCfg(
            asset_name="robot",
            joint_names=["suspension_joint_yaw_f", "suspension_joint_yaw_r"],
            scale=radians(30.0),
        )
    )
    joint_pos_suspension_roll: RelativeJointPositionActionCfg | None = (
        RelativeJointPositionActionCfg(
            asset_name="robot",
            joint_names=["suspension_joint_roll_f", "suspension_joint_roll_r"],
            scale=radians(15.0),
        )
    )


class Nifty(WheeledRobot):
    ## Links
    regex_wheels_links: str = ".*wheel.*"

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/nifty/chassis",
        ),
        "contacts_wheels": ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/nifty/.*wheel.*",
        ),
        "cam_front": CameraCfg(
            prim_path="{ENV_REGEX_NS}/nifty/chassis",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.1875, 0.0, 0.15),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }

    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/nifty",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("nifty.usdz")
            .as_posix(),
            **wheeled_rover_spawn_properties(),
        ),
        actuators={
            "wheel_joints": ImplicitActuatorCfg(
                joint_names_expr=[
                    "wheel_joint_fl",
                    "wheel_joint_fr",
                    "wheel_joint_rl",
                    "wheel_joint_rr",
                ],
                effort_limit_sim=80.0,
                velocity_limit_sim=radians(90.0),
                stiffness=0.0,
                damping=80.0,
                armature=1.0,
                friction=2.0,
                dynamic_friction=1.0,
                viscous_friction=1.5,
            ),
            "steer_joints": ImplicitActuatorCfg(
                joint_names_expr=[
                    "steer_joint_fl",
                    "steer_joint_fr",
                    "steer_joint_rl",
                    "steer_joint_rr",
                ],
                effort_limit_sim=200.0,
                velocity_limit_sim=radians(30.0),
                stiffness=100.0,
                damping=50.0,
                armature=2.0,
                friction=0.5,
                dynamic_friction=0.25,
                viscous_friction=2.5,
            ),
            "suspension_joint_yaw": ImplicitActuatorCfg(
                joint_names_expr=["suspension_joint_yaw_f", "suspension_joint_yaw_r"],
                effort_limit_sim=200.0,
                velocity_limit_sim=radians(15.0),
                stiffness=1250.0,
                damping=750.0,
                armature=100.0,
                friction=40.0,
                dynamic_friction=12.5,
                viscous_friction=4.5,
            ),
            "suspension_joint_roll": ImplicitActuatorCfg(
                joint_names_expr=["suspension_joint_roll_f", "suspension_joint_roll_r"],
                effort_limit_sim=200.0,
                velocity_limit_sim=radians(15.0),
                stiffness=15000.0,
                damping=5000.0,
                armature=75.0,
                friction=60.0,
                dynamic_friction=40.0,
                viscous_friction=2.5,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "nifty": NiftyActionGroup(),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="chassis")
    frame_payload_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.0, 0.0, 0.2),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="chassis",
        offset=Transform(
            pos=(0.0, 0.0, 0.2),
        ),
    )
