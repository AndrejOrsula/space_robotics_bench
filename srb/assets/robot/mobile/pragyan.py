from srb.core.action import ActionGroup, WheeledDriveActionCfg, WheeledDriveActionGroup
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, Transform, WheeledRobot
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Pragyan(WheeledRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/pragyan",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("pragyan.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.02, rest_offset=0.005
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_linear_velocity=1.5,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
            ),
        ),
        actuators={
            "drive_joints": ImplicitActuatorCfg(
                joint_names_expr=["wheel_drive_joint_.*"],
                velocity_limit=40.0,
                effort_limit=150.0,
                damping=5000.0,
                stiffness=0.0,
            ),
            "rocker_joints": ImplicitActuatorCfg(
                joint_names_expr=["rocker_joint_.*"],
                velocity_limit=5.0,
                effort_limit=2500.0,
                damping=400.0,
                stiffness=1000.0,
            ),
            "bogie_joints": ImplicitActuatorCfg(
                joint_names_expr=["boogie_joint_.*"],
                velocity_limit=4.0,
                effort_limit=500.0,
                damping=200.0,
                stiffness=250.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = WheeledDriveActionGroup(
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
            scale=1.0,
        )
    )

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
    frame_front_camera: Frame = Frame(
        prim_relpath="body/camera_front",
        offset=Transform(
            pos=(0.1961, -0.41564, 0.4044),
            rot=rpy_to_quat(0.0, 0.0, -90.0),
        ),
    )
