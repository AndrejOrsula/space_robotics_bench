from srb.core.action import ActionGroup, WheeledDriveActionCfg, WheeledDriveActionGroup
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, Transform, WheeledRobot
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class LeoRover(WheeledRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/leo_rover",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("leo_rover.usdc")
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
            "drive": ImplicitActuatorCfg(
                joint_names_expr=["wheel_joint_.*"],
                effort_limit=80.0,
                velocity_limit=30.0,
                stiffness=0.0,
                damping=5000.0,
            ),
            "rocker": ImplicitActuatorCfg(
                joint_names_expr=["rocker_joint_.*"],
                velocity_limit=2.0,
                effort_limit=500.0,
                damping=0.5,
                stiffness=2.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = WheeledDriveActionGroup(
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
            scale_angular=deg_to_rad(60),
        )
    )

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
    frame_front_camera: Frame = Frame(
        prim_relpath="chassis/camera_front",
        offset=Transform(
            # pos=(0.098, 0.0, 0.157), # Embedded
            pos=(0.105, 0.0, 0.22),  # Top-mounted
        ),
    )
