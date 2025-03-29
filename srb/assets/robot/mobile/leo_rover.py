from srb.core.action import ActionGroup, JointVelocityActionGroup
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
        init_state=ArticulationCfg.InitialStateCfg(),
        actuators={
            "base_drive": ImplicitActuatorCfg(
                joint_names_expr=["wheel_.*_joint"],
                velocity_limit=6.0,
                effort_limit=12.0,
                stiffness=100.0,
                damping=4000.0,
            ),
            "passive_joints": ImplicitActuatorCfg(
                joint_names_expr=["rocker_.*_joint"],
                velocity_limit=15.0,
                effort_limit=0.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = JointVelocityActionGroup()

    ## Frames
    frame_base: Frame = Frame(prim_relpath="body")
    frame_payload_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.0, -0.125, 0.07),
            rot=rpy_to_quat(0.0, 0.0, 90.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.0, 0.1, 0.07),
            rot=rpy_to_quat(0.0, 0.0, 90.0),
        ),
    )
    frame_front_camera: Frame = Frame(
        prim_relpath="body/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )
