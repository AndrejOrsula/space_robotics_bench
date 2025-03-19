from srb.core.action import (
    ActionGroup,
    WheeledRoverActionGroup,
    WheeledRoverDriveActionCfg,
)
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


class MarsRover(WheeledRobot):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/mars_rover",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("mars_rover")
            .joinpath("rover.usd")
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
            "base_steering": ImplicitActuatorCfg(
                joint_names_expr=[".*Steer.*"],
                velocity_limit=6.0,
                effort_limit=12.0,
                stiffness=8000.0,
                damping=1000.0,
            ),
            "base_drive": ImplicitActuatorCfg(
                joint_names_expr=[".*Drive.*"],
                velocity_limit=6.0,
                effort_limit=12.0,
                stiffness=100.0,
                damping=4000.0,
            ),
            "passive_joints_boogie": ImplicitActuatorCfg(
                joint_names_expr=[".*RevoluteJoint"],
                velocity_limit=15.0,
                effort_limit=0.0,
                stiffness=0.0,
                damping=0.0,
            ),
            "passive_joints_rocker": ImplicitActuatorCfg(
                joint_names_expr=[".*Rocker.*"],
                velocity_limit=15.0,
                effort_limit=0.0,
                stiffness=0.0,
                damping=0.0,
            ),
            "passive_joints_differential": ImplicitActuatorCfg(
                joint_names_expr=[".*Differential.*"],
                velocity_limit=15.0,
                effort_limit=0.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = WheeledRoverActionGroup(
        WheeledRoverDriveActionCfg(
            asset_name="robot",
            wheelbase=(0.849, 0.77),
            wheelbase_mid=0.894,
            wheel_radius=0.1,
            steering_joint_names=[".*Steer_Revolute"],
            drive_joint_names=[
                "FL_Drive_Continous",
                "FR_Drive_Continous",
                "CL_Drive_Continous",
                "CR_Drive_Continous",
                "RL_Drive_Continous",
                "RR_Drive_Continous",
            ],
            scale=1.0,
        )
    )

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
    frame_front_camera: Frame = Frame(
        prim_relpath="Body/camera_front",
        offset=Transform(
            pos=(-0.7675, 0.0, 1.9793),
            rot=rpy_to_quat(0.0, 15.0, -90.0),
        ),
    )
