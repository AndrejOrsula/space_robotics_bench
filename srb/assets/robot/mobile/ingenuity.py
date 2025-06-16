from torch import pi

from srb.core.action import (
    ActionGroup,
    MulticopterBodyAccelerationActionCfg,
    MulticopterBodyAccelerationActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, Multicopter, Transform
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Ingenuity(Multicopter):
    ## Model
    asset_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/ingenuity",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("copter")
            .joinpath("ingenuity.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(
                contact_offset=0.005, rest_offset=0.0
            ),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
            ),
            rigid_props=RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
        ),
        actuators={
            "rotors": ImplicitActuatorCfg(
                joint_names_expr=["rotor_joint_[1-2]"],
                velocity_limit=2500 / 60 * 2 * pi,
                effort_limit=7.5,
                stiffness=0.0,
                damping=1000.0,
            ),
        },
    )

    ## Actions
    actions: ActionGroup = MulticopterBodyAccelerationActionGroup(
        MulticopterBodyAccelerationActionCfg(
            asset_name="robot",
            frame_base="body",
            regex_rotor_joints="rotor_joint_[1-2]",
            nominal_rpm={
                "rotor_joint_1": 2500.0,
                "rotor_joint_2": -2500.0,
            },
            tilt_magnitude=0.125,
            scale=5.0,
        )
    )

    ## Frames
    frame_base: Frame = Frame(prim_relpath="body")
    frame_payload_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="body",
        offset=Transform(
            pos=(0.0, 0.0, 0.13),
            rot=rpy_to_quat(0.0, 180.0, 0.0),
        ),
    )
    frame_onboard_camera: Frame = Frame(
        prim_relpath="body/camera_onboard",
        offset=Transform(
            pos=(0.045, 0.0, 0.1275),
            rot=rpy_to_quat(0.0, 90.0, 0.0),
        ),
    )
