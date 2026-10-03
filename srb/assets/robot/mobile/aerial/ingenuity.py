from typing import ClassVar

from torch import pi

from srb.core.action import (
    ActionGroup,
    MulticopterBodyAccelerationActionCfg,
    MulticopterBodyAccelerationActionGroup,
)
from srb.core.actuator import ImplicitActuatorCfg
from srb.core.asset import ArticulationCfg, Frame, Multicopter, Transform
from srb.core.sensor import (
    CameraCfg,
    HeightmapSensorCfg,
    HeightmapSpec,
    ImuCfg,
    PinholeCameraCfg,
    SensorBaseCfg,
)
from srb.core.sensor.heightmap import build_heightmap_pattern_from_spec
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT

# One downward cell acts as a body-frame altimeter; share it between `spec`
# and the required `pattern_cfg` to keep the raycaster settings consistent.
_ALTIMETER_SPEC = HeightmapSpec(
    nx=1,
    ny=1,
    cell_size_m=0.1,
    forward_offset_m=0.0,
    lateral_offset_m=0.0,
    dead_zone_m=0.0,
    frame="body",
    max_range_m=100.0,
)


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
                # Approximate aerodynamic drag, meaningful only in an atmosphere.
                # Ingenuity is a Mars rotorcraft; tasks moved off the airless Moon.
                linear_damping=0.5,
                angular_damping=0.5,
            ),
        ),
        actuators={
            "rotors": ImplicitActuatorCfg(
                joint_names_expr=["rotor_joint_[1-2]"],
                velocity_limit_sim=2500 / 60 * 2 * pi,
                effort_limit_sim=7.5,
                stiffness=0.0,
                damping=1000.0,
            ),
        },
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "body_acc": MulticopterBodyAccelerationActionGroup(
            MulticopterBodyAccelerationActionCfg(
                asset_name="robot",
                frame_base="body",
                regex_rotor_joints="rotor_joint_[1-2]",
                nominal_rpm={
                    # Targets use rad/s, matching the 2500 RPM limit; passing
                    # 2500 directly would exceed it by about 9.5x.
                    "rotor_joint_1": 2500 / 60 * 2 * pi,
                    "rotor_joint_2": -2500 / 60 * 2 * pi,
                },
                tilt_magnitude=0.125,
                scale=0.5,
            )
        ),
    }

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
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/ingenuity/body",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/ingenuity/body/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(0.045, 0.0, 0.1275),
                rot=rpy_to_quat(0.0, 90.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
        "altimeter": HeightmapSensorCfg(
            prim_path="{ENV_REGEX_NS}/ingenuity/body",
            offset=HeightmapSensorCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
            mesh_prim_paths=[".*"],
            pattern_cfg=build_heightmap_pattern_from_spec(_ALTIMETER_SPEC),
            spec=_ALTIMETER_SPEC,
        ),
    }
