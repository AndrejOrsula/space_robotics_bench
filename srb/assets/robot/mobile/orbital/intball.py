from typing import ClassVar

from srb.core.action import (
    ActionGroup,
    ThrustActionCfg,
    ThrustActionGroup,
    ThrusterCfg,
)
from srb.core.asset import Frame, OrbitalRobot, RigidObjectCfg, Transform
from srb.core.sensor import (
    CameraCfg,
    ImuCfg,
    PinholeCameraCfg,
    SensorBaseCfg,
)
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class IntBall1(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/intball1",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("intball1.usdc")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Int-Ball operates inside the ISS, so retain physical air damping.
                angular_damping=2.0,
                linear_damping=0.1,
            ),
            mass_props=MassPropertiesCfg(mass=1.0),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust": ThrustActionGroup(
            ThrustActionCfg(
                asset_name="robot",
                thrusters=(
                    ThrusterCfg(
                        offset=(0.075, 0.0, 0.0),
                        direction=(1.0, 0.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(-0.075, 0.0, 0.0),
                        direction=(-1.0, 0.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, 0.075, 0.0),
                        direction=(0.0, 1.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, -0.075, 0.0),
                        direction=(0.0, -1.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, 0.0, 0.075),
                        direction=(0.0, 0.0, 1.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, 0.0, -0.075),
                        direction=(0.0, 0.0, -1.0),
                        power=0.5,
                    ),
                ),
                fuel_capacity=5.0,
                fuel_consumption_rate=0.0,
            )
        ),
    }

    ## Frames
    ## The authored body is "base_link" (/root/base_link), not "base"; the latter
    ## breaks IMU, camera, and frame initialization.
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(pos=(0.0, 0.0, 0.0)),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(pos=(0.0, 0.0, 0.075)),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/intball1/base_link",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/intball1/base_link/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.075, 0.0, 0.0),
                rot=rpy_to_quat(0.0, 90.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }


class IntBall2(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/intball2",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("intball2.usdc")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Int-Ball operates inside the ISS, so retain physical air damping.
                angular_damping=2.0,
                linear_damping=0.1,
            ),
            mass_props=MassPropertiesCfg(mass=1.5),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust": ThrustActionGroup(
            ThrustActionCfg(
                asset_name="robot",
                thrusters=(
                    ThrusterCfg(
                        offset=(0.075, 0.0, 0.0),
                        direction=(1.0, 0.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(-0.075, 0.0, 0.0),
                        direction=(-1.0, 0.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, 0.075, 0.0),
                        direction=(0.0, 1.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, -0.075, 0.0),
                        direction=(0.0, -1.0, 0.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, 0.0, 0.075),
                        direction=(0.0, 0.0, 1.0),
                        power=0.5,
                    ),
                    ThrusterCfg(
                        offset=(0.0, 0.0, -0.075),
                        direction=(0.0, 0.0, -1.0),
                        power=0.5,
                    ),
                ),
                fuel_capacity=5.0,
                fuel_consumption_rate=0.0,
            )
        ),
    }

    ## Frames
    ## The authored body is "base_link" (/root/base_link), not "base"; the latter
    ## breaks IMU, camera, and frame initialization.
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(pos=(0.0, 0.0, 0.0)),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(pos=(0.0, 0.0, 0.075)),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/intball2/base_link",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/intball2/base_link/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.075, 0.0, 0.0),
                rot=rpy_to_quat(0.0, 90.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }
