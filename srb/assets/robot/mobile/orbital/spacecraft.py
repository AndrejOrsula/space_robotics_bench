from typing import ClassVar

import simforge_foundry

from srb.core.action import (
    ActionGroup,
    BodyAccelerationActionCfg,
    BodyAccelerationActionGroup,
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
    SimforgeAssetCfg,
    UsdFileCfg,
)
from srb.utils.math import radians, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class ISS(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/iss",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("iss.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Vacuum has no drag; PhysX defaults angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            mass_props=MassPropertiesCfg(density=1500.0),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "body_acc": BodyAccelerationActionGroup(
            # Full action uses m/s² linear and rad/s² angular; 2.5 yields
            # 0.05 per 50 Hz substep.
            BodyAccelerationActionCfg(asset_name="robot", scale=2.5)
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 19.9631, 5.81683),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/iss/base",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/iss/base/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, -16.2149, 2.27954),
            ),
            width=64,
            height=64,
            spawn=PinholeCameraCfg(),
        ),
    }


class Gateway(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/gateway",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("gateway.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Vacuum has no drag; PhysX defaults angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            mass_props=MassPropertiesCfg(density=1500.0),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "body_acc": BodyAccelerationActionGroup(
            # Full action uses m/s² linear and rad/s² angular; 2.5 yields
            # 0.05 per 50 Hz substep.
            BodyAccelerationActionCfg(asset_name="robot", scale=2.5)
        ),
    }

    ## Frames
    ## The USD body is ``gateway``; no ``base`` prim exists.
    frame_base: Frame = Frame(prim_relpath="gateway")
    frame_payload_mount: Frame = Frame(
        prim_relpath="gateway",
        offset=Transform(
            pos=(-0.5, 0.0, -0.1),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="gateway",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/gateway/gateway",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/gateway/gateway/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(11.148, 0.05865, -1.63578),
            ),
            width=64,
            height=64,
            spawn=PinholeCameraCfg(),
        ),
    }


class Cubesat(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cubesat",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.Cubesat()],
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Vacuum has no drag; PhysX defaults angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    ## Two thrusters per face, offset tangentially: firing both translates, and
    ## opposite-face pairs give torque about every axis.
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust": ThrustActionGroup(
            ThrustActionCfg(
                asset_name="robot",
                thrusters=tuple(
                    ThrusterCfg(offset=offset, direction=direction, power=0.75)
                    for direction, offsets in (
                        ((1.0, 0.0, 0.0), ((0.05, 0.03, 0.0), (0.05, -0.03, 0.0))),
                        ((-1.0, 0.0, 0.0), ((-0.05, 0.03, 0.0), (-0.05, -0.03, 0.0))),
                        ((0.0, 1.0, 0.0), ((0.0, 0.05, 0.03), (0.0, 0.05, -0.03))),
                        ((0.0, -1.0, 0.0), ((0.0, -0.05, 0.03), (0.0, -0.05, -0.03))),
                        ((0.0, 0.0, 1.0), ((0.03, 0.0, 0.05), (-0.03, 0.0, 0.05))),
                        ((0.0, 0.0, -1.0), ((0.03, 0.0, -0.05), (-0.03, 0.0, -0.05))),
                    )
                    for offset in offsets
                ),
                fuel_capacity=5.0,
                fuel_consumption_rate=0.0,
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="cubesat")
    frame_payload_mount: Frame = Frame(
        prim_relpath="cubesat",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="cubesat",
        offset=Transform(
            pos=(0.0, 0.0, 0.05),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/cubesat/cubesat",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/cubesat/cubesat/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.1, 0.0),
                rot=rpy_to_quat(0.0, 0.0, 90.0),
            ),
            width=64,
            height=64,
            spawn=PinholeCameraCfg(),
        ),
    }


class SatelliteMockup(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/satellite_mockup",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("satellite_mockup.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Vacuum has no drag; PhysX defaults angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            mass_props=MassPropertiesCfg(density=1500.0),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    ## Opposite-corner pairs provide torque-free translation; other pairs rotate.
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust": ThrustActionGroup(
            ThrustActionCfg(
                asset_name="robot",
                thrusters=(
                    # Corner (+X, +Y, +Z)
                    ThrusterCfg(
                        offset=(0.15, 0.15, 0.15),
                        direction=(1.0, 0.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(0.15, 0.15, 0.15),
                        direction=(0.0, 1.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(0.15, 0.15, 0.15),
                        direction=(0.0, 0.0, 1.0),
                        power=10.0,
                    ),
                    # Corner (+X, -Y, -Z)
                    ThrusterCfg(
                        offset=(0.15, -0.15, -0.15),
                        direction=(1.0, 0.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(0.15, -0.15, -0.15),
                        direction=(0.0, -1.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(0.15, -0.15, -0.15),
                        direction=(0.0, 0.0, -1.0),
                        power=10.0,
                    ),
                    # Corner (-X, +Y, -Z)
                    ThrusterCfg(
                        offset=(-0.15, 0.15, -0.15),
                        direction=(-1.0, 0.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.15, 0.15, -0.15),
                        direction=(0.0, 1.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.15, 0.15, -0.15),
                        direction=(0.0, 0.0, -1.0),
                        power=10.0,
                    ),
                    # Corner (-X, -Y, +Z)
                    ThrusterCfg(
                        offset=(-0.15, -0.15, 0.15),
                        direction=(-1.0, 0.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.15, -0.15, 0.15),
                        direction=(0.0, -1.0, 0.0),
                        power=10.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.15, -0.15, 0.15),
                        direction=(0.0, 0.0, 1.0),
                        power=10.0,
                    ),
                ),
                fuel_capacity=50.0,
                fuel_consumption_rate=0.0,
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="satellite_mockup")
    frame_payload_mount: Frame = Frame(
        prim_relpath="satellite_mockup",
        offset=Transform(
            pos=(0.0, 0.0, -0.15),
            rot=rpy_to_quat(0.0, 180.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="satellite_mockup",
        offset=Transform(
            pos=(0.0, 0.0, 0.15),
        ),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/satellite_mockup/satellite_mockup",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/satellite_mockup/satellite_mockup/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.15, 0.0, 0.0),
                rot=rpy_to_quat(0.0, 90.0, 0.0),
            ),
            width=64,
            height=64,
            spawn=PinholeCameraCfg(),
        ),
    }


class VenusExpress(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/venus_express",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("venus_express.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Vacuum has no drag; PhysX defaults angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            mass_props=MassPropertiesCfg(density=1500.0),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust": ThrustActionGroup(
            ThrustActionCfg(
                asset_name="robot",
                thrusters=(
                    # Gimbaled (1)
                    ThrusterCfg(
                        offset=(0.0, 0.0, -1.15),
                        direction=(0.0, 0.0, -1.0),
                        power=20000.0,
                        gimbal_limits=(radians(20.0), radians(20.0)),
                    ),
                    # Sixteen modeled; the real spacecraft has eight.
                    ThrusterCfg(
                        offset=(-0.8085, -0.693, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.86, -0.746, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.8085, 0.693, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.86, 0.746, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.8085, -0.693, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.86, -0.746, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.8085, 0.693, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.86, 0.746, -1.45),
                        direction=(0.0, 0.0, -1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.8085, -0.693, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.86, -0.746, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.8085, 0.693, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.86, 0.746, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.8085, -0.693, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.86, -0.746, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.8085, 0.693, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.86, 0.746, 0.425),
                        direction=(0.0, 0.0, 1.0),
                        power=2000.0,
                    ),
                ),
                fuel_capacity=2500.0,
                fuel_consumption_rate=(2500.0 / (1 * 20000.0 + (16 * 2000.0))) / 20.0,
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.06691, -0.838982, 0.051877),
            rot=rpy_to_quat(90.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, -0.543748, 0.417019),
            rot=rpy_to_quat(0.0, 0.0, -90.0),
        ),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/venus_express/base",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/venus_express/base/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(-0.69557, -0.5479, 0.51),
                rot=rpy_to_quat(0.0, 0.0, 0.0),
            ),
            width=64,
            height=64,
            spawn=PinholeCameraCfg(),
        ),
    }


class Starship(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/starship",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("starship.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Vacuum has no drag; PhysX defaults angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            mass_props=MassPropertiesCfg(mass=100000.0),
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
                        offset=(0.0, 0.7726, 0.76),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-0.669, -0.3863, 0.76),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(0.669, -0.3863, 0.76),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-1.70384, 0.9837, 0.54),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(0.0, -1.96743, 0.54),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(1.70384, 0.9837, 0.54),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                ),
                fuel_capacity=0.5 * 1200000.0,
                fuel_consumption_rate=(0.5 * 1200000.0 / (6 * 2300000.0)) / 20.0,
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/starship/base",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/starship/base/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 3.6437, -0.0155),
                rot=rpy_to_quat(0.0, 90.0, 0.0),
            ),
            width=64,
            height=64,
            spawn=PinholeCameraCfg(),
        ),
    }


class SuperHeavy(OrbitalRobot):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/super_heavy",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("super_heavy.usdz")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                # Vacuum has no drag; PhysX defaults angular damping to 0.05.
                linear_damping=0.0,
                angular_damping=0.0,
            ),
            mass_props=MassPropertiesCfg(mass=200000.0),
            semantic_tags=[("class", "spacecraft")],
        ),
    )

    ## Actions
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust": ThrustActionGroup(
            ThrustActionCfg(
                asset_name="robot",
                thrusters=(
                    # Gimbaled (13)
                    ThrusterCfg(
                        offset=(0.750003, 0.0, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-0.375002, 0.649522, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-0.375002, -0.649522, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(1.78697, 0.478816, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(1.16425, 1.43772, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(0.096822, 1.84747, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-1.00758, 1.55154, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-1.72713, 0.662982, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-1.78697, -0.478816, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-1.16425, -1.43772, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(-0.096822, -1.84747, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(1.00758, -1.55154, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    ThrusterCfg(
                        offset=(1.72713, -0.662982, 1.8),
                        power=2300000.0,
                        gimbal_limits=(radians(15.0), radians(15.0)),
                    ),
                    # Fixed (20)
                    ThrusterCfg(
                        offset=(3.0, 0.0, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(2.85317, 0.927052, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(2.42705, 1.76336, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(1.76335, 2.42705, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.92705, 2.85317, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.0, 3.0, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.92705, 2.85317, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-1.76335, 2.42705, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-2.42705, 1.76336, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-2.85317, 0.927052, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-3.0, 0.0, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-2.85317, -0.927048, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-2.42705, -1.76335, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-1.76335, -2.42705, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.92705, -2.85317, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.0, -3.0, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(0.92705, -2.85317, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(1.76335, -2.42705, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(2.42705, -1.76335, 1.8),
                        power=2300000.0,
                    ),
                    ThrusterCfg(
                        offset=(2.85317, -0.927048, 1.8),
                        power=2300000.0,
                    ),
                ),
                fuel_capacity=0.5 * 3400000.0,
                fuel_consumption_rate=(0.5 * 3400000.0 / (33 * 2300000.0)) / 20.0,
            )
        ),
    }

    ## Frames
    frame_base: Frame = Frame(prim_relpath="base")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base",
        offset=Transform(
            pos=(0.0, 0.0, 0.0),
            rot=rpy_to_quat(0.0, 0.0, 0.0),
        ),
    )

    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/super_heavy/base",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/super_heavy/base/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, -4.45, 2.25),
                rot=rpy_to_quat(0.0, 90.0, 0.0),
            ),
            width=64,
            height=64,
            spawn=PinholeCameraCfg(),
        ),
    }
