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
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    SimforgeAssetCfg,
    UsdFileCfg,
)
from srb.utils.math import deg_to_rad, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


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
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = BodyAccelerationActionGroup(
        BodyAccelerationActionCfg(asset_name="robot", scale=0.05)
    )

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
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = BodyAccelerationActionGroup(
        BodyAccelerationActionCfg(asset_name="robot", scale=0.05)
    )

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
            ),
            mass_props=MassPropertiesCfg(mass=100000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                ThrusterCfg(
                    offset=(0.0, 0.7726, 0.76),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.669, -0.3863, 0.76),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(0.669, -0.3863, 0.76),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.70384, 0.9837, 0.54),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(0.0, -1.96743, 0.54),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.70384, 0.9837, 0.54),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
            ),
            fuel_capacity=0.5 * 1200000.0,
            fuel_consumption_rate=(0.5 * 1200000.0 / (6 * 2300000.0)) / 20.0,
        )
    )

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
            ),
            mass_props=MassPropertiesCfg(mass=200000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(),
    )

    ## Actions
    actions: ActionGroup = ThrustActionGroup(
        ThrustActionCfg(
            asset_name="robot",
            thrusters=(
                # Gimbaled (13)
                ThrusterCfg(
                    offset=(0.750003, 0.0, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.375002, 0.649522, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.375002, -0.649522, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.78697, 0.478816, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.16425, 1.43772, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(0.096822, 1.84747, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.00758, 1.55154, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.72713, 0.662982, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.78697, -0.478816, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-1.16425, -1.43772, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(-0.096822, -1.84747, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.00758, -1.55154, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
                ),
                ThrusterCfg(
                    offset=(1.72713, -0.662982, 1.8),
                    power=2300000.0,
                    gimbal_limits=(deg_to_rad(15.0), deg_to_rad(15.0)),
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
    )

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
