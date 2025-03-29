import simforge_foundry

from srb.core.action import (
    ActionGroup,
    BodyAccelerationActionCfg,
    BodyAccelerationActionGroup,
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
from srb.utils.math import rpy_to_quat
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
