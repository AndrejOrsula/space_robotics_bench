from srb.core.asset import Frame, Object, RigidObjectCfg, Transform
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class BoltM8(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/peg",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("bolt_and_nut")
                .joinpath("bolt_hex_m8.usdz")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="sdf",
                sdf_resolution=512,
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=0.1),
        ),
    )

    ## Frames
    frame_driver_slot: Frame = Frame(offset=Transform(pos=(0.0, 0.0, 0.01905)))


class NutM8(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/hole",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("bolt_and_nut")
                .joinpath("nut_hex_m8.usdz")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="sdf",
                sdf_resolution=512,
            ),
            # TODO[high]: Do not use kinematic rigid objects
            rigid_props=RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=MassPropertiesCfg(mass=0.05),
        ),
    )
