import simforge_foundry

from srb.core.asset import Object, RigidObjectCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    PreviewSurfaceCfg,
    RigidBodyPropertiesCfg,
    SimforgeAssetCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class Peg(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/peg",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.PegGeo()],
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
    )


class Hole(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/hole",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.HoleGeo()],
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )


class ProfilePeg(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/peg",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("peg_in_hole")
                .joinpath("profile.usdc")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="boundingCube"
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1000.0),
            visual_material=PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6)),
        ),
    )


class ShortProfilePeg(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/peg",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("peg_in_hole")
                .joinpath("profile_short.usdc")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="boundingCube"
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1000.0),
            visual_material=PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6)),
        ),
    )


class ProfileHole(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/hole",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("peg_in_hole")
                .joinpath("hole.usdc")
                .as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(kinematic_enabled=True),
            visual_material=PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6)),
        ),
    )
