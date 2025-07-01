import simforge_foundry

from srb.core.asset import Object, RigidObjectCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    MultiAssetSpawnerCfg,
    RigidBodyPropertiesCfg,
    SimforgeAssetCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class Asteroid(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/asteroid",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.Asteroid()],
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexHull"
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=2000.0),
        ),
    )


class MoonRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.MoonRock()],
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexHull"
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=2000.0),
        ),
    )


class MarsRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.MarsRock()],
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexHull"
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=2000.0),
        ),
    )


class ApolloSample(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(
            assets_cfg=[
                UsdFileCfg(
                    usd_path=(
                        SRB_ASSETS_DIR_SRB_OBJECT.joinpath("rock")
                        .joinpath(f"apollo_sample{i}.usdz")
                        .as_posix()
                    ),
                    collision_props=CollisionPropertiesCfg(),
                    mesh_collision_props=MeshCollisionPropertiesCfg(
                        mesh_approximation="convexHull"
                    ),
                    rigid_props=RigidBodyPropertiesCfg(),
                    mass_props=MassPropertiesCfg(density=2000.0),
                )
                for i in range(1, 23)
            ],
        ),
    )


class SpaceportMoonRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(
            assets_cfg=[
                UsdFileCfg(
                    usd_path=(
                        SRB_ASSETS_DIR_SRB_OBJECT.joinpath("rock")
                        .joinpath(f"spaceport_moon_rock{i}.usdz")
                        .as_posix()
                    ),
                    collision_props=CollisionPropertiesCfg(),
                    mesh_collision_props=MeshCollisionPropertiesCfg(
                        mesh_approximation="convexHull"
                    ),
                    rigid_props=RigidBodyPropertiesCfg(),
                    mass_props=MassPropertiesCfg(density=2000.0),
                )
                for i in range(1, 8)
            ],
        ),
    )


class RandomRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(
            assets_cfg=[
                Asteroid().asset_cfg.spawn,
                MoonRock().asset_cfg.spawn,
                MarsRock().asset_cfg.spawn,
                *ApolloSample().asset_cfg.spawn.assets_cfg,  # type: ignore
                *SpaceportMoonRock().asset_cfg.spawn.assets_cfg,  # type: ignore
            ],
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=2000.0),
        ),
    )


class LunalabBoulder(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(
            assets_cfg=[
                UsdFileCfg(
                    usd_path=(
                        SRB_ASSETS_DIR_SRB_OBJECT.joinpath("rock")
                        .joinpath(f"lunalab_boulder{i}.usdz")
                        .as_posix()
                    ),
                    collision_props=CollisionPropertiesCfg(),
                    mesh_collision_props=MeshCollisionPropertiesCfg(
                        mesh_approximation="convexHull"
                    ),
                    rigid_props=RigidBodyPropertiesCfg(),
                    mass_props=MassPropertiesCfg(density=2000.0),
                )
                for i in range(1, 5)
            ],
        ),
    )
