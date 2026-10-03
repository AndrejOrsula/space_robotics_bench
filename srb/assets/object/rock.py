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


def _rock_physics() -> dict:
    """Shared collision and mass setup for every rock variant."""
    return {
        "collision_props": CollisionPropertiesCfg(),
        "mesh_collision_props": MeshCollisionPropertiesCfg(
            mesh_approximation="convexHull"
        ),
        "rigid_props": RigidBodyPropertiesCfg(),
        "mass_props": MassPropertiesCfg(density=2000.0),
    }


def _rock_usd_cfgs(stem: str, count: int) -> list[UsdFileCfg]:
    """Per-file spawn configs for ``rock/{stem}1.usdz`` .. ``rock/{stem}{count}.usdz``."""
    return [
        UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_OBJECT.joinpath("rock")
            .joinpath(f"{stem}{i}.usdz")
            .as_posix(),
            **_rock_physics(),
        )
        for i in range(1, count + 1)
    ]


class Asteroid(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/asteroid",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.Asteroid()],
            **_rock_physics(),
        ),
    )


class MoonRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.MoonRock()],
            **_rock_physics(),
        ),
    )


class MarsRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.MarsRock()],
            **_rock_physics(),
        ),
    )


class ApolloSample(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(assets_cfg=_rock_usd_cfgs("apollo_sample", 22)),
    )


class SpaceportMoonRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(assets_cfg=_rock_usd_cfgs("spaceport_moon_rock", 7)),
    )


class RandomRock(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(
            assets_cfg=[  # type: ignore[union-attr]
                Asteroid().asset_cfg.spawn,
                MoonRock().asset_cfg.spawn,
                MarsRock().asset_cfg.spawn,
                *ApolloSample().asset_cfg.spawn.assets_cfg,
                *SpaceportMoonRock().asset_cfg.spawn.assets_cfg,
            ],
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=2000.0),
        ),
    )


class LunalabBoulder(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/rock",
        spawn=MultiAssetSpawnerCfg(assets_cfg=_rock_usd_cfgs("lunalab_boulder", 4)),
    )
