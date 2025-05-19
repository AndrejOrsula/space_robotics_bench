from srb.core.asset import AssetBaseCfg, Subterrane, Terrain
from srb.core.sim import CollisionPropertiesCfg, GridParticlesSpawnerCfg, UsdFileCfg
from srb.utils.path import SRB_ASSETS_DIR_SRB_SCENERY


class LunaLab(Subterrane):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/lunalab",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_SCENERY.joinpath("lunalab.usdc").as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-4.25, -5.5, 0.0)),
    )

    PARTICLE_SIZE: float = 0.025
    _regolith: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/regolith",
        spawn=GridParticlesSpawnerCfg(
            ratio=0.5,
            particle_size=PARTICLE_SIZE,
            dim_x=round(6.5 / PARTICLE_SIZE),
            dim_y=round(11.0 / PARTICLE_SIZE),
            dim_z=round(0.5 / PARTICLE_SIZE),
            velocity=((-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.0)),
            fluid=False,
            density=1500.0,
            friction=0.85,
            cohesion=0.65,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.3)),
    )


class Oberpfaffenhofen(Terrain):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/oberpfaffenhofen",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_SCENERY.joinpath(
                "oberpfaffenhofen_test_site.usdc"
            ).as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
    )
