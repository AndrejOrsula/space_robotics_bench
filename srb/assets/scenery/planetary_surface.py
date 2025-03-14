from typing import ClassVar, Sequence

import simforge_foundry

from srb.core.asset import AssetBaseCfg, Terrain
from srb.core.domain import Domain
from srb.core.sim import CollisionPropertiesCfg, SimforgeAssetCfg


### ANCHOR: example (docs)
class MarsSurface(Terrain):
    ## Scenario - The asset is suitable for the Mars domain
    DOMAINS: ClassVar[Sequence[Domain]] = (Domain.MARS,)

    ## Model - Static asset
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/mars_surface",
        ## Spawner procedurally generates SimForge models
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.MarsSurface()],
            collision_props=CollisionPropertiesCfg(),
        ),
    )
    ### ANCHOR_END: example (docs)


class MoonSurface(Terrain):
    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = (Domain.MOON,)

    ## Model
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/moon_surface",
        spawn=SimforgeAssetCfg(
            assets=[simforge_foundry.MoonSurface()],
            collision_props=CollisionPropertiesCfg(),
        ),
    )
