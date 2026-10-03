from __future__ import annotations

from typing import ClassVar, Sequence

from srb.core.asset import AssetBaseCfg, TerrainImporterCfg
from srb.core.asset.scenery.scenery import Scenery
from srb.core.asset.scenery.scenery_type import SceneryType
from srb.core.domain import Domain


class Terrain(
    Scenery, scenery_entrypoint=SceneryType.TERRAIN, arbitrary_types_allowed=True
):
    ## Model
    asset_cfg: AssetBaseCfg | TerrainImporterCfg

    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = (
        Domain.ASTEROID,
        Domain.EARTH,
        Domain.MARS,
        Domain.MOON,
    )
