"""Layer-B runtime: build an AssetBaseCfg referencing the baked lights USD."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb.core.asset import AssetBaseCfg


class RealLighting:
    """Stateless lighting asset config factory."""

    PRIM_PATH = "{ENV_REGEX_NS}/lighting"

    @staticmethod
    def build_asset_cfg(preset_name: str) -> "AssetBaseCfg":
        from srb.core.asset import AssetBaseCfg
        from srb.core.sim import UsdFileCfg
        from srb.terrain.lights.pipeline import bake_lighting

        path = bake_lighting(preset_name)
        return AssetBaseCfg(
            prim_path=RealLighting.PRIM_PATH,
            spawn=UsdFileCfg(usd_path=str(path)),
        )
