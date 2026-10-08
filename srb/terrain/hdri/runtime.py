"""Layer-B runtime: build an AssetBaseCfg referencing the baked dome USD."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb.core.asset import AssetBaseCfg


class RealSkydome:
    """Stateless namespace; ``build_asset_cfg(name)`` returns an AssetBaseCfg."""

    PRIM_PATH = "{ENV_REGEX_NS}/skydome"

    @staticmethod
    def build_asset_cfg(name: str) -> "AssetBaseCfg":
        from srb.core.asset import AssetBaseCfg
        from srb.core.sim import UsdFileCfg
        from srb.terrain.hdri.pipeline import bake_skydome

        path = bake_skydome(name)
        return AssetBaseCfg(
            prim_path=RealSkydome.PRIM_PATH,
            spawn=UsdFileCfg(usd_path=str(path)),
        )
