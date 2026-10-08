"""Layer-B runtime: build per-rock AssetBaseCfg list."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb.core.asset import AssetBaseCfg


class RealRockSet:
    """Stateless namespace; ``build_asset_cfgs(set_name)`` returns list."""

    PRIM_PATH_TEMPLATE = "{{ENV_REGEX_NS}}/rocks/{idx:02d}"

    @staticmethod
    def build_asset_cfgs(set_name: str) -> "list[AssetBaseCfg]":
        from srb.core.asset import AssetBaseCfg
        from srb.core.sim import UsdFileCfg
        from srb.terrain.rocks.pipeline import bake_rock_set

        paths = bake_rock_set(set_name)
        return [
            AssetBaseCfg(
                prim_path=RealRockSet.PRIM_PATH_TEMPLATE.format(idx=i),
                spawn=UsdFileCfg(usd_path=str(p)),
            )
            for i, p in enumerate(paths)
        ]
