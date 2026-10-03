from srb.core.asset import AssetBaseCfg, Light
from srb.core.sim import (
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_LIGHT


class ProjectorLight(Light):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/light_projector",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_LIGHT.joinpath("light_projector.usdc").as_posix()
            ),
        ),
    )
