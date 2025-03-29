from srb.core.asset import AssetBaseCfg, ExtravehicularScenery
from srb.core.sim import CollisionPropertiesCfg, UsdFileCfg
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class GatewayScenery(ExtravehicularScenery):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/gateway",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("gateway.usdz")
            .as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
    )
