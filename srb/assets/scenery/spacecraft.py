from srb.core.asset import AssetBaseCfg, ExtravehicularScenery
from srb.core.sim import CollisionPropertiesCfg, UsdFileCfg
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class StaticGateway(ExtravehicularScenery):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/gateway",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("gateway.usdz")
            .as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
    )


class StaticVenusExpress(ExtravehicularScenery):
    ## Model
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/venus_express",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("spacecraft")
            .joinpath("venus_express.usdz")
            .as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-0.55, 0.0, -0.35), rot=rpy_to_quat(0.0, 0.0, 90.0)
        ),
    )
