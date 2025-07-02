from srb.assets.robot.mobile.spacecraft import Gateway, VenusExpress
from srb.core.asset import AssetBaseCfg, ExtravehicularScenery
from srb.utils.math import rpy_to_quat


class StaticGateway(ExtravehicularScenery):
    asset_cfg: AssetBaseCfg = Gateway().as_asset_base_cfg()


class StaticVenusExpress(ExtravehicularScenery):
    asset_cfg: AssetBaseCfg = VenusExpress().as_asset_base_cfg()
    asset_cfg.init_state = AssetBaseCfg.InitialStateCfg(
        pos=(-0.55, 0.0, -0.35), rot=rpy_to_quat(0.0, 0.0, 90.0)
    )
