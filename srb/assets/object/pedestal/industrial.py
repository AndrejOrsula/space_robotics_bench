from srb.core.asset import AssetBaseCfg, Frame, Pedestal, Transform
from srb.core.sim import CollisionPropertiesCfg, UsdFileCfg
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class IndustrialPedestal25(Pedestal):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/pedestal",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_OBJECT.joinpath(
                "industrial_pedestal_25cm.usdz"
            ).as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
    )

    frame_manipulator_mount: Frame = Frame(
        prim_relpath="pedestal", offset=Transform(pos=(0.0, 0.0, 0.25))
    )


class IndustrialPedestal50(Pedestal):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/pedestal",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_OBJECT.joinpath(
                "industrial_pedestal_50cm.usdz"
            ).as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
    )

    frame_manipulator_mount: Frame = Frame(
        prim_relpath="pedestal", offset=Transform(pos=(0.0, 0.0, 0.5))
    )


class IndustrialPedestal100(Pedestal):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/pedestal",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_OBJECT.joinpath(
                "industrial_pedestal_100cm.usdz"
            ).as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
    )

    frame_manipulator_mount: Frame = Frame(
        prim_relpath="pedestal", offset=Transform(pos=(0.0, 0.0, 1.0))
    )
