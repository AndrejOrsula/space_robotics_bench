from srb.core.asset import AssetBaseCfg, Payload
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT, SRB_ASSETS_DIR_SRB_ROBOT


class CargoBay(Payload):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/cargo_bay",
        spawn=UsdFileCfg(
            usd_path=(SRB_ASSETS_DIR_SRB_OBJECT.joinpath("cargo_bay.usdz").as_posix()),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
    )


class HuskyPayloadLunalab(Payload):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/husky_payload_lunalab",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("rover")
            .joinpath("husky_payload_lunalab.usdc")
            .as_posix(),
            collision_props=CollisionPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=500.0),
        ),
    )
