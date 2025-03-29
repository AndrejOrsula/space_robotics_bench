from srb.core.asset import Object, RigidObjectCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class SolarPanel(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/panel",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("solar_panel.usdz").as_posix()
            ),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="sdf"),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=500.0),
        ),
    )
