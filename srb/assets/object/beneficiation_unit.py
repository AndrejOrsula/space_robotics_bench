from srb.core.asset import Object, RigidObjectCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class BeneficiationUnit(Object):
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/beneficiation_unit",
        spawn=UsdFileCfg(
            usd_path=(SRB_ASSETS_DIR_SRB_OBJECT.joinpath("PRIVATE").as_posix()),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=1000.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(1.0, 3.05, 0.967),
        ),
    )
