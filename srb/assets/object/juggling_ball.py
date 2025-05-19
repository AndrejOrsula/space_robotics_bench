from srb.core.asset import DeformableObjectCfg, Object
from srb.core.sim import DeformableBodyPropertiesCfg, MassPropertiesCfg, UsdFileCfg
from srb.utils.path import SRB_ASSETS_DIR_SRB_OBJECT


class JugglingBall(Object):
    asset_cfg: DeformableObjectCfg = DeformableObjectCfg(
        prim_path="{ENV_REGEX_NS}/juggling_ball",
        spawn=UsdFileCfg(
            usd_path=(
                SRB_ASSETS_DIR_SRB_OBJECT.joinpath("juggling_ball.usdc").as_posix()
            ),
            mass_props=MassPropertiesCfg(density=1000.0),
            deformable_props=DeformableBodyPropertiesCfg(
                rest_offset=0.0, contact_offset=0.001
            ),
            # physics_material=DeformableBodyMaterialCfg(
            #     poissons_ratio=0.4, youngs_modulus=1e5
            # ),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
        debug_vis=True,
    )
