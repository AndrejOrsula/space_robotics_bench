from dataclasses import MISSING

from srb import assets
from srb.core.asset import RigidObjectCfg
from srb.utils.cfg import configclass


@configclass
class PanelCfg:
    asset_cfg: RigidObjectCfg = MISSING
    offset_pos: tuple[float, float, float] = (0.0, 0.0, 0.15)


def select_solar_panel(*, init_state: RigidObjectCfg.InitialStateCfg) -> PanelCfg:
    offset_pos = (0.0, 0.0, 0.15)
    init_state.pos = (
        init_state.pos[0] + offset_pos[0],
        init_state.pos[1] + offset_pos[1],
        init_state.pos[2] + offset_pos[2],
    )

    cfg = assets.SolarPanel().asset_cfg
    cfg.init_state = init_state

    return PanelCfg(
        asset_cfg=cfg,
        offset_pos=offset_pos,
    )
