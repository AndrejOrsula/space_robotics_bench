from typing import TYPE_CHECKING, Any, Dict, Tuple

from pydantic import BaseModel, NonNegativeInt
from simforge import TexResConfig

from srb import assets
from srb.core.asset import AssetVariant, RigidObjectCfg
from srb.core.sim import SimforgeAssetCfg

if TYPE_CHECKING:
    from .task import TaskCfg


class PegCfg(BaseModel, arbitrary_types_allowed=True):
    asset_cfg: RigidObjectCfg
    offset_pos_ends: Tuple[
        Tuple[float, float, float],
        Tuple[float, float, float],
    ]

    ## Rotational symmetry of the peg represented as integer
    #  0: Circle (infinite symmetry)
    #  1: No symmetry (exactly one fit)
    #  n: n-fold symmetry (360/n deg between each symmetry)
    symmetry: NonNegativeInt = 1


class HoleCfg(BaseModel):
    asset_cfg: RigidObjectCfg
    offset_pos_bottom: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_pos_entrance: Tuple[float, float, float]


class PegInHoleCfg(BaseModel):
    peg: PegCfg
    hole: HoleCfg


def select_peg_in_hole_assembly(
    env_cfg: "TaskCfg",
    *,
    prim_path_peg: str = "{ENV_REGEX_NS}/peg",
    prim_path_hole: str = "{ENV_REGEX_NS}/hole",
    seed: int = 0,
    init_state: RigidObjectCfg.InitialStateCfg = RigidObjectCfg.InitialStateCfg(),
    peg_kwargs: Dict[str, Any] = {},
    hole_kwargs: Dict[str, Any] = {},
    scale: Tuple[float, float, float] = (0.05, 0.05, 0.05),
    texture_resolution: TexResConfig | None = None,
    short_peg: bool = False,
    **kwargs,
) -> PegInHoleCfg:
    match env_cfg.peg:
        case AssetVariant.DATASET:
            peg_cfg = (
                assets.ShortProfilePeg() if short_peg else assets.ProfilePeg()
            ).asset_cfg
            hole_cfg = assets.ProfileHole().asset_cfg

            rot_symmetry_n = 4
            offset_pos_ends = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2))
            offset_pos_entrance = (0.0, 0.0, 0.02)

        case AssetVariant.PROCEDURAL:
            peg_cfg = assets.Peg(
                scale=scale, texture_resolution=texture_resolution
            ).asset_cfg
            hole_cfg = assets.Hole(
                scale=scale, texture_resolution=texture_resolution
            ).asset_cfg

            rot_symmetry_n = 1
            offset_pos_ends = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            offset_pos_entrance = (0.0, 0.0, 0.0)

    if isinstance(peg_cfg.spawn, SimforgeAssetCfg):
        peg_cfg.spawn.seed = seed
    if isinstance(hole_cfg.spawn, SimforgeAssetCfg):
        hole_cfg.spawn.seed = seed

    peg_cfg.prim_path = prim_path_peg
    peg_kwargs.update(**kwargs)
    peg_cfg.spawn.replace(**peg_kwargs)  # type: ignore

    hole_cfg.prim_path = prim_path_hole
    hole_kwargs.update(**kwargs)
    hole_cfg.spawn.replace(**hole_kwargs)  # type: ignore

    peg_cfg.init_state = init_state
    hole_cfg.init_state = init_state

    return PegInHoleCfg(
        peg=PegCfg(
            asset_cfg=peg_cfg,
            offset_pos_ends=offset_pos_ends,
            symmetry=rot_symmetry_n,
        ),
        hole=HoleCfg(
            asset_cfg=hole_cfg,
            offset_pos_entrance=offset_pos_entrance,
        ),
    )
