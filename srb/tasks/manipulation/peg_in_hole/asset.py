from dataclasses import MISSING
from typing import TYPE_CHECKING, Any

from simforge import TexResConfig

from srb import assets
from srb.core.asset import AssetBaseCfg, AssetVariant, RigidObjectCfg
from srb.core.sim import SimforgeAssetCfg
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from .task import TaskCfg


# Dataset peg lengths from the shipped USD bounds; endpoint offsets use the tip.
PROFILE_PEG_LENGTH_M = 0.2
SHORT_PROFILE_PEG_LENGTH_M = 0.1


@configclass
class PegCfg:
    asset_cfg: RigidObjectCfg = MISSING
    offset_pos_ends: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
    ] = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    ## Rotational symmetry: 0 for circular, 1 for none, n for n-fold.
    symmetry: int = 1


@configclass
class HoleCfg:
    asset_cfg: AssetBaseCfg = MISSING
    offset_pos_bottom: tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_pos_entrance: tuple[float, float, float] = MISSING


@configclass
class PegInHoleCfg:
    peg: PegCfg = MISSING
    hole: HoleCfg = MISSING


def _procedural_peg_end_offsets(
    peg_cfg: RigidObjectCfg,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Fix procedural peg height and return its local end positions.

    ``PegGeo`` randomizes height, but task endpoint offsets are static; use the
    declared minimum for both bounds.
    """
    peg_geometry = peg_cfg.spawn.assets[0].ops[0]
    peg_length = peg_geometry.height_min
    peg_geometry.height_max = peg_length
    half_length = peg_length / 2.0
    return ((0.0, 0.0, -half_length), (0.0, 0.0, half_length))


def select_peg_in_hole_assembly(
    env_cfg: "TaskCfg",
    *,
    prim_path_peg: str = "{ENV_REGEX_NS}/peg",
    prim_path_hole: str = "{ENV_REGEX_NS}/hole",
    seed: int = 0,
    init_state: RigidObjectCfg.InitialStateCfg | None = None,
    peg_kwargs: dict[str, Any] | None = None,
    hole_kwargs: dict[str, Any] | None = None,
    scale: tuple[float, float, float] = (0.05, 0.05, 0.05),
    texture_resolution: TexResConfig | None = None,
    short_peg: bool = False,
    **kwargs,
) -> PegInHoleCfg:
    # Copy caller options to avoid mutating shared defaults.
    peg_kwargs = dict(peg_kwargs or {})
    hole_kwargs = dict(hole_kwargs or {})
    if init_state is None:
        init_state = RigidObjectCfg.InitialStateCfg()
    match env_cfg.peg:
        case AssetVariant.DATASET:
            peg_cfg = (
                assets.ShortProfilePeg() if short_peg else assets.ProfilePeg()
            ).asset_cfg
            hole_cfg = assets.ProfileHole().asset_cfg

            rot_symmetry_n = 4
            peg_length = (
                SHORT_PROFILE_PEG_LENGTH_M if short_peg else PROFILE_PEG_LENGTH_M
            )
            offset_pos_ends = ((0.0, 0.0, 0.0), (0.0, 0.0, peg_length))
            offset_pos_entrance = (0.0, 0.0, 0.02)

        case AssetVariant.PROCEDURAL:
            peg_cfg = assets.Peg(
                scale=scale, texture_resolution=texture_resolution
            ).asset_cfg
            hole_cfg = assets.Hole(
                scale=scale, texture_resolution=texture_resolution
            ).asset_cfg

            rot_symmetry_n = 1
            offset_pos_ends = _procedural_peg_end_offsets(peg_cfg)
            offset_pos_entrance = (0.0, 0.0, 0.0)

        case _:
            raise ValueError(
                f"Unsupported peg variant for peg-in-hole assembly: {env_cfg.peg!r}; "
                "expected AssetVariant.DATASET or AssetVariant.PROCEDURAL."
            )

    if isinstance(peg_cfg.spawn, SimforgeAssetCfg):
        peg_cfg.spawn.seed = seed
    if isinstance(hole_cfg.spawn, SimforgeAssetCfg):
        hole_cfg.spawn.seed = seed

    peg_cfg.prim_path = prim_path_peg
    peg_kwargs.update(**kwargs)
    peg_cfg.spawn = peg_cfg.spawn.replace(**peg_kwargs)  # type: ignore[union-attr]
    hole_cfg.prim_path = prim_path_hole
    hole_kwargs.update(**kwargs)
    hole_cfg.spawn = hole_cfg.spawn.replace(**hole_kwargs)  # type: ignore[union-attr]
    # Keep peg, hole, and caller initial states independent.
    peg_cfg.init_state = init_state.replace()
    hole_cfg.init_state = init_state.replace()

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
