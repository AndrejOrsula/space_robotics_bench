from typing import TYPE_CHECKING, Tuple

from srb import assets
from srb.core.asset import AssetVariant, RigidObjectCfg

if TYPE_CHECKING:
    from .task import TaskCfg


def select_debris(
    env_cfg: "TaskCfg",
    *,
    prim_path: str = "{ENV_REGEX_NS}/debris",
    scale: Tuple[float, float, float] = (0.05, 0.05, 0.05),
    init_state: RigidObjectCfg.InitialStateCfg = RigidObjectCfg.InitialStateCfg(),
    **kwargs,
) -> RigidObjectCfg:
    match env_cfg.debris:
        case AssetVariant.PRIMITIVE:
            debris_cfg = assets.RandomShape(scale=scale).asset_cfg

        case AssetVariant.DATASET:
            debris_cfg = assets.ProfilePeg().asset_cfg

        case AssetVariant.PROCEDURAL:
            debris_cfg = assets.Cubesat(scale=scale).asset_cfg

    debris_cfg.prim_path = prim_path
    debris_cfg.init_state = init_state
    debris_cfg.spawn.replace(**kwargs)  # type: ignore

    return debris_cfg
