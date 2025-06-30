from typing import TYPE_CHECKING, Tuple

from simforge import TexResConfig

from srb import assets
from srb.core.asset import RigidObjectCfg
from srb.core.sim import SimforgeAssetCfg

if TYPE_CHECKING:
    from .task import TaskCfg


def select_obstacle(
    env_cfg: "TaskCfg",
    *,
    prim_path: str = "{ENV_REGEX_NS}/obstacle",
    seed: int = 0,
    init_state: RigidObjectCfg.InitialStateCfg = RigidObjectCfg.InitialStateCfg(),
    scale: Tuple[float, float, float] = (5.0, 5.0, 5.0),
    texture_resolution: TexResConfig | None = None,
    **kwargs,
) -> RigidObjectCfg:
    obstacle_cfg = assets.Asteroid(
        scale=scale, texture_resolution=texture_resolution
    ).asset_cfg

    if isinstance(obstacle_cfg.spawn, SimforgeAssetCfg):
        obstacle_cfg.spawn.seed = seed

    obstacle_cfg.prim_path = prim_path
    obstacle_cfg.init_state = init_state
    obstacle_cfg.spawn.replace(**kwargs)  # type: ignore

    obstacle_cfg.spawn.assets[0].geo.ops[0].scale_std = (  # type: ignore
        0.2 * scale[0],
        0.2 * scale[1],
        0.2 * scale[2],
    )

    return obstacle_cfg
