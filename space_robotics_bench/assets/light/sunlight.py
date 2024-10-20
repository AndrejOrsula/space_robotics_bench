from typing import Any, Dict

import space_robotics_bench.core.envs as env_utils
import space_robotics_bench.core.sim as sim_utils
from space_robotics_bench.core.assets import AssetBaseCfg


def sunlight_from_env_cfg(
    env_cfg: env_utils.EnvironmentConfig,
    *,
    prim_path: str = "/World/light",
    spawn_kwargs: Dict[str, Any] = {},
    **kwargs,
) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path=prim_path,
        spawn=sim_utils.DistantLightCfg(
            intensity=env_cfg.scenario.light_intensity,
            angle=env_cfg.scenario.light_angular_diameter,
            color_temperature=env_cfg.scenario.light_color_temperature,
            enable_color_temperature=True,
            **spawn_kwargs,
        ),
        **kwargs,
    )
