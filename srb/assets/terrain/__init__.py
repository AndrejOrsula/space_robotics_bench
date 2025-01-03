from .ground_plane import *  # noqa: F403
from .planetary_surface import *  # noqa: F403

# import math
# from typing import Any, Dict, Tuple
# import srb.core.envs as env_utils
# import srb.core.sim as sim_utils
# from srb.core.asset import AssetBaseCfg

# def terrain_from_env_cfg(
#     env_cfg: env_utils.EnvironmentConfig,
#     *,
#     size: Tuple[float, float] = (10.0, 10.0),
#     num_assets: int = 1,
#     prim_path: str = "{ENV_REGEX_NS}/terrain",
#     spawn_kwargs: Dict[str, Any] = {},
#     procgen_kwargs: Dict[str, Any] = {},
#     **kwargs,
# ) -> AssetBaseCfg | None:
#     spawn = None

#     match env_cfg.assets.terrain.variant:
#         case env_utils.AssetVariant.PRIMITIVE:
#             prim_path = "/World/terrain"
#             size = (
#                 10 * math.sqrt(num_assets) * size[0],
#                 10 * math.sqrt(num_assets) * size[1],
#             )
#             spawn = sim_utils.GroundPlaneCfg(
#                 size=size, color=(0.0, 158.0 / 255.0, 218.0 / 255.0), **spawn_kwargs
#             )

#         case env_utils.AssetVariant.PROCEDURAL:
#             if spawn_kwargs.get("collision_props") is None:
#                 spawn_kwargs["collision_props"] = sim_utils.CollisionPropertiesCfg()
#             usd_file_cfg = sim_utils.UsdFileCfg(
#                 usd_path="IGNORED",
#                 **spawn_kwargs,
#             )

#             match env_cfg.scenario:
#                 case env_utils.Scenario.MOON:
#                     spawn = LunarSurfaceCfg(
#                         num_assets=num_assets, seed=env_cfg.seed, **spawn_kwargs
#                     )

#                 # case env_utils.Scenario.MARS:
#                 #     spawn = MartianSurfaceProcgenCfg(
#                 #         num_assets=num_assets,
#                 #         usd_file_cfg=usd_file_cfg,
#                 #         seed=env_cfg.seed,
#                 #         detail=env_cfg.detail,
#                 #     )
#                 case _:
#                     return None

#             # Fix this setting
#             # # Set height to 10% of the average planar size
#             # scale = (*size, (size[0] + size[1]) / 20.0)
#             # for node_cfg in spawn.geometry_nodes.values():
#             #     if node_cfg.get("scale") is not None:
#             #         node_cfg["scale"] = scale

#             # for key, value in procgen_kwargs.items():
#             #     for node_cfg in spawn.geometry_nodes.values():
#             #         if node_cfg.get(key) is not None:
#             #             node_cfg[key] = value
#             #         elif hasattr(spawn, key):
#             #             setattr(spawn, key, value)

#     if spawn is None:
#         return None
#     return AssetBaseCfg(
#         prim_path=prim_path,
#         spawn=spawn,
#         **kwargs,
#     )