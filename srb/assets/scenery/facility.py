import math
from typing import TYPE_CHECKING, Tuple

import torch
from pydantic import PositiveFloat, PositiveInt
from simforge import BakeType

from srb.assets.object.rock import LunalabBoulder
from srb.assets.scenery.planetary_surface import MoonSurface
from srb.core.asset import AssetBaseCfg, Object, Subterrane, Terrain
from srb.core.env import ViewerCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_xforms_uniform_poisson_disk_2d
from srb.core.sim import CollisionPropertiesCfg, GridParticlesSpawnerCfg, UsdFileCfg
from srb.utils.path import SRB_ASSETS_DIR_SRB_SCENERY

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


class Lunalab(Subterrane):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/lunalab",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_SCENERY.joinpath("lunalab.usdc").as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-4.25, -6.0, 0.0)),
    )

    ## Terrain
    terrain: Terrain = MoonSurface(
        scale=(7.5, 11.00, 0.4),
        density=0.1,
        texture_resolution={
            BakeType.ALBEDO: 2048,
            BakeType.NORMAL: 4096,
            BakeType.ROUGHNESS: 1024,
        },
    )
    terrain.asset_cfg.prim_path = "{ENV_REGEX_NS}/lunalab_terrain"
    terrain.asset_cfg.init_state.pos = (-0.5, -0.5, -0.2)

    ## Boulders
    boulder: Object = LunalabBoulder()
    boulder_count: PositiveInt = 4

    ## Basalt
    basalt_n_systems: PositiveInt = 3
    basalt_size: Tuple[PositiveFloat, PositiveFloat] = (0.002, 0.01)
    basalt_ratio: Tuple[PositiveFloat, PositiveFloat] = (0.1, 0.001)
    basalt_height: Tuple[PositiveInt, PositiveInt] = (1, 1)

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        scene = env_cfg.scene
        events = env_cfg.events

        ## Scene
        scene.env_spacing = 0.0 if env_cfg.stack else 12.5

        ## Viewer
        env_cfg.viewer = (  # type: ignore
            ViewerCfg(eye=(-4.0, -5.0, 1.5), lookat=(0.0, -1.0, 0.0), origin_type="env")
        )

        ## Light
        scene.sunlight = None
        events.randomize_sunlight_orientation = None
        events.randomize_sunlight_intensity = None
        events.randomize_sunlight_angular_diameter = None
        events.randomize_sunlight_color_temperature = None

        ## Skydome
        scene.skydome = None
        events.randomize_skydome_orientation = None

        ## Terrain
        self.terrain.asset_cfg.prim_path = (
            "/World/lunalab_terrain"
            if env_cfg.stack
            else "{ENV_REGEX_NS}/lunalab_terrain"
        )
        scene.lunalab_terrain = self.terrain.as_asset_base_cfg()  # type: ignore

        ## Boulders
        if self.boulder_count > 0:
            for i in range(self.boulder_count):
                boulder = self.boulder.as_asset_base_cfg()
                boulder.prim_path = (
                    f"/World/lunalab_boulder{i}"
                    if env_cfg.stack
                    else f"{{ENV_REGEX_NS}}/lunalab_boulder{i}"
                )
                setattr(scene, f"lunalab_boulder{i}", boulder)
            events.randomize_lunalab_boulders = (  # type: ignore
                EventTermCfg(
                    func=reset_xforms_uniform_poisson_disk_2d,
                    mode="reset",
                    params={
                        "asset_cfg": [
                            SceneEntityCfg(f"lunalab_boulder{i}")
                            for i in range(self.boulder_count)
                        ],
                        "pose_range": {
                            "x": (-3.0, 3.0),
                            "y": (-4.5, 4.5),
                            "z": (-0.1, 0.0),
                            "roll": (-torch.pi, torch.pi),
                            "pitch": (-torch.pi, torch.pi),
                            "yaw": (-torch.pi, torch.pi),
                        },
                        "radius": (1.5),
                    },
                )
            )

        ## Basalt
        if env_cfg.particles and self.basalt_n_systems:
            # Disable default particles
            scene.particles = None  # type: ignore

            # Spawn multiple regolith layers with different granularity
            spawn_height = 0.1
            min_id = 0 if self.basalt_size[0] <= self.basalt_size[1] else 1
            max_id = (min_id + 1) % 2
            for i in range(self.basalt_n_systems):
                f = (self.basalt_n_systems - i) / self.basalt_n_systems
                basalt_size = (
                    f * (self.basalt_size[max_id] - self.basalt_size[min_id])
                    + self.basalt_size[min_id]
                )
                basalt_ratio = (
                    f * (self.basalt_ratio[max_id] - self.basalt_ratio[min_id])
                    + self.basalt_ratio[min_id]
                )
                basalt_height = math.floor(
                    f * (self.basalt_height[max_id] - self.basalt_height[min_id])
                    + self.basalt_height[min_id]
                )
                basalt = AssetBaseCfg(  # type: ignore
                    prim_path=f"{{ENV_REGEX_NS}}/basalt_{i}",
                    spawn=GridParticlesSpawnerCfg(
                        ratio=basalt_ratio,
                        particle_size=basalt_size,
                        dim_x=round(6.5 / basalt_size),
                        dim_y=round(10.0 / basalt_size),
                        dim_z=basalt_height,
                        velocity=((-0.1, 0.1), (-0.1, 0.1), (-0.05, 0.0)),
                        fluid=False,
                        density=1500.0,
                        friction=0.85,
                        cohesion=0.65,
                    ),
                    init_state=AssetBaseCfg.InitialStateCfg(
                        pos=(0.0, 0.0, spawn_height)
                    ),
                )
                setattr(scene, f"basalt_{i}", basalt)
                spawn_height += basalt_size * basalt_height

        ## Events | task: waypoint_navigation
        if hasattr(events, "target_pose_evolution"):
            events.target_pose_evolution.params["pos_bounds"] = {  # type: ignore
                "hardcoded": True,
                "x": (-3.0, 3.0),
                "y": (-4.75, 4.75),
            }
        elif hasattr(events, "target_pos_evolution"):
            events.target_pos_evolution.params["pos_bounds"] = {  # type: ignore
                "hardcoded": True,
                "x": (-3.0, 3.0),
                "y": (-4.75, 4.75),
            }


class Oberpfaffenhofen(Terrain):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/oberpfaffenhofen",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_SCENERY.joinpath(
                "oberpfaffenhofen_test_site.usdc"
            ).as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
    )
