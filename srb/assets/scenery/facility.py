from typing import TYPE_CHECKING, Tuple

import torch  # noqa: F401
from pydantic import PositiveFloat, PositiveInt
from simforge import BakeType

from srb.assets.object.rock import LunalabBoulder
from srb.assets.scenery.planetary_surface import MoonSurface
from srb.core.asset import RigidObjectCollectionCfg  # noqa: F401
from srb.core.asset import AssetBaseCfg, Object, Subterrane, Terrain
from srb.core.env import ViewerCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg  # noqa: F401
from srb.core.mdp import (  # noqa: F401
    reset_collection_root_state_uniform_poisson_disk_3d,
    reset_root_state_uniform_poisson_disk_3d,
)
from srb.core.sim import RigidBodyPropertiesCfg  # noqa: F401
from srb.core.sim import CollisionPropertiesCfg, GridParticlesSpawnerCfg, UsdFileCfg
from srb.utils.path import SRB_ASSETS_DIR_SRB_SCENERY
from srb.utils.sampling import sample_poisson_disk_3d

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
    bounder_count: PositiveInt = 4
    boulder: Object = LunalabBoulder()

    ## Basalt
    basalt_n_systems: PositiveInt = 5
    basalt_size: Tuple[PositiveFloat, PositiveFloat] = (0.002, 0.01)
    basalt_ratio: Tuple[PositiveFloat, PositiveFloat] = (0.1, 0.001)

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        scene = env_cfg.scene
        events = env_cfg.events

        ## Scene
        scene.env_spacing = 0.0 if env_cfg.stack else 12.5

        ## Viewer
        env_cfg.viewer = (  # type: ignore
            ViewerCfg(eye=(-4.0, -5.0, 1.5), lookat=(0.0, -1.0, 0.0), origin_type="env")
        )

        ## Terrain
        self.terrain.asset_cfg.prim_path = (
            "/World/lunalab_terrain"
            if env_cfg.stack
            else "{ENV_REGEX_NS}/lunalab_terrain"
        )
        scene.lunalab_terrain = self.terrain.as_asset_base_cfg()  # type: ignore

        ## Boulders ~ Static
        pos = sample_poisson_disk_3d(
            self.bounder_count,
            (
                (-3.0, -4.5, -0.1),
                (3.0, 4.5, 0.0),
            ),
            1.5,
        )
        for i in range(self.bounder_count):
            boulder = self.boulder.as_asset_base_cfg()
            boulder.prim_path = (
                f"/World/lunalab_boulder{i}"
                if env_cfg.stack
                else f"{{ENV_REGEX_NS}}/lunalab_boulder{i}"
            )
            boulder.init_state.pos = pos[i]
            setattr(scene, f"lunalab_boulder{i}", boulder)
        # events.randomize_lunalab_boulders = (  # type: ignore
        #     EventTermCfg(
        #         func=reset_root_state_uniform_poisson_disk_3d,
        #         mode="reset",
        #         params={
        #             "asset_cfg": [
        #                 SceneEntityCfg(f"lunalab_boulder{i}")
        #                 for i in range(self.bounder_count)
        #             ],
        #             "pose_range": {
        #                 "x": (-3.0, 3.0),
        #                 "y": (-4.5, 4.5),
        #                 "z": (-0.1, 0.0),
        #                 "roll": (-torch.pi, torch.pi),
        #                 "pitch": (-torch.pi, torch.pi),
        #                 "yaw": (-torch.pi, torch.pi),
        #             },
        #             "velocity_range": None,
        #             "radius": (1.5),
        #         },
        #     )
        # )

        # ## Boulders ~ Dynamic
        # rigid_objects = {}
        # for i in range(self.bounder_count):
        #     boulder = self.boulder.as_rigid_object_cfg()
        #     if isinstance(
        #         boulder.spawn.rigid_props,  # type: ignore
        #         RigidBodyPropertiesCfg,
        #     ):
        #         boulder.spawn.rigid_props.kinematic_enabled = True  # type: ignore
        #     else:
        #         boulder.spawn.rigid_props = RigidBodyPropertiesCfg(  # type: ignore
        #             kinematic_enabled=True
        #         )
        #     boulder.prim_path = (
        #         f"/World/lunalab_boulder{i}"
        #         if env_cfg.stack
        #         else f"{{ENV_REGEX_NS}}/lunalab_boulder{i}"
        #     )
        #     boulder.init_state.pos = (0.0, 0.0, -0.15)
        #     rigid_objects[f"boulder{i}"] = boulder
        # scene.lunalab_boulders = (  # type: ignore
        #     RigidObjectCollectionCfg(rigid_objects=rigid_objects)
        # )
        # events.randomize_lunalab_boulders = (  # type: ignore
        #     EventTermCfg(
        #         func=reset_collection_root_state_uniform_poisson_disk_3d,
        #         mode="reset",
        #         params={
        #             "asset_cfg": SceneEntityCfg("lunalab_boulders"),
        #             "pose_range": {
        #                 "x": (-3.0, 3.0),
        #                 "y": (-4.5, 4.5),
        #                 "z": (-0.1, 0.0),
        #                 "roll": (-torch.pi, torch.pi),
        #                 "pitch": (-torch.pi, torch.pi),
        #                 "yaw": (-torch.pi, torch.pi),
        #             },
        #             "velocity_range": None,
        #             "radius": (1.5),
        #         },
        #     )
        # )

        ## Basalt
        if env_cfg.particles:
            # Disable any existing particles
            env_cfg.scatter_particles = False
            if hasattr(scene, "particles"):
                scene.particles = None  # type: ignore

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
                dim_z = 1
                basalt = AssetBaseCfg(  # type: ignore
                    prim_path=f"{{ENV_REGEX_NS}}/basalt_{i}",
                    spawn=GridParticlesSpawnerCfg(
                        ratio=basalt_ratio,
                        particle_size=basalt_size,
                        dim_x=round(6.5 / basalt_size),
                        dim_y=round(10.0 / basalt_size),
                        dim_z=dim_z,
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
                spawn_height += dim_z * basalt_size

        ## Events
        if hasattr(events, "target_pos_evolution"):
            events.target_pos_evolution.params["pos_bounds"] = {  # type: ignore
                "hardcoded": True,
                "x": (-3.0, 3.0),
                "y": (-4.75, 4.75),
            }
            events.target_pos_evolution.params["step_range"] = (0.05, 0.25)  # type: ignore


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
