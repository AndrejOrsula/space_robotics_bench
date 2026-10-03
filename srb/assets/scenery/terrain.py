import math
from dataclasses import replace
from typing import TYPE_CHECKING, ClassVar, Literal, Sequence

import simforge_foundry
import torch
from simforge_foundry.geometry.terrain.moon_surface import MoonSurfaceNodes

from srb.assets.object import SpaceportMoonRock
from srb.assets.scenery.procedural_terrain import (
    build_default_terrain_generator_asset_cfg,
)
from srb.assets.scenery.real_terrain import build_real_terrain_asset_cfg
from srb.core.asset import AssetBaseCfg, Object, Terrain
from srb.core.domain import Domain
from srb.core.particle_layout import particle_pitch_m
from srb.core.sim import (  # noqa: F401
    CollisionPropertiesCfg,
    GridParticlesSpawnerCfg,
    GroundPlaneCfg,
    MeshCollisionPropertiesCfg,
    PlaneCfg,
    SimforgeAssetCfg,
    TerrainImporterCfg,
    UsdFileCfg,
)
from srb.terrain.surface_source import resolve_surface_asset_cfg
from srb.utils.math import degrees, quat_apply, quat_mul, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_SCENERY

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


class GroundPlane(Terrain):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/ground_plane",
        spawn=GroundPlaneCfg(
            color=(0.0, 158.0 / 255.0, 218.0 / 255.0),
        ),
    )


class Oberpfaffenhofen(Terrain):
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/oberpfaffenhofen",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_SCENERY.joinpath(
                "oberpfaffenhofen_test_site.usdc"
            ).as_posix(),
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="none"),
        ),
    )


### ANCHOR: example (docs)
class MarsSurface(Terrain):
    ## Scenario - The asset is suitable for the Mars domain
    DOMAINS: ClassVar[Sequence[Domain]] = (Domain.MARS,)

    ## Source selector — opt in to real DEMs by passing `source="real"`.
    source: Literal["procedural", "real"] = "procedural"
    site: str = "jezero"

    ## Model - filled in model_post_init
    asset_cfg: AssetBaseCfg | None = None  # type: ignore[assignment]

    def _build_procedural_asset_cfg(self) -> AssetBaseCfg:
        return AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/mars_surface",
            ## Spawner procedurally generates SimForge models
            spawn=SimforgeAssetCfg(
                assets=[simforge_foundry.MarsSurface()],
                collision_props=CollisionPropertiesCfg(),
                # Static triangle-mesh collision is required; otherwise
                # particles and objects fall through the SimForge mesh.
                mesh_collision_props=MeshCollisionPropertiesCfg(
                    mesh_approximation="none"
                ),
            ),
        )

    def model_post_init(self, __context):
        # Build first so the parent forwards density, scale, and texture
        # settings to the newly constructed spawner config.
        self.asset_cfg = resolve_surface_asset_cfg(
            surface_name=self.__class__.__name__,
            body="mars",
            site=self.site,
            source=self.source,
            build_procedural_asset_cfg=self._build_procedural_asset_cfg,
            build_real_asset_cfg=build_real_terrain_asset_cfg,
        )
        super().model_post_init(__context)

    ### ANCHOR_END: example (docs)


class MoonSurface(Terrain):
    ## Scenario
    DOMAINS: ClassVar[Sequence[Domain]] = (Domain.MOON,)

    ## Model
    source: Literal["procedural", "real"] = "procedural"
    site: str = "apollo17"

    asset_cfg: AssetBaseCfg | None = None  # type: ignore[assignment]

    def _build_procedural_asset_cfg(self) -> AssetBaseCfg:
        node_kwargs = {}
        if self.scale is not None:
            node_kwargs["scale"] = self.scale
        if self.density is not None:
            node_kwargs["density"] = self.density
        if self.flat_area_size is not None:
            node_kwargs["flat_area_size"] = self.flat_area_size

        model_kwargs = {}
        if self.texture_resolution is not None:
            model_kwargs["texture_resolution"] = self.texture_resolution

        return AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/moon_surface",
            spawn=SimforgeAssetCfg(
                assets=[
                    simforge_foundry.MoonSurface(
                        geo=simforge_foundry.MoonSurfaceGeo(
                            ops=[MoonSurfaceNodes(**node_kwargs)]
                        ),
                        **model_kwargs,
                    )
                ],
                collision_props=CollisionPropertiesCfg(),
                mesh_collision_props=MeshCollisionPropertiesCfg(
                    mesh_approximation="none"
                ),
            ),
        )

    def model_post_init(self, __context):
        # Build before the parent forwards spawner settings, as in MarsSurface.
        self.asset_cfg = resolve_surface_asset_cfg(
            surface_name=self.__class__.__name__,
            body="moon",
            site=self.site,
            source=self.source,
            build_procedural_asset_cfg=self._build_procedural_asset_cfg,
            build_real_asset_cfg=build_real_terrain_asset_cfg,
        )
        super().model_post_init(__context)


class TerrainGenerator(Terrain):
    ## Model
    asset_cfg: TerrainImporterCfg = build_default_terrain_generator_asset_cfg()


class Plane(Terrain):
    ## Surface
    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/terrain",
        spawn=PlaneCfg(
            size=(0.75, 2.0),
            collision_props=CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )

    ## Walls
    wall_height: float = 1.0
    walls: Sequence[AssetBaseCfg] = [
        AssetBaseCfg(
            prim_path=f"{{ENV_REGEX_NS}}/wall{i}",
            spawn=PlaneCfg(collision_props=CollisionPropertiesCfg(), visible=False),
            init_state=AssetBaseCfg.InitialStateCfg(),
        )
        for i in range(4)
    ]

    ## Obstacles
    obstacle: Object = SpaceportMoonRock()
    obstacle_count: int = 0

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        scene = env_cfg.scene

        ## Scene
        scene.env_spacing = (
            0.0
            if hasattr(env_cfg, "stack") and env_cfg.stack
            else max(
                self.asset_cfg.spawn.size[0],  # type: ignore[attr-defined]
                self.asset_cfg.spawn.size[1],  # type: ignore[attr-defined]
            )
            + 2.0
        )

        ## Viewer
        env_cfg.viewer = (  # type: ignore[misc]
            replace(
                env_cfg.viewer,
                eye=(
                    -self.asset_cfg.spawn.size[0],  # type: ignore[attr-defined]
                    -self.asset_cfg.spawn.size[1],  # type: ignore[attr-defined]
                    max(self.asset_cfg.spawn.size[0], self.asset_cfg.spawn.size[1])  # type: ignore[attr-defined]
                    * 0.8,
                ),
                lookat=self.asset_cfg.init_state.pos,
                origin_type="env",
            )
        )

        ## Walls
        pos_wall_local = [
            torch.tensor(
                (
                    0.0,
                    self.asset_cfg.spawn.size[1],  # type: ignore[attr-defined]
                    self.wall_height / 2,
                )
            ),
            torch.tensor(
                (
                    0.0,
                    -self.asset_cfg.spawn.size[1],  # type: ignore[attr-defined]
                    self.wall_height / 2,
                )
            ),
            torch.tensor(
                (
                    -self.asset_cfg.spawn.size[0],  # type: ignore[attr-defined]
                    0.0,
                    self.wall_height / 2,
                )
            ),
            torch.tensor(
                (
                    self.asset_cfg.spawn.size[0],  # type: ignore[attr-defined]
                    0.0,
                    self.wall_height / 2,
                )
            ),
        ]
        quat_wall_local = [
            torch.tensor(rpy_to_quat(90.0, 0, 0)),
            torch.tensor(rpy_to_quat(-90.0, 0, 0)),
            torch.tensor(rpy_to_quat(90.0, 0, 90.0)),
            torch.tensor(rpy_to_quat(-90.0, 0, 90.0)),
        ]
        # Plane sizes are half-extents, like the terrain size above.
        wall_sizes = [
            (
                self.asset_cfg.spawn.size[0],  # type: ignore[attr-defined]
                self.wall_height / 2,
            ),
            (
                self.asset_cfg.spawn.size[0],  # type: ignore[attr-defined]
                self.wall_height / 2,
            ),
            (
                self.asset_cfg.spawn.size[1],  # type: ignore[attr-defined]
                self.wall_height / 2,
            ),
            (
                self.asset_cfg.spawn.size[1],  # type: ignore[attr-defined]
                self.wall_height / 2,
            ),
        ]
        pos_slope_world = torch.tensor(
            self.asset_cfg.init_state.pos, dtype=torch.float32
        )
        quat_slope_world = torch.tensor(
            self.asset_cfg.init_state.rot, dtype=torch.float32
        )
        for i in range(4):
            self.walls[i].spawn.size = wall_sizes[i]  # type: ignore[union-attr]
            p_wall_world = pos_slope_world + quat_apply(
                quat_slope_world, pos_wall_local[i]
            )
            self.walls[i].init_state.pos = tuple(p_wall_world.tolist())
            q_wall_world = quat_mul(quat_slope_world, quat_wall_local[i])
            self.walls[i].init_state.rot = tuple(q_wall_world.tolist())
            setattr(scene, f"wall_{i}", self.walls[i])

        ## Obstacles
        # Config setup runs before the env seeds Torch; seed placement explicitly.
        rng = torch.Generator()
        if getattr(env_cfg, "seed", None) is not None:
            rng.manual_seed(int(env_cfg.seed))
        else:
            rng.seed()
        for i in range(self.obstacle_count):
            obstacle = self.obstacle.as_asset_base_cfg()
            obstacle.prim_path = (
                f"/World/obstacle{i}"
                if hasattr(env_cfg, "stack") and env_cfg.stack
                else f"{{ENV_REGEX_NS}}/obstacle{i}"
            )
            rand_x = (
                2
                * (torch.rand(1, generator=rng).item() - 0.5)
                * self.asset_cfg.spawn.size[0]
            )  # type: ignore[attr-defined]
            rand_y = (
                2
                * (torch.rand(1, generator=rng).item() - 0.5)
                * self.asset_cfg.spawn.size[1]
            )  # type: ignore[attr-defined]
            p_obs_local = torch.tensor([rand_x, rand_y, 0.0], dtype=torch.float32)
            rand_yaw_rad = (torch.rand(1, generator=rng).item() - 0.5) * 2 * torch.pi
            q_obs_local = torch.tensor(
                rpy_to_quat(0.0, 0.0, degrees(rand_yaw_rad)), dtype=torch.float32
            )
            p_obs_world = pos_slope_world + quat_apply(quat_slope_world, p_obs_local)
            q_obs_world = quat_mul(quat_slope_world, q_obs_local)
            obstacle.init_state.pos = tuple(p_obs_world.tolist())
            obstacle.init_state.rot = tuple(q_obs_world.tolist())
            setattr(scene, f"obstacle{i}", obstacle)

        ## Particles
        if env_cfg.particles and env_cfg.particles_systems > 0:
            particles_size = (
                env_cfg.particles_size
                if isinstance(env_cfg.particles_size, (tuple, list))
                else (env_cfg.particles_size,) * 2
            )
            particles_area = (
                env_cfg.particles_area
                if isinstance(env_cfg.particles_area, (tuple, list))
                else (env_cfg.particles_area,) * 2
            )
            particles_height = (
                env_cfg.particles_height
                if isinstance(env_cfg.particles_height, (tuple, list))
                else (env_cfg.particles_height,) * 2
            )

            min_id = 0 if particles_size[0] <= particles_size[1] else 1
            max_id = (min_id + 1) % 2
            total_particle_z_offset_local = 0.0
            for i in range(env_cfg.particles_systems):
                _f = (
                    0
                    if env_cfg.particles_systems == 1
                    else (i / (env_cfg.particles_systems - 1))
                )
                p_size = particles_size[min_id] + _f * (
                    particles_size[max_id] - particles_size[min_id]
                )
                p_area = particles_area[min_id] + _f * (
                    particles_area[max_id] - particles_area[min_id]
                )
                p_height = math.floor(
                    particles_height[min_id]
                    + _f * (particles_height[max_id] - particles_height[min_id])
                )

                particles_cfg: AssetBaseCfg = getattr(scene, f"particles{i}")
                # ``spawn.size`` is the half-extent; preserve explicit grid dims.
                if env_cfg.particles_dim_xy is None:
                    particles_cfg.spawn.dim_x = round(  # type: ignore[union-attr]
                        p_area
                        * (2.0 * self.asset_cfg.spawn.size[0])  # type: ignore[attr-defined]
                        / p_size
                    )
                    particles_cfg.spawn.dim_y = round(  # type: ignore[union-attr]
                        p_area
                        * (2.0 * self.asset_cfg.spawn.size[1])  # type: ignore[attr-defined]
                        / p_size
                    )
                # Start particles 10 cm above the possibly rotated slope unless
                # the task supplies an explicit initial position.
                if env_cfg.particles_init_pos is None:
                    particles_cfg.init_state = AssetBaseCfg.InitialStateCfg(
                        pos=tuple(
                            (
                                pos_slope_world
                                + quat_apply(
                                    quat_slope_world,
                                    torch.tensor(
                                        [
                                            0.0,
                                            0.0,
                                            total_particle_z_offset_local + 0.1,
                                        ]
                                    ),
                                )
                            ).tolist()
                        ),
                        rot=self.asset_cfg.init_state.rot,
                    )
                setattr(scene, f"particles{i}", particles_cfg)

                total_particle_z_offset_local += p_height * particle_pitch_m(
                    p_size,
                    particles_cfg.spawn.particle_spacing,  # type: ignore[union-attr]
                    particles_cfg.spawn.ratio,  # type: ignore[union-attr]
                )

        ## Robot
        env_cfg.robot.asset_cfg.init_state.rot = self.asset_cfg.init_state.rot  # type: ignore[attr-defined]
