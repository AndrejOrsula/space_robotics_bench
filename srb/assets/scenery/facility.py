import math
from dataclasses import replace
from typing import TYPE_CHECKING, ClassVar

import torch
from pydantic import PositiveInt
from simforge import BakeType

from srb.assets.object.light import ProjectorLight
from srb.assets.object.rock import LunalabBoulder
from srb.assets.scenery.terrain import MoonSurface
from srb.core.asset import (
    AssetBaseCfg,
    IntravehicularScenery,
    Light,
    Object,
    Subterrane,
    Terrain,
)
from srb.core.asset.common.thumbnail import ThumbnailCameraPose
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import (
    reset_root_state_uniform,
    reset_xform_orientation_uniform,
    reset_xform_positions_uniform,
    reset_xforms_uniform_poisson_disk_2d,
)
from srb.core.particle_layout import particle_pitch_m
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.sim import CollisionPropertiesCfg, UsdFileCfg
from srb.core.sim.kit_visualizer import (
    sync_viewer_to_kit_visualizer as _sync_viewer_to_kit_visualizer,
)
from srb.core.sim.light_exposure import record_light_exposure_override
from srb.utils.math import radians, rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_SCENERY

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


class Lunalab(Subterrane):
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(-4.0, -5.0, 1.5), lookat=(0.0, -1.0, 0.0)
    )

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
        density=0.04,
        texture_resolution={
            BakeType.ALBEDO: 4 * 2048,
            BakeType.NORMAL: 2 * 4096,
            BakeType.ROUGHNESS: 4 * 1024,
        },
    )
    terrain.asset_cfg.prim_path = "{ENV_REGEX_NS}/lunalab_terrain"
    terrain.asset_cfg.init_state.pos = (-0.5, -0.5, 0.02)

    ## Light
    lamp: Light = ProjectorLight()
    lamp.asset_cfg.init_state = AssetBaseCfg.InitialStateCfg(  # type: ignore[union-attr]
        pos=(-0.5, -5.25, 2.5), rot=rpy_to_quat(0.0, 0.0, 0.0)
    )

    ## Boulders
    boulder: Object = LunalabBoulder()
    boulder_count: PositiveInt = 2

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        scene = env_cfg.scene
        events = env_cfg.events

        ## Scene
        scene.env_spacing = 0.0 if env_cfg.stack else 12.5

        ## Viewer
        env_cfg.viewer = (  # type: ignore[misc]
            replace(
                env_cfg.viewer,
                eye=(-4.0, -5.0, 1.5),
                lookat=(0.5, -1.5, 0.0),
                origin_type="env",
            )
        )

        ## Light
        scene.sunlight = None
        events.randomize_sunlight_orientation = None
        events.randomize_sunlight_intensity = None
        events.randomize_sunlight_angular_diameter = None
        events.randomize_sunlight_color_temperature = None
        # Event terms are built before setup_extras, so clear ephemeris too:
        # it may already target the sunlight entity removed by this scenery.
        events.randomize_sun_ephemeris = None
        self.lamp.asset_cfg.prim_path = (
            "/World/light_projector"
            if env_cfg.stack
            else "{ENV_REGEX_NS}/light_projector"
        )
        scene.lamp = self.lamp.as_asset_base_cfg()  # type: ignore[misc]
        events.randomize_lamp_position = SrbEventTermCfg(  # type: ignore[misc]
            func=reset_xform_positions_uniform,
            mode="reset",
            category=RandomizationCategory.VISUAL,
            params={
                "asset_cfg": SceneEntityCfg("lamp"),
                "position_distribution_params": {
                    "x": (-3.25, 2.0),
                    "y": (-5.5, -5.0),
                    "z": (1.0, 2.5),
                },
            },
        )
        events.randomize_lamp_orientation = SrbEventTermCfg(  # type: ignore[misc]
            func=reset_xform_orientation_uniform,
            mode="reset",
            category=RandomizationCategory.VISUAL,
            params={
                "asset_cfg": SceneEntityCfg("lamp"),
                "orientation_distribution_params": {
                    "roll": (radians(-10.0 - 30.0), radians(-10.0 + 30.0)),
                    "pitch": (radians(-30.0), radians(+30.0)),
                    "yaw": (radians(-30.0), radians(+30.0)),
                },
            },
        )

        ## Skydome
        scene.skydome = None
        events.randomize_skydome_orientation = None

        ## Terrain
        # Isaac Lab plans variants at concrete sibling paths; `.*` is invalid.
        self.terrain.asset_cfg.prim_path = (
            "/World/lunalab_terrain"
            if env_cfg.stack
            else "{ENV_REGEX_NS}/lunalab_terrain"
        )
        scene.lunalab_terrain = self.terrain.as_asset_base_cfg()  # type: ignore[misc]
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
            events.randomize_lunalab_boulders = (  # type: ignore[misc]
                SrbEventTermCfg(
                    func=reset_xforms_uniform_poisson_disk_2d,
                    mode="reset",
                    category=RandomizationCategory.TASK,
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
                        "keep_clear_of": [
                            SceneEntityCfg(name)
                            for name in ("robot", *env_cfg.extra_robots)
                        ],
                    },
                )
            )

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
            # Give each layer a different granularity.
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
                # The embedded regolith bed is 6.5 m × 10 m. Preserve an
                # explicitly configured absolute grid.
                if env_cfg.particles_dim_xy is None:
                    particles_cfg.spawn.dim_x = round(  # type: ignore[union-attr]
                        p_area * 6.5 / p_size
                    )
                    particles_cfg.spawn.dim_y = round(  # type: ignore[union-attr]
                        p_area * 10.0 / p_size
                    )
                # Start particles 50 cm above the floor unless overridden.
                if env_cfg.particles_init_pos is None:
                    particles_cfg.init_state = AssetBaseCfg.InitialStateCfg(
                        pos=(
                            self.terrain.asset_cfg.init_state.pos[0],
                            self.terrain.asset_cfg.init_state.pos[1],
                            total_particle_z_offset_local + 0.5,
                        ),
                    )
                setattr(scene, f"particles{i}", particles_cfg)

                total_particle_z_offset_local += p_height * particle_pitch_m(
                    p_size,
                    particles_cfg.spawn.particle_spacing,  # type: ignore[union-attr]
                    particles_cfg.spawn.ratio,  # type: ignore[union-attr]
                )

        ## Events | task: waypoint_navigation
        if hasattr(events, "target_pose_evolution"):
            events.target_pose_evolution.params["pos_bounds"] = {  # type: ignore[misc]
                "hardcoded": True,
                "x": (-3.0, 3.0),
                "y": (-4.75, 4.75),
            }
        elif hasattr(events, "target_pos_evolution"):
            events.target_pos_evolution.params["pos_bounds"] = {  # type: ignore[misc]
                "hardcoded": True,
                "x": (-3.0, 3.0),
                "y": (-4.75, 4.75),
            }


class Zeroglab(IntravehicularScenery):
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(0.0, 0.0, 1.6), lookat=(2.5, -1.5, 0.5)
    )

    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/zeroglab",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_SCENERY.joinpath(
                "zerog_viewable.usdc"
            ).as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0)
        ),
    )

    lamp: Light = ProjectorLight()
    lamp.asset_cfg.init_state = AssetBaseCfg.InitialStateCfg(  # type: ignore[union-attr]
        pos=(0.1, -1.5, 2.15), rot=rpy_to_quat(-15.0, 0.0, -90.0)
    )

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        scene = env_cfg.scene
        events = env_cfg.events

        ## Scene
        scene.env_spacing = 0.0 if env_cfg.stack else 6.0

        ## Viewer
        env_cfg.viewer = (  # type: ignore[misc]
            replace(
                env_cfg.viewer,
                eye=(0.0, 0.0, 1.6),
                lookat=(2.5, -1.5, 0.5),
                origin_type="env",
            )
        )
        _sync_viewer_to_kit_visualizer(env_cfg)

        ## Light
        scene.sunlight = None
        events.randomize_sunlight_orientation = None
        events.randomize_sunlight_intensity = None
        events.randomize_sunlight_angular_diameter = None
        events.randomize_sunlight_color_temperature = None
        # See Lunalab.setup_extras for why this must also be nulled here:
        # events.update() (-> _update_sunlight) runs before setup_extras, so a
        # sampled-epoch ephemeris task can have already registered this term
        # against the "sunlight" scene entity this facility deletes.
        events.randomize_sun_ephemeris = None
        self.lamp.asset_cfg.prim_path = (
            "/World/light_projector"
            if env_cfg.stack
            else "{ENV_REGEX_NS}/light_projector"
        )
        self.lamp.asset_cfg.init_state = AssetBaseCfg.InitialStateCfg(
            pos=(0.1, -1.5, 2.15), rot=rpy_to_quat(-15.0, 0.0, -90.0)
        )
        scene.lamp = self.lamp.as_asset_base_cfg()  # type: ignore[misc]
        events.randomize_lamp_position = SrbEventTermCfg(  # type: ignore[misc]
            func=reset_xform_positions_uniform,
            mode="reset",
            category=RandomizationCategory.VISUAL,
            params={
                "asset_cfg": SceneEntityCfg("lamp"),
                "position_distribution_params": {
                    "x": (0.05, 0.15),
                    "y": (-1.9, -1.1),
                    "z": (1.95, 2.35),
                },
            },
        )
        events.randomize_lamp_orientation = SrbEventTermCfg(  # type: ignore[misc]
            func=reset_xform_orientation_uniform,
            mode="reset",
            category=RandomizationCategory.VISUAL,
            params={
                "asset_cfg": SceneEntityCfg("lamp"),
                "orientation_distribution_params": {
                    "roll": (radians(-20.0), radians(-10.0)),
                    "pitch": (radians(-3.0), radians(3.0)),
                    "yaw": (radians(-100.0), radians(-80.0)),
                },
            },
        )

        ## Robot
        if hasattr(events, "randomize_robot_state"):
            events.randomize_robot_state = SrbEventTermCfg(  # type: ignore[misc]
                func=reset_root_state_uniform,
                mode="reset",
                category=RandomizationCategory.PHYSICS,
                params={
                    "asset_cfg": SceneEntityCfg("robot"),
                    "pose_range": {
                        "x": (1.0, 4.0),
                        "y": (-2.0, -1.0),
                        "yaw": (-torch.pi, torch.pi),
                    },
                    "velocity_range": {},
                },
            )

        ## Skydome
        scene.skydome = None
        events.randomize_skydome_orientation = None


class Kibou(IntravehicularScenery):
    # Kibou's lit volume is the corridor running along -y: the module ships 14
    # `CylinderLight` prims at intensity 1e5, all of them at x ~ +/-1 and
    # y in [-8.15, -0.65]. The former (2, -2, 1.5) -> (0, 0, 0) framing looked
    # across that corridor into an unlit rack from inside the hull and rendered
    # a mean RGB of 0.25; looking back up the corridor from (0, -2.1, 1.1)
    # renders the module at a mean of 144.6 with the robot in frame.
    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(0.0, -2.1, 1.1), lookat=(0.0, 0.4, 0.7)
    )

    asset_cfg: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/kibou",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_SCENERY.joinpath("kibou.usdz").as_posix(),
            collision_props=CollisionPropertiesCfg(),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )

    lamp: Light = ProjectorLight()
    lamp.asset_cfg.init_state = AssetBaseCfg.InitialStateCfg(  # type: ignore[union-attr]
        pos=(0.0, 0.0, 1.5), rot=rpy_to_quat(-30.0, 0.0, 0.0)
    )

    LAMP_EXPOSURE: ClassVar[float] = -6.0
    """Stops to dim the projector lamp by inside this module.

    Kibou ships its own 14 ``CylinderLight`` prims, so the lamp is a fill light
    here rather than the light source it is in an unlit facility. At its
    authored emission it blew out 27.4% of the corridor view (pixels at 250+ in
    all three channels, p95 = 255); the module's own lights are not responsible
    -- hiding them instead *raises* that to 31.4%, and hiding only the lamp
    takes it to 0% at mean RGB 91.1. Measured across the exposure range, -6
    (emission / 64) is the first value that brings saturation under 2%:

    ====== ====== ============
    stops  mean   saturated
    ====== ====== ============
    0      162.5  27.1 %
    -2     160.2  19.0 %
    -4     150.1  6.3 %
    -6     132.6  1.6 %
    ====== ====== ============

    It keeps the lamp's contribution (mean 132.6 against 91.1 unlit) and with
    it the randomization the reset events apply to its pose.
    """

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        scene = env_cfg.scene
        events = env_cfg.events

        ## Scene
        scene.env_spacing = 0.0 if env_cfg.stack else 6.0

        ## Viewer
        env_cfg.viewer = replace(  # type: ignore[misc]
            env_cfg.viewer,
            eye=(0.0, -2.1, 1.1),
            lookat=(0.0, 0.4, 0.7),
            origin_type="env",
        )

        ## Light
        scene.sunlight = None
        events.randomize_sunlight_orientation = None
        events.randomize_sunlight_intensity = None
        events.randomize_sunlight_angular_diameter = None
        events.randomize_sunlight_color_temperature = None
        # See Lunalab.setup_extras for why this must also be nulled here:
        # events.update() (-> _update_sunlight) runs before setup_extras, so a
        # sampled-epoch ephemeris task can have already registered this term
        # against the "sunlight" scene entity this facility deletes.
        events.randomize_sun_ephemeris = None
        self.lamp.asset_cfg.prim_path = (
            "/World/light_projector"
            if env_cfg.stack
            else "{ENV_REGEX_NS}/light_projector"
        )
        scene.lamp = self.lamp.as_asset_base_cfg()  # type: ignore[misc]
        # See ``LAMP_EXPOSURE``: the lamp's emission is authored inside its USD,
        # so it is dimmed on the stage rather than through a config field.
        record_light_exposure_override(
            env_cfg, self.lamp.asset_cfg.prim_path, self.LAMP_EXPOSURE
        )
        events.randomize_lamp_position = SrbEventTermCfg(  # type: ignore[misc]
            func=reset_xform_positions_uniform,
            mode="reset",
            category=RandomizationCategory.VISUAL,
            params={
                "asset_cfg": SceneEntityCfg("lamp"),
                "position_distribution_params": {
                    "x": (-1.0, 1.0),
                    "y": (-1.0, 1.0),
                    "z": (1.0, 2.0),
                },
            },
        )
        events.randomize_lamp_orientation = SrbEventTermCfg(  # type: ignore[misc]
            func=reset_xform_orientation_uniform,
            mode="reset",
            category=RandomizationCategory.VISUAL,
            params={
                "asset_cfg": SceneEntityCfg("lamp"),
                "orientation_distribution_params": {
                    "roll": (radians(-30.0 - 15.0), radians(-30.0 + 15.0)),
                    "pitch": (radians(-15.0), radians(+15.0)),
                    "yaw": (radians(-15.0), radians(+15.0)),
                },
            },
        )

        ## Skydome
        scene.skydome = None
        events.randomize_skydome_orientation = None
