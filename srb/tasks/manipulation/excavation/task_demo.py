"""Public excavation demo defaults and task behavior.

Adds manual-control defaults, HUD metrics, presentation cameras, and optional
regolith effects while preserving the production excavation task. Select
LunaLab with ``env.scenery=lunalab``.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Literal, Sequence

import torch
from isaaclab.sim import PreviewSurfaceCfg
from simforge import BakeType

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import AssetVariant, Subterrane, Terrain
from srb.core.env import ViewerCfg
from srb.core.sensor.category import SensorCategory
from srb.core.sim.kit_visualizer import sync_viewer_to_kit_visualizer
from srb.terrain.companion_scene import disabled_companion_kwargs
from srb.utils import logging
from srb.utils.cfg import configclass

from .demo_visuals import ExcavationDemoVisuals
from .task import Task as ExcavationTask
from .task import TaskCfg as ExcavationTaskCfg
from .task import _TaskMixin

_DEMO_MOON_TEXTURE_RESOLUTION = {
    BakeType.ALBEDO: 2048,
    BakeType.NORMAL: 4096,
    BakeType.ROUGHNESS: 1024,
}


def _make_demo_moon_surface() -> Terrain:
    terrain = assets.MoonSurface(
        scale=(8.0, 8.0, 0.42),
        density=0.16,
        # Level ground under the static Husky and the regolith pile.
        flat_area_size=2.0,
        texture_resolution=dict(_DEMO_MOON_TEXTURE_RESOLUTION),
    )
    # Demo fallback terrain must reflect launch-time visual tuning immediately.
    # SimForge's cache is normally useful, but stale MoonSurface bakes make this
    # task look like the old flat environment even when metadata is correct.
    if terrain.asset_cfg is not None and terrain.asset_cfg.spawn is not None:
        terrain.asset_cfg.spawn.use_cache = False
    return terrain


@configclass
class DemoTaskCfg(ExcavationTaskCfg):
    """Configuration for the public excavation demo."""

    ## Demo mode
    demo: bool = True
    demo_manual_reset: bool = True
    demo_regolith_quality: Literal["particles", "visual", "full"] = "particles"
    demo_cameras: bool = True
    demo_scenery_source: Literal["procedural", "real"] = "real"
    demo_real_site: str = "apollo17"
    demo_real_lod: Literal["low", "medium", "high"] = "high"
    demo_real_patch_size_m: float | None = 98.0
    demo_real_auto_fetch: bool = True
    demo_real_fallback: bool = True
    demo_vehicle_terrain_clearance_m: float = 0.03
    teleop_devices: Sequence[str] | None = ("spacemouse", "haptic", "keyboard")

    ## Fallback Moon scene when the selected real DEM site is not baked locally.
    scenery: Terrain | Subterrane | AssetVariant | None = _make_demo_moon_surface()

    ## Visual regolith effects
    demo_surface_resolution: int = 64
    demo_surface_size_m: tuple[float, float] = (0.72, 0.56)
    demo_surface_max_depth_m: float = 0.11
    demo_surface_brush_radius_m: float = 0.09
    demo_surface_deform_rate_m: float = 0.007
    demo_surface_berm_height_m: float = 0.028
    demo_dust_particles: int = 120
    demo_deformation_affects_physics: bool = False

    ## Public demo defaults
    num_envs: int | None = 1
    stack: bool = True
    # HUD and regolith metrics use privileged particle and penetration data,
    # emitted only when expose_privileged is True. The demo opts in to avoid
    # zero-valued fallbacks; the base task keeps it disabled for RL.
    expose_privileged: bool = True
    skydome: Literal["low_res", "high_res"] | bool | None = "high_res"
    episode_length_s: float = 30.0 * 60.0
    truncate_episodes: bool = False
    particles_update_interval: float = 0.1
    particles_material: PreviewSurfaceCfg = PreviewSurfaceCfg(
        diffuse_color=(0.25, 0.25, 0.25),
        roughness=0.95,
        metallic=0.0,
    )

    ## HUD
    # Defaults show excavation, scoop, status, reward, time, and reset hint;
    # operators can suppress lines through Hydra (for example, ``env.hud=false``).
    hud: bool = True
    hud_title: str = "Excavation"
    hud_max_lines: int = 6

    ## Camera sensors, available via Hydra with ``env.demo_cameras=false``
    camera_resolution: tuple[int, int] | None = (320, 240)
    camera_update_period: float = 0.1
    camera_data_types: Sequence[str] | None = ("rgb",)

    ## Viewer: fixed third-person presentation view for the outdoor Moon scene.
    viewer: ViewerCfg = ViewerCfg(
        eye=(1.55, 0.0, 1.05),
        lookat=(0.52, 0.0, 0.16),
        origin_type="env",
    )

    def __post_init__(self):
        if self.demo_regolith_quality not in ("full", "visual", "particles"):
            raise ValueError(
                "demo_regolith_quality must be one of: full, visual, particles"
            )
        if self.demo_scenery_source not in ("procedural", "real"):
            raise ValueError("demo_scenery_source must be one of: procedural, real")
        if not math.isfinite(self.demo_surface_brush_radius_m) or (
            self.demo_surface_brush_radius_m <= 0.0
        ):
            raise ValueError("demo_surface_brush_radius_m must be positive and finite")

        if self.demo and self.demo_cameras:
            self.active_sensors.add(SensorCategory.VISUAL)

        if self.demo and self._uses_demo_moon_scenery_slot():
            # Rebuild after Hydra extraction so procedural and real-site
            # scenery paths start from the same demo terrain configuration.
            self.scenery = _make_demo_moon_surface()
            if self.demo_scenery_source == "real":
                self._try_use_real_demo_scenery()
        elif self.demo and isinstance(self.scenery, assets.Lunalab):
            # Keep LunaLab stable; movable boulders can mismatch reset prim views.
            self.scenery.boulder_count = 0

        super().__post_init__()

        if self.demo:
            if isinstance(self._scenery, assets.Lunalab):
                for event_name in (
                    "randomize_lamp_position",
                    "randomize_lamp_orientation",
                    "randomize_lunalab_boulders",
                ):
                    if hasattr(self.events, event_name):
                        setattr(self.events, event_name, None)
            # Manual reset hides timeouts; otherwise the demo horizon applies.
            self.truncate_episodes = not self.demo_manual_reset
            if self.demo_regolith_quality == "particles":
                self.demo_dust_particles = 0
                self.demo_surface_deform_rate_m = 0.0
            elif self.demo_regolith_quality == "visual":
                self.demo_deformation_affects_physics = False

            self._lift_outdoor_demo_vehicle()

            # Scenery extras can replace the presentation camera; restore it last.
            self.viewer = replace(
                self.viewer,
                eye=(1.55, -1.75, 1.05),
                lookat=(0.52, 0.0, 0.16),
                origin_type="env",
            )
            self._sync_demo_viewer_to_kit_visualizer()

    def _sync_demo_viewer_to_kit_visualizer(self) -> None:
        """Keep the visible Kit visualizer aligned with the demo viewer pose.

        ``self`` is the env cfg the shared helper expects: it reads
        ``.sim.visualizer_cfgs`` and ``.viewer`` and writes the former back.
        """
        sync_viewer_to_kit_visualizer(self)  # type: ignore[arg-type]

    def _try_use_real_demo_scenery(self) -> None:
        try:
            terrain_cls = getattr(
                assets, f"LunarSite{_camel_site_key(self.demo_real_site)}"
            )
            self.scenery = terrain_cls(
                patch_size_m=self.demo_real_patch_size_m,
                lod=self.demo_real_lod,
                use_ortho=False,
                auto_fetch=self.demo_real_auto_fetch,
                augment="none",
                **disabled_companion_kwargs(),
            )
        except Exception as e:
            if not self.demo_real_fallback:
                raise
            logging.warning(
                "Falling back to procedural Moon demo scenery because real site "
                f"{self.demo_real_site!r} is not ready: {e}"
            )

    def _uses_demo_moon_scenery_slot(self) -> bool:
        if isinstance(self.scenery, AssetVariant):
            return self.scenery == AssetVariant.PROCEDURAL
        return isinstance(self.scenery, assets.MoonSurface)

    def _lift_outdoor_demo_vehicle(self) -> None:
        """Give the static Husky a small clearance on rough outdoor terrain."""

        if self.demo_vehicle_terrain_clearance_m <= 0.0:
            return
        if isinstance(self._scenery, assets.Lunalab):
            return

        dz = float(self.demo_vehicle_terrain_clearance_m)
        for asset_cfg in (
            getattr(self.scene, "robot", None),
            getattr(self.scene, "pedestal", None),
        ):
            if asset_cfg is None or getattr(asset_cfg, "init_state", None) is None:
                continue
            pos = asset_cfg.init_state.pos or (0.0, 0.0, 0.0)
            asset_cfg.init_state.pos = (pos[0], pos[1], pos[2] + dz)


class _DemoTaskMixin(_TaskMixin):
    """Demo state and extraction shared by owned and legacy proof arms."""

    cfg: DemoTaskCfg

    def __init__(self, cfg: DemoTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._demo_visuals: ExcavationDemoVisuals | None = None
        if cfg.demo and cfg.demo_regolith_quality in ("full", "visual"):
            self._demo_visuals = ExcavationDemoVisuals(self)

        self._demo_best_collected_volume_l = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )
        self._demo_excavated_volume_l = torch.zeros(
            self.num_envs, dtype=torch.float32, device=self.device
        )
        self._demo_metrics = self._empty_demo_metrics()
        self._demo_status = "Ready"

        # Prime metrics before the first reset without deforming the surface.
        if cfg.demo:
            self._update_demo_metrics(
                super().extract_step_return(), update_visuals=False
            )

    def _empty_demo_metrics(self) -> dict[str, torch.Tensor]:
        zeros = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        return {
            "demo_excavated_volume_l": zeros.clone(),
            "demo_collected_volume_l": zeros.clone(),
            "demo_best_scoop_volume_l": zeros.clone(),
            "demo_fill_fraction": zeros.clone(),
        }

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        self._demo_best_collected_volume_l[env_ids_t] = 0.0
        self._demo_excavated_volume_l[env_ids_t] = 0.0
        if bool((env_ids_t == 0).any()):
            self._demo_status = "Ready"
        if self._demo_visuals is not None:
            self._demo_visuals.reset(env_ids)

        # Refresh reset rows without visuals: their mesh writes and dust RNG
        # operate on the full batch and would advance unrelated envs.
        if self.cfg.demo and env_ids_t.numel():
            self._update_demo_metrics(
                super().extract_step_return(),
                env_ids=env_ids_t,
                update_visuals=False,
            )

    def advance_step_state(self) -> None:
        super().advance_step_state()
        if self.cfg.demo:
            # Advance telemetry once before extraction, including USD effects.
            self._update_demo_metrics(super().extract_step_return())

    def _horizon_reached(self) -> torch.Tensor:
        if not self.cfg.demo:
            return super()._horizon_reached()
        # Teleop clears ``truncate_episodes``; without manual reset the demo
        # horizon still applies.
        if self.cfg.demo_manual_reset:
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return self.episode_length_buf >= self.max_episode_length

    def extract_step_return(self) -> StepReturn:
        step = super().extract_step_return()
        if not self.cfg.demo:
            return step

        info = dict(step.info or {})
        info.update(self._demo_metrics)

        return StepReturn(
            step.observation, step.reward, step.termination, step.truncation, info
        )

    def _hud_custom_lines(self, reward: torch.Tensor) -> Sequence[tuple[str, str]]:
        if not self.cfg.demo or not self._demo_metrics:
            return ()

        excavated_l = _first_metric(self._demo_metrics["demo_excavated_volume_l"])
        collected_l = _first_metric(self._demo_metrics["demo_collected_volume_l"])
        fill_fraction = _first_metric(self._demo_metrics["demo_fill_fraction"])

        return (
            ("", f"{excavated_l:0.2f} L excavated"),
            ("Scoop", f"{collected_l:0.2f} L | {100.0 * fill_fraction:0.0f}%"),
            ("Status", self._demo_status),
        )

    def _update_demo_metrics(
        self,
        step: StepReturn,
        *,
        env_ids: torch.Tensor | None = None,
        update_visuals: bool = True,
    ) -> dict[str, torch.Tensor]:
        obs = step.observation
        num_envs = self.num_envs
        device = self.device

        if env_ids is None:
            env_ids = torch.arange(num_envs, dtype=torch.long, device=device)
        else:
            env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=device)
        if env_ids.numel() == 0:
            return self._demo_metrics

        proprio = obs[ObservationGroup.PROPRIO]
        proprio_dyn = obs[ObservationGroup.PROPRIO_DYN]
        privileged = obs.get(ObservationGroup.STATE_PRIVILEGED, {})

        ee_pos_b = proprio["fk_pos_end_effector_base"]
        contact_f = proprio_dyn.get(
            "contact_force_scoop",
            torch.zeros((num_envs, 3), dtype=torch.float32, device=device),
        )
        particle_count = privileged.get(
            "particle_count_in_scoop_volume",
            torch.zeros((num_envs, 1), dtype=torch.float32, device=device),
        ).squeeze(-1)
        pen_depth = privileged.get(
            "ee_penetration_depth_into_pile",
            torch.zeros((num_envs, 1), dtype=torch.float32, device=device),
        )

        if update_visuals and self._demo_visuals is not None:
            if env_ids.numel() != num_envs:
                raise ValueError("visual demo metrics require all environment rows")
            visual_volume_l = self._demo_visuals.update(
                ee_pos_b=ee_pos_b,
                pen_depth=pen_depth,
                contact_force=contact_f,
            )
            visual_volume_t = torch.as_tensor(
                visual_volume_l, dtype=torch.float32, device=device
            )
            self._demo_excavated_volume_l = torch.maximum(
                self._demo_excavated_volume_l, visual_volume_t
            )

        particle_volume_l = _particle_volume_l(self.cfg.particles_size)
        collected_volume_l = particle_count * particle_volume_l
        self._demo_best_collected_volume_l[env_ids] = torch.maximum(
            self._demo_best_collected_volume_l[env_ids], collected_volume_l[env_ids]
        )

        fill_fraction = (particle_count / max(1, int(self.cfg.n_scoop_capacity))).clamp(
            0.0, 1.0
        )
        excavated_volume_l = torch.maximum(
            self._demo_excavated_volume_l,
            self._demo_best_collected_volume_l,
        )

        if bool((env_ids == 0).any()):
            self._demo_status = self._status_text(
                particle_count=particle_count,
                fill_fraction=fill_fraction,
                pen_depth=pen_depth.squeeze(-1),
                ee_pos_z=ee_pos_b[:, 2],
            )

        metrics = {
            "demo_excavated_volume_l": excavated_volume_l,
            "demo_collected_volume_l": collected_volume_l,
            "demo_best_scoop_volume_l": self._demo_best_collected_volume_l,
            "demo_fill_fraction": fill_fraction,
        }
        # Copy retained telemetry so later advances cannot mutate earlier steps.
        next_metrics = {
            key: value.detach().clone() for key, value in self._demo_metrics.items()
        }
        for key, value in metrics.items():
            next_metrics[key][env_ids] = value[env_ids].detach()
        self._demo_metrics = next_metrics
        return self._demo_metrics

    def _status_text(
        self,
        *,
        particle_count: torch.Tensor,
        fill_fraction: torch.Tensor,
        pen_depth: torch.Tensor,
        ee_pos_z: torch.Tensor,
    ) -> str:
        count0 = _first_metric(particle_count)
        fill0 = _first_metric(fill_fraction)
        pen0 = _first_metric(pen_depth)
        z0 = _first_metric(ee_pos_z)

        if fill0 >= 0.75:
            return "Scoop loaded"
        if count0 > 0.0 and z0 > 0.18:
            return "Lift and carry"
        if pen0 > 0.01:
            return "Digging"
        if _first_metric(self._demo_best_collected_volume_l) > 0.0:
            return "Try another pass"
        return "Ready"


def _particle_volume_l(particles_size: float | tuple[float, float]) -> float:
    diameter = (
        particles_size[0] if isinstance(particles_size, tuple) else particles_size
    )
    radius = 0.5 * float(diameter)
    return (4.0 / 3.0) * math.pi * radius**3 * 1000.0


def _first_metric(value: torch.Tensor) -> float:
    return float(value.reshape(-1)[0].detach().cpu().item())


def _camel_site_key(site: str) -> str:
    return "".join(part.capitalize() for part in site.split("_"))


class DemoTask(_DemoTaskMixin, ExcavationTask):
    """Excavation task variant tuned for manual public demos."""

    cfg: DemoTaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: DemoTaskCfg, **kwargs):
        super().__init__(
            cfg, runtime_mode=kwargs.pop("runtime_mode", "isaac"), **kwargs
        )
