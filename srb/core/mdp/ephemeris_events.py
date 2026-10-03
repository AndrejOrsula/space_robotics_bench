"""Ephemeris sun-event policies and live-prim application.

``srb.core.mdp.events`` remains the public façade.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import torch

from srb.core.ephemeris import EphemerisCfg, SiteSpec
from srb.core.ephemeris.env_support import (
    kernels_present,
    orbit_sun_light_params,
    query_body_requirements,
    sample_epoch,
    sample_epoch_windows,
    shift_epoch,
    surface_occluder_body,
    surface_sun_light_params,
    warn_eclipse_occluder_missing_once,
)
from srb.core.ephemeris.orbit import OrbitSpec
from srb.core.manager import SceneEntityCfg
from srb.core.scene_introspection import find_scene_entity
from srb.utils.usd import safe_set_attribute_on_usd_prim

if TYPE_CHECKING:
    from srb._typing import AnyEnv


_EMITTED_EPHEMERIS_RESET_KERNELS_MISSING_WARNINGS: set[str] = set()
"""Bodies already warned for missing kernels on sampled-epoch resets.

This module-local registry keys by body, not epoch, so each reset cannot
repeat the warning.
"""


def _warn_ephemeris_reset_kernels_missing_once(body: str, term_name: str) -> None:
    """Warn once per body when sampled-epoch queries fall back without kernels."""
    if body in _EMITTED_EPHEMERIS_RESET_KERNELS_MISSING_WARNINGS:
        return
    _EMITTED_EPHEMERIS_RESET_KERNELS_MISSING_WARNINGS.add(body)

    from srb.core.ephemeris.kernels import kernel_cache_root
    from srb.utils import logging

    logging.warning(
        "%s reset (site body=%s): SPICE kernels missing under %s; sun left as "
        "last set this reset. Run 'srb ephemeris download' for ephemeris-driven "
        "lighting, or set env.ephemeris.on_missing='error' to fail fast.",
        term_name,
        body,
        kernel_cache_root(),
    )


_EMITTED_EPHEMERIS_ECLIPSE_NO_OCCLUDER_WARNINGS: set[str] = set()
"""Site bodies already warned that surface eclipse modeling is unavailable."""


def _warn_ephemeris_eclipse_no_occluder_once(body: str) -> None:
    """Warn once when surface eclipse modeling is unavailable; factor stays 1.0."""
    if body in _EMITTED_EPHEMERIS_ECLIPSE_NO_OCCLUDER_WARNINGS:
        return
    _EMITTED_EPHEMERIS_ECLIPSE_NO_OCCLUDER_WARNINGS.add(body)

    from srb.utils import logging

    logging.warning(
        "env.ephemeris.eclipse=True has no effect on site body=%s: no solar "
        "occluder is modeled for a surface observer on this body (only the "
        "Moon, occluded by Earth, is supported) -- the eclipse factor stays "
        "1.0 for the whole run.",
        body,
    )


_EMITTED_EPHEMERIS_RESET_DARK_EPISODE_WARNINGS: set[
    tuple[str, float | None, float | None]
] = set()
"""Resolved sites already warned about a sampled reset below the horizon.

Key by body and coordinates, not epoch, to distinguish sites without
repeating the warning for each sampled reset.
"""


def _warn_ephemeris_reset_dark_episode_once(
    site: SiteSpec, elevation_deg: float
) -> None:
    """Warn once per site when a sampled reset puts the sun below the horizon."""
    key = (site.body, site.lat_deg, site.lon_deg)
    if key in _EMITTED_EPHEMERIS_RESET_DARK_EPISODE_WARNINGS:
        return
    _EMITTED_EPHEMERIS_RESET_DARK_EPISODE_WARNINGS.add(key)

    from srb.utils import logging

    logging.warning(
        "randomize_sun_ephemeris reset (site body=%s lat=%s lon=%s) sampled "
        "an epoch with the sun at or below the horizon (elevation=%.3f deg) "
        "-- this episode renders fully dark. A wide env.ephemeris.epoch."
        "utc_range can produce a large fraction of dark episodes at this "
        "site; narrow the range if this is unwanted, or budget for it in "
        "the task/reward design.",
        site.body,
        site.lat_deg,
        site.lon_deg,
        elevation_deg,
    )


def _sim_time(env: "AnyEnv") -> float:
    """Return RL simulation time, excluding warmup and rendering steps."""
    return float(getattr(env, "common_step_counter", 0)) * float(
        getattr(env, "step_dt", 0.0)
    )


def _resolve_sun_event_context(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    asset_cfg: SceneEntityCfg | None,
    term_name: str = "randomize_sun_ephemeris",
) -> tuple["AnyEnv", Any] | None:
    """Resolve the sun entity; return ``None`` when query/application must skip."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("sunlight")

    _env: "AnyEnv" = env.unwrapped

    asset: Any | None = find_scene_entity(_env.scene, asset_cfg.name)
    if asset is None:
        return None

    if env_ids is not None and len(env_ids) == 0:
        return None

    # Probe before touching any RNG or querying -- NOT resolve_kernels(),
    # which would auto-fetch on a miss and defeat the offline fallback.
    # The "table" provider needs no SPICE kernels at all, so it must
    # not consult (or be gated by) this probe.
    if ephemeris_cfg.provider != "table" and not kernels_present():
        if ephemeris_cfg.on_missing == "error":
            from srb.core.ephemeris.kernels import (
                KernelsMissingError,
                kernel_cache_root,
            )

            raise KernelsMissingError(
                f"{term_name} reset (site body={site.body}) "
                f"requires SPICE kernels, which are missing under "
                f"{kernel_cache_root()}; run: srb ephemeris download"
            )
        # on_missing == "fallback" (EphemerisCfg.validate() already rejects
        # any other value): leave the sun exactly as it already is.
        _warn_ephemeris_reset_kernels_missing_once(site.body, term_name)
        return None

    return _env, asset


def apply_skydome_from_ephemeris(
    env: "AnyEnv", ephemeris_cfg: EphemerisCfg, state: Any
) -> None:
    """Rotate an available skydome by negative solar azimuth when enabled."""
    if not ephemeris_cfg.drive_skydome:
        return
    _env: "AnyEnv" = env.unwrapped
    skydome: Any | None = find_scene_entity(_env.scene, "skydome")
    if skydome is None:
        return
    azimuth_deg = state.bodies["sun"].azimuth_deg
    if np.isnan(azimuth_deg):
        return
    half_yaw = 0.5 * np.radians(-azimuth_deg)
    rot_xyzw = [0.0, 0.0, float(np.sin(half_yaw)), float(np.cos(half_yaw))]
    orientations = torch.tensor([rot_xyzw], dtype=torch.float32, device=skydome._device)
    skydome.set_world_poses(orientations=orientations)


def _apply_sun_state(
    env: "AnyEnv",
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    orbit: OrbitSpec | None,
    state: Any,
    asset: Any,
    dt_s: float,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
) -> None:
    """Apply light parameters and stash live ephemeris state on the env.

    Reset appliers own epoch anchors; progression does not overwrite them.
    """
    if orbit is not None:
        rot_xyzw, intensity, angle, occluder, factor = orbit_sun_light_params(
            state,
            orbit=orbit,
            dt_s=dt_s,
            domain_light_intensity=domain_light_intensity,
            domain_light_angular_diameter=domain_light_angular_diameter,
            eclipse_enabled=ephemeris_cfg.eclipse,
        )
    else:
        occluder_body = surface_occluder_body(site.body)
        if ephemeris_cfg.eclipse and occluder_body is None:
            _warn_ephemeris_eclipse_no_occluder_once(site.body)
        occluder_in = (
            state.bodies.get(occluder_body) if occluder_body is not None else None
        )
        if ephemeris_cfg.eclipse and occluder_body is not None and occluder_in is None:
            warn_eclipse_occluder_missing_once(
                body=site.body,
                provider=ephemeris_cfg.provider,
            )
        rot_xyzw, intensity, angle, occluder, factor = surface_sun_light_params(
            state,
            body=site.body,
            domain_light_intensity=domain_light_intensity,
            domain_light_angular_diameter=domain_light_angular_diameter,
            north_yaw_deg=site.north_yaw_deg,
            eclipse_enabled=ephemeris_cfg.eclipse,
            occluder=occluder_in,
        )

    orientations = torch.tensor([rot_xyzw], dtype=torch.float32, device=asset._device)
    asset.set_world_poses(orientations=orientations)

    for prim in asset.prims:
        safe_set_attribute_on_usd_prim(
            prim, "inputs:intensity", float(intensity), camel_case=True
        )
        safe_set_attribute_on_usd_prim(
            prim, "inputs:angle", float(angle), camel_case=True
        )

    env._ephemeris_state = state
    env._ephemeris_eclipse_factor = factor
    env._ephemeris_occluder = occluder

    apply_skydome_from_ephemeris(env, ephemeris_cfg, state)


def randomize_sun_ephemeris(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    orbit: OrbitSpec | None = None,
    asset_cfg: SceneEntityCfg | None = None,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
) -> None:
    """Sample an epoch and update the shared sunlight prim on reset.

    One query updates the global light, independent of ``env_ids``. The
    env-seeded RNG stays on the unwrapped env; missing kernels under
    ``on_missing="fallback"`` leave the light and RNG untouched. Below-horizon
    results are applied and warned once per resolved site.
    """
    ctx = _resolve_sun_event_context(
        env,
        env_ids,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        asset_cfg=asset_cfg,
    )
    if ctx is None:
        # A missing-kernels reset never advances `_ephemeris_rng` (the RNG is
        # not even created), so the epoch sequence a LATER, successful reset
        # draws is unaffected by how many earlier resets were no-ops.
        return
    _env, asset = ctx

    rng = getattr(_env, "_ephemeris_rng", None)
    if rng is None:
        rng = np.random.default_rng(_env.cfg.seed)
        _env._ephemeris_rng = rng

    if ephemeris_cfg.epoch.utc_windows is not None:
        epoch_utc = sample_epoch_windows(ephemeris_cfg.epoch.utc_windows, rng)
    else:
        utc_range = ephemeris_cfg.epoch.utc_range
        assert utc_range is not None
        epoch_utc = sample_epoch(utc_range, rng)

    from srb.core.ephemeris import query

    required_bodies, optional_bodies = query_body_requirements(ephemeris_cfg, site)
    state = query(
        site,
        epoch_utc,
        bodies=required_bodies,
        optional_bodies=optional_bodies,
        provider=ephemeris_cfg.provider,
    )
    if orbit is None and state.bodies["sun"].elevation_deg <= 0.0:
        _warn_ephemeris_reset_dark_episode_once(site, state.bodies["sun"].elevation_deg)

    # Anchor the moving-sun progression: epoch0 = the epoch
    # this reset applied, at the sim time it was applied.
    _env._ephemeris_epoch0_utc = epoch_utc
    _env._ephemeris_epoch0_sim_t = _sim_time(_env)

    _apply_sun_state(
        _env,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        orbit=orbit,
        state=state,
        asset=asset,
        # Elapsed-from-epoch0 is 0 at the reset that (re)anchors epoch0.
        dt_s=0.0,
        domain_light_intensity=domain_light_intensity,
        domain_light_angular_diameter=domain_light_angular_diameter,
    )


def apply_sun_ephemeris_fixed(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    orbit: OrbitSpec | None = None,
    asset_cfg: SceneEntityCfg | None = None,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
) -> None:
    """Query fixed ``epoch.utc`` and stash live state on each reset."""
    ctx = _resolve_sun_event_context(
        env,
        env_ids,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        asset_cfg=asset_cfg,
        term_name="apply_sun_ephemeris_fixed",
    )
    if ctx is None:
        return
    _env, asset = ctx

    epoch_utc = ephemeris_cfg.epoch.utc
    assert epoch_utc is not None, (
        "apply_sun_ephemeris_fixed requires a fixed ephemeris_cfg.epoch.utc"
    )

    from srb.core.ephemeris import query

    required_bodies, optional_bodies = query_body_requirements(ephemeris_cfg, site)
    state = query(
        site,
        epoch_utc,
        bodies=required_bodies,
        optional_bodies=optional_bodies,
        provider=ephemeris_cfg.provider,
    )

    _env._ephemeris_epoch0_utc = epoch_utc
    _env._ephemeris_epoch0_sim_t = _sim_time(_env)

    _apply_sun_state(
        _env,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        orbit=orbit,
        state=state,
        asset=asset,
        # Re-anchoring above requires orbit phase to restart at `arg_lat`.
        dt_s=0.0,
        domain_light_intensity=domain_light_intensity,
        domain_light_angular_diameter=domain_light_angular_diameter,
    )


def progress_sun_ephemeris(
    env: "AnyEnv",
    env_ids: torch.Tensor | None,
    *,
    ephemeris_cfg: EphemerisCfg,
    site: SiteSpec,
    orbit: OrbitSpec | None = None,
    asset_cfg: SceneEntityCfg | None = None,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
) -> None:
    """Advance the sun from its reset anchor on each interval firing.

    Query at ``epoch0 + time_scale * elapsed_sim_time``; use fixed
    ``epoch.utc`` when no reset anchor exists, or skip if neither exists.
    Out-of-coverage shifts raise ``ValueError``.
    """
    ctx = _resolve_sun_event_context(
        env,
        env_ids,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        asset_cfg=asset_cfg,
        term_name="progress_sun_ephemeris",
    )
    if ctx is None:
        return
    _env, asset = ctx

    epoch0_utc = getattr(_env, "_ephemeris_epoch0_utc", None)
    epoch0_sim_t = getattr(_env, "_ephemeris_epoch0_sim_t", None)
    if epoch0_utc is None or epoch0_sim_t is None:
        if ephemeris_cfg.epoch.utc is None:
            return
        epoch0_utc = ephemeris_cfg.epoch.utc
        epoch0_sim_t = 0.0

    elapsed = _sim_time(_env) - epoch0_sim_t
    scaled_elapsed = ephemeris_cfg.epoch.time_scale * elapsed
    epoch_utc = shift_epoch(epoch0_utc, scaled_elapsed)

    from srb.core.ephemeris import query

    required_bodies, optional_bodies = query_body_requirements(ephemeris_cfg, site)
    state = query(
        site,
        epoch_utc,
        bodies=required_bodies,
        optional_bodies=optional_bodies,
        provider=ephemeris_cfg.provider,
    )

    _apply_sun_state(
        _env,
        ephemeris_cfg=ephemeris_cfg,
        site=site,
        orbit=orbit,
        state=state,
        asset=asset,
        dt_s=scaled_elapsed,
        domain_light_intensity=domain_light_intensity,
        domain_light_angular_diameter=domain_light_angular_diameter,
    )
