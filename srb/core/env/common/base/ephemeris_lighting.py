"""Build ephemeris-driven sunlight without importing environment config classes."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

from srb.core.asset import AssetBaseCfg
from srb.core.ephemeris import EphemerisCfg, SiteSpec
from srb.core.ephemeris.env_support import (
    DOMAIN_TO_BODY,
    TerrainEphemerisFacts,
    check_ephemeris_precedence,
    kernels_present,
    orbit_sun_light_params,
    query_body_requirements,
    resolve_orbit_site,
    resolve_site,
    surface_occluder_body,
    surface_sun_light_params,
)
from srb.core.sim import DistantLightCfg
from srb.utils.json_types import ImmutableValueMixin
from srb.utils.math import rpy_to_quat

type OrbitSiteResolver = Callable[[SiteSpec | None], SiteSpec]
type SurfaceSiteResolver = Callable[
    [SiteSpec | None, str, tuple[float, float] | None, float], SiteSpec
]


def _resolve_surface_site(
    ephemeris_site: SiteSpec | None,
    domain_str: str,
    terrain_site_coords: tuple[float, float] | None,
    terrain_north_convergence_deg: float,
) -> SiteSpec:
    """Call the concrete surface resolver through the positional seam."""
    return resolve_site(
        ephemeris_site=ephemeris_site,
        domain_str=domain_str,
        terrain_site_coords=terrain_site_coords,
        terrain_north_convergence_deg=terrain_north_convergence_deg,
    )


@dataclass(frozen=True, slots=True)
class SiteResolvers:
    """Site resolvers shared by config-time lighting and event registration."""

    orbit: OrbitSiteResolver = resolve_orbit_site
    surface: SurfaceSiteResolver = _resolve_surface_site


@dataclass(frozen=True, slots=True)
class SunlightBuildRequest(ImmutableValueMixin):
    """Immutable light construction request, including asset extension kwargs."""

    prim_path: str = "/World/sunlight"
    spawn_kwargs: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "spawn_kwargs",
            MappingProxyType(dict(self.spawn_kwargs)),
        )


_EMITTED_EPHEMERIS_BELOW_HORIZON_WARNINGS: set[tuple[str, str, str, float, float]] = (
    set()
)
"""Fixed-epoch below-horizon warnings, keyed by domain/epoch/resolved site."""


def _warn_ephemeris_below_horizon_once(
    site: Any, domain_str: str, epoch_utc: str, elevation_deg: float
) -> None:
    key = (domain_str, epoch_utc, site.body, site.lat_deg, site.lon_deg)
    if key in _EMITTED_EPHEMERIS_BELOW_HORIZON_WARNINGS:
        return
    _EMITTED_EPHEMERIS_BELOW_HORIZON_WARNINGS.add(key)

    from srb.utils import logging

    logging.warning(
        "env.ephemeris fixed epoch %s (domain=%s, site=%s lat=%.4f lon=%.4f) "
        "yields a sun elevation of %.3f deg (at or below the horizon) -- the "
        "scene will render dark.",
        epoch_utc,
        domain_str,
        site.body,
        site.lat_deg,
        site.lon_deg,
        elevation_deg,
    )


_EMITTED_EPHEMERIS_TERRAIN_BODY_MISMATCH_WARNINGS: set[tuple[str, str, str]] = set()
"""Real-terrain/domain mismatch warnings, keyed by site/body/domain."""


def _warn_ephemeris_terrain_body_mismatch_once(
    site_key: str, scenery_body: str, domain_str: str, expected_body: str
) -> None:
    key = (site_key, scenery_body, domain_str)
    if key in _EMITTED_EPHEMERIS_TERRAIN_BODY_MISMATCH_WARNINGS:
        return
    _EMITTED_EPHEMERIS_TERRAIN_BODY_MISMATCH_WARNINGS.add(key)

    from srb.utils import logging

    logging.warning(
        "env.scenery is a real terrain on body %r (site=%r) but env.domain=%r "
        "maps to body %r -- ignoring the terrain's coordinates for the "
        "ephemeris site (they would be silently relabelled as %r) and falling "
        "back to an explicit env.ephemeris.site or the domain default. Set "
        "env.domain to match the terrain's body, or pick a %s terrain.",
        scenery_body,
        site_key,
        domain_str,
        expected_body,
        expected_body,
        expected_body,
    )


_EMITTED_EPHEMERIS_KERNELS_MISSING_WARNINGS: set[str] = set()
"""Fixed-epoch kernel fallback warnings, keyed by domain."""


def _warn_ephemeris_kernels_missing_once(domain_str: str) -> None:
    if domain_str in _EMITTED_EPHEMERIS_KERNELS_MISSING_WARNINGS:
        return
    _EMITTED_EPHEMERIS_KERNELS_MISSING_WARNINGS.add(domain_str)

    from srb.core.ephemeris.kernels import kernel_cache_root
    from srb.utils import logging

    logging.warning(
        "env.ephemeris fixed epoch (domain=%s) could not be queried: SPICE "
        "kernels are missing under %s -- falling back to the static default "
        "sun. Run 'srb ephemeris download' to enable ephemeris-driven "
        "lighting, or set env.ephemeris.on_missing='error' to fail fast "
        "instead.",
        domain_str,
        kernel_cache_root(),
    )


def resolve_ephemeris_site(
    env_cfg: Any,
    *,
    orbit_mode: bool | None = None,
    site_resolvers: SiteResolvers | None = None,
    terrain_facts: TerrainEphemerisFacts | None = None,
) -> SiteSpec:
    """Resolve one site from explicit config and caller-supplied terrain facts."""
    cfg: EphemerisCfg = env_cfg.ephemeris
    domain_str = str(env_cfg.domain)
    is_orbit = domain_str == "orbit" if orbit_mode is None else orbit_mode
    resolvers = site_resolvers or SiteResolvers()
    if is_orbit:
        return resolvers.orbit(cfg.site)

    terrain_site_coords: tuple[float, float] | None = None
    terrain_north_convergence_deg = 0.0
    if terrain_facts is not None:
        expected_body = DOMAIN_TO_BODY.get(domain_str)
        if expected_body is not None and terrain_facts.body != expected_body:
            _warn_ephemeris_terrain_body_mismatch_once(
                terrain_facts.site_key,
                terrain_facts.body,
                domain_str,
                expected_body,
            )
        else:
            terrain_site_coords = (terrain_facts.lat_deg, terrain_facts.lon_deg)
            terrain_north_convergence_deg = terrain_facts.north_convergence_deg
    return resolvers.surface(
        cfg.site,
        domain_str,
        terrain_site_coords,
        terrain_north_convergence_deg,
    )


def _sunlight_asset(
    env_cfg: Any,
    *,
    request: SunlightBuildRequest,
    rot: tuple[float, ...],
    intensity: float,
    angle: float,
) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path=request.prim_path,
        spawn=DistantLightCfg(
            intensity=intensity,
            angle=angle,
            color_temperature=env_cfg.domain.light_color_temperature,
            enable_color_temperature=True,
            **request.spawn_kwargs,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(rot=rot),
    )


def build_sunlight_asset(
    env_cfg: Any,
    *,
    request: SunlightBuildRequest | None = None,
) -> AssetBaseCfg | None:
    """Build the normal sun or the ephemeris static placeholder."""
    request = request or SunlightBuildRequest()
    if env_cfg.domain.light_intensity <= 0.0:
        return None

    if env_cfg.ephemeris.enabled and env_cfg.ephemeris.drive_sunlight:
        env_cfg.ephemeris.validate()
        check_ephemeris_precedence(
            ephemeris_enabled=True,
            sunlight_rpy_set=env_cfg.sunlight_rpy is not None,
            domain_str=str(env_cfg.domain),
            ephemeris_site=env_cfg.ephemeris.site,
            orbit_mode=env_cfg.ephemeris.orbit is not None,
        )
        rot = rpy_to_quat(45.0, 30.0, 0.0)
    elif env_cfg.sunlight_rpy is not None:
        roll, pitch, yaw = (float(v) for v in env_cfg.sunlight_rpy)
        env_cfg.events.randomize_sunlight_orientation = None
        rot = rpy_to_quat(roll, pitch, yaw)
    else:
        rot = rpy_to_quat(45.0, 30.0, 0.0)

    return _sunlight_asset(
        env_cfg,
        request=request,
        rot=rot,
        intensity=env_cfg.domain.light_intensity,
        angle=env_cfg.domain.light_angular_diameter,
    )


def build_ephemeris_sunlight_asset(
    env_cfg: Any,
    *,
    request: SunlightBuildRequest | None = None,
    terrain_facts: TerrainEphemerisFacts | None = None,
) -> AssetBaseCfg:
    """Apply all fixed/sampled config-time ephemeris policy in one place."""
    request = request or SunlightBuildRequest()
    cfg: EphemerisCfg = env_cfg.ephemeris
    cfg.validate()
    check_ephemeris_precedence(
        ephemeris_enabled=True,
        sunlight_rpy_set=env_cfg.sunlight_rpy is not None,
        domain_str=str(env_cfg.domain),
        ephemeris_site=cfg.site,
        orbit_mode=cfg.orbit is not None,
    )

    domain_str = str(env_cfg.domain)
    site = resolve_ephemeris_site(
        env_cfg,
        terrain_facts=terrain_facts,
    )

    rot: tuple[float, ...] = rpy_to_quat(45.0, 30.0, 0.0)
    intensity: float = env_cfg.domain.light_intensity
    angle: float = env_cfg.domain.light_angular_diameter

    if cfg.epoch.utc is not None:
        if cfg.provider == "table" or kernels_present():
            from srb.core.ephemeris import query

            required_bodies, optional_bodies = query_body_requirements(cfg, site)
            state = query(
                site,
                cfg.epoch.utc,
                bodies=required_bodies,
                provider=cfg.provider,
                optional_bodies=optional_bodies,
            )
            if domain_str == "orbit":
                orbit = cfg.orbit
                assert orbit is not None
                rot, intensity, angle, _occluder, _factor = orbit_sun_light_params(
                    state,
                    orbit=orbit,
                    dt_s=0.0,
                    domain_light_intensity=env_cfg.domain.light_intensity,
                    domain_light_angular_diameter=env_cfg.domain.light_angular_diameter,
                    eclipse_enabled=cfg.eclipse,
                )
            else:
                if state.bodies["sun"].elevation_deg <= 0.0:
                    _warn_ephemeris_below_horizon_once(
                        site,
                        domain_str,
                        cfg.epoch.utc,
                        state.bodies["sun"].elevation_deg,
                    )
                occluder_body = (
                    surface_occluder_body(site.body) if cfg.eclipse else None
                )
                occluder = (
                    state.bodies.get(occluder_body)
                    if occluder_body is not None
                    else None
                )
                rot, intensity, angle, _occluder, _factor = surface_sun_light_params(
                    state,
                    body=site.body,
                    domain_light_intensity=env_cfg.domain.light_intensity,
                    domain_light_angular_diameter=env_cfg.domain.light_angular_diameter,
                    north_yaw_deg=site.north_yaw_deg,
                    eclipse_enabled=cfg.eclipse,
                    occluder=occluder,
                )
        elif cfg.on_missing == "error":
            from srb.core.ephemeris.kernels import (
                KernelsMissingError,
                kernel_cache_root,
            )

            raise KernelsMissingError(
                f"env.ephemeris fixed epoch {cfg.epoch.utc!r} "
                f"(domain={domain_str}) requires SPICE kernels, which are "
                f"missing under {kernel_cache_root()}; run: srb ephemeris "
                "download"
            )
        else:
            _warn_ephemeris_kernels_missing_once(domain_str)
    elif (
        cfg.provider != "table" and cfg.on_missing == "error" and not kernels_present()
    ):
        from srb.core.ephemeris.kernels import KernelsMissingError, kernel_cache_root

        raise KernelsMissingError(
            f"env.ephemeris sampled epoch range {cfg.epoch.utc_range!r} "
            f"(domain={domain_str}) requires SPICE kernels, which are missing "
            f"under {kernel_cache_root()}; run: srb ephemeris download"
        )

    return _sunlight_asset(
        env_cfg,
        request=request,
        rot=rot,
        intensity=intensity,
        angle=angle,
    )
