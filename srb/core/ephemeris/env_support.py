"""Isaac-free environment and ephemeris helpers; imports avoid Kit and torch."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import timedelta

import numpy as np

from srb.core.ephemeris.config import EphemerisCfg, SiteSpec, _parse_epoch
from srb.core.ephemeris.geometry import (
    eclipse_factor,
    light_quat_from_direction,
    light_quat_xyzw,
    sun_visibility_factor,
)
from srb.core.ephemeris.orbit import OrbitSpec, central_body_state, orbit_position_j2000
from srb.core.ephemeris.state import BodyState, EphemerisState
from srb.utils.failures import InvalidInputError

# Use stdlib logging to preserve the no-Isaac/no-torch import path. The
# logging utility configures this same named logger when imported.
logger = logging.getLogger("srb")

DOMAIN_TO_BODY: dict[str, str] = {
    "moon": "moon",
    "mars": "mars",
    "earth": "earth",
}
"""Maps ``str(Domain)`` (see ``srb.core.domain.Domain.__str__``) to a SPICE
body name. Deliberately excludes ``"asteroid"``/``"orbit"`` -- ephemeris-driven
lighting for those domains is not supported."""

DEFAULT_SITES: dict[str, SiteSpec] = {
    "moon": SiteSpec(body="moon", lat_deg=0.0, lon_deg=0.0),
    "mars": SiteSpec(body="mars", lat_deg=18.44, lon_deg=77.45),  # Jezero crater
    "earth": SiteSpec(body="earth", lat_deg=0.0, lon_deg=0.0),
}
"""Per-domain default observer site, used when neither an explicit
``EphemerisCfg.site`` nor terrain-derived coordinates are available."""


@dataclass(frozen=True, slots=True)
class TerrainEphemerisFacts:
    """Terrain coordinates and grid-north convergence resolved by the scenery owner."""

    body: str
    site_key: str
    lat_deg: float
    lon_deg: float
    north_convergence_deg: float = 0.0


def region_terrain_ephemeris_facts(
    *,
    body: str,
    region_key: str,
    lat_deg: float,
    lon_deg: float,
    north_convergence_deg: float = 0.0,
) -> TerrainEphemerisFacts:
    """Build ephemeris facts from a terrain adapter’s canonical region and coordinates."""
    return TerrainEphemerisFacts(
        body=body,
        site_key=region_key,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        north_convergence_deg=north_convergence_deg,
    )


def domain_body(domain_str: str) -> str:
    """Map a domain string to its SPICE body; unsupported domains raise ValueError."""
    try:
        return DOMAIN_TO_BODY[domain_str]
    except KeyError as exc:
        raise ValueError(
            f"domain {domain_str!r} has no ephemeris body mapping; supported "
            f"domains: {sorted(DOMAIN_TO_BODY)}"
        ) from exc


def resolve_site(
    *,
    ephemeris_site: SiteSpec | None,
    domain_str: str,
    terrain_site_coords: tuple[float, float] | None = None,
    terrain_north_convergence_deg: float = 0.0,
) -> SiteSpec:
    """Resolve observer site: explicit site, then terrain coordinates, then domain
    default.

        Terrain convergence applies only with terrain coordinates. The terrain path uses
        az_true + gamma; rotating the reference frame requires north_yaw_deg = -gamma
    (active/passive duality), verified against direction_world. Explicit sites ignore gamma.
    """
    if ephemeris_site is not None:
        return ephemeris_site

    body = domain_body(domain_str)

    if terrain_site_coords is not None:
        lat_deg, lon_deg = terrain_site_coords
        return SiteSpec(
            body=body,
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            north_yaw_deg=-terrain_north_convergence_deg,
        )

    return DEFAULT_SITES[body]


MEAN_IRRADIANCE_SCALE: dict[str, float] = {
    "moon": 1.0,
    "earth": 1.0,
    "mars": 1.0 / 1.523679**2,  # a_mars = 1.523679 AU (mean semi-major axis)
}
"""``(1 AU / a)**2`` for each body's own mean heliocentric semi-major axis
``a`` -- the same distance reference :class:`~srb.core.domain.Domain`'s
``light_intensity`` is quoted at (Moon/Orbit at 1 AU, Mars at its ~1.523679 AU
mean/surface value). See :func:`irradiance_multiplier`."""


def irradiance_multiplier(body: str, irradiance_scale: float) -> float:
    """Scale domain light intensity by irradiance relative to the body's mean distance.

        Domain intensity already includes mean-distance scaling (and surface attenuation);
    using raw irradiance_scale would double-count it, under-lighting Mars by about 2.3x.
    """
    try:
        mean_scale = MEAN_IRRADIANCE_SCALE[body]
    except KeyError as exc:
        raise ValueError(
            f"irradiance_multiplier: body {body!r} not in "
            f"{sorted(MEAN_IRRADIANCE_SCALE)}"
        ) from exc
    return irradiance_scale / mean_scale


def sun_light_params(
    state: EphemerisState,
    *,
    body: str,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
    north_yaw_deg: float = 0.0,
) -> tuple[tuple[float, ...], float, float]:
    """Shared surface-sun orientation and radiometry for fixed- and sampled-epoch paths.

        Returns rotation, intensity, and effective angular diameter. A zero reported angle
    falls back to the domain angle, which also preserves smooth horizon visibility.
    """
    sun = state.bodies["sun"]
    rot_xyzw = tuple(
        float(v)
        for v in light_quat_xyzw(
            sun.azimuth_deg, sun.elevation_deg, north_yaw_deg=north_yaw_deg
        )
    )
    angle = sun.angular_diameter_deg or domain_light_angular_diameter
    intensity = (
        domain_light_intensity
        * irradiance_multiplier(body, state.irradiance_scale)
        * sun_visibility_factor(sun.elevation_deg, 0.5 * angle)
    )
    return rot_xyzw, intensity, angle


def resolve_orbit_site(ephemeris_site: SiteSpec | None) -> SiteSpec:
    """Resolve an orbit observer; default to Earth and reject surface sites."""
    if ephemeris_site is None:
        return SiteSpec(body="earth")
    if ephemeris_site.lat_deg is not None:
        raise ValueError(
            "env.ephemeris.site for orbit mode must be a central-body "
            f"observer (lat_deg/lon_deg None), not a surface site; got "
            f"lat_deg={ephemeris_site.lat_deg!r}"
        )
    return ephemeris_site


def surface_occluder_body(site_body: str) -> str | None:
    """Return the modeled surface eclipse occluder (Earth for lunar sites)."""
    return "earth" if site_body == "moon" else None


def query_bodies(ephemeris_cfg: EphemerisCfg, site: SiteSpec) -> tuple[str, ...]:
    """Return configured bodies plus a missing surface eclipse occluder, preserving
    order.
    """
    bodies = list(ephemeris_cfg.bodies)
    if ephemeris_cfg.eclipse and site.is_surface:
        occluder = surface_occluder_body(site.body)
        if occluder is not None and occluder not in bodies:
            bodies.append(occluder)
    return tuple(bodies)


def query_body_requirements(
    ephemeris_cfg: EphemerisCfg, site: SiteSpec
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """Split configured bodies from the optional eclipse-only occluder."""
    required = tuple(ephemeris_cfg.bodies)
    occluder = (
        surface_occluder_body(site.body)
        if ephemeris_cfg.eclipse and site.is_surface
        else None
    )
    optional = (occluder,) if occluder is not None and occluder not in required else ()
    return required, optional


def eclipse_factor_for(
    state: EphemerisState,
    *,
    eclipse_enabled: bool,
    occluder: BodyState | None,
) -> float:
    """Return the unocculted solar fraction, or 1.0 when checking is unavailable."""
    if not eclipse_enabled or occluder is None:
        return 1.0
    return eclipse_factor(state.bodies["sun"], occluder)


_ECLIPSE_OCCLUDER_WARNED: set[tuple[str, str]] = set()


def reset_eclipse_occluder_warnings() -> None:
    """Test hook: clear once-per-body/provider warning state."""
    _ECLIPSE_OCCLUDER_WARNED.clear()


def warn_eclipse_occluder_missing_once(*, body: str, provider: str) -> bool:
    """Warn once when an eclipse provider omits the required occluder state.

    A table product without earth_azel has no Earth state, so lunar eclipses stay at
    1.0.
    """
    key = (body, provider)
    if key in _ECLIPSE_OCCLUDER_WARNED:
        return False
    _ECLIPSE_OCCLUDER_WARNED.add(key)
    logger.warning(
        "ephemeris: eclipse=True on a %s site with provider=%r, but the "
        "provider returned no occluder body state (a 'table' product "
        "without `earth_azel`?). The eclipse factor stays pinned at 1.0 "
        "for this run.",
        body,
        provider,
    )
    return True


def orbit_sun_light_params(
    state: EphemerisState,
    *,
    orbit: OrbitSpec,
    dt_s: float,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
    eclipse_enabled: bool,
) -> tuple[tuple[float, ...], float, float, BodyState | None, float]:
    """Compute orbit-mode sun lighting from J2000 direction, irradiance, and eclipse
    state.

        There is no local horizon term. When enabled, propagate the central-body occluder
    at dt_s; Domain.ORBIT intensity is referenced to 1 AU. Return the shared 5-tuple.
    """
    sun = state.bodies["sun"]
    rot_xyzw = tuple(float(v) for v in light_quat_from_direction(sun.direction_world))
    angle = sun.angular_diameter_deg or domain_light_angular_diameter

    if eclipse_enabled:
        pos_j2000 = orbit_position_j2000(orbit, state.site.body, dt_s)
        occluder = central_body_state(pos_j2000, state.site.body)
        factor = eclipse_factor(sun, occluder)
    else:
        occluder = None
        factor = 1.0

    intensity = domain_light_intensity * state.irradiance_scale * factor
    return rot_xyzw, intensity, angle, occluder, factor


def surface_sun_light_params(
    state: EphemerisState,
    *,
    body: str,
    domain_light_intensity: float,
    domain_light_angular_diameter: float,
    north_yaw_deg: float = 0.0,
    eclipse_enabled: bool = False,
    occluder: BodyState | None = None,
) -> tuple[tuple[float, ...], float, float, BodyState | None, float]:
    """Extend shared surface lighting to the orbit path's 5-tuple shape.

    The caller supplies the surface occluder; eclipse scaling is applied here."""
    rot_xyzw, intensity, angle = sun_light_params(
        state,
        body=body,
        domain_light_intensity=domain_light_intensity,
        domain_light_angular_diameter=domain_light_angular_diameter,
        north_yaw_deg=north_yaw_deg,
    )
    factor = eclipse_factor_for(
        state, eclipse_enabled=eclipse_enabled, occluder=occluder
    )
    return rot_xyzw, intensity * factor, angle, occluder, factor


def sample_epoch(utc_range: tuple[str, str], rng: np.random.Generator) -> str:
    """Uniformly sample an ISO epoch using a seeded, reusable RNG for reproducibility.

        Use naive-datetime offset arithmetic: timestamp() would interpret naive values in
    the host timezone and make samples machine-dependent.
    """
    start = _parse_epoch(utc_range[0])
    end = _parse_epoch(utc_range[1])
    span_s = (end - start).total_seconds()
    offset_s = float(rng.uniform(0.0, span_s))
    return (start + timedelta(seconds=offset_s)).isoformat()


def sample_epoch_windows(
    windows: tuple[tuple[str, str], ...], rng: np.random.Generator
) -> str:
    """Sample an epoch from duration-weighted windows using the shared RNG contract.

        Weights are max(duration_s, 1), keeping zero-length windows sampleable; draw uniformly
    within the selected window with timezone-independent offset arithmetic.
    """
    parsed = [(_parse_epoch(start), _parse_epoch(end)) for start, end in windows]
    durations = [(end - start).total_seconds() for start, end in parsed]
    weights = [max(duration, 1.0) for duration in durations]
    total_weight = sum(weights)

    pick = float(rng.uniform(0.0, total_weight))
    cumulative = 0.0
    chosen = len(weights) - 1
    for i, weight in enumerate(weights):
        cumulative += weight
        if pick < cumulative:
            chosen = i
            break

    start, _end = parsed[chosen]
    span_s = durations[chosen]
    offset_s = float(rng.uniform(0.0, span_s)) if span_s > 0.0 else 0.0
    return (start + timedelta(seconds=offset_s)).isoformat()


def shift_epoch(utc: str, seconds: float) -> str:
    """Shift and revalidate an epoch; reject non-finite shifts and kernel-range
    overflow.

        Naive-datetime arithmetic avoids host-timezone dependence; result validation prevents
    returning epochs outside kernel coverage.
    """
    try:
        finite_seconds = float(seconds)
    except (TypeError, ValueError, OverflowError) as exc:
        raise InvalidInputError(
            code="ephemeris.epoch.shift.seconds",
            message=f"epoch shift seconds must be finite, got {seconds!r}",
            component="ephemeris.env_support",
            details={
                "seconds": repr(seconds),
                "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
            },
            retryable=False,
            fallback_allowed=False,
            evidence_valid=False,
        ) from exc
    if not np.isfinite(finite_seconds):
        raise InvalidInputError(
            code="ephemeris.epoch.shift.seconds",
            message=f"epoch shift seconds must be finite, got {seconds!r}",
            component="ephemeris.env_support",
            details={"seconds": repr(seconds)},
            retryable=False,
            fallback_allowed=False,
            evidence_valid=False,
        )
    try:
        shifted = (_parse_epoch(utc) + timedelta(seconds=finite_seconds)).isoformat()
    except OverflowError as exc:
        raise InvalidInputError(
            code="ephemeris.epoch.shift.seconds",
            message=f"epoch shift seconds is outside datetime range: {seconds!r}",
            component="ephemeris.env_support",
            details={
                "seconds": repr(seconds),
                "cause_type": f"{type(exc).__module__}.{type(exc).__qualname__}",
            },
            retryable=False,
            fallback_allowed=False,
            evidence_valid=False,
        ) from exc
    _parse_epoch(shifted)  # range-check the RESULT against kernel coverage
    return shifted


def check_ephemeris_precedence(
    *,
    ephemeris_enabled: bool,
    sunlight_rpy_set: bool,
    domain_str: str,
    ephemeris_site: SiteSpec | None = None,
    orbit_mode: bool = False,
    drive_skydome: bool = False,
    drive_sunlight: bool = True,
    expose_observations: bool = False,
    eclipse: bool = False,
    time_scale: float = 0.0,
) -> None:
    """Reject conflicting, unsupported, or inert ephemeris settings; no-op when
    disabled.
    """
    if not ephemeris_enabled:
        return

    if drive_skydome and not drive_sunlight:
        raise ValueError(
            "env.ephemeris.drive_skydome=True requires env.ephemeris."
            "drive_sunlight=True -- the skydome co-drive runs inside the "
            "sunlight apply path, so it is silently inert otherwise; enable "
            "drive_sunlight or disable drive_skydome"
        )

    if not drive_sunlight:
        inert_knobs = []
        if expose_observations:
            inert_knobs.append("expose_observations")
        if eclipse:
            inert_knobs.append("eclipse")
        if time_scale > 0.0:
            inert_knobs.append("epoch.time_scale")
        if orbit_mode:
            inert_knobs.append("orbit")
        if inert_knobs:
            raise ValueError(
                "env.ephemeris."
                + ", env.ephemeris.".join(inert_knobs)
                + " requires env.ephemeris.drive_sunlight=True -- none of "
                "the ephemeris sun event terms (per-reset apply, eclipse-"
                "occluder query, moving-sun interval term) are registered "
                "when drive_sunlight=False, so the ephemeris state is never "
                "written and the knob(s) above are silently inert (e.g. "
                'obs["ephemeris"] would ship permanently zeroed); enable '
                "drive_sunlight or disable the knob(s) above"
            )

    if sunlight_rpy_set:
        raise ValueError(
            "env.ephemeris.enabled=True conflicts with env.sunlight_rpy being "
            "set -- both would drive the sun orientation; unset sunlight_rpy "
            "or disable ephemeris"
        )

    if domain_str == "orbit":
        if not orbit_mode:
            raise ValueError(
                "env.ephemeris.enabled=True on domain 'orbit' requires an "
                "orbit-mode configuration -- set env.ephemeris.orbit"
            )
        return

    if orbit_mode:
        # Surface-domain code does not propagate OrbitSpec; reject it here.
        raise ValueError(
            f"env.ephemeris.orbit is only valid on Domain.ORBIT (domain is "
            f"{domain_str!r}); unset env.ephemeris.orbit or switch the env "
            "to an orbital domain"
        )

    if domain_str not in DOMAIN_TO_BODY:
        raise ValueError(
            f"env.ephemeris.enabled=True is not supported on domain "
            f"{domain_str!r}; supported domains: {sorted(DOMAIN_TO_BODY)} "
            "(asteroid remains unsupported; orbit requires "
            "env.ephemeris.orbit)"
        )

    if ephemeris_site is not None:
        expected_body = DOMAIN_TO_BODY[domain_str]
        if not ephemeris_site.is_surface:
            raise ValueError(
                f"env.ephemeris.site for surface domain {domain_str!r} must be "
                "a surface site with latitude and longitude"
            )
        if ephemeris_site.body != expected_body:
            raise ValueError(
                f"env.ephemeris.site.body {ephemeris_site.body!r} does not "
                f"match domain {domain_str!r}'s body {expected_body!r}; "
                "either unset env.ephemeris.site (to inherit the domain's "
                f"default site) or set it to body={expected_body!r}"
            )


def kernels_present() -> bool:
    """Check pinned kernel hashes offline without importing spiceypy or accessing
    network.

    Fixed- and sampled-epoch paths share this probe, so “present” has one meaning.
    """
    from srb.core.ephemeris.kernels import kernel_bundle_present

    return kernel_bundle_present()


__all__ = [
    "DEFAULT_SITES",
    "DOMAIN_TO_BODY",
    "MEAN_IRRADIANCE_SCALE",
    "TerrainEphemerisFacts",
    "check_ephemeris_precedence",
    "domain_body",
    "eclipse_factor_for",
    "irradiance_multiplier",
    "kernels_present",
    "orbit_sun_light_params",
    "query_bodies",
    "query_body_requirements",
    "region_terrain_ephemeris_facts",
    "reset_eclipse_occluder_warnings",
    "resolve_orbit_site",
    "resolve_site",
    "sample_epoch",
    "shift_epoch",
    "sun_light_params",
    "surface_occluder_body",
    "surface_sun_light_params",
    "warn_eclipse_occluder_missing_once",
]
