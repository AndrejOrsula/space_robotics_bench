"""SPICE backend; the only module that imports spiceypy."""

from __future__ import annotations

import os
import threading
import warnings

import numpy as np

from srb.core.ephemeris import geometry
from srb.core.ephemeris.config import SiteSpec, _parse_epoch
from srb.core.ephemeris.kernels import KernelsMissingError, _resolve_verified_kernels
from srb.core.ephemeris.provider import (
    EphemerisProvider,
    register_provider,
    validate_query_inputs,
)
from srb.core.ephemeris.state import BodyState, EphemerisState
from srb.utils.extras import require_extra

_LOCK = threading.Lock()
_FURNISHED: tuple[int, tuple[tuple[int, int, int, int, int], ...]] | None = None
_POOL_UNCERTAIN = False


def _reset_furnished_after_fork() -> None:
    """Reset child locks and mark inherited CSPICE state uncertain."""
    global _LOCK, _FURNISHED, _POOL_UNCERTAIN
    _LOCK = threading.Lock()
    _FURNISHED = None
    _POOL_UNCERTAIN = True


if hasattr(os, "register_at_fork"):
    os.register_at_fork(after_in_child=_reset_furnished_after_fork)

_AU_M = 149_597_870_700.0
_BODY_RADIUS_M = {
    "sun": 695_700_000.0,
    "earth": 6_371_000.0,
    "moon": 1_737_400.0,
    "mars": 3_389_500.0,
    "venus": 6_051_800.0,
}
# body -> (body-fixed frame, SPICE observer name, mean surface radius [m])
_SITE_TABLE = {
    "moon": ("MOON_ME", "MOON", 1_737_400.0),
    "mars": (
        "IAU_MARS",
        "MARS BARYCENTER",
        3_389_500.0,
    ),  # de440s has no 499 (see design note)
    "earth": ("IAU_EARTH", "EARTH", 6_371_000.0),
}
_SPICE_NAME = {
    "sun": "SUN",
    "earth": "EARTH",
    "moon": "MOON",
    "mars": "MARS BARYCENTER",
    "venus": "VENUS",
}


def _spice():
    with require_extra("spice"):
        import spiceypy
    return spiceypy


def _clear_pool(sp) -> None:
    """Clear CSPICE state; retain uncertainty unless the clear succeeds."""
    global _FURNISHED, _POOL_UNCERTAIN
    _POOL_UNCERTAIN = True
    clear = getattr(sp, "kclear", None)
    if clear is None:
        raise RuntimeError(
            "CSPICE kclear is unavailable; kernel pool remains uncertain"
        )
    clear()
    _FURNISHED = None
    _POOL_UNCERTAIN = False


def _ensure_furnished(sp) -> list[str]:
    global _FURNISHED, _POOL_UNCERTAIN
    if _POOL_UNCERTAIN:
        _clear_pool(sp)
    try:
        paths, verified_identities = _resolve_verified_kernels()
    except KernelsMissingError as error:
        if _FURNISHED is not None:
            try:
                _clear_pool(sp)
            except BaseException as clear_error:
                error.add_note(
                    f"CSPICE kclear failed after kernel refusal: {clear_error!r}"
                )
        raise
    identity = (os.getpid(), verified_identities)

    def identity_is_current() -> bool:
        try:
            return (
                tuple(
                    (
                        stat.st_dev,
                        stat.st_ino,
                        stat.st_size,
                        stat.st_mtime_ns,
                        stat.st_ctime_ns,
                    )
                    for stat in (path.lstat() for path in paths)
                )
                == verified_identities
            )
        except OSError:
            return False

    def refuse_changed_bundle() -> None:
        _clear_pool(sp)
        raise KernelsMissingError(
            "SPICE kernel files changed after verification; run: srb ephemeris download"
        )

    if not identity_is_current():
        refuse_changed_bundle()

    if _FURNISHED != identity:
        if _FURNISHED is not None:
            _clear_pool(sp)
        if not identity_is_current():
            refuse_changed_bundle()
        for p in paths:
            try:
                sp.furnsh(str(p))
            except BaseException as furnish_error:
                _POOL_UNCERTAIN = True
                try:
                    _clear_pool(sp)
                except BaseException as clear_error:
                    furnish_error.add_note(
                        f"CSPICE kclear failed after furnish error: {clear_error!r}"
                    )
                raise
            if not identity_is_current():
                refuse_changed_bundle()
        if not identity_is_current():
            refuse_changed_bundle()
        _FURNISHED = identity
    return [p.name for p in paths]


@register_provider("spice")
class SpiceProvider(EphemerisProvider):
    def query(
        self,
        site: SiteSpec,
        epoch_utc: str,
        bodies: tuple[str, ...] = ("sun",),
        *,
        optional_bodies: tuple[str, ...] = (),
    ) -> EphemerisState:
        site, epoch_utc, bodies = validate_query_inputs(
            site, epoch_utc, (*bodies, *optional_bodies)
        )
        sp = _spice()
        with _LOCK:
            kernel_names = _ensure_furnished(sp)
            frame, observer, radius_m = _SITE_TABLE[site.body]
            # CSPICE rejects UTC offsets; pass the epoch normalized as naive UTC.
            et = float(sp.str2et(_parse_epoch(epoch_utc).isoformat()))
            rot = (
                np.asarray(sp.pxform("J2000", frame, et))
                if site.is_surface
                else np.eye(3)
            )
            if site.is_surface:
                pos = geometry.site_body_fixed(
                    site.lat_deg, site.lon_deg, site.alt_m, radius_m
                )
                east, north, up = geometry.enu_basis(pos)
            out: dict[str, BodyState] = {}
            query_bodies = bodies if "sun" in bodies else ("sun", *bodies)
            for body in query_bodies:  # sun always; query inputs are unique
                bp_km, _ = sp.spkpos(_SPICE_NAME[body], et, "J2000", "LT+S", observer)
                vec = rot @ (np.asarray(bp_km) * 1000.0)
                if site.is_surface:
                    d = vec - pos
                    az, el = geometry.azel_from_direction(d, east, north, up)
                    direction = geometry.direction_world(az, el, site.north_yaw_deg)
                else:
                    d = vec
                    az = el = float("nan")
                dist = float(np.linalg.norm(d))
                if dist == 0.0:
                    warnings.warn(
                        f"SPICE target {body!r} coincides with the observer; "
                        "direction is undefined, using +Z",
                        RuntimeWarning,
                        stacklevel=2,
                    )
                    direction = np.array((0.0, 0.0, 1.0))
                    ang = 180.0
                else:
                    if not site.is_surface:
                        direction = d / dist
                    ang = 2.0 * float(
                        np.degrees(
                            np.arcsin(min(1.0, _BODY_RADIUS_M.get(body, 0.0) / dist))
                        )
                    )
                out[body] = BodyState(az, el, tuple(direction), dist, ang)
        return EphemerisState(
            epoch_utc=epoch_utc,
            epoch_et=et,
            site=site,
            bodies=out,
            irradiance_scale=(_AU_M / out["sun"].distance_m) ** 2,
            provenance={
                "provider": "spice",
                "frame": frame if site.is_surface else "J2000",
                "observer": observer,
                "kernels": ",".join(kernel_names),
            },
        )
