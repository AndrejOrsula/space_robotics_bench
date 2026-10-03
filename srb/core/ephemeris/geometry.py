"""Pure-numpy site/direction geometry. No Isaac, no spiceypy.

Conventions: z-up world; compass azimuth (0=N, 90=E) in the local ENU frame;
scene +Y is ENU north when north_yaw_deg == 0; quaternions are xyzw.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from srb.core.ephemeris.state import BodyState

_Z = np.array([0.0, 0.0, 1.0])
_POLE_EAST_FALLBACK = np.array(
    [0.0, 1.0, 0.0]
)  # matches lunarlab spice_illum.body_azel


def site_body_fixed(
    lat_deg: float, lon_deg: float, alt_m: float, radius_m: float
) -> np.ndarray:
    lam, phi = np.radians(lon_deg), np.radians(lat_deg)
    r = radius_m + alt_m
    return r * np.array(
        [np.cos(phi) * np.cos(lam), np.cos(phi) * np.sin(lam), np.sin(phi)]
    )


def enu_basis(site_vec: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    up = site_vec / np.linalg.norm(site_vec)
    east = np.cross(_Z, up)
    n = np.linalg.norm(east)
    east = _POLE_EAST_FALLBACK if n < 1e-12 else east / n
    north = np.cross(up, east)
    return east, north, up


def azel_from_direction(
    direction: np.ndarray, east: np.ndarray, north: np.ndarray, up: np.ndarray
) -> tuple[float, float]:
    d = direction / np.linalg.norm(direction)
    el = float(np.degrees(np.arcsin(np.clip(d @ up, -1.0, 1.0))))
    az = float(np.degrees(np.arctan2(d @ east, d @ north)) % 360.0)
    return az, el


def direction_world(
    az_deg: float, el_deg: float, north_yaw_deg: float = 0.0
) -> np.ndarray:
    psi = np.radians(north_yaw_deg)
    north_w = np.array([-np.sin(psi), np.cos(psi), 0.0])
    east_w = np.array([np.cos(psi), np.sin(psi), 0.0])
    az, el = np.radians(az_deg), np.radians(el_deg)
    return np.cos(el) * (np.cos(az) * north_w + np.sin(az) * east_w) + np.sin(el) * _Z


def light_quat_xyzw(
    az_deg: float, el_deg: float, north_yaw_deg: float = 0.0
) -> np.ndarray:
    """Quaternion (xyzw) orienting a UsdLux light (local -Z emission) to shine
    from the body at (az, el) toward the origin: local +Z maps onto the
    body direction.

    NOTE: `terrain/lights/usd_writer._orient_light` builds its horizontal
    heading as `(+cos az, +sin az)`, i.e. it places the light at azimuth
    `az + 180°` measured from +X — a different (non-compass) convention.
    This function is the compass-correct replacement; it reconciles
    the two conventions.
    """
    v = direction_world(az_deg, el_deg, north_yaw_deg)
    return light_quat_from_direction(v)


def light_quat_from_direction(v: np.ndarray) -> np.ndarray:
    """Quaternion (xyzw) mapping local +Z onto unit(*v*) — the direction-vector
    form of :func:`light_quat_xyzw`, for orbit mode where the body direction is
    a J2000/world vector with no az/el."""
    v = np.asarray(v, dtype=float)
    v = v / np.linalg.norm(v)
    c = float(np.clip(_Z @ v, -1.0, 1.0))
    if c > 1.0 - 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    if c < -1.0 + 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])  # 180° about +X
    axis = np.cross(_Z, v)
    axis /= np.linalg.norm(axis)
    half = 0.5 * np.arccos(c)
    return np.array([*(np.sin(half) * axis), np.cos(half)])


def eclipse_factor(sun: "BodyState", occluder: "BodyState") -> float:
    """Unocculted fraction of the solar disk (1.0 = no eclipse, 0.0 = total).

    Disk-overlap model on apparent angular radii: both disks are treated as
    planar circles of radius ``angular_diameter_deg / 2`` separated by the
    angular distance between the two direction vectors (valid for the small
    angular sizes involved; exact in the degenerate branches).
    """
    a_s = 0.5 * sun.angular_diameter_deg
    a_o = 0.5 * occluder.angular_diameter_deg
    if a_o <= 0.0 or a_s <= 0.0:
        return 1.0
    ds = np.asarray(sun.direction_world, dtype=float)
    do = np.asarray(occluder.direction_world, dtype=float)
    cos_c = float(
        np.clip((ds @ do) / (np.linalg.norm(ds) * np.linalg.norm(do)), -1.0, 1.0)
    )
    c = float(np.degrees(np.arccos(cos_c)))
    if c >= a_s + a_o:
        return 1.0
    if c <= abs(a_o - a_s):
        if a_o >= a_s:
            return 0.0
        return 1.0 - (a_o / a_s) ** 2
    # partial: circular-lens overlap area
    d1 = (c * c + a_s * a_s - a_o * a_o) / (2.0 * c)
    d2 = c - d1
    overlap = (
        a_s * a_s * np.arccos(np.clip(d1 / a_s, -1.0, 1.0))
        + a_o * a_o * np.arccos(np.clip(d2 / a_o, -1.0, 1.0))
        - d1 * np.sqrt(max(a_s * a_s - d1 * d1, 0.0))
        - d2 * np.sqrt(max(a_o * a_o - d2 * d2, 0.0))
    )
    return float(np.clip(1.0 - overlap / (np.pi * a_s * a_s), 0.0, 1.0))


def earthshine_fraction(sun: "BodyState", earth: "BodyState") -> float:
    """Illuminated fraction of Earth's disk seen from the site:
    ``(1 + cos(phase)) / 2`` with the phase angle measured AT Earth between
    Earth->site and Earth->sun (site-relative vectors + distances suffice)."""
    e_dir = np.asarray(earth.direction_world, dtype=float)
    s_dir = np.asarray(sun.direction_world, dtype=float)
    earth_to_site = -e_dir * earth.distance_m
    earth_to_sun = s_dir * sun.distance_m - e_dir * earth.distance_m
    cos_phase = float(
        np.clip(
            (earth_to_site @ earth_to_sun)
            / (np.linalg.norm(earth_to_site) * np.linalg.norm(earth_to_sun)),
            -1.0,
            1.0,
        )
    )
    return 0.5 * (1.0 + cos_phase)


def sun_visibility_factor(el_deg: float, angular_radius_deg: float) -> float:
    """Fraction of the solar disk above the horizon (disk-segment model,
    lunarlab build.py:_disk_segment_area)."""
    x = float(np.clip(el_deg / max(angular_radius_deg, 1e-9), -1.0, 1.0))
    if x <= -1.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return float((np.arccos(-x) + x * np.sqrt(1.0 - x * x)) / np.pi)
