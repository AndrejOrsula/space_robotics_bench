"""Normalize planetary CRSes and convert between projected and geographic coordinates.

Canonical matching checks known authority codes first, then parameters for
aliases unknown to PROJ. Matching checks projection,
origin, radius, offsets, longitude, and effective origin scale; shifted grids
do not match an unshifted canonical CRS. Authority codes provide a fast path
when pyproj exposes a known canonical code.

Angles are degrees; lengths are metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal

from pyproj import CRS

from srb.core import geodesy as _geodesy

_TRUE_NORTH_STEP_DEG = _geodesy._TRUE_NORTH_STEP_DEG
_crs_pair = _geodesy._crs_pair
_forward_transformer = _geodesy._forward_transformer
_inverse_transformer = _geodesy._inverse_transformer
latlon_to_xy = _geodesy.latlon_to_xy
north_convergence_deg = _geodesy.north_convergence_deg
xy_to_latlon = _geodesy.xy_to_latlon

CANONICAL_CRS: dict[str, str] = {
    "moon_south": "IAU_2015:30135",
    "moon_north": "IAU_2015:30130",
    "moon_geographic": "IAU_2015:30100",
    "mars_south": "IAU_2015:49935",
    "mars_north": "IAU_2015:49930",
    "mars_geographic": "IAU_2015:49900",
    "earth_south": "EPSG:3031",
    "earth_north": "EPSG:3413",
    "earth_geographic": "EPSG:4326",
}

# Mean radii for local oblique projections; Earth uses WGS84 there.
BODY_RADIUS_M: dict[str, float] = {
    "moon": 1_737_400.0,
    "mars": 3_396_190.0,
    "earth": 6_371_000.0,
}

# Signatures used by `normalize_crs`; zero offsets and longitude avoid matching
# shifted grids. Geographic entries use `lat_0=None`.
_CANONICAL_PARAM_SIGNATURES: dict[str, dict[str, Any]] = {
    CANONICAL_CRS["moon_south"]: {
        "proj": "stere",
        "lat_0": -90.0,
        "radius_m": 1737400.0,
        "k_0": 1.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
    CANONICAL_CRS["moon_north"]: {
        "proj": "stere",
        "lat_0": 90.0,
        "radius_m": 1737400.0,
        "k_0": 1.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
    CANONICAL_CRS["moon_geographic"]: {
        "proj": "longlat",
        "lat_0": None,
        "radius_m": 1737400.0,
        "k_0": 1.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
    CANONICAL_CRS["mars_south"]: {
        "proj": "stere",
        "lat_0": -90.0,
        "radius_m": 3396190.0,
        "k_0": 1.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
    CANONICAL_CRS["mars_north"]: {
        "proj": "stere",
        "lat_0": 90.0,
        "radius_m": 3396190.0,
        "k_0": 1.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
    CANONICAL_CRS["mars_geographic"]: {
        "proj": "longlat",
        "lat_0": None,
        "radius_m": 3396190.0,
        "k_0": 1.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
    CANONICAL_CRS["earth_south"]: {
        "proj": "stere",
        "lat_0": -90.0,
        # EPSG:3031 uses lat_ts=-71; preserve its scaled pole so an unscaled
        # south-polar WGS84 grid does not match it.
        "radius_m": 6378137.0,
        "semi_minor_m": 6356752.314245,
        "k_0": (1.0 + math.sin(math.radians(71.0))) / 2.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
    # EPSG:3413 has lon_0=-45, outside the zero-longitude invariant. Authority
    # matching handles it; parameter matching could misidentify rotated grids.
    CANONICAL_CRS["earth_geographic"]: {
        "proj": "longlat",
        "lat_0": None,
        "radius_m": 6378137.0,
        "semi_minor_m": 6356752.314245,
        "k_0": 1.0,
        "x_0": 0.0,
        "y_0": 0.0,
        "lon_0": 0.0,
    },
}

_RADIUS_TOL_M = 0.5
_LAT0_TOL_DEG = 1e-6
_XY0_TOL_M = 0.5
_LON0_TOL_DEG = 1e-6
# Allows EPSG:3031 scale spellings to match while distinguishing scaled poles.
# Near the pole, scale may fall within tolerance of an unscaled grid.
_K0_TOL = 1e-4


@dataclass(frozen=True)
class NormalizedCrs:
    """Result of matching an arbitrary CRS string against the canonical set."""

    canonical_id: str | None
    proj_params: dict[str, Any]
    is_polar_south: bool
    matched_by: Literal["authority", "parameters", "body_default"]


@dataclass(frozen=True)
class RasterGridSig:
    """Raster grid identity: CRS, affine transform, and shape."""

    crs: str
    transform: tuple[float, float, float, float, float, float]
    """Affine coefficients in rasterio order: (a, b, c, d, e, f)."""
    width: int
    height: int


# --- internals ---------------------------------------------------------------


def _radius_m(crs: CRS) -> float | None:
    """Return radius in metres, preferring PROJ's sphere parameter."""
    d = crs.to_dict()
    if "R" in d:
        return float(d["R"])
    ell = crs.ellipsoid
    if ell is not None:
        return float(ell.semi_major_metre)
    return None


def _authority_id(crs: CRS) -> str | None:
    auth = crs.to_authority()
    if auth is None:
        return None
    return f"{auth[0]}:{auth[1]}"


_ProjSignature = tuple[
    "str | None",
    "float | None",
    "float | None",
    "float | None",
    float,
    float,
    float,
    float,
]


def _scale_at_origin(
    d: dict[str, Any],
    proj_type: str | None,
    lat_0: float | None,
    crs: CRS | None = None,
) -> float:
    """Normalize scale spellings to the effective factor at the origin.

    Polar stereographic derives scale from ``lat_ts`` only at a pole. For
    cylindrical projections, ``lat_ts`` takes precedence; invalid values yield NaN.
    """
    lat_ts = d.get("lat_ts")
    if lat_ts is not None and proj_type in {"eqc", "merc", "cea"}:
        try:
            lat_ts_deg = float(lat_ts)
        except (TypeError, ValueError, OverflowError):
            return math.nan
        if not math.isfinite(lat_ts_deg) or abs(lat_ts_deg) >= 90.0:
            return math.nan

        if "R" in d:
            eccentricity_squared = 0.0
        else:
            if crs is None or crs.ellipsoid is None:
                return math.nan
            try:
                semi_major = float(crs.ellipsoid.semi_major_metre)
                semi_minor = float(crs.ellipsoid.semi_minor_metre)
            except (TypeError, ValueError, OverflowError):
                return math.nan
            if (
                not math.isfinite(semi_major)
                or not math.isfinite(semi_minor)
                or semi_major <= 0.0
                or semi_minor <= 0.0
                or semi_minor > semi_major
            ):
                return math.nan
            eccentricity_squared = 1.0 - (semi_minor / semi_major) ** 2
            if not math.isfinite(eccentricity_squared) or not (
                0.0 <= eccentricity_squared < 1.0
            ):
                return math.nan

        phi_ts = math.radians(lat_ts_deg)
        denominator = math.sqrt(1.0 - eccentricity_squared * math.sin(phi_ts) ** 2)
        scale = math.cos(phi_ts) / denominator
        return scale if math.isfinite(scale) else math.nan

    k_0 = d.get("k_0", d.get("k"))
    if k_0 is not None:
        try:
            scale = float(k_0)
        except (TypeError, ValueError, OverflowError):
            return math.nan
        return scale if math.isfinite(scale) else math.nan

    is_polar = lat_0 is not None and abs(abs(lat_0) - 90.0) <= _LAT0_TOL_DEG
    if lat_ts is not None and proj_type == "stere" and is_polar:
        return (1.0 + math.sin(math.radians(abs(float(lat_ts))))) / 2.0
    return 1.0


def _proj_signature(crs: CRS) -> _ProjSignature:
    """Return projection parameters that determine grid identity.

    Missing offsets and longitude use PROJ's zero defaults; normalized scale
    lets equivalent parameter spellings compare equally.
    """
    d = crs.to_dict()
    proj_type = d.get("proj")
    lat_0_raw = d.get("lat_0")
    lat_0 = float(lat_0_raw) if lat_0_raw is not None else None
    x_0 = float(d.get("x_0", 0.0))
    y_0 = float(d.get("y_0", 0.0))
    lon_0 = float(d.get("lon_0", 0.0))
    return (
        proj_type,
        lat_0,
        _radius_m(crs),
        float(crs.ellipsoid.semi_minor_metre) if crs.ellipsoid is not None else None,
        x_0,
        y_0,
        lon_0,
        _scale_at_origin(d, proj_type, lat_0, crs),
    )


def _signatures_match(
    sig_a: _ProjSignature,
    sig_b: _ProjSignature,
    *,
    lat0_tol: float = _LAT0_TOL_DEG,
    radius_tol: float = _RADIUS_TOL_M,
    xy0_tol: float = _XY0_TOL_M,
    lon0_tol: float = _LON0_TOL_DEG,
    k0_tol: float = _K0_TOL,
) -> bool:
    proj_a, lat0_a, r_a, b_a, x0_a, y0_a, lon0_a, k0_a = sig_a
    proj_b, lat0_b, r_b, b_b, x0_b, y0_b, lon0_b, k0_b = sig_b
    if proj_a != proj_b:
        return False
    if r_a is None or r_b is None or abs(r_a - r_b) > radius_tol:
        return False
    if b_a is None or b_b is None or abs(b_a - b_b) > radius_tol:
        return False
    if (lat0_a is None) != (lat0_b is None):
        return False
    if lat0_a is not None and abs(lat0_a - lat0_b) > lat0_tol:  # type: ignore[operator]
        return False
    if abs(x0_a - x0_b) > xy0_tol:
        return False
    if abs(y0_a - y0_b) > xy0_tol:
        return False
    if abs(lon0_a - lon0_b) > lon0_tol:
        return False
    if not math.isfinite(k0_a) or not math.isfinite(k0_b):
        return False
    if abs(k0_a - k0_b) > k0_tol:
        return False
    return True


def _apply_affine(
    transform: tuple[float, float, float, float, float, float],
    col: float,
    row: float,
) -> tuple[float, float]:
    a, b, c, d, e, f = transform
    return a * col + b * row + c, d * col + e * row + f


# --- public API ----------------------------------------------------------------


def normalize_crs(wkt_or_code: str | None, *, body: str) -> NormalizedCrs:
    """Match a CRS to the canonical set by authority, parameters, or body default."""
    if not wkt_or_code:
        return NormalizedCrs(
            canonical_id=CANONICAL_CRS.get(f"{body}_geographic"),
            proj_params={},
            is_polar_south=False,
            matched_by="body_default",
        )

    crs = CRS.from_user_input(wkt_or_code)
    proj_params = crs.to_dict()
    proj_type, lat_0, *_rest = _proj_signature(crs)
    is_polar_south = (
        proj_type == "stere"
        and lat_0 is not None
        and abs(lat_0 - (-90.0)) <= _LAT0_TOL_DEG
    )

    authority_id = _authority_id(crs)
    if authority_id is not None and authority_id in CANONICAL_CRS.values():
        return NormalizedCrs(
            canonical_id=authority_id,
            proj_params=proj_params,
            is_polar_south=is_polar_south,
            matched_by="authority",
        )

    crs_sig = _proj_signature(crs)
    for canonical_id, sig in _CANONICAL_PARAM_SIGNATURES.items():
        target_sig = (
            sig["proj"],
            sig["lat_0"],
            sig["radius_m"],
            sig.get("semi_minor_m", sig["radius_m"]),
            sig["x_0"],
            sig["y_0"],
            sig["lon_0"],
            sig["k_0"],
        )
        if _signatures_match(crs_sig, target_sig):
            return NormalizedCrs(
                canonical_id=canonical_id,
                proj_params=proj_params,
                is_polar_south=is_polar_south,
                matched_by="parameters",
            )

    return NormalizedCrs(
        canonical_id=CANONICAL_CRS.get(f"{body}_geographic"),
        proj_params=proj_params,
        is_polar_south=is_polar_south,
        matched_by="body_default",
    )


def canonical_crs_for(body: str, lat_deg: float, *, polar_lat_deg: float = 60.0) -> str:
    """Return a polar CRS at or beyond the threshold, else the geographic CRS.

    Raises ``KeyError`` for unknown bodies.
    """
    if abs(lat_deg) >= polar_lat_deg:
        pole = "south" if lat_deg < 0 else "north"
        return CANONICAL_CRS[f"{body}_{pole}"]
    return CANONICAL_CRS[f"{body}_geographic"]


def local_stereographic_crs(body: str, lat_deg: float, lon_deg: float) -> str:
    """Build an oblique stereographic CRS centered on ``(lat, lon)``.

    Raises ``ValueError`` at the poles, where it can collide with a canonical
    polar CRS; use `canonical_crs_for` there. Unknown bodies raise ``KeyError``.
    """
    if abs(lat_deg) >= 90.0 - 1e-9:
        raise ValueError(
            f"local_stereographic_crs is not valid at the poles (lat_deg={lat_deg!r}); "
            "use canonical_crs_for for polar regions instead."
        )
    if body == "earth":
        radius_or_ellps = "+ellps=WGS84"
    else:
        radius_or_ellps = f"+R={BODY_RADIUS_M[body]:.0f}"
    return (
        f"+proj=stere +lat_0={lat_deg:.10g} +lon_0={lon_deg:.10g} "
        f"{radius_or_ellps} +units=m +no_defs"
    )


def grids_identical(a: RasterGridSig, b: RasterGridSig, tol_m: float = 0.5) -> bool:
    """Check whether two raster grids share CRS, affine placement, and shape.

    Equivalent CRS spellings match by signature; sample pixels verify the
    affine grids agree within ``tol_m``.
    """
    if a.width != b.width or a.height != b.height:
        return False

    sig_a = _proj_signature(CRS.from_user_input(a.crs))
    sig_b = _proj_signature(CRS.from_user_input(b.crs))
    if not _signatures_match(sig_a, sig_b):
        return False

    sample_pixels = (
        (0, 0),
        (a.width // 2, a.height // 2),
        (a.width - 1, a.height - 1),
    )
    for col, row in sample_pixels:
        xa, ya = _apply_affine(a.transform, col, row)
        xb, yb = _apply_affine(b.transform, col, row)
        if math.hypot(xa - xb, ya - yb) > tol_m:
            return False
    return True
