"""Parse cap, WKT polygon, and geographic box footprints; estimate coverage.

Coverage uses deterministic grid samples, not exact area. CRSs must match or
sampling raises `FootprintInvalidError`; full coverage compares integer counts.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Literal

from srb.terrain.errors import FootprintInvalidError
from srb.terrain.ingest.planetary_crs import xy_to_latlon

COVERAGE_SAMPLES: int = 33  # odd -> grid includes the region center + all 4 corners

_CAP_RE = re.compile(r"^cap:(?P<lat>\d+(?:\.\d+)?)(?P<hemi>[NS])$")
_MULTIPOLYGON_PREFIX = "MULTIPOLYGON"
_POLYGON_PREFIX = "POLYGON"
_BBOX_RE = re.compile(
    r"^bbox:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$"
)
"""`bbox:<lonmin>,<latmin>,<lonmax>,<latmax>` in degrees.

This shorthand supports non-polar rectangles. `lonmin > lonmax` denotes an
antimeridian-spanning box; `latmin > latmax` is invalid because latitude has
no wraparound."""

Ring = tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class Region:
    """An axis-aligned square patch on a canonical PROJECTED grid (metres)."""

    crs: str
    center_xy_m: tuple[float, float]
    size_m: float

    def corners_xy_m(self) -> tuple[tuple[float, float], ...]:
        """Return corners counter-clockwise, starting at `(min_x, min_y)`."""
        cx, cy = self.center_xy_m
        half = self.size_m / 2.0
        min_x, max_x = cx - half, cx + half
        min_y, max_y = cy - half, cy + half
        return ((min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y))

    def sample_points_xy_m(
        self, n: int = COVERAGE_SAMPLES
    ) -> tuple[tuple[float, float], ...]:
        """Return deterministic row-major samples spanning the square.

        Center-relative offsets preserve the exact center and corner floats
        returned by `corners_xy_m()` when `n` is odd.
        """
        # Zero samples would make coverage divide by zero or vacuously "full".
        if isinstance(n, bool) or not isinstance(n, int) or n < 1:
            raise ValueError(f"coverage sample count must be a positive int, got {n!r}")
        cx, cy = self.center_xy_m
        step = self.size_m / (n - 1) if n > 1 else 0.0
        mid = (n - 1) / 2.0
        xs = [cx + (col - mid) * step for col in range(n)]
        ys = [cy + (row - mid) * step for row in range(n)]
        return tuple((x, y) for y in ys for x in xs)


@dataclass(frozen=True, slots=True)
class Footprint:
    """A footprint compared in a region's projected CRS.

    Cap and box samples convert to lat/lon; polygon coordinates use the CRS.
    """

    crs: str
    kind: Literal["cap", "polygon", "bbox"]
    cap_lat_deg: float | None = None
    rings: tuple[Ring, ...] = ()
    bbox_deg: tuple[float, float, float, float] | None = None
    """`(lonmin, latmin, lonmax, latmax)` degrees for `kind == "bbox"`.

    Longitudes are normalized to `[-180, 180]`; `lonmin > lonmax` spans the
    antimeridian. Latitude bounds are ordered during parsing.
    """
    cap_hemisphere: Literal["N", "S"] | None = None


# --- WKT parsing (pure Python; no shapely) -----------------------------------


def _strip_parens(s: str) -> str:
    s = s.strip()
    if len(s) < 2 or s[0] != "(" or s[-1] != ")":
        raise FootprintInvalidError(f"expected a parenthesized group, got {s!r}")
    return s[1:-1].strip()


def _split_top_level(s: str) -> list[str]:
    """Split on top-level commas; raise for unbalanced parentheses."""
    parts: list[str] = []
    depth = 0
    start = 0
    for i, ch in enumerate(s):
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth < 0:
                raise FootprintInvalidError(f"unbalanced parentheses in {s!r}")
        elif ch == "," and depth == 0:
            parts.append(s[start:i])
            start = i + 1
    if depth != 0:
        raise FootprintInvalidError(f"unbalanced parentheses in {s!r}")
    parts.append(s[start:])
    return [p.strip() for p in parts]


def _parse_coordinate(token: str) -> tuple[float, float]:
    fields = token.split()
    if len(fields) != 2:
        raise FootprintInvalidError(f"malformed coordinate pair: {token!r}")
    try:
        x, y = float(fields[0]), float(fields[1])
    except ValueError as exc:
        raise FootprintInvalidError(f"malformed coordinate pair: {token!r}") from exc
    if not (math.isfinite(x) and math.isfinite(y)):
        raise FootprintInvalidError(f"non-finite coordinate pair: {token!r}")
    return x, y


def _parse_ring(s: str) -> Ring:
    points = tuple(_parse_coordinate(tok) for tok in _split_top_level(_strip_parens(s)))
    if len(points) < 4:
        raise FootprintInvalidError(
            "a polygon ring needs at least 3 points (4 coordinates including "
            f"closure), got {len(points)}: {s!r}"
        )
    if points[0] != points[-1]:
        raise FootprintInvalidError(f"a polygon ring must be closed: {s!r}")
    return points


def _parse_polygon_rings(s: str) -> tuple[Ring, ...]:
    return tuple(
        _parse_ring(ring_str) for ring_str in _split_top_level(_strip_parens(s))
    )


def _parse_multipolygon_rings(s: str) -> tuple[Ring, ...]:
    rings: list[Ring] = []
    for polygon_str in _split_top_level(_strip_parens(s)):
        rings.extend(_parse_polygon_rings(polygon_str))
    return tuple(rings)


def parse_footprint(spec: str, *, crs: str) -> Footprint:
    """Parse a cap, WKT polygon, or box in `crs`.

    Invalid forms raise `FootprintInvalidError`. Polygon coordinates use the
    projected CRS; box coordinates use degrees.
    """
    text = spec.strip()

    cap_match = _CAP_RE.match(text)
    if cap_match is not None:
        lat = float(cap_match.group("lat"))
        hemisphere = cap_match.group("hemi")
        if not 0.0 < lat <= 90.0:
            raise FootprintInvalidError(
                f"malformed 'cap:' footprint {spec!r}: latitude magnitude "
                "must be in (0, 90]; use 'cap:<lat>N' or 'cap:<lat>S'"
            )
        signed = -lat if hemisphere == "S" else lat
        return Footprint(
            crs=crs,
            kind="cap",
            cap_lat_deg=signed,
            cap_hemisphere=hemisphere,
        )

    bbox_match = _BBOX_RE.match(text)
    if bbox_match is not None:
        lonmin, latmin, lonmax, latmax = (float(g) for g in bbox_match.groups())
        if not (-360.0 <= lonmin <= 360.0 and -360.0 <= lonmax <= 360.0):
            raise FootprintInvalidError(
                f"malformed 'bbox:' footprint spec {spec!r}: lonmin/lonmax "
                "must each be in [-360, 360] (pre-normalization range)"
            )
        if not (-90.0 <= latmin <= 90.0 and -90.0 <= latmax <= 90.0):
            raise FootprintInvalidError(
                f"malformed 'bbox:' footprint spec {spec!r}: latmin/latmax "
                "must each be in [-90, 90]"
            )
        if latmin > latmax:
            raise FootprintInvalidError(
                f"malformed 'bbox:' footprint spec {spec!r}: latmin must be "
                "<= latmax -- unlike lonmin/lonmax, an inverted latitude "
                "band has no antimeridian-style meaning and is always an "
                "authoring error"
            )
        # Normalize stored bounds like sampled longitudes. Preserve full-globe
        # spans before normalization, which would otherwise collapse both
        # bounds to the same longitude.
        raw_span = lonmax - lonmin
        if raw_span != 0.0 and raw_span % 360.0 == 0.0:
            lonmin, lonmax = -180.0, 180.0
        else:
            lonmin = _normalize_lon_deg(lonmin)
            lonmax = _normalize_lon_deg(lonmax)
        return Footprint(
            crs=crs, kind="bbox", bbox_deg=(lonmin, latmin, lonmax, latmax)
        )

    upper = text.upper()
    if upper.startswith(_MULTIPOLYGON_PREFIX):
        rest = text[len(_MULTIPOLYGON_PREFIX) :].strip()
        rings = _parse_multipolygon_rings(rest)
        return Footprint(crs=crs, kind="polygon", rings=rings)
    if upper.startswith(_POLYGON_PREFIX):
        rest = text[len(_POLYGON_PREFIX) :].strip()
        rings = _parse_polygon_rings(rest)
        return Footprint(crs=crs, kind="polygon", rings=rings)

    raise FootprintInvalidError(
        f"unrecognized footprint spec {spec!r}: expected 'cap:<lat><N|S>', "
        "a WKT POLYGON/MULTIPOLYGON, or 'bbox:<lonmin>,<latmin>,<lonmax>,"
        "<latmax>'"
    )


# --- coverage sampling --------------------------------------------------------


def _cap_contains(
    cap_lat_deg: float,
    crs: str,
    x: float,
    y: float,
    hemisphere: Literal["N", "S"] | None = None,
) -> bool:
    lat_deg, _lon_deg = xy_to_latlon(x, y, crs)
    # Parsed caps preserve the explicit N/S token. The sign fallback retains
    # compatibility for callers constructing Footprint directly.
    is_north = hemisphere == "N" if hemisphere is not None else cap_lat_deg >= 0
    if is_north:
        return lat_deg > cap_lat_deg
    return lat_deg < cap_lat_deg


def _normalize_lon_deg(lon_deg: float) -> float:
    """Wrap an arbitrary longitude into `[-180, 180]`."""
    return ((lon_deg + 180.0) % 360.0) - 180.0


def _bbox_contains(
    bbox_deg: tuple[float, float, float, float], crs: str, x: float, y: float
) -> bool:
    """Convert a sample to lat/lon and test bounds, including antimeridian wrap."""
    lonmin, latmin, lonmax, latmax = bbox_deg
    lat_deg, lon_deg = xy_to_latlon(x, y, crs)
    if not (latmin <= lat_deg <= latmax):
        return False
    lon_deg = _normalize_lon_deg(lon_deg)
    if lonmin <= lonmax:
        return lonmin <= lon_deg <= lonmax
    return lon_deg >= lonmin or lon_deg <= lonmax


def _rings_bbox(rings: tuple[Ring, ...]) -> tuple[float, float, float, float] | None:
    xs = [x for ring in rings for x, _y in ring]
    ys = [y for ring in rings for _x, y in ring]
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def _ring_crossings(ring: Ring, x: float, y: float) -> int:
    """Count half-open even-odd crossings toward +x.

    For counter-clockwise rings, bottom/left edges count inside and top/right
    edges outside.
    """
    count = 0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_at_y:
                count += 1
    return count


def _polygon_contains(
    rings: tuple[Ring, ...],
    bbox: tuple[float, float, float, float] | None,
    x: float,
    y: float,
) -> bool:
    if bbox is None:
        return False
    min_x, min_y, max_x, max_y = bbox
    if not (min_x <= x <= max_x and min_y <= y <= max_y):
        return False
    # Summing parity across all rings handles both holes and disjoint exteriors.
    crossings = sum(_ring_crossings(ring, x, y) for ring in rings)
    return crossings % 2 == 1


def coverage_samples_inside(
    fp: Footprint, region: Region, *, samples: int = COVERAGE_SAMPLES
) -> tuple[int, int]:
    """Return `(inside, total)` sample counts for a footprint and region.

    Raises `FootprintInvalidError` when their CRSs differ. Caps use strict
    poleward latitude, polygons use even-odd ring crossings, and boxes handle
    antimeridian wrapping."""
    if fp.crs != region.crs:
        raise FootprintInvalidError(
            f"footprint crs {fp.crs!r} does not match region crs {region.crs!r} "
            "(coverage math must not compare a footprint and a region "
            "expressed in different CRSes)"
        )

    points = region.sample_points_xy_m(samples)
    total = len(points)

    if fp.kind == "cap":
        if fp.cap_lat_deg is None:
            raise FootprintInvalidError("a 'cap' footprint requires cap_lat_deg")
        inside = sum(
            1
            for x, y in points
            if _cap_contains(fp.cap_lat_deg, fp.crs, x, y, hemisphere=fp.cap_hemisphere)
        )
    elif fp.kind == "bbox":
        if fp.bbox_deg is None:
            raise FootprintInvalidError("a 'bbox' footprint requires bbox_deg")
        inside = sum(1 for x, y in points if _bbox_contains(fp.bbox_deg, fp.crs, x, y))
    else:
        bbox = _rings_bbox(fp.rings)
        inside = sum(1 for x, y in points if _polygon_contains(fp.rings, bbox, x, y))

    return inside, total


def coverage_fraction(
    fp: Footprint, region: Region, *, samples: int = COVERAGE_SAMPLES
) -> float:
    """Return the sampled fraction inside the footprint, from 0 to 1."""
    inside, total = coverage_samples_inside(fp, region, samples=samples)
    return inside / total


def covers_fully(
    fp: Footprint, region: Region, *, samples: int = COVERAGE_SAMPLES
) -> bool:
    """Return whether every sampled point is inside, using integer counts."""
    inside, total = coverage_samples_inside(fp, region, samples=samples)
    return inside == total


__all__ = [
    "COVERAGE_SAMPLES",
    "Footprint",
    "Region",
    "coverage_fraction",
    "coverage_samples_inside",
    "covers_fully",
    "parse_footprint",
]
