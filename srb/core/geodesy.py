"""Optional CRS transforms shared by core consumers.

The terrain package owns CRS catalogs, normalization and raster policies. This
module owns only the task-independent coordinate transforms and true-north
convergence that those consumers need. ``pyproj`` remains optional at import
sites that use this module: callers can import it lazily and translate a
missing dependency into their own failure taxonomy.

All angles are degrees, all lengths are metres. Transformer caches are
bounded because ``pyproj`` objects hold native PROJ resources.
"""

from __future__ import annotations

import math
from functools import lru_cache

from pyproj import CRS, Transformer

# Latitude step used by ``north_convergence_deg``'s finite difference. Keep
# this exact value with the original terrain implementation: it is large
# enough to stay above float64 noise and small enough for the local-linear
# approximation used by the convergence calculation.
_TRUE_NORTH_STEP_DEG = 1e-5


# Bounded. Keyed by a CRS string; ``pyproj`` ``Transformer``
# objects hold native PROJ resources, so an unbounded cache pins one per CRS
# ever seen for the process lifetime. The shipped manifest uses three CRSs.
@lru_cache(maxsize=64)
def _crs_pair(crs: str) -> tuple[CRS, CRS]:
    target = CRS.from_user_input(crs)
    source = target.geodetic_crs or target
    return source, target


@lru_cache(maxsize=64)
def _forward_transformer(crs: str) -> Transformer:
    """lat/lon (degrees) -> grid XY (metres), source geodetic_crs -> target."""
    source, target = _crs_pair(crs)
    return Transformer.from_crs(source, target, always_xy=True)


@lru_cache(maxsize=64)
def _inverse_transformer(crs: str) -> Transformer:
    """grid XY (metres) -> lat/lon (degrees), target -> source geodetic_crs."""
    source, target = _crs_pair(crs)
    return Transformer.from_crs(target, source, always_xy=True)


def latlon_to_xy(lat_deg: float, lon_deg: float, crs: str) -> tuple[float, float]:
    x, y = _forward_transformer(crs).transform(lon_deg, lat_deg)
    return float(x), float(y)


def xy_to_latlon(x_m: float, y_m: float, crs: str) -> tuple[float, float]:
    lon, lat = _inverse_transformer(crs).transform(x_m, y_m)
    return float(lat), float(lon)


def north_convergence_deg(lat_deg: float, lon_deg: float, crs: str) -> float:
    """Grid convergence angle γ such that ``az_grid = az_true + γ``.

    Azimuths are clockwise from grid +Y (true north is clockwise-positive
    from true north too). Computed by finite difference: a true-north step
    is exactly a constant-longitude step toward increasing latitude (the
    local meridian direction), so its image in grid XY -- transformed via
    the same forward transform ``latlon_to_xy`` uses -- gives the true-north
    direction expressed in grid coordinates. γ is that vector's bearing off
    grid +Y, measured clockwise (``atan2(dx, dy)``), never assumed from a
    closed-form formula.
    """
    x0, y0 = latlon_to_xy(lat_deg, lon_deg, crs)
    # Latitude cannot increase beyond the north pole. Step equatorward there
    # and reverse the projected vector so the bearing still points true north.
    step = (
        -_TRUE_NORTH_STEP_DEG
        if lat_deg + _TRUE_NORTH_STEP_DEG > 90.0
        else _TRUE_NORTH_STEP_DEG
    )
    x1, y1 = latlon_to_xy(lat_deg + step, lon_deg, crs)
    direction = 1.0 if step > 0.0 else -1.0
    return math.degrees(math.atan2(direction * (x1 - x0), direction * (y1 - y0)))
