"""Per-env location-variety request derivation.

One region request can fan out into K distinct :class:`RegionRequest`s:
index 0 is ALWAYS the requested center verbatim; indices 1..K-1 are
deterministic seeded offsets sampled uniformly in a disk of radius
``location_spread_m`` in the request's own projected CRS (polar-stereographic
near the poles, local oblique stereographic elsewhere -- see
`RegionRequest.resolved_crs`). The RNG stream depends only on ``seed`` (fixed
salt 905), never on K, so growing ``num_locations`` preserves the existing
slug prefix and its cache entries.

Variety derivation is not polar-only: `RegionRequest.resolved_crs` resolves
a valid projected CRS at every latitude, so non-polar centers fan out
exactly like polar ones.
"""

from __future__ import annotations

import math
from collections import Counter
from typing import Sequence

import numpy as np

from srb.terrain.catalog.resolve import RegionRequest
from srb.terrain.errors import RegionInvalidError
from srb.terrain.ingest.planetary_crs import latlon_to_xy, xy_to_latlon
from srb.terrain.log import get_logger
from srb.terrain.schema import Body, Lod

# Fixed salt decoupling this RNG stream from every other consumer of `seed`
# Changing it re-derives every non-primary patch center -- treat it
# like a cache-format constant.
_LOCATION_STREAM_SALT = 905

_log = get_logger("locations")


def derive_location_requests(
    *,
    body: Body,
    lat_deg: float,
    lon_deg: float,
    size_m: float,
    num_locations: int = 1,
    location_spread_m: float | None = None,
    seed: int = 0,
    lod: Lod = "medium",
    locations: Sequence[tuple[float, float]] | None = None,
) -> list[RegionRequest]:
    """Derive the ordered list of region requests for a variety bake.

    Raises:
        RegionInvalidError: invalid counts/spread, or post-normalization
            slug collisions.
    """
    if locations is not None:
        if num_locations != 1:
            raise RegionInvalidError(
                size_m,
                reason=(
                    "`locations` and `num_locations` are mutually exclusive; "
                    "the list length already fixes the count"
                ),
                remediation="Pass either `locations` or `num_locations`, not both.",
            )
        if not locations:
            raise RegionInvalidError(
                size_m,
                reason="`locations` must be non-empty",
                remediation="Pass at least one `(lat_deg, lon_deg)` entry in `locations`.",
            )
        if location_spread_m is not None:
            _log.warning(
                "location_spread_m=%s is ignored: explicit `locations` fix every "
                "patch center",
                location_spread_m,
            )
        requests = [
            RegionRequest(body=body, lat_deg=lat, lon_deg=lon, size_m=size_m, lod=lod)
            for lat, lon in locations
        ]
        _require_distinct(requests, size_m, hint="deduplicate `locations`")
        return requests

    if num_locations < 1:
        raise RegionInvalidError(
            size_m,
            reason=f"num_locations must be >= 1, got {num_locations}",
            remediation="Pass --num-locations as an integer >= 1.",
        )

    primary = RegionRequest(
        body=body, lat_deg=lat_deg, lon_deg=lon_deg, size_m=size_m, lod=lod
    )
    if num_locations == 1:
        if location_spread_m is not None:
            _log.warning(
                "location_spread_m=%s is ignored: num_locations=1 bakes only the "
                "requested center",
                location_spread_m,
            )
        return [primary]

    spread = float(location_spread_m) if location_spread_m is not None else size_m
    if spread <= 0.0:
        raise RegionInvalidError(
            size_m,
            reason=f"location_spread_m must be > 0, got {spread}",
            remediation="Pass --location-spread-m as a positive number of meters.",
        )

    crs = primary.resolved_crs
    cx, cy = latlon_to_xy(primary.lat_deg, primary.lon_deg, crs)
    rng = np.random.default_rng([_LOCATION_STREAM_SALT, seed])
    requests = [primary]
    for _ in range(num_locations - 1):
        u, v = rng.random(), rng.random()
        r = spread * math.sqrt(u)
        theta = 2.0 * math.pi * v
        lat, lon = xy_to_latlon(cx + r * math.cos(theta), cy + r * math.sin(theta), crs)
        requests.append(
            RegionRequest(body=body, lat_deg=lat, lon_deg=lon, size_m=size_m, lod=lod)
        )
    _require_distinct(requests, size_m, hint="increase `location_spread_m`")
    return requests


def _require_distinct(
    requests: Sequence[RegionRequest], size_m: float, *, hint: str
) -> None:
    slugs = [r.slug for r in requests]
    dupes = sorted(slug for slug, count in Counter(slugs).items() if count > 1)
    if dupes:
        raise RegionInvalidError(
            size_m,
            reason=(
                "location variety requires distinct region slugs after "
                f"normalization (1e-4 deg quantum); duplicates: {dupes}; {hint}"
            ),
            remediation=hint[0].upper() + hint[1:] + ".",
        )


__all__ = ["derive_location_requests"]
