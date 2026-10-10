"""``srb asset list <kind> [--body]``."""

from __future__ import annotations

import sys
from typing import Optional

from srb.cli.commands._assets._catalog import get_asset_kind
from srb.cli.commands._assets._common import filter_by_body


def _list_terrain(body: Optional[str]) -> int:
    from srb.terrain.lifecycle import assess_terrain_asset
    from srb.terrain.manifest_loader import load_manifest

    sites = load_manifest().sites
    sites = filter_by_body(sites, body)
    for name, entry in sorted(sites.items()):
        availability = assess_terrain_asset(name).availability
        issue = (
            f"  issue={availability.issue.kind.value}"
            if availability.issue is not None
            else ""
        )
        print(
            f"{name:24s}  body={entry.body}  "
            f"status={availability.status.value}{issue}  name={entry.name!r}"
            f"{_ephemeris_lighting_suffix(entry)}"
        )
    return len(sites)


def _ephemeris_lighting_suffix(entry) -> str:
    """Provenance suffix for a site whose lighting is the ``ephemeris`` sentinel.

    Kernels present -> compute + show ``sun=EL°@AZ° (ephemeris @ <utc>, true-north)``
    (the RAW true-north azimuth; the bake folds ``+gamma`` onto grid north);
    kernels absent -> ``(ephemeris @ <utc>, kernels missing)`` WITHOUT any query.
    Any other lighting value returns ``""`` so the plain columns are unchanged.
    """
    from srb.terrain.lights import ephemeris_bridge

    if getattr(entry, "lighting", None) != ephemeris_bridge.EPHEMERIS_LIGHTING:
        return ""
    epoch = getattr(entry, "epoch", None) or "?"
    if not ephemeris_bridge._kernels_present():
        return f"  (ephemeris @ {epoch}, kernels missing)"
    try:
        import srb.core.ephemeris as eph
        from srb.core.ephemeris.config import SiteSpec

        state = eph.query(
            SiteSpec(
                body=entry.body,
                lat_deg=entry.coords.lat,
                lon_deg=entry.coords.lon,
            ),
            epoch,
            bodies=("sun",),
        )
        sun = state.bodies["sun"]
        # RAW true-north sun azimuth (the bake folds +gamma onto grid north);
        # labeled so it is not misread as the grid azimuth at a gamma != 0 site.
        return (
            f"  sun={sun.elevation_deg:.1f}°@{sun.azimuth_deg:.0f}° "
            f"(ephemeris @ {epoch}, true-north)"
        )
    except Exception:  # noqa: BLE001 -- provenance is best-effort presentation
        from srb.utils import logging

        logging.debug(
            "ephemeris query failed for list suffix of terrain site %r @ %s; "
            "showing no-detail provenance",
            getattr(entry, "name", "?"),
            epoch,
            exc_info=True,
        )
        return f"  (ephemeris @ {epoch})"


def _list_rocks(body: Optional[str]) -> int:
    from srb.terrain.rocks.catalog_loader import load_rocks_catalog

    sets = load_rocks_catalog().sets
    sets = filter_by_body(sets, body)
    for name, rs in sorted(sets.items()):
        print(f"{name:24s}  body={rs.body}  rocks={len(rs.rocks)}")
    return len(sets)


def _list_hdri(body: Optional[str]) -> int:
    from srb.terrain.hdri.catalog_loader import load_hdri_catalog

    domes = load_hdri_catalog().skydomes
    domes = filter_by_body(domes, body)
    for name, entry in sorted(domes.items()):
        print(
            f"{name:24s}  body={entry.body}  format={entry.format}  size={entry.resize_to}"
        )
    return len(domes)


def _list_lights(body: Optional[str]) -> int:
    from srb.terrain.lights.catalog_loader import load_lighting_catalog

    presets = load_lighting_catalog().presets
    presets = filter_by_body(presets, body)
    for name, p in sorted(presets.items()):
        print(
            f"{name:24s}  body={p.body}  sun={p.sun_elevation_deg:.1f}°@{p.sun_azimuth_deg:.0f}°"
        )
    return len(presets)


def run(*, kind: str, body: Optional[str] = None, **_unused) -> int:
    asset_kind = get_asset_kind(kind)
    if not asset_kind.supports_list:
        raise ValueError(f"list: unsupported asset kind: {kind!r}")
    if not _LIST_HANDLERS[kind](body):
        where = f" for body {body!r}" if body else ""
        print(f"No {kind} entries{where}.", file=sys.stderr)
    return 0


_LIST_HANDLERS = {
    "terrain": _list_terrain,
    "rocks": _list_rocks,
    "hdri": _list_hdri,
    "lights": _list_lights,
}
