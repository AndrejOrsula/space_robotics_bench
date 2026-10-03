"""IAU 2015 spheroid registry per body.

The values are short symbolic identifiers that the manifest, the runtime
DEM-loader, and the bake's `meta.json` all share. Real proj/wkt strings live
in the DEM files themselves; we only normalize the identifier here.
"""

from __future__ import annotations

from typing import Literal

from srb.terrain.errors import MissingCRSError

Body = Literal["moon", "mars", "earth"]

BODY_DEFAULT_CRS: dict[str, str] = {
    "moon": "IAU_Moon_2015",
    "mars": "IAU_Mars_2015",
    "earth": "IAU_Earth_2015",
}

KNOWN_CRS: frozenset[str] = frozenset(
    {
        "IAU_Moon_2015",
        "IAU_Mars_2015",
        "IAU_Earth_2015",
        # Future: add Phobos/Deimos/Mercury/asteroid CRSes when they're added.
    }
)


def body_crs(body: Body) -> str:
    return BODY_DEFAULT_CRS[body]


def is_known_crs(crs: str) -> bool:
    return crs in KNOWN_CRS


def resolve_crs(
    *,
    body: Body,
    explicit: str | None,
    from_dem: str | None,
) -> str:
    """Pick the effective CRS in priority order: explicit → from_dem → body default."""
    if explicit is not None:
        if not is_known_crs(explicit):
            raise MissingCRSError(
                f"CRS '{explicit}' is unsupported; choose one of "
                f"{', '.join(sorted(KNOWN_CRS))}"
            )
        return explicit
    if from_dem is not None and is_known_crs(from_dem):
        return from_dem
    return body_crs(body)
