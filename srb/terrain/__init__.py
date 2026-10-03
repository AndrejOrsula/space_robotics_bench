"""Offline terrain pipeline; `srb.terrain.api.bake_or_load` is its entry point."""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb.terrain.api import bake_or_load
    from srb.terrain.availability import (
        TerrainAvailability,
        TerrainAvailabilityStatus,
        assess_terrain_site,
        assess_terrain_spec,
    )
    from srb.terrain.spec import TerrainSpec

__all__ = [
    "TerrainAvailability",
    "TerrainAvailabilityStatus",
    "TerrainSpec",
    "assess_terrain_site",
    "assess_terrain_spec",
    "bake_or_load",
]


def __getattr__(name: str):
    if name == "bake_or_load":
        from srb.terrain.api import bake_or_load

        return bake_or_load
    if name in (
        "TerrainAvailability",
        "TerrainAvailabilityStatus",
        "assess_terrain_site",
        "assess_terrain_spec",
    ):
        availability = importlib.import_module("srb.terrain.availability")
        return getattr(availability, name)
    if name == "TerrainSpec":
        from srb.terrain.spec import TerrainSpec

        return TerrainSpec
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
