"""Optional dependency probes for terrain fetch/bake workflows."""

from __future__ import annotations

from importlib.util import find_spec

_BAKE_MODULES = ("rasterio", "pxr")
_IMG_MODULES = ("pdr",)


def missing_bake_dependencies() -> tuple[str, ...]:
    """Return import module names required by the terrain bake pipeline."""
    return tuple(module for module in _BAKE_MODULES if find_spec(module) is None)


def missing_img_dependencies() -> tuple[str, ...]:
    """Optional modules required for the PDS3 `.IMG` fallback path
    (`srb.terrain.sources.pds_img`) when GDAL's own PDS driver is
    unavailable."""
    return tuple(module for module in _IMG_MODULES if find_spec(module) is None)
