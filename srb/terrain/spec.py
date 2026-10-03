"""Immutable description of a terrain bake."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from srb.terrain.errors import LicenseMissingError

if TYPE_CHECKING:
    from srb.terrain.catalog.resolve import RegionRequest

Body = Literal["moon", "mars", "earth"]
Lod = Literal["low", "medium", "high"]


@dataclass(frozen=True, slots=True)
class TerrainSpec:
    body: Body
    site: str | None = None
    dem_path: Path | None = None
    site_name: str | None = None
    attribution: str | None = None
    region: "RegionRequest | None" = None

    patch: str = "default"
    patch_size_m: float | None = None
    patch_center_xy_m: tuple[float, float] | None = None

    lod: Lod = "medium"
    seed: int = 0

    use_ortho: bool = True
    materials_version: int | None = None

    # Request-level deshade. Effective deshade is manifest OR request; v1
    # bakes reject an effective `True` value.
    deshade: bool = False

    # Request-level opt-in for synthetic catalog layers. Region specs take
    # this setting from `region.allow_synthetic`; other specs use this field.
    allow_synthetic: bool = False

    # Bake-time opt-in to stamp generated craters into the DEM. Included in
    # the cache key only when true; valid for v1 and v2 specs.
    augment_stamp: bool = False

    # Optional preset override for augmentation generation and crater
    # stamping. `None` keeps the body default; the selected preset is keyed.
    augment_preset: str | None = None

    def __post_init__(self) -> None:
        selected = [
            name
            for name, value in (
                ("dem_path", self.dem_path),
                ("site", self.site),
                ("region", self.region),
            )
            if value is not None
        ]
        if len(selected) != 1:
            raise ValueError(
                "TerrainSpec needs exactly one of site or dem_path or region "
                f"(got {selected or 'none'})"
            )
        if self.dem_path is not None:
            if not self.site_name:
                raise ValueError("`site_name` is required when `dem_path` is set")
            # `local_<site_name>` becomes one cache directory name.
            if "/" in self.site_name or "\x00" in self.site_name:
                raise ValueError(
                    f"`site_name` must be a single path component, got {self.site_name!r}"
                )
            if not (self.attribution and self.attribution.strip()):
                raise LicenseMissingError(
                    "user-supplied DEMs require a non-blank `attribution` string"
                )
        if self.region is not None:
            if self.region.body != self.body:
                raise ValueError(
                    f"region body {self.region.body!r} does not match terrain body {self.body!r}"
                )
            # CRS overrides are absent from the slug and cache key, so reject
            # them to keep region bakes deterministic.
            if self.region.crs is not None:
                raise ValueError(
                    "region CRS override is not supported on terrain specs; "
                    "the canonical CRS is derived from the request latitude"
                )
            # Region bakes use the region's size and center; reject patch
            # overrides that would be ignored but still affect the cache key.
            if self.patch_size_m is not None or self.patch_center_xy_m is not None:
                raise ValueError(
                    "region terrain specs do not support `patch_size_m` or "
                    "`patch_center_xy_m`; the region's own `size_m` and "
                    "resolved center already determine the bake footprint"
                )
            # Region resolution and caching use `region.allow_synthetic`.
            if self.allow_synthetic and not self.region.allow_synthetic:
                raise ValueError(
                    "region terrain specs read `allow_synthetic` from the "
                    "RegionRequest itself; set it on the region "
                    "(RegionRequest(allow_synthetic=True)) instead of the spec"
                )
        # Preset validation stays in the shared resolver to avoid duplicate
        # policy. RealTerrain construction resolves the cache key eagerly;
        # unknown or body-mismatched presets therefore fail before baking.

    @property
    def is_local(self) -> bool:
        return self.dem_path is not None

    @property
    def cache_site(self) -> str:
        if self.region is not None:
            return self.region.slug
        if self.is_local:
            assert self.site_name is not None
            return f"local_{self.site_name}"
        assert self.site is not None
        return self.site
