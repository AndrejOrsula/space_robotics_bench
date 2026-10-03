"""Pydantic v2 models and validation for the terrain manifest."""

from __future__ import annotations

import math
import re
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, StringConstraints, model_validator

Body = Literal["moon", "mars", "earth"]
Lod = Literal["low", "medium", "high"]
ProviderName = Literal[
    "usgs_astro",
    "lroc_pds",
    "pds_geosciences",
    "openplanetary",
    "local",
    "pgda",
    "zenodo",
    "uahirise",
    "copernicus_s3",
    "usgs_3dep",
    "imbrium",
]

Sha256 = Annotated[
    str,
    StringConstraints(pattern=r"^[0-9a-fA-F]{64}$", min_length=64, max_length=64),
]


class _ManifestModel(BaseModel):
    # Reject unknown keys so a typo in an optional setting fails validation
    # instead of silently falling back to the default.
    model_config = ConfigDict(extra="forbid")


class Coords(_ManifestModel):
    lat: float = Field(..., ge=-90.0, le=90.0)
    lon: float = Field(..., ge=-360.0, le=360.0)


class SourceEntry(_ManifestModel):
    provider: ProviderName
    url: str
    sha256: Sha256
    crs: str | None = None
    gsd_m: float = Field(..., gt=0.0, allow_inf_nan=False)
    license: str = Field(..., min_length=1)
    attribution: str = Field(..., min_length=1)
    allow_large_download: bool = False


class StackRef(_ManifestModel):
    """Reference a v2 catalog layer in a DEM or color stack.

    Manifest order is ignored: resolved stacks use priority order, with the
    declared DEM base first.
    """

    layer_id: str
    role: Literal["base", "inset"] = "inset"
    max_gsd_m: float | None = Field(default=None, gt=0.0, allow_inf_nan=False)


class Sources(_ManifestModel):
    dem: SourceEntry | None = None
    ortho: SourceEntry | None = None
    dem_stack: list[StackRef] | None = None
    color_stack: list[StackRef] | None = None

    @model_validator(mode="after")
    def _validate_dem_and_stacks(self) -> "Sources":
        has_dem = self.dem is not None
        has_dem_stack = self.dem_stack is not None
        if has_dem == has_dem_stack:
            raise ValueError("sources must define exactly one of 'dem' or 'dem_stack'")
        if self.ortho is not None and self.color_stack is not None:
            raise ValueError("sources cannot define both 'ortho' and 'color_stack'")
        if has_dem_stack:
            base_count = sum(1 for ref in self.dem_stack if ref.role == "base")
            if base_count != 1:
                raise ValueError(
                    "dem_stack must contain exactly one entry with role='base' "
                    f"(found {base_count})"
                )
        return self


class PatchEntry(_ManifestModel):
    center_xy_m: tuple[float, float] | None = None
    center_latlon: tuple[float, float] | None = None
    size_m: float = Field(..., gt=0.0, allow_inf_nan=False)
    lod: Lod = "medium"

    @model_validator(mode="after")
    def _validate_center(self) -> "PatchEntry":
        has_xy = self.center_xy_m is not None
        has_latlon = self.center_latlon is not None
        if has_xy == has_latlon:
            raise ValueError(
                "patch must define exactly one of 'center_xy_m' or 'center_latlon'"
            )
        if self.center_xy_m is not None and not all(
            math.isfinite(v) for v in self.center_xy_m
        ):
            raise ValueError("patch center_xy_m must be finite")
        if self.center_latlon is not None:
            lat, lon = self.center_latlon
            if not math.isfinite(lat) or not -90.0 <= lat <= 90.0:
                raise ValueError(
                    "patch center_latlon latitude must be finite and in [-90, 90]"
                )
            if not math.isfinite(lon) or not -360.0 <= lon <= 360.0:
                raise ValueError(
                    "patch center_latlon longitude must be finite and in [-360, 360]"
                )
        return self


class SiteEntry(_ManifestModel):
    body: Body
    name: str
    coords: Coords
    sources: Sources
    patches: dict[str, PatchEntry] = Field(default_factory=dict)
    sky: str | None = None
    lighting: str | None = None
    rocks: str | None = None
    # Default preset for scene rock scatter and bake generation.
    augment_preset: str | None = None
    epoch: str | None = None
    # Site default; a request may enable deshading but cannot disable this.
    deshade: bool = False

    @model_validator(mode="after")
    def _patches_have_default(self) -> "SiteEntry":
        if "default" not in self.patches:
            raise ValueError("site must define a 'default' patch")
        return self

    @model_validator(mode="after")
    def _validate_epoch(self) -> "SiteEntry":
        if self.epoch is not None:
            from srb.core.ephemeris.config import EpochSpec

            EpochSpec(utc=self.epoch).validate()  # ISO-8601 + de440s bounds
        return self


_SITE_KEY_RE = re.compile(r"^[a-z][a-z0-9_]*$")


class Manifest(_ManifestModel):
    version: Literal[1, 2]
    sites: dict[str, SiteEntry]

    @model_validator(mode="after")
    def _validate_site_keys(self) -> "Manifest":
        for key in self.sites:
            if not _SITE_KEY_RE.match(key):
                raise ValueError(f"invalid site key '{key}'; must be lower_snake_case")
        return self

    @model_validator(mode="after")
    def _validate_version_1_excludes_v2_fields(self) -> "Manifest":
        if self.version != 1:
            return self
        for site_key, site in self.sites.items():
            if site.sources.dem_stack is not None:
                raise ValueError(
                    f"site '{site_key}': version 1 manifest cannot use "
                    "'dem_stack' (v2-only field; set version: 2)"
                )
            if site.sources.color_stack is not None:
                raise ValueError(
                    f"site '{site_key}': version 1 manifest cannot use "
                    "'color_stack' (v2-only field; set version: 2)"
                )
            for patch_key, patch in site.patches.items():
                if patch.center_latlon is not None:
                    raise ValueError(
                        f"site '{site_key}' patch '{patch_key}': version 1 "
                        "manifest cannot use 'center_latlon' (v2-only field; "
                        "set version: 2)"
                    )
        return self
