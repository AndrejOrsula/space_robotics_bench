"""Pydantic models for the v2 per-body source-layer catalog (revision 3)."""

from __future__ import annotations

import math
import unicodedata
from typing import Literal
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from srb.terrain.schema import Body, ProviderName, Sha256

LayerKind = Literal["dem", "color", "mask", "quality"]

# Photometric processing class of a color layer.
Radiometry = Literal[
    "normalized_albedo",
    "illumination_averaged",
    "single_illumination",
    "secondary_illumination",
    "synthetic",
]

AlignmentSource = Literal["published", "pc_align", "cross_correlation", "assumed_zero"]

AccessMode = Literal["cog_remote", "download", "zip_member"]


def url_basename(url: str) -> str:
    """Return the URL path basename, without query or fragment.

    Public for the v2 ``LayerLoader`` to key ``layer.sha256`` consistently.
    """
    return urlsplit(url).path.rsplit("/", 1)[-1]


_url_basename = url_basename
"""Backward-compatible private alias -- every call site that imported this
under its original (module-private) name keeps working unchanged."""


def _validate_safe_basename(value: str, *, label: str) -> str:
    """Require a literal, portable filename rather than a path fragment."""
    if not value or value in {".", ".."}:
        raise ValueError(f"{label} must be a non-dot basename")
    if "/" in value or "\\" in value:
        raise ValueError(f"{label} must not contain path separators")
    if len(value) >= 2 and value[1] == ":" and value[0].isalpha():
        raise ValueError(f"{label} must not contain a drive prefix")
    if any(unicodedata.category(character) == "Cc" for character in value):
        raise ValueError(f"{label} must not contain NUL or control characters")
    return value


class AlignmentRecord(BaseModel):
    """Co-registration shift from a layer to its reference.

    ``reference=None`` denotes a layer defining its own datum. Use
    ``source="assumed_zero"`` when no published or measured shift exists.
    """

    model_config = ConfigDict(extra="forbid")

    reference: str | None = None
    dx_m: float = Field(default=0.0, allow_inf_nan=False)
    dy_m: float = Field(default=0.0, allow_inf_nan=False)
    dz_m: float = Field(default=0.0, allow_inf_nan=False)
    source: AlignmentSource = "assumed_zero"


class QualitySemantics(BaseModel):
    """Meaning of a quality layer's pixel values."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    measure: Literal["vertical_one_sigma_m"]


class MaskEncoding(BaseModel):
    """Map raw mask pixel values to masked and nodata categories.

    Applied at ingest; without a declaration, values pass through unchanged.
    Values use exact equality and must be exactly representable as floats.
    """

    model_config = ConfigDict(extra="forbid")

    masked_values: list[float] = Field(..., min_length=1)
    """Pixel values that mean "inside the mask" (e.g. `[20000]` for the raw
    PGDA LPSR). Everything not listed here and not in `nodata_values` decodes
    to `0.0` ("outside the mask")."""
    nodata_values: list[float] = Field(default_factory=list)
    """Pixel values that mean "no observation", decoded to `NaN` -- which
    `psr_mask` reads as "not in this mask", the same as `0.0`, but which
    keeps the distinction visible to anything else that inspects the array.
    Empty is legal: a product with no nodata sentinel."""

    @field_validator("masked_values", "nodata_values", mode="before")
    @classmethod
    def _coerce_exact_numeric_values(cls, values):
        if not isinstance(values, (list, tuple)):
            raise ValueError("mask values must be a sequence of numeric values")
        normalized = []
        for value in values:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(
                    f"mask value must be an integer or float, got {value!r}"
                )
            try:
                converted = float(value)
            except OverflowError as exc:
                raise ValueError(
                    f"mask value is outside float range: {value!r}"
                ) from exc
            if isinstance(value, int) and int(converted) != value:
                raise ValueError(
                    f"integer mask value is not exactly representable: {value!r}"
                )
            normalized.append(converted)
        return normalized

    @field_validator("masked_values", "nodata_values")
    @classmethod
    def _validate_finite_and_unique(cls, v: list[float]) -> list[float]:
        for value in v:
            if not math.isfinite(value):
                raise ValueError(
                    f"non-finite mask value {value!r}: decoding compares raw "
                    "pixel values for exact equality (non-finite input pixels "
                    "decode to NaN unconditionally)"
                )
        if len(set(v)) != len(v):
            raise ValueError(f"duplicate mask values in {v!r}")
        return v

    @model_validator(mode="after")
    def _validate_disjoint(self) -> "MaskEncoding":
        overlap = sorted(set(self.masked_values) & set(self.nodata_values))
        if overlap:
            raise ValueError(
                f"value(s) {overlap} are declared BOTH masked and nodata; a "
                "single DN cannot mean two things"
            )
        return self


class SourceSidecar(BaseModel):
    """One explicitly declared sibling file in a downloadable product."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    url: str = Field(..., min_length=1)
    filename: str = Field(..., min_length=1)
    sha256: str = Field(..., pattern=r"^[0-9a-f]{64}$")
    size_bytes: int = Field(..., gt=0)

    @field_validator("filename")
    @classmethod
    def _validate_filename(cls, value: str) -> str:
        return _validate_safe_basename(value, label="sidecar filename")


class SourceLayer(BaseModel):
    """One product in the per-body catalog."""

    # `extra="forbid"`: without it Pydantic's
    # default is `extra="ignore"`, so a catalog spelling `mask_encodding:`
    # would load cleanly with `mask_encoding is None` -- the mask would then
    # pass through raw and the whole south pole would read as PSR, SILENTLY.
    # `MaskEncoding`'s own `extra="forbid"` only protects keys INSIDE an
    # already-recognised block, so it cannot catch that. This is
    # curated, reviewed data (see the module docstring): an unknown key here
    # is always an authoring error, never forward compatibility.
    model_config = ConfigDict(extra="forbid")

    # `layer_id` is
    # interpolated unvalidated into a filesystem path in
    # `srb/terrain/sources/vsicurl.py::patch_cog_path`
    # (`<cache_root>/_raw/_windows/<layer_id>/<key>.tif`). A bad catalog
    # commit is the only way to exploit an unconstrained value here, but
    # `_extract_zip_members` guards the analogous zip-member-name
    # case explicitly (`acquisition.py::_is_unsafe_zip_member_name`), and
    # this constraint keeps the two symmetric.
    layer_id: str = Field(..., min_length=1, pattern=r"^[A-Za-z0-9_.-]+$")
    body: Body
    kind: LayerKind
    product_family: str = Field(..., min_length=1)
    provider: ProviderName
    urls: list[str] = Field(..., min_length=1)
    sha256: dict[str, Sha256] = Field(
        ...,
        min_length=1,
        description="Per-file hash; multiple entries for zip members.",
    )
    native_gsd_m: float = Field(..., gt=0.0, allow_inf_nan=False)
    crs: str | None = None
    """Product CRS; ``None`` means read it from the raster.

    For WKT polygon footprints, this is the coordinate frame. If omitted,
    polygons use the body's canonical south-polar CRS. Declare any other
    frame explicitly so the resolver compares coordinates correctly.
    """
    radiometry: Radiometry | None = None
    footprint: str
    vertical_datum: str | None = None
    alignment: AlignmentRecord
    license: str = Field(..., min_length=1)
    attribution: str = Field(..., min_length=1)
    size_bytes: int = Field(..., gt=0)
    access: AccessMode
    primary_size_bytes: int | None = Field(default=None, gt=0)
    """Exact primary-file size, required when ``sidecars`` is non-empty."""
    sidecars: tuple[SourceSidecar, ...] = Field(default_factory=tuple)
    """Ordered, explicit sibling files for ``access == 'download'``."""
    priority_class: int = Field(100, ge=1)
    """Lower wins. DEM rows use 1..5; color rows use 1..4.
    Ties break by (native_gsd_m, layer_id) so resolution is total."""
    allow_large_download: bool = False
    """Catalog consent for >1 GiB downloads; also requires
    ``SRB_TERRAIN_LARGE_OK``.
    """
    reference_only: bool = False
    """Exclude from footprint resolution; allowed reference stacks can resolve these layers by ID.

    Use for comparison references that should not become bake inputs.
    Priority only orders candidates after footprint resolution admits them.
    Default ``False`` preserves ordinary footprint-based resolution.
    """
    sun_azimuth_deg: float | None = None
    """Capture-time sun bearing in degrees (0=N, 90=E, 180=S, 270=W).

    Used to remove product shading. Missing geometry makes deshading skip
    the layer rather than guess a sun position.
    """
    sun_elevation_deg: float | None = None
    """Capture-time sun elevation, paired with ``sun_azimuth_deg``."""
    mask_encoding: MaskEncoding | None = None
    """Decode raw DNs for mask layers; absent means pass values through.

    Only valid for ``kind="mask"``; other layer kinds reject the field.
    """

    @field_validator("layer_id")
    @classmethod
    def _validate_layer_id_segment(cls, value: str) -> str:
        if value in {".", ".."}:
            raise ValueError("layer_id must not be '.' or '..'")
        return value

    quality_semantics: QualitySemantics | None = None
    """Meaning of a quality layer when declared.

    Optional for compatibility; fusion fails closed when required semantics
    are absent.
    """

    @field_validator("footprint")
    @classmethod
    def _validate_footprint(cls, v: str) -> str:
        from srb.terrain.catalog.footprint import parse_footprint
        from srb.terrain.errors import FootprintInvalidError

        try:
            parse_footprint(v, crs="catalog-validation")
        except FootprintInvalidError as exc:
            raise ValueError(f"invalid footprint {v!r}: {exc}") from exc
        return v

    @field_validator("license", "attribution")
    @classmethod
    def _validate_no_embedded_newlines(cls, v: str) -> str:
        """Prevent verbatim license or attribution values forging file blocks."""
        if "\n" in v or "\r" in v:
            raise ValueError(
                "must not contain newlines -- embedded verbatim as a single "
                "line in ATTRIBUTION.txt; a newline (e.g. a blank line) "
                "could forge an extra, fake block on that legal-compliance "
                "artifact"
            )
        return v

    @model_validator(mode="after")
    def _validate_sha_map_matches_access(self) -> "SourceLayer":
        """Validate SHA keys for access mode and the first URL basename.

        Zip-member sources require archive and member keys; other modes need
        only the archive key. Reject an empty basename.
        """
        archive_key = _url_basename(self.urls[0])
        if not archive_key:
            raise ValueError(
                f"urls[0]={self.urls[0]!r} has an empty basename; cannot "
                "derive a sha256 key from a bare-directory URL"
            )
        if self.access == "zip_member":
            if archive_key not in self.sha256:
                raise ValueError(
                    f"access='zip_member' requires sha256 to contain the "
                    f"archive key {archive_key!r} (basename of urls[0])"
                )
            if len(self.sha256) < 2:
                raise ValueError(
                    "access='zip_member' requires at least one "
                    "extracted-member sha256 entry in addition to the "
                    f"archive key {archive_key!r}"
                )
        elif set(self.sha256) != {archive_key}:
            raise ValueError(
                f"access={self.access!r} requires sha256 to contain exactly "
                f"one entry keyed {archive_key!r} (basename of urls[0]); got "
                f"{sorted(self.sha256)}"
            )
        return self

    @model_validator(mode="after")
    def _validate_sidecars(self) -> "SourceLayer":
        if not self.sidecars:
            if self.primary_size_bytes is not None:
                raise ValueError(
                    "primary_size_bytes is only valid when sidecars are declared"
                )
            return self

        if self.access != "download":
            raise ValueError(
                "sidecars are only valid for access='download'; sibling files "
                f"are forbidden for access={self.access!r}"
            )

        primary_filename = _url_basename(self.urls[0])
        _validate_safe_basename(primary_filename, label="primary filename")
        if self.primary_size_bytes is None:
            raise ValueError(
                "sidecar bundles require a positive primary_size_bytes field"
            )

        seen: set[str] = {primary_filename.casefold()}
        for sidecar in self.sidecars:
            folded = sidecar.filename.casefold()
            if folded in seen:
                raise ValueError(
                    "sidecar filename collides with the primary or another "
                    f"sidecar under case-folding: {sidecar.filename!r}"
                )
            seen.add(folded)

        expected_size = self.primary_size_bytes + sum(
            sidecar.size_bytes for sidecar in self.sidecars
        )
        if self.size_bytes != expected_size:
            raise ValueError(
                "size_bytes must equal primary_size_bytes plus all sidecar "
                f"sizes ({expected_size}), got {self.size_bytes}"
            )
        return self

    @model_validator(mode="after")
    def _validate_sun_geometry_both_or_neither(self) -> "SourceLayer":
        """Require both sun angles; deshading skips products missing geometry.

        A partial declaration could fabricate an angle and prevent
        ``fusion.color.deshade_hillshade_divide`` from skipping the product.
        """
        if (self.sun_azimuth_deg is None) != (self.sun_elevation_deg is None):
            raise ValueError(
                "sun_azimuth_deg and sun_elevation_deg must both be set or "
                "both be None, got "
                f"sun_azimuth_deg={self.sun_azimuth_deg!r}, "
                f"sun_elevation_deg={self.sun_elevation_deg!r}"
            )
        if self.sun_azimuth_deg is not None and (
            not math.isfinite(self.sun_azimuth_deg)
            or not 0.0 <= self.sun_azimuth_deg < 360.0
            or not math.isfinite(self.sun_elevation_deg)
            or not 0.0 <= self.sun_elevation_deg <= 90.0
        ):
            raise ValueError(
                "sun angles must be finite; azimuth in [0, 360), elevation in [0, 90]"
            )
        return self

    @model_validator(mode="after")
    def _validate_mask_encoding_only_on_masks(self) -> "SourceLayer":
        """Reject ``mask_encoding`` outside mask ingest, where it is ignored."""
        if self.mask_encoding is not None and self.kind != "mask":
            raise ValueError(
                f"mask_encoding is only meaningful for kind='mask'; layer "
                f"{self.layer_id!r} declares kind={self.kind!r}"
            )
        return self

    @model_validator(mode="after")
    def _validate_quality_semantics_only_on_quality(self) -> "SourceLayer":
        """Quality semantics are meaningful only for quality layers."""
        if self.quality_semantics is not None and self.kind != "quality":
            raise ValueError(
                f"quality_semantics is only meaningful for kind='quality'; "
                f"layer {self.layer_id!r} declares kind={self.kind!r}"
            )
        return self

    @model_validator(mode="after")
    def _validate_color_alignment_vertical_shift(self) -> "SourceLayer":
        """Color alignment may not apply a vertical datum shift."""
        if self.kind == "color" and self.alignment.dz_m != 0.0:
            raise ValueError(
                "color layer alignment must have dz_m == 0.0; "
                f"got {self.alignment.dz_m!r}"
            )
        return self


class SourceCatalog(BaseModel):
    """Per-body catalog schema for curated source-layer YAML files."""

    model_config = ConfigDict(extra="forbid")

    body: Body
    layers: list[SourceLayer]

    @model_validator(mode="after")
    def _validate_unique_layer_ids(self) -> "SourceCatalog":
        seen: set[str] = set()
        for layer in self.layers:
            if layer.body != self.body:
                raise ValueError(
                    f"layer {layer.layer_id!r} body {layer.body!r} does not match catalog body {self.body!r}"
                )
            if layer.layer_id in seen:
                raise ValueError(f"duplicate layer_id '{layer.layer_id}' in catalog")
            seen.add(layer.layer_id)
        return self
