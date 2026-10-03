"""Terrain pipeline exceptions. Each error carries remediation text shown by the CLI
instead of a traceback.
"""

from __future__ import annotations

from typing import ClassVar

from srb.terrain.asset_commands import (
    asset_download_command,
    asset_list_command,
    asset_validate_manifest_command,
)


class TerrainError(Exception):
    """Base error for terrain operations; `remediation` is the CLI recovery hint."""

    default_remediation: ClassVar[str] = ""

    def __init__(self, message: str = "", *, remediation: str | None = None) -> None:
        super().__init__(message)
        self.remediation = remediation or self.default_remediation


class ManifestSiteUnknownError(TerrainError):
    default_remediation = f"Run `{asset_list_command('terrain')}` to see valid sites."

    def __init__(self, site: str) -> None:
        super().__init__(f"site '{site}' is not in the manifest")


class ManifestInvalidError(TerrainError):
    default_remediation = f"Run `{asset_validate_manifest_command()}` for details."


class CatalogMissingError(TerrainError):
    default_remediation = (
        "No source catalog for this body yet. `moon`, `mars`, and `earth` "
        "each ship one; add `srb/terrain/catalog/<body>.yaml` (see "
        "docs/src/contributing/adding_terrain_sites.md)."
    )


class DeclaredLayerUnknownError(TerrainError):
    """A manifest names a layer absent from its source catalog. This is an authoring
    error; missing footprint coverage is a normal resolution outcome.
    """

    default_remediation = (
        f"Check the layer_id against the body's catalog "
        f"(srb/terrain/catalog/<body>.yaml), or run "
        f"`{asset_list_command('terrain')}` to see what the catalog actually "
        "contains."
    )

    def __init__(self, layer_id: str) -> None:
        super().__init__(f"declared layer_id {layer_id!r} is not in the source catalog")


class DeclaredLayerDuplicateError(TerrainError):
    """A manifest repeats a layer in one stack. Resolution would create duplicate
    contributions while fetch-byte accounting deduplicates them, making the plan
    inconsistent.
    """

    default_remediation = (
        "Remove the duplicate layer_id from the manifest's dem_stack/"
        "color_stack -- each layer may appear at most once per stack."
    )

    def __init__(self, layer_id: str) -> None:
        super().__init__(f"declared stack names layer_id {layer_id!r} more than once")


class ReferenceOnlyLayerError(TerrainError):
    """A validation-only catalog layer reached the production resolver."""

    default_remediation = (
        "Remove the reference_only layer from the production terrain manifest. "
        "Validation tooling must use resolve_declared_reference_stacks explicitly."
    )

    def __init__(self, layer_id: str) -> None:
        super().__init__(
            f"declared layer_id {layer_id!r} is reference_only and cannot be "
            "used by the production terrain resolver"
        )


class DeclaredBaseUnavailableError(TerrainError):
    """The v2 declared-stack DEM base could not load or was resolved away. V2 has no v1
    single-source fallback, so the base failure is fatal; insets may still degrade.
    """

    default_remediation = (
        "Check the layer's URL/provider/consent settings in the source "
        "catalog (srb/terrain/catalog/<body>.yaml), or run `srb asset "
        "stack` to preview what the declared plan would resolve/fetch."
    )

    def __init__(self, layer_id: str, *, site: str, reason: str | None = None) -> None:
        detail = f": {reason}" if reason else ""
        super().__init__(
            f"site {site!r}: DEM base layer {layer_id!r} could not be loaded{detail}"
        )


class DeclaredBaseCrsMismatchError(TerrainError):
    """A loaded raster CRS conflicts with the expected CRS. Compare canonical
    identities; ingest uses strict rasterio equality when body-default normalization
    cannot distinguish projected CRSs.
    """

    default_remediation = (
        "A layer's raster CRS does not match the expected CRS (either the "
        "resolved plan CRS, for the DEM base, or the catalog's own "
        "declared `crs:`, for any windowed layer read). Check the "
        "catalog's declared footprint/geometry/crs for this layer "
        "(srb/terrain/catalog/<body>.yaml) against the raster's own "
        "header -- either the catalog entry is wrong, or the raster was "
        "published/re-projected under a different grid than assumed."
    )

    def __init__(
        self,
        layer_id: str,
        *,
        site: str | None = None,
        dem_crs: str,
        plan_crs: str,
    ) -> None:
        site_prefix = f"site {site!r}: " if site is not None else ""
        super().__init__(
            f"{site_prefix}layer {layer_id!r} raster CRS {dem_crs!r} does not "
            f"match the expected CRS {plan_crs!r}"
        )


class FetchError(TerrainError):
    default_remediation = (
        "Check connectivity, verify sha256 in manifest, or retry with `--force`."
    )


class NetworkError(FetchError):
    pass


class ChecksumMismatchError(FetchError):
    """A downloaded or extracted artifact failed its pinned hash."""

    def __init__(self, url: str, expected: str, actual: str) -> None:
        super().__init__(
            f"sha256 mismatch fetching {url}: expected {expected}, got {actual}"
        )


class InvalidSourcePayloadError(FetchError):
    default_remediation = (
        "Use a direct GeoTIFF/IMG download URL, not an HTML catalog page."
    )


class Http4xxError(FetchError):
    pass


class Http5xxError(FetchError):
    pass


class SourceTooLargeError(FetchError):
    """Streamed download bytes or a zip member's declared decompressed size
    exceeded its cap."""

    default_remediation = (
        "Use a smaller source or window and verify the URL points at the "
        "expected raster. Large-download consent does not remove hard source, "
        "archive-member or window limits."
    )

    def __init__(
        self,
        url: str,
        *,
        limit: int,
        observed: int,
        observed_desc: str = "streamed and counting",
    ) -> None:
        super().__init__(
            f"source {url} exceeded the {limit}-byte download cap "
            f"({observed} bytes {observed_desc})"
        )


class LargeDownloadConsentRequiredError(FetchError):
    """Catalog fetch estimates above the unapproved cap require `SRB_TERRAIN_LARGE_OK`
    plus layer opt-in; consent is environment-only, with no prompt.
    """

    default_remediation = (
        "Set SRB_TERRAIN_LARGE_OK=1 to approve multi-gigabyte source downloads "
        "for this session (the catalog layer must also carry "
        "`allow_large_download: true`). `srb asset stack --allow-large` only "
        "previews the plan; it moves no bytes."
    )

    def __init__(self, layer_id: str, *, est_bytes: int, limit: int) -> None:
        super().__init__(
            f"layer {layer_id!r} requires consent to fetch: estimated "
            f"{est_bytes} bytes exceeds the {limit}-byte unapproved-download "
            "cap; set SRB_TERRAIN_LARGE_OK=1 to approve"
        )


class LicenseMissingError(TerrainError):
    default_remediation = (
        "Pass `attribution=...` to RealTerrain, or fix the manifest entry."
    )


class DemLoadError(TerrainError):
    default_remediation = "Inspect the input GeoTIFF with rasterio."


class UnreadableFileError(DemLoadError):
    default_remediation = "Install GDAL or check the file path / permissions."


class MissingCRSError(DemLoadError):
    """Raised for an unknown explicit CRS or a DEM without CRS. The grid call site
    supplies remediation specific to the latter case.
    """

    default_remediation = (
        "Pass a known CRS (see `srb.terrain.ingest.crs.KNOWN_CRS`) as the "
        "explicit CRS, or ensure the DEM/GeoTIFF's own CRS is set so it "
        "can be used instead."
    )


class NodataAllError(DemLoadError):
    default_remediation = "Cropped patch is mostly nodata — pick a different center."


class InvalidPatchGridError(TerrainError):
    """Patch size and GSD must produce at least one pixel. A non-positive planned GSD
    signals an empty DEM stack whose `plan.is_empty` check was skipped.
    """

    default_remediation = (
        "size_m and gsd_m must both be > 0, with size_m at least gsd_m so "
        "the patch covers at least one pixel. A non-positive target_gsd_m "
        "from a CompositionPlan means its DEM stack resolved empty -- check "
        "`plan.is_empty` before calling grid_from_plan rather than retrying "
        "the grid."
    )


class UnsupportedResamplingError(TerrainError):
    """Only bilinear and nearest resampling are accepted; unsupported values are
    rejected explicitly.
    """

    default_remediation = (
        'Pass resampling="bilinear" (continuous DEM data) or "nearest" '
        "(categorical layers: masks, provenance)."
    )


class UnsupportedTransformError(TerrainError):
    """Rotated or transposed source transforms are unsupported. Grid identity samples
    only three collinear pixels and can miss a transpose, so this check guards before
    that shortcut.
    """

    default_remediation = (
        "warp_to_grid only supports north-up source transforms (no "
        "rotation/shear): transform_a[1] and transform_a[3] must both be "
        "0. Re-derive the Dem from an axis-aligned read."
    )


class FootprintInvalidError(TerrainError):
    """Raised for an unsupported footprint format or coverage calculations across
    mismatched CRSs.
    """

    default_remediation = (
        "Footprint must be 'cap:<lat><N|S>' (e.g. 'cap:83S') or a WKT "
        "POLYGON/MULTIPOLYGON string with coordinates (metres) in the same "
        "projected CRS as the region being tested against."
    )


class RegionTooLargeError(TerrainError):
    """Regions above the maximum are refused because multi-tile composition and
    cross-tile stitching are unsupported.
    """

    default_remediation = (
        "Split the request into multiple tiles no larger than 8000 m each "
        "and compose them separately; cross-tile stitching (tiling) is not "
        "supported."
    )

    def __init__(self, size_m: float, *, max_size_m: float) -> None:
        super().__init__(
            f"requested region size_m={size_m} exceeds the {max_size_m} m region cap"
        )


class RegionUnresolvedError(TerrainError):
    """An arbitrary region has no catalog DEM coverage. This expected runtime condition
    degrades to procedural terrain; resolver warnings are retained in the error.
    """

    default_remediation = (
        "Pick a location covered by the body's source catalog "
        "(srb/terrain/catalog/<body>.yaml), or use a curated `site` "
        "instead. Run `srb asset stack` to preview what a region would "
        "resolve."
    )

    def __init__(
        self,
        *,
        body: str,
        lat_deg: float,
        lon_deg: float,
        size_m: float,
        reason: str,
        warnings: tuple[str, ...] = (),
    ) -> None:
        detail = f" (resolver warnings: {'; '.join(warnings)})" if warnings else ""
        super().__init__(
            f"region request {body} lat={lat_deg} lon={lon_deg} "
            f"size={size_m} m could not be resolved: {reason}{detail}"
        )


class RegionInvalidError(TerrainError, ValueError):
    """Invalid region size. It also subclasses `ValueError` for existing callers and
    `TerrainError` for CLI error rendering.
    """

    default_remediation = "Pass --size as a positive number of meters."

    def __init__(
        self,
        size_m: float,
        *,
        reason: str | None = None,
        remediation: str | None = None,
    ) -> None:
        super().__init__(
            reason or f"size_m must be > 0, got {size_m}", remediation=remediation
        )


class DeshadeUnsupportedError(TerrainError, ValueError):
    """Effective deshading reached a bake path without fusion support.
    The CLI rejects v1 deshading before fetching any bytes. It subclasses
    `ValueError` for callers and `TerrainError` for CLI rendering and procedural
    fallback."""

    default_remediation = (
        "Deshading is implemented by the fusion engine, which only runs for "
        "a v2 declared-stack manifest site ('dem_stack'/'color_stack', "
        "version: 2) or an arbitrary --lat/--lon/--size-m region request; "
        "drop --deshade for a v1 single-DEM site."
    )


class MeshBuildError(TerrainError):
    pass


class EmptyHeightmapError(MeshBuildError):
    default_remediation = "Patch returned 0 pixels — check center / size_m."


class DegenerateGeometryError(MeshBuildError):
    default_remediation = "Heightmap produced a degenerate mesh (likely all-equal Z)."


class MeshDecimationUnavailableError(MeshBuildError):
    """Quadric decimation failed, usually because `fast-simplification` is unavailable.
    There is no random-culling fallback: it could meet a face budget while destroying
    terrain geometry and collision fidelity.
    """

    default_remediation = (
        "Install `fast-simplification` (a declared core dependency: `pip "
        "install fast-simplification`, or `pip install -e .` to pull in "
        "SRB's full dependency set) so "
        "`trimesh.Trimesh.simplify_quadric_decimation` can run."
    )

    def __init__(
        self, *, face_budget: int, natural_face_count: int, cause: BaseException
    ) -> None:
        super().__init__(
            f"quadric decimation to face_budget={face_budget} (from "
            f"{natural_face_count} natural faces) failed: "
            f"{type(cause).__name__}: {cause}"
        )


class TerrainNotBakedError(TerrainError):
    def __init__(self, *, site: str, patch: str, lod: str) -> None:
        cmd = asset_download_command(site, patch=patch, lod=lod)
        super().__init__(
            f"terrain '{site}/{patch}@{lod}' not baked and auto_fetch=False",
            remediation=f"Run: {cmd}",
        )


class CacheLockTimeoutError(TerrainError):
    default_remediation = (
        "Another bake is still running. If stale, remove the .lock_<key> file."
    )


class MissingTerrainDependencyError(TerrainError):
    """The active environment lacks terrain runtime dependencies or an optional format
    adapter.
    """

    def __init__(
        self, missing_modules: tuple[str, ...], *, extra: str | None = None
    ) -> None:
        modules = ", ".join(missing_modules)
        remediation = (
            "Install SRB terrain/runtime dependencies in the active "
            "environment, for example `pip install -e .`, or install the "
            "missing packages directly."
        )
        if extra:
            remediation += (
                f" Or `pip install 'srb[{extra}]'` to pull "
                f"in the optional `{extra}` dependency group."
            )
        super().__init__(
            f"missing terrain bake dependencies: {modules}",
            remediation=remediation,
        )


class SimforgeMissingError(TerrainError):
    """Raised when `augment="simforge"` is requested without `simforge_foundry`
    installed.
    """

    default_remediation: ClassVar[str] = (
        "Install simforge_foundry: `pip install simforge_foundry` (or set "
        "augment='none' to keep the baked-only path)."
    )
