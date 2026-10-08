"""Define the common fusion grid and warp materialized arrays onto it."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import rasterio.warp
from rasterio.enums import Resampling
from rasterio.transform import Affine

from srb.terrain.errors import (
    InvalidPatchGridError,
    MissingCRSError,
    UnsupportedResamplingError,
    UnsupportedTransformError,
)
from srb.terrain.ingest.dem import Dem
from srb.terrain.ingest.planetary_crs import RasterGridSig, grids_identical

if TYPE_CHECKING:
    # Type-only import keeps catalog and source-provider dependencies out of
    # the fusion runtime import graph.
    from srb.terrain.catalog.resolve import CompositionPlan

# Continuous data uses bilinear; categorical layers use nearest.
_RESAMPLING_BY_NAME: dict[str, Resampling] = {
    "bilinear": Resampling.bilinear,
    "nearest": Resampling.nearest,
}


@dataclass(frozen=True, slots=True)
class PatchGrid:
    """Square north-up output grid, centered at ``center_xy_m`` in ``crs``."""

    crs: str
    center_xy_m: tuple[float, float]
    size_m: float
    gsd_m: float

    def __post_init__(self) -> None:
        # Reject degenerate grids at construction, not first use.
        _ = self.pixels

    @property
    def pixels(self) -> int:
        """Rounded side length; it must cover at least one pixel."""
        if self.gsd_m <= 0:
            raise InvalidPatchGridError(
                f"PatchGrid.gsd_m must be > 0, got {self.gsd_m!r}"
            )
        px = int(round(self.size_m / self.gsd_m))
        if px <= 0:
            raise InvalidPatchGridError(
                f"PatchGrid size_m={self.size_m!r} / gsd_m={self.gsd_m!r} "
                f"rounds to {px} pixels; the grid must cover at least one"
            )
        return px

    @property
    def transform_a(self) -> tuple[float, float, float, float, float, float]:
        """North-up affine transform centered on the rounded pixel extent."""
        half = self.pixels * self.gsd_m / 2.0
        cx, cy = self.center_xy_m
        return (
            self.gsd_m,
            0.0,
            cx - half,
            0.0,
            -self.gsd_m,
            cy + half,
        )

    @property
    def shape(self) -> tuple[int, int]:
        return (self.pixels, self.pixels)


def grid_from_plan(plan: "CompositionPlan") -> PatchGrid:
    """Build the fusion grid for a non-empty composition plan."""
    if plan.target_gsd_m <= 0:
        raise InvalidPatchGridError(
            f"CompositionPlan.target_gsd_m={plan.target_gsd_m!r} is not "
            "positive -- an empty DEM stack (plan.is_empty) resolves "
            "target_gsd_m to 0.0; check that before calling grid_from_plan"
        )
    return PatchGrid(
        crs=plan.crs,
        center_xy_m=plan.center_xy_m,
        size_m=plan.request.size_m,
        gsd_m=plan.target_gsd_m,
    )


def _identity_decision(dem: Dem, grid: PatchGrid) -> bool:
    """Return whether `dem` and `grid` share a pixel grid within the
    resolution-relative tolerance. Used by both warp paths."""
    height, width = dem.heights.shape[:2]
    src_sig = RasterGridSig(
        crs=dem.crs, transform=dem.transform_a, width=width, height=height
    )
    dst_sig = RasterGridSig(
        crs=grid.crs,
        transform=grid.transform_a,
        width=grid.pixels,
        height=grid.pixels,
    )
    tol_m = min(0.5, 0.25 * grid.gsd_m)
    return grids_identical(src_sig, dst_sig, tol_m=tol_m)


def warp_to_grid(
    dem: Dem, grid: PatchGrid, *, resampling: str = "bilinear"
) -> np.ndarray:
    """Warp arrays to ``grid`` as float32 with NaN outside valid coverage.

    Supports bilinear and nearest resampling. Matching float32 single-band
    grids return the source unchanged unless the validity mask adds nodata; other
    arrays use the generalized path.
    """
    try:
        resampling_enum = _RESAMPLING_BY_NAME[resampling]
    except KeyError:
        raise UnsupportedResamplingError(
            f"unsupported resampling {resampling!r}; must be one of "
            f"{sorted(_RESAMPLING_BY_NAME)}"
        ) from None

    if dem.crs is None:
        # The class default covers unknown explicit CRS strings; this case
        # needs guidance for a Dem already built without a CRS.
        raise MissingCRSError(
            "warp_to_grid: source Dem has no CRS to reproject from",
            remediation=(
                "Ensure the Dem's CRS is set by whatever ingest step "
                "produced it (load_dem / load_dem_window read it from the "
                "source GeoTIFF)."
            ),
        )

    if (
        dem.transform_a[0] <= 0
        or dem.transform_a[1] != 0
        or dem.transform_a[3] != 0
        or dem.transform_a[4] >= 0
    ):
        # The identity check samples three collinear pixels and can misclassify
        # transposed grids, so reject non-north-up transforms before identity.
        raise UnsupportedTransformError(
            "warp_to_grid: dem.transform_a must be east-up and north-up "
            f"(transform_a={dem.transform_a!r})"
        )

    valid_mask = (
        None if dem.valid_mask is None else np.asarray(dem.valid_mask, dtype=bool)
    )
    if valid_mask is not None and valid_mask.shape != dem.heights.shape[:2]:
        raise ValueError(
            "warp_to_grid: dem.valid_mask must match the source spatial shape; "
            f"got {valid_mask.shape}, expected {dem.heights.shape[:2]}"
        )
    explicit_mask_adds_nodata = False
    if valid_mask is not None and dem.heights.ndim in (2, 3):
        finite_pixels = (
            np.isfinite(dem.heights)
            if dem.heights.ndim == 2
            else np.all(np.isfinite(dem.heights), axis=-1)
        )
        explicit_mask_adds_nodata = bool(np.any(~valid_mask & finite_pixels))

    if (
        dem.heights.ndim != 2
        or dem.heights.dtype != np.float32
        or explicit_mask_adds_nodata
    ):
        # Preserve the single-band float32 path for its identity fast path.
        return _warp_generalized(dem, grid, resampling_enum)

    if _identity_decision(dem, grid):
        return dem.heights

    destination = np.full(grid.shape, np.nan, dtype=np.float32)
    rasterio.warp.reproject(
        source=dem.heights,
        destination=destination,
        src_transform=Affine(*dem.transform_a),
        src_crs=dem.crs,
        dst_transform=Affine(*grid.transform_a),
        dst_crs=grid.crs,
        src_nodata=np.nan,
        dst_nodata=np.nan,
        init_dest_nodata=True,
        resampling=resampling_enum,
    )
    return destination


def _warp_generalized(
    dem: Dem, grid: PatchGrid, resampling_enum: Resampling
) -> np.ndarray:
    """Warp non-float or multi-band data as float32, with NaN outside coverage."""
    if dem.heights.ndim not in (2, 3):
        raise ValueError(
            f"warp_to_grid: dem.heights.ndim={dem.heights.ndim} is not "
            "supported; must be 2 (H, W) or 3 (H, W, C)"
        )

    identical = _identity_decision(dem, grid)

    source = dem.heights
    if source.dtype == np.bool_:
        source = source.astype(np.uint8)
    valid_mask = (
        None if dem.valid_mask is None else np.asarray(dem.valid_mask, dtype=bool)
    )
    if valid_mask is not None and not np.all(valid_mask):
        source = source.astype(np.float32, copy=True)
        if source.ndim == 2:
            source[~valid_mask] = np.nan
        else:
            source[~valid_mask, :] = np.nan

    if identical:
        return source.astype(np.float32, copy=True)

    multiband = source.ndim == 3
    reproject_source = np.moveaxis(source, -1, 0) if multiband else source
    dst_shape = (reproject_source.shape[0], *grid.shape) if multiband else grid.shape
    destination = np.full(dst_shape, np.nan, dtype=np.float32)

    src_nodata = np.nan if np.issubdtype(source.dtype, np.floating) else None

    rasterio.warp.reproject(
        source=reproject_source,
        destination=destination,
        src_transform=Affine(*dem.transform_a),
        src_crs=dem.crs,
        dst_transform=Affine(*grid.transform_a),
        dst_crs=grid.crs,
        src_nodata=src_nodata,
        dst_nodata=np.nan,
        init_dest_nodata=True,
        resampling=resampling_enum,
    )
    if multiband:
        return np.ascontiguousarray(np.moveaxis(destination, 0, -1))
    return destination
