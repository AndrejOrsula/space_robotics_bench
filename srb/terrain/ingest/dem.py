"""DEM loading and the shared `Dem` dataclass.

Heights use float32 with NaN nodata; `gsd_m` averages pixel dimensions, which
must be nearly equal because downstream stages require square pixels.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional

import numpy as np
import rasterio
import rasterio.errors
from rasterio.fill import fillnodata

from srb.terrain.errors import (
    NodataAllError,
    UnreadableFileError,
    UnsupportedTransformError,
)
from srb.utils.resource_budget import checked_shape_nbytes, require_file_size

Resampling = Literal["bilinear", "nearest"]


@dataclass(frozen=True)
class Dem:
    heights: np.ndarray  # float32, shape (H, W), NaN where nodata after fill
    # -- the single-band DEM/height contract. Exception: colour reads via
    # `load_dem_window_multiband` return `(H, W, C)` in the source's own
    # native dtype (`uint8` for colour DN), no NaN fill -- see that
    # function's own docstring (`ingest/window.py`'s `read_planned_window_multiband`).
    gsd_m: float
    nodata: float
    crs: Optional[str]
    transform_a: tuple[float, float, float, float, float, float]
    """Affine transform (a, b, c, d, e, f) in rasterio/Affine native order."""
    valid_mask: np.ndarray | None = None
    """Optional explicit `(H, W)` validity for native-dtype reads.

    Single-band elevation normally carries validity in `NaN` pixels.  Native
    integer colour reads cannot use an in-band sentinel without changing their
    radiometry, so their nodata/alpha validity is carried separately here.
    """

    def __post_init__(self) -> None:
        """Publish detached, read-only snapshots of raster state."""
        if isinstance(self.heights, np.ndarray):
            heights = self.heights.copy()
            heights.setflags(write=False)
            object.__setattr__(self, "heights", heights)
        if isinstance(self.valid_mask, np.ndarray):
            valid_mask = self.valid_mask.copy()
            valid_mask.setflags(write=False)
            object.__setattr__(self, "valid_mask", valid_mask)


def _require_dem(value: object, *, source: str, expected_ndim: int) -> Dem:
    """Validate a DEM crossing an ingest adapter boundary.

    Windowed and optionally imported remote readers may have unknown static
    return types, so reject malformed arrays here before fusion.
    """
    if not isinstance(value, Dem):
        raise TypeError(f"{source} returned {type(value).__name__}, expected Dem")

    heights = value.heights
    if not isinstance(heights, np.ndarray):
        raise TypeError(f"{source} returned Dem.heights that is not an ndarray")
    if heights.ndim != expected_ndim:
        raise ValueError(
            f"{source} returned heights with ndim={heights.ndim}, "
            f"expected {expected_ndim}"
        )
    if heights.dtype.kind not in "biufc":
        raise TypeError(
            f"{source} returned non-numeric heights dtype={heights.dtype!r}"
        )
    if not np.isfinite(value.gsd_m) or value.gsd_m <= 0.0:
        raise ValueError(f"{source} returned invalid gsd_m={value.gsd_m!r}")
    if len(value.transform_a) != 6:
        raise ValueError(f"{source} returned an invalid six-term transform")

    valid_mask = value.valid_mask
    if valid_mask is not None:
        if not isinstance(valid_mask, np.ndarray):
            raise TypeError(f"{source} returned a non-array validity mask")
        if valid_mask.dtype.kind != "b":
            raise TypeError(
                f"{source} returned a non-boolean validity mask "
                f"dtype={valid_mask.dtype!r}"
            )
        if valid_mask.shape != heights.shape[:2]:
            raise ValueError(
                f"{source} returned validity shape {valid_mask.shape}, "
                f"expected {heights.shape[:2]}"
            )

    return value


def load_dem(
    path: Path,
    *,
    fill_nodata_threshold: float = 0.10,
) -> Dem:
    """Read a single-band float32 GeoTIFF.

    Interpolate nodata only when its fraction is at most
    `fill_nodata_threshold`; fully nodata rasters raise `NodataAllError`.
    """
    if not Path(path).is_file():
        raise UnreadableFileError(f"DEM not found: {path}")
    require_file_size(path, resource=f"terrain DEM {path}")

    try:
        ds = rasterio.open(path)
    except rasterio.errors.RasterioIOError as e:
        raise UnreadableFileError(f"rasterio could not open {path}: {e}") from e

    with ds:
        t = ds.transform
        if t.b != 0.0 or t.d != 0.0 or t.a <= 0.0 or t.e >= 0.0:
            raise UnsupportedTransformError(
                f"DEM requires east-up and north-up pixels without rotation or "
                f"shear in {path}: a={t.a}, b={t.b}, d={t.d}, e={t.e}"
            )
        checked_shape_nbytes(
            (ds.height, ds.width),
            np.dtype(ds.dtypes[0]),
            resource=f"terrain DEM decoded pixels {path}",
        )
        checked_shape_nbytes(
            (ds.height, ds.width),
            np.dtype(np.float32),
            resource=f"terrain DEM float32 pixels {path}",
        )
        masked_band = ds.read(1, masked=True)
        band = masked_band.astype(np.float32, copy=False).filled(np.nan)
        dataset_mask = np.ma.getmaskarray(masked_band)
        nodata = (
            ds.nodatavals[0]
            if ds.nodatavals and ds.nodatavals[0] is not None
            else float("nan")
        )
        nodata_f = (
            float(nodata)
            if nodata is not None and not np.isnan(nodata)
            else float("nan")
        )

        if not np.isnan(nodata_f):
            mask = dataset_mask | (band == nodata_f)
        else:
            mask = dataset_mask | np.isnan(band)

        nodata_frac = float(mask.mean())
        if nodata_frac == 1.0:
            raise NodataAllError(f"all pixels are nodata in {path}")

        if 0.0 < nodata_frac <= fill_nodata_threshold:
            # rasterio fillnodata expects mask where 1=valid, 0=invalid
            valid = (~mask).astype(np.uint8)
            band = fillnodata(band, mask=valid, max_search_distance=10.0)
            band = band.astype(np.float32, copy=False)
            # GDAL's fillnodata leaves a pixel UNCHANGED when no source value is
            # found within max_search_distance, so a hole wider than the search
            # radius keeps its original sentinel/NaN. Recompute the mask from the
            # filled band instead of assuming everything was filled, so the
            # remaining holes are still NaN-marked below (otherwise a raw -32768
            # sentinel would survive as a "real" elevation into the mesh).
            if not np.isnan(nodata_f):
                mask = np.isnan(band) | (band == nodata_f)
            else:
                mask = np.isnan(band)
        # nodata_frac > fill_nodata_threshold: leave as NaN (handled below)

        # Mark remaining nodata as NaN so the rest of the pipeline can detect it.
        if mask.any():
            band = band.copy()
            band[mask] = np.nan

        ax, by = ds.res  # (xsize, ysize) in CRS units (meters)
        gsd_m = float((ax + by) / 2.0)
        if abs(ax - by) / max(ax, by, 1e-9) > 1e-3:
            # We assume square pixels; surface non-square as a load error so
            # callers don't get silently warped meshes.
            raise UnreadableFileError(f"non-square pixels in {path}: x={ax}, y={by}")

        crs_str = str(ds.crs) if ds.crs else None
        transform_a = (t.a, t.b, t.c, t.d, t.e, t.f)

    return _require_dem(
        Dem(
            heights=band,
            gsd_m=gsd_m,
            nodata=nodata_f,
            crs=crs_str,
            transform_a=transform_a,
            valid_mask=np.isfinite(band),
        ),
        source=str(path),
        expected_ndim=2,
    )


def load_dem_window(
    path: Path,
    *,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: Optional[float] = None,
    pad_m: float = 0.0,
    fill_nodata_threshold: float = 0.10,
    pad_mode: Literal["edge", "nan"] = "edge",
    dst_crs: Optional[str] = None,
    resampling: Resampling = "bilinear",
    declared_crs: Optional[str] = None,
    layer_id: Optional[str] = None,
    body: Optional[str] = None,
) -> Dem:
    """Read and resample a square GeoTIFF window without loading the raster.

    Bounds expand to source pixels before resampling to a square grid. Boundless
    reads and dataset masks preserve alignment; `pad_m` expands the request and
    `pad_mode` fills outside margins. Nodata handling follows `load_dem`; COG
    overviews are used when decimating.

    `dst_crs` enables a `WarpedVRT` for reprojection or grid correction, and
    `resampling` selects its algorithm. `declared_crs` is checked against the
    raster header using `layer_id` and `body`; omitting it skips validation.
    `pad_mode="edge"` repeats edge samples; `"nan"` marks margins missing.
    Raises `EmptyHeightmapError` when the padded request misses the raster.
    Shared window logic lives in `ingest.window`; this wrapper owns the file.
    """
    # Local import: `window.py` imports `Dem` / `_fill_nodata_like_load_dem`
    # from THIS module at import time, so importing `window` back at module
    # level here would be circular. By the time this function actually
    # runs, both modules are already fully loaded, so a function-local
    # import is safe -- and keeps the eager dependency one-directional
    # (window.py depends on dem.py, never the reverse).
    from srb.terrain.ingest.window import (
        plan_window,
        read_planned_window,
        reprojected_source,
    )

    if not Path(path).is_file():
        raise UnreadableFileError(f"DEM not found: {path}")
    require_file_size(path, resource=f"terrain DEM {path}")

    try:
        ds = rasterio.open(path)
    except rasterio.errors.RasterioIOError as e:
        raise UnreadableFileError(f"rasterio could not open {path}: {e}") from e

    source_desc = str(path)
    with (
        ds,
        reprojected_source(
            ds,
            dst_crs=dst_crs,
            resampling=resampling,
            declared_crs=declared_crs,
            layer_id=layer_id,
            body=body,
        ) as read_ds,
    ):
        plan = plan_window(
            read_ds,
            center_xy_m=center_xy_m,
            size_m=size_m,
            target_gsd_m=target_gsd_m,
            pad_m=pad_m,
            source_desc=source_desc,
        )
        return _require_dem(
            read_planned_window(
                read_ds,
                plan,
                fill_nodata_threshold=fill_nodata_threshold,
                source_desc=source_desc,
                pad_mode=pad_mode,
            ),
            source=source_desc,
            expected_ndim=2,
        )


def load_dem_window_multiband(
    path: Path,
    *,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: Optional[float] = None,
    pad_m: float = 0.0,
    dst_crs: Optional[str] = None,
    resampling: Resampling = "bilinear",
    declared_crs: Optional[str] = None,
    layer_id: Optional[str] = None,
    body: Optional[str] = None,
) -> Dem:
    """Read downloaded and zip-member colour layers into `(H, W, C)` heights.

    No nodata filling is applied. Other layer kinds, including remote COG
    colour layers, use the single-band path. CRS and resampling match
    `load_dem_window`.
    """
    from srb.terrain.ingest.window import (
        plan_window,
        read_planned_window_multiband,
        reprojected_source,
    )

    if not Path(path).is_file():
        raise UnreadableFileError(f"DEM not found: {path}")
    require_file_size(path, resource=f"terrain multiband raster {path}")

    try:
        ds = rasterio.open(path)
    except rasterio.errors.RasterioIOError as e:
        raise UnreadableFileError(f"rasterio could not open {path}: {e}") from e

    source_desc = str(path)
    with (
        ds,
        reprojected_source(
            ds,
            dst_crs=dst_crs,
            resampling=resampling,
            declared_crs=declared_crs,
            layer_id=layer_id,
            body=body,
        ) as read_ds,
    ):
        plan = plan_window(
            read_ds,
            center_xy_m=center_xy_m,
            size_m=size_m,
            target_gsd_m=target_gsd_m,
            pad_m=pad_m,
            source_desc=source_desc,
        )
        return _require_dem(
            read_planned_window_multiband(read_ds, plan),
            source=source_desc,
            expected_ndim=3,
        )


def _fill_nodata_like_load_dem(
    band: np.ndarray,
    *,
    nodata_f: float,
    fill_nodata_threshold: float,
    source: str,
) -> np.ndarray:
    """Apply `load_dem` nodata handling to a windowed read."""
    if not np.isnan(nodata_f):
        mask = band == nodata_f
    else:
        mask = np.isnan(band)

    nodata_frac = float(mask.mean())
    if nodata_frac == 1.0:
        raise NodataAllError(f"all pixels in window are nodata in {source}")

    if 0.0 < nodata_frac <= fill_nodata_threshold:
        valid = (~mask).astype(np.uint8)
        band = fillnodata(band, mask=valid, max_search_distance=10.0)
        band = band.astype(np.float32, copy=False)
        if not np.isnan(nodata_f):
            mask = np.isnan(band) | (band == nodata_f)
        else:
            mask = np.isnan(band)

    if mask.any():
        band = band.copy()
        band[mask] = np.nan

    return band
