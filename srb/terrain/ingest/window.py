"""Shared window geometry and raster reads for local and remote DEM sources.

`plan_window` computes geometry without I/O; `read_planned_window` reads, validates,
fills, pads, and returns a `Dem`. Nodata filling is shared with `dem.py`; that module
imports these helpers locally to avoid a cycle.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from typing import Iterator, Literal, Optional, Union

import numpy as np
import rasterio
import rasterio.errors
import rasterio.windows
from rasterio.crs import CRS as _RioCRS
from rasterio.enums import ColorInterp, Resampling
from rasterio.transform import Affine
from rasterio.vrt import WarpedVRT

from srb.terrain.errors import (
    DeclaredBaseCrsMismatchError,
    EmptyHeightmapError,
    UnreadableFileError,
    UnsupportedTransformError,
)
from srb.terrain.ingest.dem import Dem, _fill_nodata_like_load_dem
from srb.utils.resource_budget import checked_shape_nbytes

_RESAMPLING_BY_NAME: dict[str, Resampling] = {
    "bilinear": Resampling.bilinear,
    "nearest": Resampling.nearest,
}

_SQUARE_PIXEL_REL_TOL = 1e-9


def _check_declared_crs(
    ds: "rasterio.DatasetReader",
    *,
    declared_crs: Optional[str],
    layer_id: Optional[str],
    body: Optional[str],
) -> None:
    """Check a catalog-declared CRS against the raw dataset header before reprojection
    can replace it. Missing declarations or dataset CRSes are skipped; empty or
    unparseable declarations are typed mismatches. Compare canonical IDs only when
    both normalized CRSes matched canonical entries; if either uses `body_default`,
    use strict Rasterio CRS equality because fallback IDs can collide. `body` is
    required for a declared CRS.
    """
    if declared_crs is None or ds.crs is None:
        return
    assert body is not None, (
        "_check_declared_crs: declared_crs given but body is None -- "
        "every caller that passes declared_crs must also pass body"
    )
    from pyproj.exceptions import CRSError as PyprojCRSError

    from srb.terrain.ingest.planetary_crs import normalize_crs

    if not declared_crs:
        raise DeclaredBaseCrsMismatchError(
            layer_id if layer_id is not None else "<unknown>",
            dem_crs=str(ds.crs),
            plan_crs=str(declared_crs),
        )
    try:
        declared_norm = normalize_crs(declared_crs, body=body)
    except (PyprojCRSError, rasterio.errors.CRSError) as e:
        raise DeclaredBaseCrsMismatchError(
            layer_id if layer_id is not None else "<unknown>",
            dem_crs=str(ds.crs),
            plan_crs=str(declared_crs),
        ) from e
    actual_norm = normalize_crs(str(ds.crs), body=body)

    if (
        declared_norm.matched_by != "body_default"
        and actual_norm.matched_by != "body_default"
    ):
        if declared_norm.canonical_id == actual_norm.canonical_id:
            return
        raise DeclaredBaseCrsMismatchError(
            layer_id if layer_id is not None else "<unknown>",
            dem_crs=str(ds.crs),
            plan_crs=str(declared_crs),
        )

    if _RioCRS.from_user_input(declared_crs) == _RioCRS.from_user_input(ds.crs):
        return
    raise DeclaredBaseCrsMismatchError(
        layer_id if layer_id is not None else "<unknown>",
        dem_crs=str(ds.crs),
        plan_crs=str(declared_crs),
    )


def _same_crs_for_wrap_decision(
    ds_crs: "rasterio.crs.CRS",
    dst_crs: str,
    *,
    body: Optional[str],
) -> bool:
    """Decide whether the source can be read directly in the destination CRS. Compare
    canonical IDs only when both CRSes matched canonical entries; otherwise use
    strict Rasterio CRS equality. With `body=None`, use strict equality. Keep this
    separate from `_check_declared_crs`: one validates the raw catalog claim; this
    function decides whether to reproject.
    """
    if body is not None:
        from srb.terrain.ingest.planetary_crs import normalize_crs

        src_norm = normalize_crs(str(ds_crs), body=body)
        dst_norm = normalize_crs(dst_crs, body=body)
        if (
            src_norm.matched_by != "body_default"
            and dst_norm.matched_by != "body_default"
        ):
            return src_norm.canonical_id == dst_norm.canonical_id
    return _RioCRS.from_user_input(ds_crs) == _RioCRS.from_user_input(dst_crs)


@dataclass(frozen=True)
class _VrtFillPlan:
    """Nodata strategy for a warped source. Existing nodata needs no addition; otherwise
    use NaN for floating data or an added alpha band for integer data. These fields
    are mutually exclusive because `add_alpha` removes VRT nodata. Integer sentinels
    can collide with valid data or change dtype.
    """

    nodata: Optional[float]
    add_alpha: bool


_ALPHA_BAND_ATTR = "_srb_footprint_alpha_band"

_SRC_ALPHA_BAND_ATTR = "_srb_source_alpha_band"


def _footprint_alpha_band(
    ds: Union["rasterio.DatasetReader", "WarpedVRT"],
) -> Optional[int]:
    """Return the 1-based index of an alpha band added by this module, else `None`. An
    explicit marker distinguishes it from source-owned alpha, which multiband reads
    preserve.
    """
    idx = getattr(ds, _ALPHA_BAND_ATTR, None)
    if isinstance(idx, int) and 1 <= idx <= ds.count:
        return idx
    return None


def _source_alpha_band(
    ds: Union["rasterio.DatasetReader", "WarpedVRT"],
) -> Optional[int]:
    """Return the source-owned alpha index stamped on a wrapped dataset, else `None`.
    The single-band reader masks with it; the multiband reader preserves it.
    """
    idx = getattr(ds, _SRC_ALPHA_BAND_ATTR, None)
    if isinstance(idx, int) and 1 <= idx <= ds.count:
        return idx
    return None


def _vrt_fill_nodata_for(ds: "rasterio.DatasetReader") -> _VrtFillPlan:
    """Choose a detectable fill for pixels outside a warped source's footprint. Existing
    nodata propagates; otherwise floats use NaN and integers use alpha, avoiding
    fabricated zeros, unsafe sentinels, and dtype changes. If integer data already
    has alpha, reuse it and record its index for the single-band reader. Added alpha
    is consumed into validity. This policy also runs for same-CRS non-square
    rebuilds, where the fill may be unused.
    """
    if ds.nodata is not None:
        return _VrtFillPlan(nodata=None, add_alpha=False)
    dtype = np.dtype(ds.dtypes[0])
    if np.issubdtype(dtype, np.floating):
        return _VrtFillPlan(nodata=float("nan"), add_alpha=False)
    if ColorInterp.alpha in ds.colorinterp:
        # Reuse source-owned alpha; GDAL rejects adding another.
        return _VrtFillPlan(nodata=None, add_alpha=False)
    return _VrtFillPlan(nodata=None, add_alpha=True)


def _maybe_warp_source_for_read(
    ds: "rasterio.DatasetReader",
    *,
    dst_crs: Optional[str],
    resampling: Literal["bilinear", "nearest"],
    body: Optional[str] = None,
) -> Union["rasterio.DatasetReader", "WarpedVRT"]:
    """Wrap a source in `WarpedVRT` only when `dst_crs` is set and CRS, pixel shape, or
    affine rotation/shear requires a square, north-up grid. Otherwise return the
    identical dataset. VRT resampling precedes later fusion-grid resampling.

    If the first VRT preserves the source transform exactly, a non-square source needs
    an explicit output transform and dimensions; a resolution hint alone may not square
    pixels. Rebuild over source bounds using the average-axis pixel size. Genuine
    reprojections do not take this branch.

    For missing source nodata, apply `_vrt_fill_nodata_for`. Track module-added alpha
    separately from source-owned alpha: single-band reads mask on either, while
    multiband reads exclude only the added band.
    """
    if dst_crs is None or ds.crs is None:
        return ds
    ax, by = ds.res
    non_square = abs(ax - by) > _SQUARE_PIXEL_REL_TOL * max(ax, by)
    rotated = ds.transform.b != 0.0 or ds.transform.d != 0.0
    same_crs = _same_crs_for_wrap_decision(ds.crs, dst_crs, body=body)
    if same_crs and not non_square and not rotated:
        return ds
    try:
        resampling_enum = _RESAMPLING_BY_NAME[resampling]
    except KeyError:
        raise ValueError(
            f"_maybe_warp_source_for_read: unsupported resampling "
            f"{resampling!r}; must be one of {sorted(_RESAMPLING_BY_NAME)}"
        ) from None
    fill_plan = _vrt_fill_nodata_for(ds)
    vrt_kwargs: dict = {}
    if fill_plan.nodata is not None:
        vrt_kwargs["nodata"] = fill_plan.nodata
    if fill_plan.add_alpha:
        vrt_kwargs["add_alpha"] = True
    warped = WarpedVRT(ds, crs=dst_crs, resampling=resampling_enum, **vrt_kwargs)
    if non_square and warped.transform == ds.transform:
        warped.close()
        native_gsd = (ax + by) / 2.0
        left, bottom, right, top = ds.bounds
        out_w = max(1, int(round((right - left) / native_gsd)))
        out_h = max(1, int(round((top - bottom) / native_gsd)))
        vrt_kwargs["transform"] = Affine(native_gsd, 0.0, left, 0.0, -native_gsd, top)
        vrt_kwargs["width"] = out_w
        vrt_kwargs["height"] = out_h
        warped = WarpedVRT(ds, crs=dst_crs, resampling=resampling_enum, **vrt_kwargs)
    if fill_plan.add_alpha:
        setattr(warped, _ALPHA_BAND_ATTR, warped.count)
    elif fill_plan.nodata is None and ds.nodata is None:
        if ColorInterp.alpha in warped.colorinterp:
            setattr(
                warped,
                _SRC_ALPHA_BAND_ATTR,
                warped.colorinterp.index(ColorInterp.alpha) + 1,
            )
    return warped


@contextmanager
def reprojected_source(
    ds: "rasterio.DatasetReader",
    *,
    dst_crs: Optional[str],
    resampling: Literal["bilinear", "nearest"] = "bilinear",
    declared_crs: Optional[str] = None,
    layer_id: Optional[str] = None,
    body: Optional[str] = None,
) -> Iterator[Union["rasterio.DatasetReader", "WarpedVRT"]]:
    """Context manager for optional reprojection. Yields the original dataset or a
    `WarpedVRT`, closes only the VRT, and leaves `ds` ownership to the caller.
    Declared CRS validation runs against the raw header first. Production callers
    pass `body` for wrap comparison even without a CRS declaration.
    """
    _check_declared_crs(ds, declared_crs=declared_crs, layer_id=layer_id, body=body)
    warped = _maybe_warp_source_for_read(
        ds, dst_crs=dst_crs, resampling=resampling, body=body
    )
    try:
        yield warped
    finally:
        if warped is not ds:
            warped.close()


@dataclass(frozen=True, slots=True)
class WindowPlan:
    native_gsd_m: float
    out_gsd_m: float
    out_pixels: int
    read_window: "rasterio.windows.Window"
    col_pad_before: int
    col_pad_after: int
    row_pad_before: int
    row_pad_after: int
    read_out_w: int
    read_out_h: int
    window_size_m: float
    """The (possibly padded) requested window size in meters, i.e.
    `size_m + 2 * pad_m`, computed BEFORE rounding to `out_pixels` -- kept
    verbatim (not re-derived as `out_pixels * out_gsd_m`) for requested
    geometry and allocation. The returned `Dem` transform and GSD instead
    describe the snapped source pixels actually read, including resampling
    and padding."""


def _axis_sample_coverage(
    window_offset: float, side: float, out_pixels: int, source_pixels: int
) -> np.ndarray:
    """Return which resampled pixel centers map inside one source axis."""
    centers = window_offset + (np.arange(out_pixels, dtype=np.float64) + 0.5) * (
        side / out_pixels
    )
    return (centers >= 0.0) & (centers < source_pixels)


def _axis_output_padding(
    *,
    window_offset: float,
    side: float,
    out_pixels: int,
    source_pixels: int,
) -> tuple[int, int]:
    """Crop admitted centers in O(1) memory; if none land inside, retain one
    invalid cell so the DEM path raises `NodataAllError`."""
    step = side / out_pixels

    def first_center_at_or_after(bound: float) -> int:
        low = 0
        high = out_pixels
        while low < high:
            middle = (low + high) // 2
            center = window_offset + (middle + 0.5) * step
            if center < bound:
                low = middle + 1
            else:
                high = middle
        return low

    first = first_center_at_or_after(0.0)
    stop = first_center_at_or_after(float(source_pixels))
    if first < stop:
        return first, out_pixels - stop

    scale = out_pixels / side
    before = int(round(max(0.0, -window_offset) * scale))
    after = int(round(max(0.0, window_offset + side - source_pixels) * scale))
    before = min(max(before, 0), out_pixels - 1)
    after = min(max(after, 0), out_pixels - before - 1)
    return before, after


@contextmanager
def _planned_output_grid(
    ds: "rasterio.DatasetReader | WarpedVRT", plan: WindowPlan
) -> Iterator[Optional[WarpedVRT]]:
    """Expose a `WarpedVRT` on the planned grid for bounded reads. Rasterio rejects
    boundless reads on a VRT; this virtual grid preserves the snapped window without
    allocating its pixel array.
    """
    if not isinstance(ds, WarpedVRT):
        yield None
        return

    source_transform = ds.window_transform(plan.read_window)
    gsd = source_transform.a * plan.read_window.width / plan.out_pixels
    output_transform = Affine(
        gsd,
        0.0,
        source_transform.c,
        0.0,
        -gsd,
        source_transform.f,
    )
    planned = WarpedVRT(
        ds,
        crs=ds.crs,
        transform=output_transform,
        width=plan.out_pixels,
        height=plan.out_pixels,
        resampling=Resampling.nearest,
    )
    for attr in (_ALPHA_BAND_ATTR, _SRC_ALPHA_BAND_ATTR):
        if hasattr(ds, attr):
            setattr(planned, attr, getattr(ds, attr))
    try:
        yield planned
    finally:
        planned.close()


def plan_window(
    ds: "rasterio.DatasetReader",
    *,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: float | None,
    pad_m: float,
    source_desc: str,
) -> WindowPlan:
    """Plan square-window geometry on an open dataset without reading pixels. Bounds
    expand to whole pixels; out-of-extent portions remain available for boundless
    reads and caller-selected padding. `pad_m` extends each side of `size_m`;
    `source_desc` labels errors.

    Raises `UnreadableFileError` for materially non-square pixels and
    `EmptyHeightmapError` for an empty window or one outside the dataset.
    """
    if ds.transform.b != 0.0 or ds.transform.d != 0.0:
        raise UnsupportedTransformError(
            "windowed terrain reads require a north-up source transform "
            "(no rotation/shear)"
        )
    if ds.transform.a <= 0.0 or ds.transform.e >= 0.0:
        raise UnsupportedTransformError(
            "windowed terrain reads require east-up and north-up pixels "
            "(positive x scale, negative y scale)"
        )
    ax, by = ds.res  # (xsize, ysize) in CRS units (meters)
    native_gsd_m = float((ax + by) / 2.0)
    if abs(ax - by) > _SQUARE_PIXEL_REL_TOL * max(ax, by):
        # We assume square pixels; surface non-square as a load error so
        # callers don't get silently warped meshes.
        raise UnreadableFileError(f"non-square pixels in {source_desc}: x={ax}, y={by}")

    out_gsd = target_gsd_m if target_gsd_m is not None else native_gsd_m
    window_size_m = size_m + 2.0 * pad_m
    out_pixels = int(round(window_size_m / out_gsd))
    if out_pixels <= 0:
        raise EmptyHeightmapError(
            f"size_m {size_m}m (+pad_m {pad_m}m) / gsd {out_gsd}m is < 1px"
        )

    left = center_xy_m[0] - window_size_m / 2.0
    right = center_xy_m[0] + window_size_m / 2.0
    bottom = center_xy_m[1] - window_size_m / 2.0
    top = center_xy_m[1] + window_size_m / 2.0

    ideal_window = rasterio.windows.from_bounds(
        left, bottom, right, top, transform=ds.transform
    )
    full_window = rasterio.windows.Window(0, 0, ds.width, ds.height)
    try:
        col_off = np.floor(ideal_window.col_off)
        row_off = np.floor(ideal_window.row_off)
        col_end = np.ceil(ideal_window.col_off + ideal_window.width)
        row_end = np.ceil(ideal_window.row_off + ideal_window.height)
        side = max(col_end - col_off, row_end - row_off)
        read_window = rasterio.windows.Window(col_off, row_off, side, side)
        read_window.intersection(full_window)
    except rasterio.errors.WindowError as e:
        raise EmptyHeightmapError(
            f"window center={center_xy_m} size={size_m}m (+pad_m {pad_m}m) "
            f"lies outside {source_desc} extent"
        ) from e

    col_pad_before, col_pad_after = _axis_output_padding(
        window_offset=read_window.col_off,
        side=side,
        out_pixels=out_pixels,
        source_pixels=ds.width,
    )
    row_pad_before, row_pad_after = _axis_output_padding(
        window_offset=read_window.row_off,
        side=side,
        out_pixels=out_pixels,
        source_pixels=ds.height,
    )
    read_out_w = out_pixels - col_pad_before - col_pad_after
    read_out_h = out_pixels - row_pad_before - row_pad_after

    return WindowPlan(
        native_gsd_m=native_gsd_m,
        out_gsd_m=out_gsd,
        out_pixels=out_pixels,
        read_window=read_window,
        col_pad_before=col_pad_before,
        col_pad_after=col_pad_after,
        row_pad_before=row_pad_before,
        row_pad_after=row_pad_after,
        read_out_w=read_out_w,
        read_out_h=read_out_h,
        window_size_m=window_size_m,
    )


def _output_sample_coverage(
    ds: Union["rasterio.DatasetReader", "WarpedVRT"], plan: WindowPlan
) -> np.ndarray:
    """Return which output sample centers fall inside the source pixel grid. Boundless
    reads can repeat edge values without nodata, so overlap alone is insufficient;
    dataset masks supply per-pixel validity separately.
    """
    valid_cols = _axis_sample_coverage(
        plan.read_window.col_off,
        plan.read_window.width,
        plan.out_pixels,
        ds.width,
    )
    valid_rows = _axis_sample_coverage(
        plan.read_window.row_off,
        plan.read_window.height,
        plan.out_pixels,
        ds.height,
    )
    return valid_rows[:, None] & valid_cols[None, :]


def read_planned_window(
    ds: "rasterio.DatasetReader",
    plan: WindowPlan,
    *,
    fill_nodata_threshold: float,
    source_desc: str,
    pad_mode: Literal["edge", "nan"] = "edge",
) -> Dem:
    """Read and fill a planned window into a single-band float32 `Dem`.
    `pad_mode="edge"` repeats boundary samples; use `"nan"` when uncovered regions
    must remain missing. `source_desc` labels fill errors and may differ from the
    planning descriptor.
    """
    read_shape = (plan.read_out_h, plan.read_out_w)
    checked_shape_nbytes(
        read_shape,
        np.dtype(ds.dtypes[0]),
        resource=f"terrain window decoded pixels from {source_desc}",
    )
    checked_shape_nbytes(
        read_shape,
        np.dtype(np.float32),
        resource=f"terrain window float32 pixels from {source_desc}",
    )
    checked_shape_nbytes(
        (plan.out_pixels, plan.out_pixels),
        np.dtype(np.float32),
        resource=f"terrain padded window from {source_desc}",
    )
    target_shape = (plan.out_pixels, plan.out_pixels)
    checked_shape_nbytes(
        target_shape,
        np.dtype(ds.dtypes[0]),
        resource=f"terrain target-grid decoded pixels from {source_desc}",
    )
    checked_shape_nbytes(
        target_shape,
        np.dtype(np.uint8),
        resource=f"terrain target-grid validity mask from {source_desc}",
    )
    alpha_idx = _footprint_alpha_band(ds)
    if alpha_idx is None:
        alpha_idx = _source_alpha_band(ds)
    alpha_valid: np.ndarray | None = None
    nodata = (
        ds.nodatavals[0]
        if ds.nodatavals and ds.nodatavals[0] is not None
        else float("nan")
    )
    with _planned_output_grid(ds, plan) as read_ds:
        if read_ds is None:
            band = ds.read(
                1, window=plan.read_window, boundless=True, out_shape=target_shape
            )
            if alpha_idx is not None:
                alpha_valid = (
                    ds.read(
                        alpha_idx,
                        window=plan.read_window,
                        boundless=True,
                        out_shape=target_shape,
                    )
                    > 0
                )
            source_mask = ds.read_masks(
                1,
                window=plan.read_window,
                boundless=True,
                out_shape=target_shape,
            )
        else:
            band = read_ds.read(1)
            if alpha_idx is not None:
                alpha_valid = read_ds.read(alpha_idx) > 0
            source_mask = read_ds.read_masks(1)
    band = band.astype(np.float32, copy=False)
    valid_samples = (source_mask > 0) & _output_sample_coverage(ds, plan)
    if alpha_valid is not None:
        valid_samples &= alpha_valid
    fill_value = np.float32(nodata) if nodata is not None else np.float32("nan")
    band = np.where(valid_samples, band, fill_value)

    rows = slice(plan.row_pad_before, plan.out_pixels - plan.row_pad_after)
    cols = slice(plan.col_pad_before, plan.out_pixels - plan.col_pad_after)
    band = band[rows, cols]
    nodata_f = (
        float(nodata) if nodata is not None and not np.isnan(nodata) else float("nan")
    )
    band = _fill_nodata_like_load_dem(
        band,
        nodata_f=nodata_f,
        fill_nodata_threshold=fill_nodata_threshold,
        source=source_desc,
    )

    if (
        plan.row_pad_before
        or plan.row_pad_after
        or plan.col_pad_before
        or plan.col_pad_after
    ):
        if pad_mode == "nan":
            band = np.pad(
                band,
                (
                    (plan.row_pad_before, plan.row_pad_after),
                    (plan.col_pad_before, plan.col_pad_after),
                ),
                mode="constant",
                constant_values=np.nan,
            )
        else:
            band = np.pad(
                band,
                (
                    (plan.row_pad_before, plan.row_pad_after),
                    (plan.col_pad_before, plan.col_pad_after),
                ),
                mode="edge",
            )

    crs_str = str(ds.crs) if ds.crs else None

    source_transform = ds.window_transform(plan.read_window)
    actual_gsd_m = source_transform.a * plan.read_window.width / plan.out_pixels
    transform_a = (
        actual_gsd_m,
        0.0,
        source_transform.c,
        0.0,
        -actual_gsd_m,
        source_transform.f,
    )

    return Dem(
        heights=band.astype(np.float32, copy=False),
        gsd_m=actual_gsd_m,
        nodata=nodata_f,
        crs=crs_str,
        transform_a=transform_a,
        valid_mask=np.isfinite(band),
    )


def read_planned_window_multiband(
    ds: "rasterio.DatasetReader",
    plan: WindowPlan,
) -> Dem:
    """Read every data band into `(H, W, C)`, preserving source dtype for color data.

    Do not pad to `out_pixels`: NaN padding would upcast integer color and change
    downstream byte-versus-unit scale detection. Return the true read size with an
    affine shifted past cropped padding; the destination warp leaves uncovered cells
    invalid. Exclude only module-added alpha; preserve source-owned alpha. Skip
    elevation nodata filling. Validity combines dataset masks, alpha, and
    source-footprint coverage.
    """
    alpha_idx = _footprint_alpha_band(ds)
    if alpha_idx is None:
        indexes = list(range(1, ds.count + 1))
    else:
        indexes = list(range(1, alpha_idx))
    result_dtype = np.result_type(
        *(np.dtype(ds.dtypes[index - 1]) for index in indexes)
    )
    read_shape = (len(indexes), plan.read_out_h, plan.read_out_w)
    checked_shape_nbytes(
        read_shape,
        result_dtype,
        resource="terrain multiband window decoded pixels",
    )
    checked_shape_nbytes(
        read_shape,
        np.dtype(np.uint8),
        resource="terrain multiband window validity masks",
    )
    checked_shape_nbytes(
        (plan.read_out_h, plan.read_out_w),
        np.dtype(bool),
        resource="terrain multiband window validity",
    )
    target_shape = (plan.out_pixels, plan.out_pixels)
    target_read_shape = (len(indexes), *target_shape)
    checked_shape_nbytes(
        target_read_shape,
        result_dtype,
        resource="terrain multiband window decoded pixels",
    )
    checked_shape_nbytes(
        target_read_shape,
        np.dtype(np.uint8),
        resource="terrain multiband window validity masks",
    )
    with _planned_output_grid(ds, plan) as read_ds:
        if read_ds is None:
            raw = ds.read(
                indexes,
                window=plan.read_window,
                boundless=True,
                out_shape=target_read_shape,
            )
            source_masks = ds.read_masks(
                indexes,
                window=plan.read_window,
                boundless=True,
                out_shape=target_read_shape,
            )
            if alpha_idx is not None:
                alpha = ds.read(
                    alpha_idx,
                    window=plan.read_window,
                    boundless=True,
                    out_shape=target_shape,
                )
        else:
            raw = read_ds.read(indexes)
            source_masks = read_ds.read_masks(indexes)
            if alpha_idx is not None:
                alpha = read_ds.read(alpha_idx)
    valid_mask = np.all(source_masks > 0, axis=0) & _output_sample_coverage(ds, plan)
    if alpha_idx is not None:
        valid_mask &= alpha > 0
    if np.issubdtype(raw.dtype, np.floating):
        valid_mask &= np.all(np.isfinite(raw), axis=0)

    rows = slice(plan.row_pad_before, plan.out_pixels - plan.row_pad_after)
    cols = slice(plan.col_pad_before, plan.out_pixels - plan.col_pad_after)
    raw = raw[:, rows, cols]
    valid_mask = valid_mask[rows, cols]
    band = np.ascontiguousarray(np.moveaxis(raw, 0, -1))

    crs_str = str(ds.crs) if ds.crs else None
    source_transform = ds.window_transform(plan.read_window)
    actual_gsd_m = source_transform.a * plan.read_window.width / plan.out_pixels
    transform_a = (
        actual_gsd_m,
        0.0,
        source_transform.c + plan.col_pad_before * actual_gsd_m,
        0.0,
        -actual_gsd_m,
        source_transform.f - plan.row_pad_before * actual_gsd_m,
    )

    return Dem(
        heights=band,
        gsd_m=actual_gsd_m,
        nodata=float("nan"),
        crs=crs_str,
        transform_a=transform_a,
        valid_mask=valid_mask,
    )
