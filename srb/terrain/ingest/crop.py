"""Crop a `Dem` to a fixed-size patch in DEM-projected coordinates.

`center_xy_m` is an absolute coordinate in DEM CRS meters, not a pixel offset.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from srb.terrain.errors import EmptyHeightmapError, UnsupportedTransformError
from srb.terrain.ingest.dem import Dem


def crop_to_patch(
    dem: Dem,
    *,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: Optional[float] = None,
) -> Dem:
    """Return a square patch centered at `center_xy_m`.

    `target_gsd_m=None` preserves input resolution. Samples beyond the DEM edge
    are clamped; a patch wholly outside raises `EmptyHeightmapError`.
    """
    in_gsd = dem.gsd_m
    out_gsd = target_gsd_m if target_gsd_m is not None else in_gsd
    out_pixels = int(round(size_m / out_gsd))
    if out_pixels <= 0:
        raise EmptyHeightmapError(f"size_m {size_m}m / gsd {out_gsd}m is < 1px")

    # Convert north-up affine coordinates (x=c+a*col, y=f+e*row) to pixels.
    h, w = dem.heights.shape
    a, b, c, d, e, f = dem.transform_a
    if b != 0.0 or d != 0.0:
        raise UnsupportedTransformError(
            "crop_to_patch requires a north-up transform: "
            "transform_a[1] and transform_a[3] must both be 0"
        )
    if a == 0 or e == 0:
        raise EmptyHeightmapError(f"degenerate transform: a={a}, e={e}")
    if a < 0 or e > 0:
        # The output transform below is emitted north-up; reversed axes would
        # be sampled correctly but mislabeled.
        raise UnsupportedTransformError(
            "crop_to_patch requires a north-up transform: "
            f"transform_a[0] must be positive and transform_a[4] negative, got a={a}, e={e}"
        )
    # The affine maps pixel corners, while the sampler indexes pixel centers.
    # Subtract half a pixel so the patch center matches the requested coordinate.
    cx_px = (center_xy_m[0] - c) / a - 0.5
    cy_px = (center_xy_m[1] - f) / e - 0.5

    # Rounding the pixel count can change the span from `size_m`. Use the actual
    # span for sampling and the transform so spacing and georeference agree.
    span_m = out_pixels * out_gsd
    half_in_px = (span_m / in_gsd) / 2.0
    x0 = cx_px - half_in_px
    y0 = cy_px - half_in_px
    x1 = cx_px + half_in_px
    y1 = cy_px + half_in_px

    if x1 <= -0.5 or y1 <= -0.5 or x0 >= w - 0.5 or y0 >= h - 0.5:
        raise EmptyHeightmapError(
            f"patch center=({center_xy_m}) size={size_m}m lies outside DEM extent"
        )

    # Sample pixel centers over the half-open patch interval at declared spacing.
    # Endpoint-inclusive linspace uses span/(n-1), but output GSD declares span/n.
    # This mismatch offsets samples and steepens slopes. Use input-pixel offsets
    # on both axes to preserve the north-up grid.
    step_px = out_gsd / in_gsd
    offsets_px = (np.arange(out_pixels, dtype=np.float64) + 0.5) * step_px
    ys = y0 + offsets_px
    xs = x0 + offsets_px
    out = _bilinear_sample(
        dem.heights,
        xs[None, :].repeat(out_pixels, 0),
        ys[:, None].repeat(out_pixels, 1),
    )

    return Dem(
        heights=out.astype(np.float32, copy=False),
        gsd_m=out_gsd,
        nodata=dem.nodata,
        crs=dem.crs,
        transform_a=(
            out_gsd,
            0.0,
            center_xy_m[0] - span_m / 2.0,
            0.0,
            -out_gsd,
            center_xy_m[1] + span_m / 2.0,
        ),
    )


def _bilinear_sample(arr: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """Bilinearly sample `arr` at fractional (xs, ys); edge-clamp out-of-bounds."""
    h, w = arr.shape
    x0 = np.floor(xs).astype(np.int64)
    y0 = np.floor(ys).astype(np.int64)
    x1 = x0 + 1
    y1 = y0 + 1

    fx = xs - x0
    fy = ys - y0

    x0c = np.clip(x0, 0, w - 1)
    x1c = np.clip(x1, 0, w - 1)
    y0c = np.clip(y0, 0, h - 1)
    y1c = np.clip(y1, 0, h - 1)

    a = arr[y0c, x0c]
    b = arr[y0c, x1c]
    c = arr[y1c, x0c]
    d = arr[y1c, x1c]

    top = a * (1 - fx) + b * fx
    bot = c * (1 - fx) + d * fx
    return top * (1 - fy) + bot * fy
