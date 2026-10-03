"""Load an orthorectified RGB image warped onto the DEM grid.

The ortho is reprojected with bilinear resampling onto the DEM's CRS,
transform, and shape, so rasters with different grids or extents still align.
Rasters without a CRS must already share the DEM grid exactly.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.transform import Affine, array_bounds
from rasterio.vrt import WarpedVRT
from rasterio.warp import transform_bounds

from srb.terrain.ingest.dem import Dem
from srb.utils.resource_budget import (
    MAX_IMAGE_PIXELS,
    checked_shape_nbytes,
    require_file_size,
)


def load_ortho(path: Path, *, target: Dem) -> np.ndarray | None:
    """Return an HxWx3 uint8 array on the target grid, or None if all nodata."""
    if not Path(path).is_file():
        raise FileNotFoundError(f"ortho not found: {path}")
    require_file_size(path, resource=f"terrain ortho {path}")

    h_out, w_out = target.heights.shape[:2]
    checked_shape_nbytes(
        (h_out, w_out),
        np.dtype(np.uint8),
        resource="terrain ortho target pixels",
        max_elements=MAX_IMAGE_PIXELS,
    )
    checked_shape_nbytes(
        (3, h_out, w_out),
        np.dtype(np.uint8),
        resource=f"terrain ortho decoded RGB {path}",
    )
    dst_transform = Affine(*target.transform_a)

    with rasterio.open(path) as ds:
        if ds.count < 3:
            raise ValueError(
                f"terrain ortho requires at least 3 bands, found {ds.count}: {path}"
            )
        if ds.crs is None or target.crs is None:
            # Without both CRSs the grids cannot be related; accept only an
            # identical grid instead of silently stretching by shape.
            if (
                ds.crs is not None
                or target.crs is not None
                or (ds.height, ds.width) != (h_out, w_out)
                or not ds.transform.almost_equals(dst_transform)
            ):
                raise ValueError(
                    f"terrain ortho {path} lacks a CRS shared with the DEM and "
                    "does not match the DEM grid exactly"
                )
            bands = ds.read((1, 2, 3), out_dtype=np.uint8)
            valid = ds.dataset_mask()
        else:
            _require_coverage(ds, target, dst_transform, path=path)
            with WarpedVRT(
                ds,
                crs=target.crs,
                transform=dst_transform,
                width=w_out,
                height=h_out,
                resampling=Resampling.bilinear,
            ) as vrt:
                bands = vrt.read((1, 2, 3), out_dtype=np.uint8)
                valid = vrt.dataset_mask()

    if not np.any(valid):
        return None

    # bands: (3, H, W). Transpose to (H, W, 3).
    return np.ascontiguousarray(np.transpose(bands, (1, 2, 0)))


def _require_coverage(
    ds: rasterio.DatasetReader, target: Dem, dst_transform: Affine, *, path: Path
) -> None:
    """Reject orthos that do not cover the DEM extent (half-pixel tolerance)."""
    h_out, w_out = target.heights.shape[:2]
    west, south, east, north = array_bounds(h_out, w_out, dst_transform)
    src_west, src_south, src_east, src_north = transform_bounds(
        ds.crs, target.crs, *ds.bounds
    )
    tol_x = 0.5 * abs(dst_transform.a)
    tol_y = 0.5 * abs(dst_transform.e)
    if (
        src_west > west + tol_x
        or src_east < east - tol_x
        or src_south > south + tol_y
        or src_north < north - tol_y
    ):
        raise ValueError(
            f"terrain ortho {path} does not cover the DEM extent "
            f"({west}, {south}, {east}, {north})"
        )
