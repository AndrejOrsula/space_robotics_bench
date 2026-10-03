"""Decode mask-layer DNs into fusion's `NaN` / `0.0` / `1.0` convention.

Fusion treats finite, nonzero pixels as masked and `NaN` as outside the
layer's extent. Encodings are catalog data (`SourceLayer.mask_encoding`),
not hardcoded here.
"""

from __future__ import annotations

import numpy as np

from srb.terrain.catalog_schema import MaskEncoding

__all__ = ["decode_mask_dn"]


def decode_mask_dn(values: np.ndarray, encoding: MaskEncoding) -> np.ndarray:
    """Decode mask DNs to `float32` values in `{0.0, 1.0, NaN}`.

    Declared masked values become `1.0`, nodata and non-finite values become
    `NaN`, and all other values become `0.0`. Exact comparisons assume mask
    reads use nearest-neighbor resampling; nodata filling happens before this
    decode, so filled values normally become `0.0`.

    Shape is preserved. `float32` carries `NaN` through fusion's grid warp.
    """
    arr = np.asarray(values, dtype=np.float64)
    out = np.where(
        np.isin(arr, np.asarray(encoding.masked_values, dtype=np.float64)), 1.0, 0.0
    ).astype(np.float32)
    if encoding.nodata_values:
        out[np.isin(arr, np.asarray(encoding.nodata_values, dtype=np.float64))] = np.nan
    out[~np.isfinite(arr)] = np.nan
    return out
