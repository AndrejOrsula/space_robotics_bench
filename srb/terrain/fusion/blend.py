"""Feather co-registered DEM layers in front-to-back priority order.

The gap-free base is last with zero feather width. Helpers return diagnostics;
`fusion.engine` emits warnings.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from srb.terrain.fusion.filters import chamfer_distance_px, nan_lowpass

_WEIGHT_EPS = 1e-6
"""Minimum covered alpha; smaller totals are treated as uncovered."""

_BLEND_LEN_MIN_M = 50.0
_BLEND_LEN_MAX_M = 200.0
"""Clamp on the priority-blend feather length, in metres."""

_BLEND_LEN_DEFAULT_FACTOR = 20.0
"""Default feather length uses base GSD, so coarse bases get wider feathers."""

_MAX_LAYERS = 254
"""`uint8` reserves 0 for none; contributor indices are capped at 254."""


def feather_weights(valid: np.ndarray, *, blend_px: float) -> np.ndarray:
    """Return float32 weights: zero at the edge, one at `blend_px`, invalid 0.

    Zero width gives hard priority without division.
    """
    if blend_px <= 0:
        return valid.astype(np.float32)
    dist = chamfer_distance_px(valid, max_px=float(blend_px)).astype(np.float64)
    w = np.clip(dist / float(blend_px), 0.0, 1.0)
    w = np.where(valid, w, 0.0)
    return w.astype(np.float32)


def blend_len_px(
    base_gsd_m: float, out_gsd_m: float, *, blend_len_m: float | None = None
) -> tuple[float, bool]:
    """Clamp default `20 * base_gsd_m` or explicit length to [50, 200] m.

    Return output pixels and whether the raw length was outside the bounds;
    `engine.py` emits the warning. Values on either bound are not clamped.
    """
    raw_m = (
        blend_len_m
        if blend_len_m is not None
        else _BLEND_LEN_DEFAULT_FACTOR * base_gsd_m
    )
    clamped_m = min(max(raw_m, _BLEND_LEN_MIN_M), _BLEND_LEN_MAX_M)
    return clamped_m / out_gsd_m, clamped_m != raw_m


def detrend_inset(
    inset: np.ndarray, base: np.ndarray, *, sigma_px: float
) -> np.ndarray:
    """Remove long-wavelength datum/tilt differences from `inset`.

    NaN-safe; unsupported regions get zero correction and pass through.
    """
    both_finite = np.isfinite(inset) & np.isfinite(base)
    diff = np.where(
        both_finite,
        inset.astype(np.float64) - base.astype(np.float64),
        np.nan,
    )
    trend = nan_lowpass(diff, sigma_px=sigma_px).astype(np.float64)
    correction = np.where(np.isfinite(trend), trend, 0.0)
    return (inset.astype(np.float64) - correction).astype(np.float32)


def priority_blend(
    layers: Sequence[np.ndarray], *, blend_px: Sequence[float]
) -> tuple[np.ndarray, np.ndarray]:
    """Composite highest priority first, with a gap-free base last.

    Remaining alpha lets feathered edges reveal lower layers; fully covered
    pixels exclude lower layers. Return float32
    data and int16 provenance (`-1` uncovered); reject empty or mismatched
    inputs and stacks over 254 layers.
    """
    layers = list(layers)
    blend_px = list(blend_px)
    if len(layers) != len(blend_px):
        raise ValueError(
            f"priority_blend: len(layers)={len(layers)} != "
            f"len(blend_px)={len(blend_px)}"
        )
    if not layers:
        raise ValueError("priority_blend: layers must be non-empty")
    if len(layers) > _MAX_LAYERS:
        raise ValueError(
            f"priority_blend: {len(layers)} layers exceeds the "
            f"{_MAX_LAYERS}-layer cap (the provenance raster is uint8, "
            "1-based 1..N indices)"
        )

    shape = layers[0].shape
    for index, arr in enumerate(layers[1:], start=1):
        if arr.shape != shape:
            raise ValueError(
                f"priority_blend: layer {index} shape {arr.shape} differs "
                f"from layer 0 shape {shape}"
            )
    acc = np.zeros(shape, np.float64)
    remaining = np.ones(shape, np.float64)  # unclaimed alpha
    best_c = np.zeros(shape, np.float64)
    prov = np.full(shape, -1, np.int16)
    for i, (arr, bpx) in enumerate(zip(layers, blend_px, strict=True)):  # front-to-back
        valid = np.isfinite(arr)
        w = feather_weights(valid, blend_px=bpx).astype(np.float64)
        c = w * remaining  # what THIS layer actually claims
        acc += np.where(valid, arr, 0.0) * c
        remaining -= c
        take = c > best_c
        best_c = np.where(take, c, best_c)
        prov = np.where(take, np.int16(i), prov)
    covered = 1.0 - remaining
    fused = np.where(
        covered > _WEIGHT_EPS, acc / np.maximum(covered, _WEIGHT_EPS), np.nan
    )
    # Keep provenance consistent with coverage; otherwise NaN pixels inflate
    # the consumer's area_fraction.
    prov = np.where(covered > _WEIGHT_EPS, prov, np.int16(-1))
    return fused.astype(np.float32), prov
