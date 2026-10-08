"""Colour compositing and radiometric normalization for terrain fusion.

Inputs are warped arrays in catalog priority order. Diagnostics are returned as
data; `fusion.engine` emits warnings.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Literal

import numpy as np

from srb.terrain.fusion.blend import _MAX_LAYERS
from srb.terrain.fusion.filters import nan_lowpass

_GAIN_DEN_EPS = 1e-6
"""Treat low-pass denominators with absolute value <= 1e-6 as zero."""

_NORM_LEN_FACTOR = 10.0
"""`default_norm_len_m`'s multiple of the anchor's own native GSD."""

_NORM_LEN_FLOOR_M = 500.0
"""`default_norm_len_m`'s floor, in metres -- see its docstring."""


# --- shared promotion helper -------------------------------------------------


def _to_hwc3_float64(arr: np.ndarray) -> np.ndarray:
    """Promote `(H, W)` or `(H, W, 3)` input to `(H, W, 3)` `float64`.

    Grayscale channels are replicated; RGB order is preserved. Other shapes
    raise `ValueError`.
    """
    a = np.asarray(arr)
    if a.ndim == 2:
        return np.repeat(a[:, :, None], 3, axis=2).astype(np.float64)
    if a.ndim == 3 and a.shape[2] == 3:
        return a.astype(np.float64)
    raise ValueError(f"expected a (H, W) or (H, W, 3) array, got shape {a.shape!r}")


# --- to_rgb_u8 ----------------------------------------------------------------


def to_rgb_u8(
    arr: np.ndarray,
    *,
    assume_scale: Literal["auto", "unit", "byte"] = "auto",
) -> np.ndarray:
    """Convert `(H, W)` or `(H, W, 3)` input to RGB `uint8` in 0..255.

    Floats use `assume_scale`; auto selects unit scale when all finite values
    are at most 1. Integers are not rescaled, and non-finite values become zero.
    """
    a = np.asarray(arr)
    rgb = _to_hwc3_float64(a)
    finite = np.isfinite(rgb)

    if not np.issubdtype(a.dtype, np.floating):
        scale = 1.0
    elif assume_scale == "unit":
        scale = 255.0
    elif assume_scale == "byte":
        scale = 1.0
    elif assume_scale == "auto":
        sample = rgb[finite]
        unit_range = sample.size == 0 or float(np.max(sample)) <= 1.0
        scale = 255.0 if unit_range else 1.0
    else:
        raise ValueError(
            "to_rgb_u8: assume_scale must be one of 'auto', 'unit', "
            f"'byte', got {assume_scale!r}"
        )

    scaled = np.where(finite, rgb * scale, 0.0)
    # Matches materials.baker._write_png_*'s own "+0.5 then clip" rounding
    # convention (baker.py), rather than np.round's round-half-to-
    # even -- house style for a float->uint8 pixel conversion.
    return np.clip(scaled + 0.5, 0.0, 255.0).astype(np.uint8)


# --- psr_mask -------------------------------------------------------------


def psr_mask(masks: Sequence[np.ndarray]) -> np.ndarray | None:
    """Union warped mask layers as bool `(H, W)`; return `None` when empty.

    Nearest-warped masks are NaN outside their extent and 0/1 inside; only
    finite, nonzero cells belong to the union.
    """
    masks = list(masks)
    if not masks:
        return None
    union = np.zeros(masks[0].shape, dtype=bool)
    for m in masks:
        union |= np.isfinite(m) & (m != 0)
    return union


# --- radiometric normalization: lowpass_gain + its default length -----------


def default_norm_len_m(anchor_native_gsd_m: float, *, patch_size_m: float) -> float:
    """Return the normalization length `max(10 * anchor_native_gsd_m, 500 m)`,
    limited to half the patch size."""
    raw_m = max(_NORM_LEN_FACTOR * anchor_native_gsd_m, _NORM_LEN_FLOOR_M)
    return min(raw_m, patch_size_m / 2.0)


def lowpass_gain(
    detail: np.ndarray,
    anchor: np.ndarray,
    *,
    sigma_px: float,
    gain_clamp: tuple[float, float] = (0.5, 2.0),
) -> tuple[np.ndarray, float]:
    """Compute a NaN-safe per-pixel gain that matches the detail layer's
    low-frequency trend to the anchor. Returns `(gain, clamped_fraction)`, the
    fraction taken over supported pixels; unsupported pixels use gain 1.0.
    Warning emission belongs to `fusion.engine`."""
    lo_d = nan_lowpass(detail, sigma_px=sigma_px)
    lo_a = nan_lowpass(anchor, sigma_px=sigma_px)
    ok = np.isfinite(lo_d) & np.isfinite(lo_a) & (np.abs(lo_d) > _GAIN_DEN_EPS)
    g = np.where(ok, lo_a / np.where(ok, lo_d, 1.0), 1.0)
    clipped = np.clip(g, gain_clamp[0], gain_clamp[1])
    # Fraction of supported pixels; unsupported ones would dilute the warning.
    n_ok = int(np.count_nonzero(ok))
    clamped_fraction = (
        float(np.count_nonzero(ok & (clipped != g))) / n_ok if n_ok else 0.0
    )
    return clipped.astype(np.float32), clamped_fraction


# --- hillshade / deshade -----------------------------------------------------


def hillshade(
    heights: np.ndarray,
    *,
    gsd_m: float,
    sun_azimuth_deg: float,
    sun_elevation_deg: float,
) -> np.ndarray:
    """Compute clipped Lambertian illumination from central-difference DEM
    gradients. Returns float32 with the input shape; self-shadowed slopes are 0.
    Azimuth is clockwise from north and elevation is above the horizon."""
    h = np.asarray(heights, dtype=np.float64)
    grad_row, grad_col = np.gradient(h, gsd_m)
    dzdx = grad_col
    dzdy = -grad_row

    normal_mag = np.sqrt(dzdx**2 + dzdy**2 + 1.0)
    nx = -dzdx / normal_mag
    ny = -dzdy / normal_mag
    nz = 1.0 / normal_mag

    az = math.radians(sun_azimuth_deg)
    el = math.radians(sun_elevation_deg)
    sx = math.cos(el) * math.sin(az)
    sy = math.cos(el) * math.cos(az)
    sz = math.sin(el)

    cos_incidence = nx * sx + ny * sy + nz * sz
    return np.clip(cos_incidence, 0.0, None).astype(np.float32)


def deshade_hillshade_divide(
    image: np.ndarray,
    heights: np.ndarray,
    *,
    gsd_m: float,
    sun_azimuth_deg: float,
    sun_elevation_deg: float,
    floor: float = 0.15,
    normalize: bool = True,
) -> np.ndarray:
    """Divide out modeled hillshade, flooring its denominator at `floor`.

    `normalize` preserves average brightness; grayscale and RGB are supported.
    Pixels with non-finite shade retain their input values.
    """
    hs = hillshade(
        heights,
        gsd_m=gsd_m,
        sun_azimuth_deg=sun_azimuth_deg,
        sun_elevation_deg=sun_elevation_deg,
    )
    hs64 = hs.astype(np.float64)
    denom = np.maximum(hs64, float(floor))
    if normalize:
        finite_hs = hs64[np.isfinite(hs64)]
        mean_hs = float(np.mean(finite_hs)) if finite_hs.size else 1.0
        if abs(mean_hs) <= np.finfo(np.float64).eps:
            return np.asarray(image, dtype=np.float32)
    else:
        mean_hs = 1.0

    img = np.asarray(image, dtype=np.float64)
    denom_b = denom[..., None] if img.ndim == 3 else denom
    with np.errstate(divide="ignore", invalid="ignore"):
        deshaded = img * mean_hs / denom_b
    out = np.where(np.isfinite(denom_b), deshaded, img)
    return out.astype(np.float32)


# --- composite_color ----------------------------------------------------------


def composite_color(
    layers: Sequence[np.ndarray],
    *,
    psr: np.ndarray | None,
    psr_layers: Sequence[np.ndarray],
    assume_scale: Literal["auto", "unit", "byte"] = "auto",
) -> tuple[np.ndarray, np.ndarray]:
    """Composite ordered RGB/grayscale layers by first-valid-pixel priority, with
    optional PSR switching. PSR pixels try `psr_layers` first, then fall back to
    `layers`; other pixels use only `layers`. Returns `(rgb_u8, provenance_index)`;
    indices are 0-based into `[*psr_layers, *layers]`, with `-1` where uncovered.
    `assume_scale` is forwarded to `to_rgb_u8`."""
    layers = list(layers)
    psr_layers = list(psr_layers)
    if not layers and not psr_layers:
        raise ValueError(
            "composite_color: layers and psr_layers are both empty -- at "
            "least one array is required to know the output shape"
        )
    if len(psr_layers) + len(layers) > _MAX_LAYERS:
        raise ValueError(
            f"composite_color: {len(psr_layers)} psr_layers + "
            f"{len(layers)} layers exceeds the {_MAX_LAYERS}-layer cap "
            "(the provenance raster is uint8, with 1-based 1..N indices)"
        )

    first = layers[0] if layers else psr_layers[0]
    shape2d = first.shape[:2]
    acc = np.full((*shape2d, 3), np.nan, dtype=np.float64)
    prov = np.full(shape2d, -1, dtype=np.int16)
    unresolved = np.ones(shape2d, dtype=bool)

    use_psr = (
        np.asarray(psr, dtype=bool)
        if psr is not None
        else np.zeros(shape2d, dtype=bool)
    )

    for i, arr in enumerate(psr_layers):
        rgb = _to_hwc3_float64(arr)
        valid = np.all(np.isfinite(rgb), axis=-1) & use_psr & unresolved
        if valid.any():
            acc[valid] = rgb[valid]
            prov[valid] = np.int16(i)
            unresolved &= ~valid

    offset = len(psr_layers)
    for i, arr in enumerate(layers):
        rgb = _to_hwc3_float64(arr)
        valid = np.all(np.isfinite(rgb), axis=-1) & unresolved
        if valid.any():
            acc[valid] = rgb[valid]
            prov[valid] = np.int16(offset + i)
            unresolved &= ~valid

    return to_rgb_u8(acc, assume_scale=assume_scale), prov
