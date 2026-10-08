"""Slope, curvature, ambient occlusion, and small numerical helpers used by
the material baker. Pure numpy.
"""

from __future__ import annotations

import numpy as np


def smooth(arr: np.ndarray, *, sigma_px: float) -> np.ndarray:
    if sigma_px <= 0:
        return arr.astype(np.float32, copy=True)

    radius = max(1, int(round(3.0 * sigma_px)))
    xs = np.arange(-radius, radius + 1, dtype=np.float64)
    kernel = np.exp(-(xs**2) / (2.0 * sigma_px**2))
    kernel /= kernel.sum()

    a = np.asarray(arr, dtype=np.float64)
    a = _convolve1d(a, kernel, axis=1)
    a = _convolve1d(a, kernel, axis=0)
    return a.astype(np.float32, copy=False)


def slope_deg(heights: np.ndarray, *, gsd_m: float) -> np.ndarray:
    gy, gx = np.gradient(heights.astype(np.float64), gsd_m)
    mag = np.sqrt(gx**2 + gy**2)
    return np.rad2deg(np.arctan(mag)).astype(np.float32)


def curvature(heights: np.ndarray, *, gsd_m: float) -> np.ndarray:
    h = heights.astype(np.float64)
    inv = 1.0 / (gsd_m**2)
    lap = (
        np.roll(h, 1, axis=0)
        + np.roll(h, -1, axis=0)
        + np.roll(h, 1, axis=1)
        + np.roll(h, -1, axis=1)
        - 4.0 * h
    ) * inv
    # Negate: positive = convex (dome/ridge), negative = concave (bowl/valley).
    lap = -lap
    # Make edges zero so we don't propagate wrap-around artifacts.
    lap[0, :] = 0
    lap[-1, :] = 0
    lap[:, 0] = 0
    lap[:, -1] = 0
    return lap.astype(np.float32)


def ambient_occlusion(
    heights: np.ndarray,
    *,
    gsd_m: float,
    num_dirs: int = 32,
    max_steps: int = 16,
    step_px: float = 1.5,
) -> np.ndarray:
    """Cheap horizon-angle AO: for each direction, find the largest elevation
    angle along a short ray; AO = 1 - mean(sin(horizon_angle))."""
    h = heights.astype(np.float64)
    H, W = h.shape
    occlusion = np.zeros_like(h)

    rng = np.linspace(0, 2 * np.pi, num_dirs, endpoint=False)
    ys, xs = np.indices((H, W), dtype=np.float64)
    for theta in rng:
        dx = np.cos(theta) * step_px
        dy = np.sin(theta) * step_px
        max_angle = np.full_like(h, -np.pi / 2)
        for s in range(1, max_steps + 1):
            sx = xs + dx * s
            sy = ys + dy * s
            xi = np.clip(np.round(sx).astype(np.int64), 0, W - 1)
            yi = np.clip(np.round(sy).astype(np.int64), 0, H - 1)
            dh = h[yi, xi] - h
            horiz = dh / (s * step_px * gsd_m)
            angle = np.arctan(horiz)
            np.maximum(max_angle, angle, out=max_angle)
        occlusion += np.sin(np.maximum(max_angle, 0.0))
    occlusion /= num_dirs
    return np.clip(occlusion, 0.0, 1.0).astype(np.float32)


def _convolve1d(arr: np.ndarray, kernel: np.ndarray, *, axis: int) -> np.ndarray:
    pad = len(kernel) // 2
    if axis == 0:
        padded = np.pad(arr, ((pad, pad), (0, 0)), mode="edge")
        out = np.zeros_like(arr)
        for i, w in enumerate(kernel):
            out += padded[i : i + arr.shape[0], :] * w
        return out
    elif axis == 1:
        padded = np.pad(arr, ((0, 0), (pad, pad)), mode="edge")
        out = np.zeros_like(arr)
        for i, w in enumerate(kernel):
            out += padded[:, i : i + arr.shape[1]] * w
        return out
    raise ValueError(f"axis must be 0 or 1, got {axis}")
