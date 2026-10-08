"""Bake-time crater stamping from the augmentation database (opt-in).

Profile: parabolic bowl to the rim radius R, then a raised cosine rim
collar decaying to grade at 1.4 R. Rim height = 0.06 * depth (fresh
simple-crater proportion; documented model, not a physical simulation).
Pure numpy, float32-deterministic, per-crater local windows.
"""

from __future__ import annotations

import numpy as np

_RIM_HEIGHT_FRAC = 0.06
_COLLAR_R_FRAC = 1.4


def stamp_craters(
    heights: np.ndarray, *, craters: np.ndarray, gsd_m: float
) -> np.ndarray:
    out = np.asarray(heights, dtype=np.float32).copy()
    craters_arr = np.asarray(craters, dtype=np.float64)
    if craters_arr.size == 0:
        return out
    h, w = out.shape
    for x_m, y_m, radius_m, depth_m in craters_arr:
        if radius_m <= 0.0:
            # Avoid division by zero for degenerate custom radius ranges.
            continue
        col_c = x_m / gsd_m + 0.5 * (w - 1)
        row_c = 0.5 * (h - 1) - y_m / gsd_m
        r_px = radius_m / gsd_m
        span = int(np.ceil(r_px * _COLLAR_R_FRAC)) + 1
        c0 = max(int(np.floor(col_c)) - span, 0)
        c1 = min(int(np.ceil(col_c)) + span + 1, w)
        r0 = max(int(np.floor(row_c)) - span, 0)
        r1 = min(int(np.ceil(row_c)) + span + 1, h)
        if c0 >= c1 or r0 >= r1:
            continue
        cols = np.arange(c0, c1, dtype=np.float64)
        rows = np.arange(r0, r1, dtype=np.float64)
        dx = (cols[None, :] - col_c) * gsd_m
        dy = (rows[:, None] - row_c) * gsd_m
        rho = np.sqrt(dx * dx + dy * dy)
        dz = np.zeros_like(rho)
        inside = rho <= radius_m
        rim_h = _RIM_HEIGHT_FRAC * depth_m
        # bowl: -depth at center, +rim_h at rho == R (continuous with collar)
        dz[inside] = -depth_m + (depth_m + rim_h) * (rho[inside] / radius_m) ** 2
        collar = (rho > radius_m) & (rho <= _COLLAR_R_FRAC * radius_m)
        t = (rho[collar] - radius_m) / ((_COLLAR_R_FRAC - 1.0) * radius_m)
        dz[collar] = rim_h * 0.5 * (1.0 + np.cos(np.pi * t))
        out[r0:r1, c0:c1] += dz.astype(np.float32)
    return out
