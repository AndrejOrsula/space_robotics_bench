"""NaN-safe low-pass filters and chamfer distances for terrain fusion."""

from __future__ import annotations

import numpy as np

_LOWPASS_DEN_EPS = 1e-4
"""Below this accumulated valid-weight fraction, `nan_lowpass` is unreliable.

It measures weight against full support, so isolated samples in large holes
can fall below the threshold by design.
"""

_CHAMFER_A = 3.0
"""Orthogonal step cost of the 3-4 chamfer distance transform."""

_CHAMFER_B = 4.0
"""Diagonal step cost; distances divide by `_CHAMFER_A` to yield pixels."""


def _box1d(a: np.ndarray, radius: int, axis: int) -> np.ndarray:
    """Moving average of width `2*radius+1` along `axis` in O(N).

    Edge padding preserves constant fields at array borders.
    """
    n = a.shape[axis]
    width = 2 * radius + 1
    pad_width = [(0, 0)] * a.ndim
    pad_width[axis] = (radius, radius)
    padded = np.pad(a, pad_width, mode="edge")

    zero_shape = list(padded.shape)
    zero_shape[axis] = 1
    csum = np.concatenate(
        [np.zeros(zero_shape, dtype=padded.dtype), np.cumsum(padded, axis=axis)],
        axis=axis,
    )

    hi = [slice(None)] * a.ndim
    hi[axis] = slice(width, width + n)
    lo = [slice(None)] * a.ndim
    lo[axis] = slice(0, n)
    window_sum = csum[tuple(hi)] - csum[tuple(lo)]
    return window_sum / width


def box_blur(a: np.ndarray, *, radius: int, passes: int = 3) -> np.ndarray:
    """Apply `passes` separable box blurs; three passes approximate a Gaussian.

    Use `radius_for_sigma` to convert a target sigma to a radius.
    """
    out = np.asarray(a, dtype=np.float64)
    for _ in range(passes):
        out = _box1d(out, radius, axis=0)
        out = _box1d(out, radius, axis=1)
    return out


def radius_for_sigma(sigma_px: float, *, passes: int = 3) -> int:
    """Return cascade radius `(sqrt(12*sigma_px**2/passes + 1) - 1) / 2`.

    Clamp to 1: the 0.5 px sigma floor can round to zero, making blur an identity.
    """
    r = (np.sqrt(12.0 * sigma_px**2 / passes + 1.0) - 1.0) / 2.0
    return max(1, int(round(r)))


def nan_lowpass(arr: np.ndarray, *, sigma_px: float) -> np.ndarray:
    """NaN-safe low-pass using normalized convolution: blur zero-filled values
    and the validity mask, then divide. Returns NaN where support is below
    `_LOWPASS_DEN_EPS`; an entirely invalid input returns all NaNs."""
    r = radius_for_sigma(sigma_px)
    valid = np.isfinite(arr)
    filled = np.where(valid, arr, 0.0).astype(np.float64)
    num = box_blur(filled, radius=r)
    den = box_blur(valid.astype(np.float64), radius=r)
    out = np.where(
        den > _LOWPASS_DEN_EPS, num / np.maximum(den, _LOWPASS_DEN_EPS), np.nan
    )
    return out.astype(np.float32)


def sigma_px_for_length(length_m: float, *, gsd_m: float) -> float:
    """Convert full suppression length to sigma pixels, floored at 0.5 px.

    Sigma is `length_m / (3 * gsd_m)`, so the three-sigma support spans the
    requested length.
    """
    return max(length_m / (3.0 * gsd_m), 0.5)


def _inrow_forward(row: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """Left-to-right running minimum with slope `_CHAMFER_A`."""
    return _CHAMFER_A * idx + np.minimum.accumulate(row - _CHAMFER_A * idx)


def _inrow_backward(row: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """Right-to-left running minimum with slope `_CHAMFER_A`."""
    return (-_CHAMFER_A * idx) + np.minimum.accumulate((row + _CHAMFER_A * idx)[::-1])[
        ::-1
    ]


def _shift_cols(row: np.ndarray, offset: int) -> np.ndarray:
    """Shift columns without wraparound; out-of-row positions become `+inf`."""
    out = np.full_like(row, np.inf)
    if offset > 0:
        out[offset:] = row[:-offset]
    elif offset < 0:
        out[:offset] = row[-offset:]
    else:
        out[:] = row
    return out


def _propagate_rows(seed: np.ndarray, *, reverse: bool) -> np.ndarray:
    """Sweep top-down or bottom-up, combining row and diagonal neighbours.

    The bottom-up pass consumes the top-down output.
    """
    h, w = seed.shape
    idx = np.arange(w, dtype=np.float64)
    row_order = range(h - 1, -1, -1) if reverse else range(h)
    inrow = _inrow_backward if reverse else _inrow_forward

    out = np.empty((h, w), dtype=np.float64)
    prev = None
    for y in row_order:
        row = seed[y]
        if prev is not None:
            vert = prev + _CHAMFER_A
            diag_a = _shift_cols(prev, 1) + _CHAMFER_B
            diag_b = _shift_cols(prev, -1) + _CHAMFER_B
            row = np.minimum(row, np.minimum(vert, np.minimum(diag_a, diag_b)))
        row = inrow(row, idx)
        out[y] = row
        prev = row
    return out


def chamfer_distance_px(valid: np.ndarray, *, max_px: float) -> np.ndarray:
    """Approximate distance from valid cells to the nearest invalid cell using a
    two-pass 3-4 chamfer transform. Distances saturate at `max_px`; array edges
    are treated as valid, so a fully valid array has no boundary feather."""
    seed = np.where(valid, np.inf, 0.0)
    forward = _propagate_rows(seed, reverse=False)
    both = _propagate_rows(forward, reverse=True)
    px = both / _CHAMFER_A
    return np.minimum(px, float(max_px)).astype(np.float32)
