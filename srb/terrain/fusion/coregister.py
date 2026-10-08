"""Co-register a warped inset DEM against a base raster with a QC gate.

Inputs are same-grid 2-D arrays.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from srb.terrain.catalog_schema import AlignmentRecord, AlignmentSource
from srb.terrain.fusion.blend import detrend_inset

# Tolerates expected DEM residuals while rejecting clear mis-registration.
QC_MAX_RESIDUAL_M = 5.0

# Bound search to plausible co-registration errors.
MAX_SHIFT_PX = 32


@dataclass(frozen=True, slots=True)
class AppliedShift:
    """Alignment attempt and QC result returned by :func:`coregister`.

    Shift fields describe the attempted alignment even when ``demoted`` is
    true and the returned array is unshifted. A missing residual means there
    was no finite overlap; estimated shifts then fall back to the record.
    """

    dx_m: float
    dy_m: float
    dz_m: float
    source: AlignmentSource
    residual_m: float | None
    demoted: bool
    reason: str | None


def _shift_m_to_px(dx_m: float, dy_m: float, *, gsd_m: float) -> tuple[float, float]:
    """Convert east/north metre corrections to column/row pixel corrections."""
    return dx_m / gsd_m, -dy_m / gsd_m


def _shift_px_to_m(dx_px: float, dy_px: float, *, gsd_m: float) -> tuple[float, float]:
    """Convert column/row pixel corrections to east/north metres."""
    return dx_px * gsd_m, -dy_px * gsd_m


def apply_shift(
    arr: np.ndarray, *, dx_px: float, dy_px: float, dz_m: float
) -> np.ndarray:
    """Apply a NaN-safe bilinear shift; out-of-bounds samples become NaN.

    Corrections are ``(dx_px, dy_px)``; output samples at
    ``(row - dy_px, col - dx_px)``. Returns float32 and preserves zero shifts
    bit-exactly.
    """
    height, width = arr.shape
    rows = np.arange(height, dtype=np.float64)
    cols = np.arange(width, dtype=np.float64)
    src_row = rows - dy_px
    src_col = cols - dx_px

    row0 = np.floor(src_row)
    col0 = np.floor(src_col)
    frac_row = (src_row - row0)[:, None]
    frac_col = (src_col - col0)[None, :]

    row0_i = row0.astype(np.int64)
    col0_i = col0.astype(np.int64)
    row1_i = row0_i + 1
    col1_i = col0_i + 1

    source = arr.astype(np.float64, copy=False)

    def _gather(rr: np.ndarray, cc: np.ndarray) -> np.ndarray:
        row_in_bounds = (rr >= 0) & (rr < height)
        col_in_bounds = (cc >= 0) & (cc < width)
        rr_clipped = np.clip(rr, 0, height - 1)
        cc_clipped = np.clip(cc, 0, width - 1)
        values = source[np.ix_(rr_clipped, cc_clipped)]
        in_bounds = row_in_bounds[:, None] & col_in_bounds[None, :]
        return np.where(in_bounds, values, np.nan)

    v00 = _gather(row0_i, col0_i)
    v01 = _gather(row0_i, col1_i)
    v10 = _gather(row1_i, col0_i)
    v11 = _gather(row1_i, col1_i)

    w00 = (1.0 - frac_row) * (1.0 - frac_col)
    w01 = (1.0 - frac_row) * frac_col
    w10 = frac_row * (1.0 - frac_col)
    w11 = frac_row * frac_col

    def _weighted(weight: np.ndarray, value: np.ndarray) -> np.ndarray:
        # Avoid IEEE 0 * NaN contaminating samples with zero weight.
        return np.where(weight == 0.0, 0.0, weight * value)

    total = (
        _weighted(w00, v00)
        + _weighted(w01, v01)
        + _weighted(w10, v10)
        + _weighted(w11, v11)
    )
    return (total + dz_m).astype(np.float32)


def _prep(a: np.ndarray) -> np.ndarray:
    """Mean-remove nodata and Hann-taper the raster before correlation."""
    valid = np.isfinite(a)
    mean = float(np.mean(a[valid], dtype=np.float64)) if valid.any() else 0.0
    x = np.where(valid, a - mean, 0.0).astype(np.float64)
    hann_y = np.hanning(x.shape[0])[:, None]
    hann_x = np.hanning(x.shape[1])[None, :]
    return x * hann_y * hann_x


def estimate_shift_px(
    base: np.ndarray, inset: np.ndarray, *, max_shift_px: int = MAX_SHIFT_PX
) -> tuple[float, float]:
    """Estimate an integer ``(dy, dx)`` correction with phase correlation.

    Search is bounded by raster size and ``max_shift_px``. Raises
    ``ValueError`` if no nonzero search band is possible.
    """
    a, b = _prep(base), _prep(inset)
    fa, fb = np.fft.rfft2(a), np.fft.rfft2(b)
    cross_power = fa * np.conj(fb)
    magnitude = np.abs(cross_power)
    cross_power = cross_power / np.where(magnitude > 0.0, magnitude, 1.0)
    corr = np.fft.fftshift(np.fft.irfft2(cross_power, s=a.shape))
    center_y, center_x = a.shape[0] // 2, a.shape[1] // 2
    # Keep slice starts nonnegative to select the intended search band.
    k = int(min(max_shift_px, min(center_y, center_x) - 1))
    if k < 1:
        raise ValueError(
            f"no search band: array {a.shape}, max_shift_px={max_shift_px}"
        )
    band = corr[center_y - k : center_y + k + 1, center_x - k : center_x + k + 1]
    peak_y, peak_x = np.unravel_index(int(np.argmax(band)), band.shape)
    return float(peak_y - k), float(peak_x - k)


def coregister(
    base: np.ndarray,
    inset: np.ndarray,
    *,
    gsd_m: float,
    record: AlignmentRecord,
    estimate: bool,
    max_shift_px: int = MAX_SHIFT_PX,
    qc_max_residual_m: float = QC_MAX_RESIDUAL_M,
    trend_sigma_px: float = 1.0,
) -> tuple[np.ndarray, AppliedShift]:
    """Apply or estimate alignment, then gate it by detrended residual.

    Estimation is limited to enabled ``assumed_zero`` records. Unverifiable
    estimates fall back to the record; residuals above the QC limit demote the
    inset. Returns the aligned array and attempt diagnostics.
    """
    estimate_requested = record.source == "assumed_zero" and estimate
    should_estimate = estimate_requested and min(base.shape) >= 4
    if should_estimate:
        dy_px, dx_px = estimate_shift_px(base, inset, max_shift_px=max_shift_px)
        estimated_dx_m, estimated_dy_m = _shift_px_to_m(dx_px, dy_px, gsd_m=gsd_m)
        dz_m = record.dz_m
        source: AlignmentSource = "cross_correlation"
    else:
        dx_m, dy_m, dz_m = record.dx_m, record.dy_m, record.dz_m
        source = record.source
        dx_px, dy_px = _shift_m_to_px(dx_m, dy_m, gsd_m=gsd_m)

    shifted = apply_shift(inset, dx_px=dx_px, dy_px=dy_px, dz_m=dz_m)

    zero_shifted = (
        apply_shift(inset, dx_px=0.0, dy_px=0.0, dz_m=dz_m)
        if should_estimate
        else shifted
    )
    # Compare both shifts on identical support after removing fusion's trend.
    support = np.isfinite(base) & np.isfinite(shifted) & np.isfinite(zero_shifted)

    def _residual(candidate: np.ndarray) -> float | None:
        if not support.any():
            return None
        comparison_base = np.where(support, base, np.nan)
        comparison_candidate = np.where(support, candidate, np.nan)
        adjusted = detrend_inset(
            comparison_candidate, comparison_base, sigma_px=trend_sigma_px
        )
        return float(np.median(np.abs(adjusted[support] - comparison_base[support])))

    baseline_residual = _residual(zero_shifted)
    residual_m = _residual(shifted)

    if should_estimate and residual_m is None:
        # A degenerate correlation still yields an argmax; without overlap,
        # keep the record shift rather than trust an unverifiable estimate.
        fallback_dx_px, fallback_dy_px = _shift_m_to_px(
            record.dx_m, record.dy_m, gsd_m=gsd_m
        )
        fallback = apply_shift(
            inset, dx_px=fallback_dx_px, dy_px=fallback_dy_px, dz_m=record.dz_m
        )
        return fallback, AppliedShift(
            dx_m=record.dx_m,
            dy_m=record.dy_m,
            dz_m=record.dz_m,
            source="assumed_zero",
            residual_m=None,
            demoted=False,
            reason="could not verify: no finite overlap between base and inset",
        )

    if should_estimate and residual_m is not None and baseline_residual is not None:
        required_improvement = max(1e-6, 0.001 * baseline_residual)
        if baseline_residual - residual_m <= required_improvement:
            shifted = zero_shifted
            dx_m, dy_m = record.dx_m, record.dy_m
            dx_px, dy_px = _shift_m_to_px(dx_m, dy_m, gsd_m=gsd_m)
            source = record.source
            residual_m = baseline_residual
            estimate_reason = (
                f"estimated shift did not improve detrended residual by more "
                f"than {required_improvement:.6g} m"
            )
        else:
            dx_m, dy_m = estimated_dx_m, estimated_dy_m
            estimate_reason = None
    else:
        estimate_reason = (
            "co-registration estimate skipped: raster is smaller than 4x4"
            if estimate_requested and not should_estimate
            else None
        )

    if residual_m is not None and residual_m > qc_max_residual_m:
        reason = (
            f"co-registration residual {residual_m:.3f} m exceeds "
            f"qc_max_residual_m={qc_max_residual_m:.3f} m"
        )
        return inset.astype(np.float32), AppliedShift(
            dx_m=dx_m,
            dy_m=dy_m,
            dz_m=dz_m,
            source=source,
            residual_m=residual_m,
            demoted=True,
            reason=reason,
        )

    return shifted, AppliedShift(
        dx_m=dx_m,
        dy_m=dy_m,
        dz_m=dz_m,
        source=source,
        residual_m=residual_m,
        demoted=False,
        reason=estimate_reason,
    )
