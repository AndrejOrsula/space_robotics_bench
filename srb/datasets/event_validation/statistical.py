"""NumPy fidelity metrics for event streams with ``t, x, y, p`` arrays.

Empty distance inputs yield NaN; JSON output encodes non-finite values as
``null`` and includes ON/OFF counts.
"""

from __future__ import annotations

import json
import math
import warnings
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from srb.core.sensor.event_camera.manifest import (
    SYNTHETIC_FIXTURE_TASK_ID,
)
from srb.utils.publication import ArtifactPublication

# ---------- core single-stream metrics -----------------------------------


def event_rate_hz(t: np.ndarray) -> float:
    """Return event rate in Hz over the timestamp span in nanoseconds.

    Uses the Prophesee RAW EVT3 convention; fewer than two timestamps or a
    nonpositive span yields NaN.
    """
    t = np.asarray(t, dtype=np.int64).ravel()
    if t.size < 2:
        return float("nan")
    span_ns = int(t[-1] - t[0])
    if span_ns <= 0:
        return float("nan")
    return float(t.size) * 1e9 / float(span_ns)


def on_off_ratio(p: np.ndarray) -> float:
    """Return ON/OFF ratio for signed polarity; zero OFF yields infinity, empty yields
    NaN.
    """
    p = np.asarray(p).ravel()
    on = int(np.sum(p > 0))
    off = int(np.sum(p < 0))
    if on == 0 and off == 0:
        return float("nan")
    if off == 0:
        return float("inf")
    return on / off


def spatial_entropy(
    x: np.ndarray,
    y: np.ndarray,
    *,
    resolution: tuple[int, int],
) -> float:
    """Return per-pixel event-density entropy in bits; empty streams yield 0."""
    w, h = resolution
    if x.size == 0:
        return 0.0
    counts = _per_pixel_counts(x, y, w=w, h=h).ravel().astype(np.float64)
    total = counts.sum()
    if total <= 0:
        return 0.0
    probs = counts / total
    nonzero = probs[probs > 0]
    return float(-np.sum(nonzero * np.log2(nonzero)))


def isi_distribution_ns(t: np.ndarray) -> np.ndarray:
    """Inter-event-time intervals in ns (the diff of sorted timestamps)."""
    t = np.asarray(t, dtype=np.int64).ravel()
    if t.size < 2:
        return np.empty(0, dtype=np.int64)
    t_sorted = np.sort(t)
    return np.diff(t_sorted)


def hot_pixel_detection_rate(
    x: np.ndarray,
    y: np.ndarray,
    *,
    resolution: tuple[int, int],
    sigma_k: float = 6.0,
) -> dict[str, Any]:
    """Flag pixels above ``mean + sigma_k * std``; return fraction, count, threshold."""
    w, h = resolution
    if x.size == 0:
        return {"fraction": 0.0, "count": 0, "threshold": 0.0}
    counts = _per_pixel_counts(x, y, w=w, h=h).ravel().astype(np.float64)
    mean = float(counts.mean())
    std = float(counts.std())
    threshold = mean + float(sigma_k) * std
    hot = int(np.sum(counts > threshold))
    return {
        "fraction": hot / float(counts.size),
        "count": hot,
        "threshold": threshold,
    }


def spatial_sparsity(
    x: np.ndarray,
    y: np.ndarray,
    t: np.ndarray,
    *,
    resolution: tuple[int, int],
    window_ns: int = 1_000_000,  # 1 ms
) -> float:
    """Return mean active-pixel fraction across non-overlapping time windows.

    Empty windows count as zero; empty streams yield ``0.0``.
    """
    w, h = resolution
    t = np.asarray(t, dtype=np.int64).ravel()
    if t.size == 0 or window_ns <= 0:
        return 0.0
    x_arr = np.asarray(x, dtype=np.int64).ravel()
    y_arr = np.asarray(y, dtype=np.int64).ravel()
    t0 = int(t[0])
    bins = ((t - t0) // int(window_ns)).astype(np.int64)
    pixel_keys = y_arr.astype(np.int64) * int(w) + x_arr.astype(np.int64)
    n_pixels = float(w * h)
    fractions: list[float] = []
    # Sort by bin so we can stream unique-per-bin without a full groupby.
    order = np.argsort(bins, kind="stable")
    bins_sorted = bins[order]
    keys_sorted = pixel_keys[order]
    unique_bins, starts = np.unique(bins_sorted, return_index=True)
    starts = np.append(starts, bins_sorted.size)
    for i in range(unique_bins.size):
        seg = keys_sorted[starts[i] : starts[i + 1]]
        fractions.append(float(np.unique(seg).size) / n_pixels)
    # Average over ALL windows spanning the stream (empty windows count as 0),
    # not just the windows that contain events — otherwise temporal gaps inflate
    # the reported sparsity, contradicting the "across the stream's span" contract.
    total_windows = int(bins_sorted[-1] - bins_sorted[0]) + 1
    return float(sum(fractions) / total_windows)


# ---------- two-stream distance metrics ----------------------------------


def ks_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Return two-sample Kolmogorov–Smirnov distance; empty inputs yield NaN."""
    a = np.sort(np.asarray(a, dtype=np.float64).ravel())
    b = np.sort(np.asarray(b, dtype=np.float64).ravel())
    if a.size == 0 or b.size == 0:
        return float("nan")
    grid = np.unique(np.concatenate([a, b]))
    cdf_a = np.searchsorted(a, grid, side="right") / a.size
    cdf_b = np.searchsorted(b, grid, side="right") / b.size
    return float(np.max(np.abs(cdf_a - cdf_b)))


def wasserstein1_1d(a: np.ndarray, b: np.ndarray) -> float:
    """Return 1-D Wasserstein-1 distance; empty inputs yield NaN."""
    a = np.sort(np.asarray(a, dtype=np.float64).ravel())
    b = np.sort(np.asarray(b, dtype=np.float64).ravel())
    if a.size == 0 or b.size == 0:
        return float("nan")
    if a.size == b.size:
        return float(np.mean(np.abs(a - b)))
    grid = np.unique(np.concatenate((a, b)))
    if grid.size < 2:
        return 0.0
    cdf_a = np.searchsorted(a, grid[:-1], side="right") / a.size
    cdf_b = np.searchsorted(b, grid[:-1], side="right") / b.size
    return float(np.sum(np.abs(cdf_a - cdf_b) * np.diff(grid)))


# ---------- scorecard composition ----------------------------------------


@dataclass
class ScoreCard:
    """Per-stream and pairwise fidelity scores in the CLI JSON layout."""

    synth: dict[str, Any] = field(default_factory=dict)
    real: dict[str, Any] = field(default_factory=dict)
    distances: dict[str, Any] = field(default_factory=dict)
    schema_version: int = 1

    def to_json(self) -> str:
        return json.dumps(
            _json_ready(asdict(self)),
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )


def compute_metrics(
    events: dict[str, Any] | Any,
    *,
    resolution: tuple[int, int] | None = None,
    sparsity_window_ns: int = 1_000_000,
) -> dict[str, Any]:
    """Compute one stream's metrics for :class:`ScoreCard`.

    Requires ``t, x, y, p`` arrays. Resolution comes from metadata unless
    explicitly supplied, which takes precedence.
    """
    t = np.asarray(events["t"])
    x = np.asarray(events["x"])
    y = np.asarray(events["y"])
    p = np.asarray(events["p"])

    res = _resolve_resolution(events, resolution)

    on_count = int(np.sum(p > 0))
    off_count = int(np.sum(p < 0))
    return {
        "n_events": int(t.size),
        "event_rate_hz": event_rate_hz(t),
        "on_off_ratio": on_off_ratio(p),
        "on_count": on_count,
        "off_count": off_count,
        "spatial_entropy_bits": spatial_entropy(x, y, resolution=res),
        "hot_pixels": hot_pixel_detection_rate(x, y, resolution=res),
        "spatial_sparsity": spatial_sparsity(
            x, y, t, resolution=res, window_ns=sparsity_window_ns
        ),
        "resolution": list(res),
    }


def score_pair(
    synth: dict[str, Any] | Any,
    real: dict[str, Any] | Any,
    *,
    resolution: tuple[int, int] | None = None,
    sparsity_window_ns: int = 1_000_000,
) -> ScoreCard:
    """Build a scorecard with per-stream and pairwise synth/real metrics.

    Declared resolutions must match, and out-of-bounds coordinates are rejected.
    """

    def declared_resolution(events: dict[str, Any] | Any) -> tuple[int, int]:
        try:
            return _resolve_resolution(events, None)
        except ValueError:
            if resolution is None:
                raise
            return resolution

    synth_resolution = declared_resolution(synth)
    real_resolution = declared_resolution(real)
    if synth_resolution != real_resolution:
        raise ValueError(
            "synth and real resolutions must match for statistical comparison: "
            f"{synth_resolution} != {real_resolution}"
        )
    res = synth_resolution
    _reject_out_of_bounds_events(synth, res, label="synth")
    _reject_out_of_bounds_events(real, res, label="real")

    s = compute_metrics(synth, resolution=res, sparsity_window_ns=sparsity_window_ns)
    r = compute_metrics(real, resolution=res, sparsity_window_ns=sparsity_window_ns)
    synth_fixture = _fixture_score_provenance(synth, label="synth")
    real_fixture = _fixture_score_provenance(real, label="real")
    if synth_fixture is not None:
        s.update(synth_fixture)
    if real_fixture is not None:
        r.update(real_fixture)

    synth_isi = isi_distribution_ns(np.asarray(synth["t"]))
    real_isi = isi_distribution_ns(np.asarray(real["t"]))
    synth_span_s = _timestamp_span_seconds(np.asarray(synth["t"]))
    real_span_s = _timestamp_span_seconds(np.asarray(real["t"]))
    if not math.isfinite(synth_span_s) or not math.isfinite(real_span_s):
        rate_emd = float("nan")
    else:
        synth_rates = (
            _per_pixel_counts(
                np.asarray(synth["x"]), np.asarray(synth["y"]), w=res[0], h=res[1]
            ).ravel()
            / synth_span_s
        )
        real_rates = (
            _per_pixel_counts(
                np.asarray(real["x"]), np.asarray(real["y"]), w=res[0], h=res[1]
            ).ravel()
            / real_span_s
        )
        rate_emd = wasserstein1_1d(synth_rates, real_rates)

    distances = {
        "isi_ks_ns": ks_distance(synth_isi, real_isi),
        "per_pixel_rate_emd": rate_emd,
    }
    return ScoreCard(synth=s, real=r, distances=distances)


def _fixture_score_provenance(
    events: dict[str, Any] | Any, *, label: str
) -> dict[str, str] | None:
    """Mark an artifact-backed score block carrying the reserved fixture ID."""
    metadata = events.get("metadata") if isinstance(events, dict) else None
    if not isinstance(metadata, dict):
        return None
    task_id = metadata.get("task_id")
    if task_id != SYNTHETIC_FIXTURE_TASK_ID:
        return None
    warnings.warn(
        f"{label} input is a synthetic fixture, not task or real-data fidelity evidence",
        UserWarning,
        stacklevel=3,
    )
    policy_id = metadata.get("policy_id", "")
    return {
        "source": "synthetic_fixture",
        "task_id": str(task_id),
        "policy_id": str(policy_id),
    }


# ---------- helpers ------------------------------------------------------


def _per_pixel_counts(x: np.ndarray, y: np.ndarray, *, w: int, h: int) -> np.ndarray:
    """Dense ``(h, w)`` count of events per pixel."""
    if x.size == 0:
        return np.zeros((h, w), dtype=np.int64)
    x_arr = np.asarray(x, dtype=np.int64).ravel()
    y_arr = np.asarray(y, dtype=np.int64).ravel()
    # Clip out-of-bounds defensively (real captures can hit border pixels
    # via lens distortion calibration). This keeps metrics finite even when
    # the upstream renderer emits a stray edge event.
    mask = (x_arr >= 0) & (x_arr < w) & (y_arr >= 0) & (y_arr < h)
    if not mask.all():
        x_arr = x_arr[mask]
        y_arr = y_arr[mask]
    flat = y_arr * w + x_arr
    counts = np.bincount(flat, minlength=w * h)
    return counts.reshape(h, w)


def _reject_out_of_bounds_events(
    events: dict[str, Any] | Any,
    resolution: tuple[int, int],
    *,
    label: str,
) -> None:
    """Reject coordinates the dense per-pixel metrics would otherwise drop."""
    w, h = resolution
    x = np.asarray(events["x"], dtype=np.int64).ravel()
    y = np.asarray(events["y"], dtype=np.int64).ravel()
    out_of_bounds = (x < 0) | (x >= w) | (y < 0) | (y >= h)
    count = int(np.count_nonzero(out_of_bounds))
    if count:
        raise ValueError(
            f"{label} stream contains {count} out-of-bounds event coordinate(s) "
            f"for resolution {resolution}"
        )


def _timestamp_span_seconds(t: np.ndarray) -> float:
    """Return the event-rate convention's timestamp span in seconds."""
    timestamps = np.asarray(t, dtype=np.int64).ravel()
    if timestamps.size < 2:
        return float("nan")
    span_ns = int(timestamps[-1] - timestamps[0])
    if span_ns <= 0:
        return float("nan")
    return span_ns / 1e9


def _resolve_resolution(
    events: dict[str, Any] | Any,
    explicit: tuple[int, int] | None,
) -> tuple[int, int]:
    if explicit is not None:
        return (int(explicit[0]), int(explicit[1]))
    meta = events.get("metadata", {}) if isinstance(events, dict) else {}
    res = meta.get("resolution")
    if res is None or len(res) != 2:
        raise ValueError(
            "resolution missing from events metadata; pass resolution= explicitly"
        )
    return (int(res[0]), int(res[1]))


def write_scorecard(card: ScoreCard, path: Path | str) -> None:
    """Persist a :class:`ScoreCard` as pretty-printed JSON."""
    _publish_scorecard(card, path, _validate_scorecard_output)


def _publish_scorecard(
    card: Any, path: Path | str, validate: Callable[[Path], None]
) -> None:
    output = Path(path)
    payload = (card.to_json() + "\n").encode("utf-8")

    def build(staging: Path) -> None:
        staging.write_bytes(payload)
        validate(staging)

    ArtifactPublication.file(output).publish(build)


def _validate_scorecard_output(path: Path) -> None:
    """Validate a statistical scorecard before publication."""

    try:
        payload = json.loads(path.read_bytes())
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"malformed statistical scorecard: {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("statistical scorecard must be a JSON object")
    if set(payload) != {"synth", "real", "distances", "schema_version"}:
        raise ValueError("statistical scorecard schema is malformed")
    if type(payload["schema_version"]) is not int or payload["schema_version"] != 1:
        raise ValueError("unsupported statistical scorecard schema version")
    if any(
        not isinstance(payload[name], dict) for name in ("synth", "real", "distances")
    ):
        raise ValueError("statistical scorecard metric blocks must be objects")


def scorecard_json_payload(card: ScoreCard) -> dict[str, Any]:
    """Return the JSON-safe scorecard payload."""
    payload = _json_ready(asdict(card))
    if not isinstance(payload, dict):
        raise TypeError("scorecard payload must be a mapping")
    return payload


def _json_ready(value: Any) -> Any:
    if isinstance(value, np.generic):
        return _json_ready(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    return value


__all__ = [
    "ScoreCard",
    "compute_metrics",
    "event_rate_hz",
    "hot_pixel_detection_rate",
    "isi_distribution_ns",
    "ks_distance",
    "on_off_ratio",
    "score_pair",
    "scorecard_json_payload",
    "spatial_entropy",
    "spatial_sparsity",
    "wasserstein1_1d",
    "write_scorecard",
]
