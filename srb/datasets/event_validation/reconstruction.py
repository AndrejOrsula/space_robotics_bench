"""Reconstruction operators and fidelity metrics for event-camera frames."""

from __future__ import annotations

import contextlib
import io
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from srb.core.sensor.event_camera.extras import require_extra
from srb.datasets.event_validation.statistical import (
    _fixture_score_provenance,
    _publish_scorecard,
)


class E2VIDModelLoader(Protocol):
    """Callable that loads an E2VID-compatible recurrent model."""

    def __call__(self, checkpoint_path: Path, *, device: Any) -> Any: ...


def baseline_polarity_accumulation(
    events: dict[str, Any] | Any,
    *,
    resolution: tuple[int, int],
    frame_times_ns: np.ndarray,
) -> np.ndarray:
    """Accumulate polarity into ``(T, H, W)`` float32 frames in ``[0, 1]``.

    Resolution is ``(width, height)``; events map to the first frame time at
    or after their timestamp. Empty frames are uniform ``0.5``.
    """
    w, h = int(resolution[0]), int(resolution[1])
    frame_times = np.asarray(frame_times_ns, dtype=np.int64).ravel()
    if frame_times.size == 0:
        return np.empty((0, h, w), dtype=np.float32)

    t = np.asarray(events["t"], dtype=np.int64).ravel()
    x = np.asarray(events["x"], dtype=np.int64).ravel()
    y = np.asarray(events["y"], dtype=np.int64).ravel()
    p = np.asarray(events["p"], dtype=np.int64).ravel()

    frames = np.full((frame_times.size, h, w), 0.5, dtype=np.float32)
    if t.size == 0:
        return frames

    # bucket: index of the first frame_time >= each event's t; events with bucket
    # == T fall after the last frame and are ignored.
    bucket = np.searchsorted(frame_times, t, side="left")
    valid = (bucket < frame_times.size) & (x >= 0) & (x < w) & (y >= 0) & (y < h)
    if not valid.any():
        return frames

    bx = x[valid]
    by = y[valid]
    bp = p[valid].astype(np.float32)
    bb = bucket[valid]

    flat = (bb * (h * w) + by * w + bx).astype(np.int64)
    summed = np.bincount(flat, weights=bp, minlength=frame_times.size * h * w)
    voltage = summed.reshape(frame_times.size, h, w).astype(np.float32)

    out = np.full_like(voltage, 0.5)
    for i in range(voltage.shape[0]):
        v = voltage[i]
        lo, hi = float(v.min()), float(v.max())
        if hi > lo:
            out[i] = (v - lo) / (hi - lo)
    return out


def e2vid_reconstruct(
    events: dict[str, Any] | Any,
    *,
    resolution: tuple[int, int],
    frame_times_ns: np.ndarray,
    checkpoint_path: Path | str | None = None,
    device: str = "cpu",
    model_loader: E2VIDModelLoader | None = None,
) -> np.ndarray:
    """Run recurrent E2VID on event windows and return ``(T, H, W)`` frames.

    Requires ``srb[event-reconstruction]`` and a checkpoint. Fetch the default with
    ``srb event-camera fetch-weights`` or pass ``checkpoint_path`` for another file.
    """
    with require_extra("event-reconstruction"):
        import torch

        torch_device = torch.device(device)
        path = _resolve_e2vid_checkpoint_path(checkpoint_path)
        loader = model_loader or _load_e2vid_model
        model = loader(path, device=torch_device)
        if hasattr(model, "to"):
            model = model.to(torch_device)
        if hasattr(model, "eval"):
            model.eval()

        voxel_grids = events_to_e2vid_voxel_grids(
            events,
            resolution=resolution,
            frame_times_ns=frame_times_ns,
            num_bins=5,
            normalize=True,
        )
        if voxel_grids.shape[0] == 0:
            return np.empty(
                (0, int(resolution[1]), int(resolution[0])), dtype=np.float32
            )

        frames: list[np.ndarray] = []
        states = None
        with torch.no_grad():
            for voxel_grid in voxel_grids:
                event_tensor = (
                    torch.from_numpy(voxel_grid).unsqueeze(0).to(torch_device)
                )
                predicted, states = model(event_tensor, states)
                frames.append(_e2vid_prediction_to_numpy(predicted))
        return np.stack(frames, axis=0).astype(np.float32, copy=False)


def events_to_e2vid_voxel_grids(
    events: dict[str, Any] | Any,
    *,
    resolution: tuple[int, int],
    frame_times_ns: np.ndarray,
    num_bins: int = 5,
    normalize: bool = True,
) -> np.ndarray:
    """Convert event arrays to ``(T, num_bins, H, W)`` float32 voxel grids.

    Events map to the first frame timestamp at or after their time, then
    interpolate between adjacent temporal bins.
    """
    w, h = int(resolution[0]), int(resolution[1])
    frame_times = np.asarray(frame_times_ns, dtype=np.int64).ravel()
    if frame_times.size == 0:
        return np.empty((0, int(num_bins), h, w), dtype=np.float32)
    if num_bins <= 0:
        raise ValueError(f"num_bins must be positive; got {num_bins}")

    t = np.asarray(events["t"], dtype=np.int64).ravel()
    x = np.asarray(events["x"], dtype=np.int64).ravel()
    y = np.asarray(events["y"], dtype=np.int64).ravel()
    p = np.asarray(events["p"], dtype=np.float32).ravel()

    grids = np.zeros((frame_times.size, int(num_bins), h, w), dtype=np.float32)
    if t.size == 0:
        return grids

    polarity = np.where(p > 0, 1.0, -1.0).astype(np.float32)
    bucket = np.searchsorted(frame_times, t, side="left")
    valid = (bucket < frame_times.size) & (x >= 0) & (x < w) & (y >= 0) & (y < h)
    if not valid.any():
        return grids

    t = t[valid]
    x = x[valid]
    y = y[valid]
    polarity = polarity[valid]
    bucket = bucket[valid]

    for frame_idx in range(frame_times.size):
        mask = bucket == frame_idx
        if not mask.any():
            continue
        window_t = t[mask]
        start = (
            int(frame_times[frame_idx - 1]) if frame_idx > 0 else int(window_t.min())
        )
        end = int(frame_times[frame_idx])
        if end <= start:
            end = int(window_t.max())
        if end <= start:
            scaled_t = np.zeros_like(window_t, dtype=np.float32)
        else:
            scaled_t = (
                (window_t - start).astype(np.float32)
                * float(num_bins - 1)
                / float(end - start)
            )
            scaled_t = np.clip(scaled_t, 0.0, float(num_bins - 1))

        left = np.floor(scaled_t).astype(np.int64)
        dt = scaled_t - left.astype(np.float32)
        xs = x[mask]
        ys = y[mask]
        ps = polarity[mask]

        np.add.at(grids[frame_idx], (left, ys, xs), ps * (1.0 - dt))
        right = left + 1
        has_right = right < num_bins
        if has_right.any():
            np.add.at(
                grids[frame_idx],
                (right[has_right], ys[has_right], xs[has_right]),
                ps[has_right] * dt[has_right],
            )

        if normalize:
            _normalize_e2vid_voxel_grid_in_place(grids[frame_idx])
    return grids


def _resolve_e2vid_checkpoint_path(checkpoint_path: Path | str | None) -> Path:
    if checkpoint_path is not None:
        path = Path(checkpoint_path).expanduser()
    else:
        from srb.core.sensor.event_camera.weights import (
            E2VID_DEFAULT,
            default_cache_dir,
        )

        path = default_cache_dir() / E2VID_DEFAULT.name
    if not path.is_file():
        raise FileNotFoundError(
            f"E2VID checkpoint not found at {path}. Fetch it with "
            "`srb event-camera fetch-weights --mirror hf-andrejorsula-e2vid`, "
            "or pass checkpoint_path= explicitly."
        )
    return path


def _load_e2vid_model(checkpoint_path: Path, *, device: Any) -> Any:
    from e2vid.utils.loading_utils import load_model

    # Upstream loading_utils prints progress to stdout; SRB CLIs reserve stdout
    # for structured JSON.
    with contextlib.redirect_stdout(io.StringIO()):
        return load_model(str(checkpoint_path), device=device)


def _normalize_e2vid_voxel_grid_in_place(voxel_grid: np.ndarray) -> None:
    nonzero = voxel_grid != 0
    if not nonzero.any():
        return
    values = voxel_grid[nonzero]
    std = float(values.std())
    if std <= 0.0:
        return
    voxel_grid[nonzero] = (values - float(values.mean())) / std


def _e2vid_prediction_to_numpy(predicted: Any) -> np.ndarray:
    if isinstance(predicted, dict):
        predicted = predicted.get("image", predicted)
    array = predicted.detach().cpu().numpy()
    array = np.asarray(array, dtype=np.float32)
    if array.ndim == 4:
        array = array[0, 0]
    elif array.ndim == 3:
        array = array[0]
    if array.size and float(array.max()) > 1.0:
        array = array / 255.0
    return np.clip(array, 0.0, 1.0).astype(np.float32, copy=False)


def ssim_score(
    a: np.ndarray,
    b: np.ndarray,
    *,
    data_range: float = 1.0,
    k1: float = 0.01,
    k2: float = 0.03,
) -> float:
    """Return mean global per-frame SSIM for matching ``(T, H, W)`` arrays.

    Uses each frame's full pixel grid, without a sliding window or SciPy.
    Returns NaN for empty or mismatched inputs.
    """
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    if a.shape != b.shape or a.size == 0:
        return float("nan")
    if a.ndim == 2:
        a = a[None, ...]
        b = b[None, ...]
    c1 = (k1 * data_range) ** 2
    c2 = (k2 * data_range) ** 2
    out = np.empty(a.shape[0], dtype=np.float64)
    for i in range(a.shape[0]):
        ai = a[i].ravel()
        bi = b[i].ravel()
        mu_a, mu_b = float(ai.mean()), float(bi.mean())
        var_a, var_b = float(ai.var()), float(bi.var())
        cov = float(((ai - mu_a) * (bi - mu_b)).mean())
        num = (2 * mu_a * mu_b + c1) * (2 * cov + c2)
        den = (mu_a * mu_a + mu_b * mu_b + c1) * (var_a + var_b + c2)
        out[i] = num / den if den != 0 else float("nan")
    return float(np.nanmean(out))


def lpips_score(
    a: np.ndarray,
    b: np.ndarray,
    *,
    net: str = "alex",
) -> float:
    """Return LPIPS distance for ``(T, H, W)`` inputs in ``[0, 1]``.

    Requires ``srb[event-reconstruction]``; inputs map to three channels in
    ``[-1, 1]`` for LPIPS.
    """
    if a.shape != b.shape or a.size == 0:
        return float("nan")
    with require_extra("event-reconstruction"):
        import lpips  # noqa: F401
        import torch

        loss_fn = lpips.LPIPS(net=net)
        loss_fn.eval()
        with torch.no_grad():
            a_t = torch.from_numpy(np.asarray(a, dtype=np.float32))
            b_t = torch.from_numpy(np.asarray(b, dtype=np.float32))
            a_t = a_t.unsqueeze(1).repeat(1, 3, 1, 1) * 2.0 - 1.0
            b_t = b_t.unsqueeze(1).repeat(1, 3, 1, 1) * 2.0 - 1.0
            d = loss_fn(a_t, b_t).squeeze().cpu().numpy()
    return float(np.mean(d))


def rgb_to_grayscale(rgb: np.ndarray) -> np.ndarray:
    """Convert ``(T, H, W, 3)`` RGB frames to BT.601 luma float32."""
    rgb = np.asarray(rgb)
    if rgb.ndim != 4 or rgb.shape[-1] != 3:
        raise ValueError(f"rgb must be (T, H, W, 3); got shape {rgb.shape}")
    if np.issubdtype(rgb.dtype, np.integer):
        rgb = rgb.astype(np.float32) / 255.0
    else:
        rgb = rgb.astype(np.float32)
    return 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]


@dataclass
class ReconstructionScoreCard:
    """Per-sequence metrics matching the validation CLI JSON schema."""

    metrics: dict[str, Any] = field(default_factory=dict)
    operator: str = "baseline"
    n_frames: int = 0
    resolution: list[int] = field(default_factory=list)
    schema_version: int = 1

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True, allow_nan=False)


def score_reconstruction(
    events: dict[str, Any] | Any,
    rgb: np.ndarray,
    frame_times_ns: np.ndarray,
    *,
    resolution: tuple[int, int] | None = None,
    operator: str = "baseline",
    include_lpips: bool = False,
) -> ReconstructionScoreCard:
    """Reconstruct at ``frame_times_ns`` and score against RGB frames.

    ``rgb`` is ``(T, H, W, 3)``; resolution defaults to event metadata then
    RGB dimensions. Select ``baseline`` or gated ``e2vid``; LPIPS is optional.
    """
    rgb = np.asarray(rgb)
    if rgb.ndim != 4 or rgb.shape[-1] != 3:
        raise ValueError(f"rgb must be (T, H, W, 3); got shape {rgb.shape}")
    if rgb.shape[0] == 0:
        raise ValueError("rgb must contain at least one frame")
    if not np.isfinite(rgb).all():
        raise ValueError("rgb must contain only finite values")
    res = _resolve_resolution(events, resolution, rgb)
    rgb_resolution = (int(rgb.shape[2]), int(rgb.shape[1]))
    if res != rgb_resolution:
        raise ValueError(
            "event resolution must match rgb frame dimensions: "
            f"{res} != {rgb_resolution}"
        )
    frame_times = np.asarray(frame_times_ns, dtype=np.int64).ravel()
    if frame_times.shape[0] != rgb.shape[0]:
        raise ValueError(
            f"frame_times_ns has {frame_times.shape[0]} entries but rgb has "
            f"{rgb.shape[0]} frames"
        )

    if operator == "baseline":
        recon = baseline_polarity_accumulation(
            events, resolution=res, frame_times_ns=frame_times
        )
    elif operator == "e2vid":
        recon = e2vid_reconstruct(events, resolution=res, frame_times_ns=frame_times)
    else:
        raise ValueError(
            f"unknown operator {operator!r}; expected 'baseline' or 'e2vid'"
        )

    if not np.isfinite(recon).all():
        raise ValueError("reconstruction must contain only finite values")
    gt_gray = rgb_to_grayscale(rgb)
    if not np.isfinite(gt_gray).all():
        raise ValueError("grayscale rgb frames must contain only finite values")
    ssim = ssim_score(recon, gt_gray)
    if not np.isfinite(ssim):
        raise ValueError("SSIM score is not finite")
    metrics: dict[str, Any] = {
        "ssim": ssim,
        "n_frames": int(recon.shape[0]),
    }
    if include_lpips:
        lpips = lpips_score(recon, gt_gray)
        if not np.isfinite(lpips):
            raise ValueError("LPIPS score is not finite")
        metrics["lpips"] = lpips
    else:
        metrics["lpips"] = None

    fixture_provenance = _fixture_score_provenance(events, label="reconstruction")
    if fixture_provenance is not None:
        metrics.update(fixture_provenance)

    return ReconstructionScoreCard(
        metrics=metrics,
        operator=operator,
        n_frames=int(recon.shape[0]),
        resolution=[res[0], res[1]],
    )


def write_scorecard(card: ReconstructionScoreCard, path: Path | str) -> None:
    """Persist a :class:`ReconstructionScoreCard` as pretty-printed JSON."""
    _publish_scorecard(card, path, _validate_scorecard_output)


def _validate_scorecard_output(path: Path) -> None:
    try:
        payload = json.loads(path.read_bytes(), parse_constant=_reject_json_constant)
    except (OSError, UnicodeDecodeError, ValueError) as exc:
        raise ValueError(f"malformed reconstruction scorecard: {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("reconstruction scorecard must be a JSON object")
    if set(payload) != {
        "metrics",
        "operator",
        "n_frames",
        "resolution",
        "schema_version",
    }:
        raise ValueError("reconstruction scorecard schema is malformed")
    if payload["schema_version"] != 1:
        raise ValueError("unsupported reconstruction scorecard schema version")
    if not isinstance(payload["metrics"], dict):
        raise ValueError("reconstruction scorecard metrics must be an object")
    _validate_finite_metrics(payload["metrics"])


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON constant {value!r}")


def _validate_finite_metrics(value: Any, *, path: str = "metrics") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"reconstruction scorecard {path} must be finite")
    if isinstance(value, dict):
        for key, item in value.items():
            _validate_finite_metrics(item, path=f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _validate_finite_metrics(item, path=f"{path}[{index}]")


def _resolve_resolution(
    events: dict[str, Any] | Any,
    explicit: tuple[int, int] | None,
    rgb: np.ndarray,
) -> tuple[int, int]:
    if explicit is not None:
        return (int(explicit[0]), int(explicit[1]))
    meta = events.get("metadata", {}) if isinstance(events, dict) else {}
    res = meta.get("resolution")
    if res is not None and len(res) == 2:
        return (int(res[0]), int(res[1]))
    # Fallback: derive from RGB (W = shape[2], H = shape[1]).
    if rgb.ndim == 4:
        return (int(rgb.shape[2]), int(rgb.shape[1]))
    raise ValueError(
        "resolution missing from events metadata; pass resolution= explicitly"
    )


__all__ = [
    "ReconstructionScoreCard",
    "baseline_polarity_accumulation",
    "e2vid_reconstruct",
    "events_to_e2vid_voxel_grids",
    "lpips_score",
    "rgb_to_grayscale",
    "score_reconstruction",
    "ssim_score",
    "write_scorecard",
]
