"""Convert canonical ``(N, 4)`` events to dense tensors.

Register representations in the shared shape/conversion table and config
allowlist.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Final, NamedTuple

import torch

if TYPE_CHECKING:
    from srb.core.sensor.event_camera.config import EventCameraCfg

__all__ = [
    "EventRawOverflowError",
    "RepFns",
    "get_representation",
    "list_representations",
    "to_voxel_grid",
    "to_histogram",
    "to_time_surface",
    "to_stacked",
    "to_raw",
]


class EventRawOverflowError(ValueError):
    """The raw representation cannot represent every supplied event row."""

    code = "event_camera.raw_overflow"

    def __init__(self, *, rows: int, max_events: int) -> None:
        self.rows = rows
        self.max_events = max_events
        super().__init__(f"raw event rows {rows} exceed raw_max_events {max_events}")


def _split_columns(
    events: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    t = events[:, 0].to(torch.int64)
    x = events[:, 1].to(torch.int64)
    y = events[:, 2].to(torch.int64)
    p = events[:, 3].to(torch.int64)
    return t, x, y, p


def _in_bounds(x: torch.Tensor, y: torch.Tensor, H: int, W: int) -> torch.Tensor:
    return (x >= 0) & (x < W) & (y >= 0) & (y < H)


def _time_bins_for(t: torch.Tensor, time_bins: int) -> torch.Tensor:
    """Map each timestamp to a bin in ``[0, time_bins)`` via uniform binning."""
    t_min = t.min()
    t_max = t.max()
    span = (t_max - t_min).item()
    if span == 0:
        return torch.zeros_like(t, dtype=torch.int64)
    norm = (t - t_min).to(torch.float64) / float(span)
    bins = torch.floor(norm * time_bins).to(torch.int64)
    return bins.clamp(min=0, max=time_bins - 1)


def to_voxel_grid(
    events: torch.Tensor,
    *,
    H: int,
    W: int,
    time_bins: int,
    normalize: bool = True,
) -> torch.Tensor:
    """Signed-polarity voxel grid of shape ``(T, H, W)`` float32.

    Events binned uniformly in time across ``[t_min, t_max]`` into
    ``time_bins`` slices; per-pixel signed contributions accumulate. Optional
    normalisation divides by the max absolute value (zero events ⇒ zeros).
    """
    out = torch.zeros((time_bins, H, W), dtype=torch.float32, device=events.device)
    if events.numel() == 0:
        return out
    t, x, y, p = _split_columns(events)
    mask = _in_bounds(x, y, H, W)
    t, x, y, p = t[mask], x[mask], y[mask], p[mask]
    if t.numel() == 0:
        return out
    bins = _time_bins_for(t, time_bins)
    flat_idx = bins * (H * W) + y * W + x
    out.view(-1).index_add_(0, flat_idx, p.to(torch.float32))
    if normalize:
        m = out.abs().max()
        if m > 0:
            out = out / m
    return out


def to_histogram(
    events: torch.Tensor,
    *,
    H: int,
    W: int,
    normalize: bool = True,
) -> torch.Tensor:
    """Two-channel event-count histogram of shape ``(2, H, W)``.

    Channel 0 = ON (p=+1) count, channel 1 = OFF (p=-1) count. Optional
    normalisation divides by the max count.
    """
    out = torch.zeros((2, H, W), dtype=torch.float32, device=events.device)
    if events.numel() == 0:
        return out
    _, x, y, p = _split_columns(events)
    mask = _in_bounds(x, y, H, W)
    x, y, p = x[mask], y[mask], p[mask]
    pos = p > 0
    neg = p < 0
    if pos.any():
        idx = y[pos] * W + x[pos]
        out[0].view(-1).index_add_(
            0, idx, torch.ones(idx.shape, dtype=torch.float32, device=out.device)
        )
    if neg.any():
        idx = y[neg] * W + x[neg]
        out[1].view(-1).index_add_(
            0, idx, torch.ones(idx.shape, dtype=torch.float32, device=out.device)
        )
    if normalize:
        m = out.max()
        if m > 0:
            out = out / m
    return out


def to_time_surface(
    events: torch.Tensor,
    *,
    H: int,
    W: int,
    tau_s: float,
    t_ref_ns: int | None = None,
) -> torch.Tensor:
    """Per-polarity exponential-decay time surface of shape ``(2, H, W)``.

    For each (polarity, pixel), records the latest event time and computes
    ``exp(-(t_ref - t_last) / tau_ns)``. Pixels with no events stay zero.
    ``t_ref_ns`` defaults to the latest event timestamp. Rows need not be time
    sorted; the per-pixel latest time is a deterministic max reduction.
    """
    out = torch.zeros((2, H, W), dtype=torch.float32, device=events.device)
    if events.numel() == 0:
        return out
    t, x, y, p = _split_columns(events)
    mask = _in_bounds(x, y, H, W)
    t, x, y, p = t[mask], x[mask], y[mask], p[mask]
    if t.numel() == 0:
        return out
    if t_ref_ns is None:
        t_ref = int(t.max().item())
    else:
        t_ref = int(t_ref_ns)
    tau_ns = float(tau_s) * 1.0e9
    if tau_ns <= 0:
        raise ValueError(f"tau_s must be positive; got {tau_s}")
    # Max reduction keeps duplicate-pixel results deterministic for any row order.
    last_pos = torch.full((H * W,), -1, dtype=torch.int64, device=t.device)
    last_neg = torch.full((H * W,), -1, dtype=torch.int64, device=t.device)
    flat = y * W + x
    pos_mask = p > 0
    neg_mask = p < 0
    if pos_mask.any():
        last_pos.scatter_reduce_(
            0, flat[pos_mask], t[pos_mask], reduce="amax", include_self=True
        )
    if neg_mask.any():
        last_neg.scatter_reduce_(
            0, flat[neg_mask], t[neg_mask], reduce="amax", include_self=True
        )
    pos_dt = (t_ref - last_pos).to(torch.float64)
    neg_dt = (t_ref - last_neg).to(torch.float64)
    valid_pos = last_pos >= 0
    valid_neg = last_neg >= 0
    out[0].view(-1)[valid_pos] = torch.exp(-pos_dt[valid_pos] / tau_ns).to(
        torch.float32
    )
    out[1].view(-1)[valid_neg] = torch.exp(-neg_dt[valid_neg] / tau_ns).to(
        torch.float32
    )
    return out


def to_stacked(
    events: torch.Tensor,
    *,
    H: int,
    W: int,
    time_bins: int,
    normalize: bool = True,
) -> torch.Tensor:
    """Polarity × time-bin stack of shape ``(2 * time_bins, H, W)``.

    Channel layout: ``ON_0, ON_1, ..., ON_{T-1}, OFF_0, ..., OFF_{T-1}`` —
    i.e. polarity is the major axis. Counts accumulate per cell.
    """
    out = torch.zeros((2 * time_bins, H, W), dtype=torch.float32, device=events.device)
    if events.numel() == 0:
        return out
    t, x, y, p = _split_columns(events)
    mask = _in_bounds(x, y, H, W)
    t, x, y, p = t[mask], x[mask], y[mask], p[mask]
    if t.numel() == 0:
        return out
    bins = _time_bins_for(t, time_bins)
    pol_ch = torch.where(p > 0, torch.zeros_like(p), torch.ones_like(p))
    ch = pol_ch * time_bins + bins
    flat_idx = ch * (H * W) + y * W + x
    out.view(-1).index_add_(
        0, flat_idx, torch.ones(flat_idx.shape, dtype=torch.float32, device=out.device)
    )
    if normalize:
        m = out.max()
        if m > 0:
            out = out / m
    return out


def to_raw(
    events: torch.Tensor,
    *,
    max_events: int,
) -> tuple[torch.Tensor, int]:
    """Zero-padded raw event list of shape ``(max_events, 4)`` + length scalar.

    Preserves every supplied row or raises when the configured capacity is
    insufficient; pads with zeros otherwise. Length is the exact number of
    events represented.
    """
    if isinstance(max_events, bool) or not isinstance(max_events, int):
        raise ValueError(
            f"max_events must be a non-negative integer; got {max_events!r}"
        )
    if max_events < 0:
        raise ValueError(f"max_events must be >= 0; got {max_events}")
    n = int(events.shape[0])
    if n > max_events:
        raise EventRawOverflowError(rows=n, max_events=max_events)
    out = torch.zeros((max_events, 4), dtype=torch.int64, device=events.device)
    if n == 0 or max_events == 0:
        return out, 0
    out[:n] = events[:n].to(torch.int64)
    return out, n


# ---------- Dispatch table ----------------------------------------------
#
# Shape inference and tensor construction share this dispatch; keep the
# ``cfg.representation`` switch here.


class RepFns(NamedTuple):
    """Shape inference and conversion functions for one representation."""

    shape: Callable[["EventCameraCfg"], tuple[int, ...]]
    convert_one_env: Callable[[torch.Tensor, "EventCameraCfg"], dict[str, torch.Tensor]]


def _hw(cfg: "EventCameraCfg") -> tuple[int, int]:
    """Return ``(H, W)`` for the rep's spatial axes (cfg gives ``(W, H)``)."""
    w, h = cfg.effective_resolution
    return h, w


def _voxel_shape(cfg: "EventCameraCfg") -> tuple[int, int, int]:
    h, w = _hw(cfg)
    return (cfg.time_bins, h, w)


def _voxel_convert(
    events: torch.Tensor, cfg: "EventCameraCfg"
) -> dict[str, torch.Tensor]:
    h, w = _hw(cfg)
    return {
        "main": to_voxel_grid(
            events,
            H=h,
            W=w,
            time_bins=cfg.time_bins,
            normalize=cfg.normalize,
        )
    }


def _histogram_shape(cfg: "EventCameraCfg") -> tuple[int, int, int]:
    h, w = _hw(cfg)
    return (2, h, w)


def _histogram_convert(
    events: torch.Tensor, cfg: "EventCameraCfg"
) -> dict[str, torch.Tensor]:
    h, w = _hw(cfg)
    return {"main": to_histogram(events, H=h, W=w, normalize=cfg.normalize)}


def _time_surface_shape(cfg: "EventCameraCfg") -> tuple[int, int, int]:
    h, w = _hw(cfg)
    return (2, h, w)


def _time_surface_convert(
    events: torch.Tensor, cfg: "EventCameraCfg"
) -> dict[str, torch.Tensor]:
    h, w = _hw(cfg)
    return {"main": to_time_surface(events, H=h, W=w, tau_s=cfg.time_surface_tau_s)}


def _stacked_shape(cfg: "EventCameraCfg") -> tuple[int, int, int]:
    h, w = _hw(cfg)
    return (2 * cfg.time_bins, h, w)


def _stacked_convert(
    events: torch.Tensor, cfg: "EventCameraCfg"
) -> dict[str, torch.Tensor]:
    h, w = _hw(cfg)
    return {
        "main": to_stacked(
            events,
            H=h,
            W=w,
            time_bins=cfg.time_bins,
            normalize=cfg.normalize,
        )
    }


def _raw_shape(cfg: "EventCameraCfg") -> tuple[int, int]:
    return (cfg.raw_max_events, 4)


def _raw_convert(
    events: torch.Tensor, cfg: "EventCameraCfg"
) -> dict[str, torch.Tensor]:
    padded, n = to_raw(events, max_events=cfg.raw_max_events)
    return {
        "main": padded,
        "count": torch.tensor(n, dtype=torch.int64, device=padded.device),
    }


_REPRESENTATIONS: Final[dict[str, RepFns]] = {
    "voxel_grid": RepFns(shape=_voxel_shape, convert_one_env=_voxel_convert),
    "histogram": RepFns(shape=_histogram_shape, convert_one_env=_histogram_convert),
    "time_surface": RepFns(
        shape=_time_surface_shape, convert_one_env=_time_surface_convert
    ),
    "stacked": RepFns(shape=_stacked_shape, convert_one_env=_stacked_convert),
    "raw": RepFns(shape=_raw_shape, convert_one_env=_raw_convert),
}


def get_representation(name: str) -> RepFns:
    """Return the :class:`RepFns` entry for ``name``."""
    try:
        return _REPRESENTATIONS[name]
    except KeyError as e:
        raise KeyError(
            f"Unknown event representation {name!r}; "
            f"available: {sorted(_REPRESENTATIONS)}"
        ) from e


def list_representations() -> tuple[str, ...]:
    """Sorted tuple of registered representation names."""
    return tuple(sorted(_REPRESENTATIONS))
