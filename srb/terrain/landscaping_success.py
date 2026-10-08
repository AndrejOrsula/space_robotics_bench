"""Terrain quality and sustained-success evaluation, without Isaac imports.

Quality is ``1 / (1 + mae_m / success_tolerance_m)`` over manifest grading
cells. Oracle and observable maps use separate evaluator instances.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import torch


def quality_from_mae(
    mae_m: torch.Tensor, *, success_tolerance_m: float
) -> torch.Tensor:
    """Exact stationary quality reward, finite and in ``(0, 1]``."""
    if not math.isfinite(success_tolerance_m) or success_tolerance_m <= 0.0:
        raise ValueError("success_tolerance_m must be positive and finite")
    return 1.0 / (1.0 + mae_m / success_tolerance_m)


@dataclass(frozen=True, slots=True)
class StateQualityParams:
    """Saved success parameters plus provenance of the graded contract."""

    success_tolerance_m: float
    fraction_within_tolerance: float
    sustained_frames: int
    manifest_sha256: str = ""
    io_schema_fingerprint: str = ""

    def __post_init__(self) -> None:
        if not math.isfinite(self.success_tolerance_m) or self.success_tolerance_m <= 0:
            raise ValueError("success_tolerance_m must be positive and finite")
        if not 0.0 < self.fraction_within_tolerance <= 1.0:
            raise ValueError("fraction_within_tolerance must be in (0, 1]")
        if self.sustained_frames < 1:
            raise ValueError("sustained_frames must be >= 1")


@dataclass(frozen=True, slots=True)
class StateQualityResult:
    """Per-env outputs of one :class:`LandscapingStateQuality` evaluation."""

    grading_mae_m: torch.Tensor
    fraction_within_tolerance: torch.Tensor
    quality: torch.Tensor
    instantaneous_success: torch.Tensor
    local_mae_m: torch.Tensor
    manifest_sha256: str
    io_schema_fingerprint: str
    #: Above-target grading volume in m^3; ``None`` without ``cell_size_m``.
    excess_volume_m3: torch.Tensor | None = None


class LandscapingStateQuality:
    """Stateless evaluator over fixed manifest grading cells."""

    def __init__(
        self,
        *,
        params: StateQualityParams,
        map_shape: tuple[int, int],
        grading_indices: torch.Tensor | None = None,
        cell_centres_xy: torch.Tensor | None = None,
        local_radius_m: float = 1.0,
        cell_size_m: float | None = None,
        device: torch.device | str = "cpu",
    ) -> None:
        """Build a fixed-domain evaluator.

        Args:
            params: Success thresholds and graded-contract provenance.
            map_shape: Shape ``(H, W)`` of each evaluated map.
            grading_indices: ``(K, 2)`` integer ``(row, col)`` cells to grade;
                ``None`` grades every cell of the map.
            cell_centres_xy: Mission-frame ``(K, 2)`` centres for local MAE;
                ``None`` makes local MAE equal grading MAE.
            local_radius_m: Local-MAE neighbourhood radius.
            cell_size_m: Grading-cell edge length; enables ``excess_volume_m3``.
            device: Device for cached index tensors.
        """
        self.params = params
        if cell_size_m is not None and cell_size_m <= 0.0:
            raise ValueError("cell_size_m must be positive")
        self._cell_area_m2 = None if cell_size_m is None else float(cell_size_m) ** 2
        self._device = torch.device(device)
        h, w = int(map_shape[0]), int(map_shape[1])
        if h < 1 or w < 1:
            raise ValueError("map_shape must be positive")
        self._map_shape = (h, w)
        if local_radius_m <= 0.0:
            raise ValueError("local_radius_m must be positive")
        self._local_radius_m = float(local_radius_m)

        if grading_indices is None:
            rows = torch.arange(h, device=self._device).repeat_interleave(w)
            cols = torch.arange(w, device=self._device).repeat(h)
        else:
            idx = torch.as_tensor(grading_indices, device=self._device).long()
            if idx.ndim != 2 or idx.shape[-1] != 2 or idx.shape[0] < 1:
                raise ValueError("grading_indices must be a non-empty (K, 2) array")
            rows, cols = idx[:, 0], idx[:, 1]
            if rows.min() < 0 or rows.max() >= h or cols.min() < 0 or cols.max() >= w:
                raise ValueError("grading_indices outside the map shape")
        self._flat_indices = rows * w + cols

        if cell_centres_xy is not None:
            centres = torch.as_tensor(
                cell_centres_xy, device=self._device, dtype=torch.float32
            )
            if centres.shape != (int(self._flat_indices.shape[0]), 2):
                raise ValueError("cell_centres_xy must be (K, 2) matching grading")
            self._cell_centres_xy: torch.Tensor | None = centres
        else:
            self._cell_centres_xy = None

    @property
    def grading_cell_count(self) -> int:
        return int(self._flat_indices.shape[0])

    def evaluate(
        self,
        current: torch.Tensor,
        desired: torch.Tensor,
        *,
        rover_xy: torch.Tensor | None = None,
    ) -> StateQualityResult:
        """Compare ``(n, H, W)`` current maps with static or batched targets.

        Optional mission-frame ``rover_xy`` has shape ``(n, 2)`` and selects
        local MAE cells.
        """
        h, w = self._map_shape
        if current.ndim != 3 or current.shape[-2:] != (h, w):
            raise ValueError(f"current must be (n, {h}, {w}), got {current.shape}")
        if desired.ndim == 2:
            desired = desired.unsqueeze(0).expand_as(current)
        if desired.shape != current.shape:
            raise ValueError("desired shape must match current")

        n = current.shape[0]
        cur = current.reshape(n, h * w)[:, self._flat_indices]
        des = desired.reshape(n, h * w)[:, self._flat_indices]
        abs_err = (cur - des).abs()

        mae = abs_err.mean(dim=-1)
        excess_volume = (
            None
            if self._cell_area_m2 is None
            else (cur - des).clamp(min=0.0).sum(dim=-1) * self._cell_area_m2
        )
        tol = self.params.success_tolerance_m
        fraction = (abs_err <= tol).float().mean(dim=-1)
        quality = quality_from_mae(mae, success_tolerance_m=tol)
        instantaneous = fraction >= self.params.fraction_within_tolerance

        if rover_xy is not None and self._cell_centres_xy is not None:
            dist = torch.cdist(
                rover_xy.to(self._cell_centres_xy).unsqueeze(1),
                self._cell_centres_xy.unsqueeze(0).expand(n, -1, -1),
            ).squeeze(1)
            local = (dist <= self._local_radius_m).float()
            count = local.sum(dim=-1)
            local_sum = (abs_err * local).sum(dim=-1)
            local_mae = torch.where(count > 0, local_sum / count.clamp(min=1.0), mae)
        else:
            local_mae = mae

        return StateQualityResult(
            grading_mae_m=mae,
            fraction_within_tolerance=fraction,
            quality=quality,
            instantaneous_success=instantaneous,
            local_mae_m=local_mae,
            manifest_sha256=self.params.manifest_sha256,
            io_schema_fingerprint=self.params.io_schema_fingerprint,
            excess_volume_m3=excess_volume,
        )


def sustained_success_window(
    ok_frac: torch.Tensor,
    history: torch.Tensor,
    history_idx: torch.Tensor,
    window_steps: int,
    thresh: float,
) -> torch.Tensor:
    """Update the per-env success ring buffer in place and return its status."""
    if window_steps < 1:
        raise ValueError("window_steps must be >= 1")

    n_envs = history.shape[0]
    device = history.device
    idx = history_idx % window_steps
    history[torch.arange(n_envs, device=device), idx] = (ok_frac >= thresh).float()
    history_idx += 1
    has_enough = history_idx >= window_steps
    return history.all(dim=-1) & has_enough


class SustainedSuccessTracker:
    """Track success windows; repeated source sequences do not advance them."""

    _NO_SEQUENCE = -1

    def __init__(
        self,
        *,
        num_envs: int,
        window_steps: int,
        device: torch.device | str,
    ) -> None:
        if window_steps < 1:
            raise ValueError("window_steps must be >= 1")
        self.window_steps = int(window_steps)
        self.history = torch.zeros(
            (int(num_envs), self.window_steps),
            device=device,
            dtype=torch.float32,
        )
        self.history_idx = torch.zeros(int(num_envs), device=device, dtype=torch.long)
        self.last_sequence = torch.full(
            (int(num_envs),), self._NO_SEQUENCE, device=device, dtype=torch.long
        )

    def reset(self, env_ids: Sequence[int] | torch.Tensor | None = None) -> None:
        """Clear sustained-success history for all or selected environments."""
        if env_ids is None:
            self.history.zero_()
            self.history_idx.zero_()
            self.last_sequence.fill_(self._NO_SEQUENCE)
            return
        ids = torch.as_tensor(env_ids, device=self.history.device, dtype=torch.long)
        if ids.numel() == 0:
            return
        self.history[ids] = 0.0
        self.history_idx[ids] = 0
        self.last_sequence[ids] = self._NO_SEQUENCE

    def update(
        self,
        ok_frac: torch.Tensor,
        *,
        threshold: float,
        sequence: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Record success and return status; unchanged sequences keep the window.

        ``sequence`` contains each env's accepted-frame sequence number.
        """
        if sequence is None:
            return sustained_success_window(
                ok_frac=ok_frac,
                history=self.history,
                history_idx=self.history_idx,
                window_steps=self.window_steps,
                thresh=threshold,
            )

        seq = torch.as_tensor(
            sequence, device=self.history.device, dtype=torch.long
        ).reshape(-1)
        if seq.shape != self.history_idx.shape:
            raise ValueError("sequence must have one entry per environment")
        advance = seq != self.last_sequence
        n_envs = self.history.shape[0]
        device = self.history.device
        idx = self.history_idx % self.window_steps
        rows = torch.arange(n_envs, device=device)[advance]
        self.history[rows, idx[advance]] = (ok_frac[advance] >= threshold).float()
        self.history_idx += advance.long()
        self.last_sequence = torch.where(advance, seq, self.last_sequence)
        has_enough = self.history_idx >= self.window_steps
        return self.history.all(dim=-1) & has_enough


__all__ = [
    "LandscapingStateQuality",
    "StateQualityParams",
    "StateQualityResult",
    "SustainedSuccessTracker",
    "quality_from_mae",
    "sustained_success_window",
]
