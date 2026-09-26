"""Seeded, batched Poisson-disk placement on torch.

Candidates use torch's generator on the target device, so a fixed environment
seed reproduces layouts. Each object takes the first of a fixed number of
uniform candidates at least ``radius`` from prior placements. If none qualify,
it takes the candidate farthest from its nearest neighbour to fill the layout.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

__all__ = ["sample_poisson_disk"]


def sample_poisson_disk(
    shape: tuple[int, int],
    bounds: tuple[Sequence[float], Sequence[float]],
    radius: float,
    *,
    device: torch.device | str = "cpu",
    num_candidates: int = 64,
    occupied: torch.Tensor | None = None,
) -> torch.Tensor:
    """Return seeded ``(num_envs, num_objects, dim)`` positions within bounds."""
    num_envs, num_objects = (int(shape[0]), int(shape[1]))
    if num_envs < 0 or num_objects < 0:
        raise ValueError(f"shape must be non-negative, got {shape!r}")
    if num_candidates < 1:
        raise ValueError(f"num_candidates must be positive, got {num_candidates}")
    # NaN would mark every candidate occupied; negative would mark all free.
    if not math.isfinite(radius) or radius < 0.0:
        raise ValueError(f"radius must be finite and non-negative, got {radius!r}")
    low = torch.as_tensor(bounds[0], dtype=torch.float32, device=device)
    high = torch.as_tensor(bounds[1], dtype=torch.float32, device=device)
    if low.ndim != 1 or low.shape != high.shape:
        raise ValueError(f"bounds must be two equal-length vectors, got {bounds!r}")
    dim = low.shape[0]
    extent = high - low
    if (
        not torch.isfinite(low).all()
        or not torch.isfinite(high).all()
        or not torch.isfinite(extent).all()
        or (extent < 0).any()
    ):
        raise ValueError("bounds must be finite and ordered componentwise")

    placed = torch.empty(
        (num_envs, num_objects, dim), dtype=torch.float32, device=device
    )
    if num_envs == 0 or num_objects == 0:
        return placed

    fixed = (
        torch.empty((num_envs, 0, dim), dtype=torch.float32, device=device)
        if occupied is None
        else torch.as_tensor(occupied, dtype=torch.float32, device=device)
    )
    if fixed.shape[0] != num_envs or fixed.ndim != 3 or fixed.shape[2] != dim:
        raise ValueError(
            f"occupied must have shape ({num_envs}, k, {dim}), got {tuple(fixed.shape)}"
        )

    env_index = torch.arange(num_envs, device=device)
    for i in range(num_objects):
        candidates = low + extent * torch.rand(
            (num_envs, num_candidates, dim), dtype=torch.float32, device=device
        )
        others = torch.cat((fixed, placed[:, :i]), dim=1)
        if others.shape[1] == 0:
            placed[:, 0] = candidates[:, 0]
            continue
        offsets = candidates.unsqueeze(2) - others.unsqueeze(1)
        nearest = offsets.square().sum(dim=-1).sqrt().amin(dim=-1)
        free = nearest >= radius
        first_free = free.to(torch.int8).argmax(dim=-1)
        farthest = nearest.argmax(dim=-1)
        choice = torch.where(free.any(dim=-1), first_free, farthest)
        placed[:, i] = candidates[env_index, choice]
    return placed
