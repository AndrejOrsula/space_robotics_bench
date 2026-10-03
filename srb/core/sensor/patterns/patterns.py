from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from . import patterns_cfg


def multi_pattern(
    cfg: "patterns_cfg.MultiPatternBaseCfg", device: str
) -> tuple[torch.Tensor, torch.Tensor]:
    if not cfg.offsets_pos:
        cfg.offsets_pos = (None,) * len(cfg.patterns)
    elif len(cfg.offsets_pos) != len(cfg.patterns):
        raise ValueError("Offsets must match the number of patterns.")

    ray_starts = []
    ray_directions = []
    for pattern, pattern_offset in zip(
        cfg.patterns,
        cfg.offsets_pos,
        strict=True,
    ):
        pattern_starts, pattern_directions = pattern.func(pattern, device)
        if pattern_offset is not None and (
            pattern_offset[0] != 0.0
            or pattern_offset[1] != 0.0
            or pattern_offset[2] != 0.0
        ):
            pattern_starts += torch.tensor(
                pattern_offset, device=device, dtype=pattern_starts.dtype
            ).unsqueeze(0)
        ray_starts.append(pattern_starts)
        ray_directions.append(pattern_directions)
    # Preserve pattern order and duplicate rays: ray-casters reshape by index
    # into a fixed grid, so sorting or deduplicating breaks the cell mapping.
    return torch.cat(ray_starts), torch.cat(ray_directions)


def explicit_pattern(
    cfg: "patterns_cfg.ExplicitPatternBaseCfg", device: str
) -> tuple[torch.Tensor, torch.Tensor]:
    if not cfg.ray_starts:
        raise ValueError("ExplicitPatternBaseCfg.ray_starts must be non-empty")
    ray_starts = [
        torch.tensor(start, device=device, dtype=torch.float32)
        for start in cfg.ray_starts
    ]
    if isinstance(cfg.ray_directions[0], tuple):
        ray_directions = [
            torch.tensor(direction, device=device, dtype=torch.float32)
            for direction in cfg.ray_directions
        ]
    else:
        ray_directions = [
            torch.tensor(cfg.ray_directions, device=device, dtype=torch.float32)
        ] * len(cfg.ray_starts)

    return torch.stack(ray_starts), torch.stack(ray_directions)
