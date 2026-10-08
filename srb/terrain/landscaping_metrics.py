"""Pure terrain-landscaping diagnostic metrics.

Only metric helpers live here. The task reward is the single
stationary quality term in :mod:`srb.terrain.landscaping_success`. Everything
in this module feeds named metrics, never reward.
"""

from __future__ import annotations

import torch

from srb.terrain.landscaping_heightmap import (
    DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY,
    LandscapingHeightmapGeometry,
    sample_global_map_at_heightmap_with_validity,
)


def global_mae_from_error(global_err: torch.Tensor) -> torch.Tensor:
    """Mean absolute target-current error per environment."""
    return global_err.abs().mean(dim=(-2, -1)).clamp(min=0.0)


def local_workzone_success_fraction(
    *,
    global_err: torch.Tensor,
    robot_pos_w: torch.Tensor,
    env_origins: torch.Tensor,
    robot_yaw: torch.Tensor,
    extent: float,
    resolution: int,
    tolerance_m: float,
    geometry: LandscapingHeightmapGeometry = DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY,
) -> torch.Tensor:
    """Fraction of heightmap cells whose absolute global error is within tolerance."""
    err_local, valid_mask = sample_global_map_at_heightmap_with_validity(
        global_map=global_err.abs(),
        robot_pos_w=robot_pos_w,
        env_origins=env_origins,
        robot_yaw=robot_yaw,
        geometry=geometry,
        extent=extent,
        resolution=resolution,
    )
    valid = valid_mask.float()
    valid_count = valid.sum(dim=(-2, -1))
    within = ((err_local <= tolerance_m) & valid_mask).float().sum(dim=(-2, -1))
    return torch.where(
        valid_count > 0,
        within / valid_count.clamp(min=1.0),
        torch.zeros_like(valid_count),
    )


def update_stuck_counter(
    *,
    stuck_counter: torch.Tensor,
    base_vel_b: torch.Tensor,
    prev_action: torch.Tensor,
    velocity_threshold: float,
    command_threshold: float,
    window_steps: int,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return updated stuck counter and tanh-style saturated stuck score."""
    if window_steps < 1:
        raise ValueError("window_steps must be at least 1")
    speed = base_vel_b[:, :2].norm(dim=-1)
    cmd_mag = prev_action.abs().max(dim=-1).values
    is_stuck = (speed < velocity_threshold) & (cmd_mag > command_threshold)
    new_counter = torch.where(
        is_stuck,
        stuck_counter + 1,
        torch.zeros_like(stuck_counter),
    )
    score = (new_counter.float() / window_steps).clamp(0.0, 1.0)
    return new_counter, score


__all__ = [
    "global_mae_from_error",
    "local_workzone_success_fraction",
    "update_stuck_counter",
]
