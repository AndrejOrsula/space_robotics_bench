"""Pure geometry, state, and reset helpers for the spacewalk task."""

import torch

__all__ = [
    "advance_waypoints",
    "compute_approach_alignment",
    "compute_nearest_handrail_distance",
    "compute_reset_pose_range",
    "compute_signed_progress",
    "legacy_handrail_specs_to_segments",
    "update_success_streak",
    "waypoint_path_to_handrail_segments",
]


def compute_nearest_handrail_distance(
    *,
    points: torch.Tensor,
    seg_starts: torch.Tensor,
    seg_ends: torch.Tensor,
    seg_radii: torch.Tensor,
    fallback: float,
    eps: float = 1.0e-8,
) -> torch.Tensor:
    """Minimum nonnegative surface distance to capsule rail segments.

    Args:
        points: ``(num_envs, 3)`` query positions in world frame.
        seg_starts: ``(num_envs, num_segments, 3)`` segment start points.
        seg_ends: ``(num_envs, num_segments, 3)`` segment end points.
        seg_radii: ``(num_segments,)`` rail radii.
        fallback: distance returned when ``num_segments == 0``.
        eps: guards the zero-length-segment division.

    Returns:
        ``(num_envs,)`` nearest rail-surface distance, ``>= 0``.
    """
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (num_envs, 3)")
    num_envs = points.shape[0]
    if (
        seg_starts.ndim != 3
        or seg_starts.shape[0] != num_envs
        or seg_starts.shape[2] != 3
    ):
        raise ValueError("seg_starts must have shape (num_envs, num_segments, 3)")
    if seg_ends.shape != seg_starts.shape:
        raise ValueError("seg_ends must match seg_starts shape")
    if eps <= 0.0:
        raise ValueError("eps must be positive")
    if not fallback >= 0.0:
        raise ValueError("fallback must be non-negative")

    num_segments = seg_starts.shape[1]
    if num_segments == 0:
        return torch.full(
            (num_envs,), float(fallback), dtype=points.dtype, device=points.device
        )
    if seg_radii.shape != (num_segments,):
        raise ValueError("seg_radii must have shape (num_segments,)")

    segment = seg_ends - seg_starts
    point = points[:, None, :]
    projection = torch.sum((point - seg_starts) * segment, dim=-1) / (
        torch.sum(segment * segment, dim=-1) + eps
    )
    projection = torch.clamp(projection, 0.0, 1.0)
    closest = seg_starts + projection[:, :, None] * segment
    distance = torch.norm(point - closest, dim=-1) - seg_radii[None, :]
    return torch.clamp(torch.min(distance, dim=1).values, min=0.0)


def advance_waypoints(
    *,
    waypoint_index: torch.Tensor,
    is_near_target: torch.Tensor,
    last_index: int,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Advance one waypoint when near the target and before the final index.

    Args:
        waypoint_index: ``(num_envs,)`` long tensor of current indices.
        is_near_target: ``(num_envs,)`` bool, robot within success radius.
        last_index: index of the final waypoint.

    Returns:
        ``(new_index, advanced_mask)`` — both ``(num_envs,)``; ``advanced_mask``
        is True for environments that advanced this step.
    """
    if waypoint_index.ndim != 1:
        raise ValueError("waypoint_index must have shape (num_envs,)")
    if is_near_target.shape != waypoint_index.shape:
        raise ValueError("is_near_target must match waypoint_index shape")

    advanced_mask = is_near_target & (waypoint_index < last_index)
    new_index = torch.where(advanced_mask, waypoint_index + 1, waypoint_index)
    return new_index, advanced_mask


def compute_signed_progress(
    *,
    previous_dist: torch.Tensor,
    current_dist: torch.Tensor,
    advanced_mask: torch.Tensor,
) -> torch.Tensor:
    """Signed distance reduction, excluding steps that advance the waypoint."""
    if previous_dist.shape != current_dist.shape:
        raise ValueError("previous_dist must match current_dist shape")
    if advanced_mask.shape != current_dist.shape:
        raise ValueError("advanced_mask must match current_dist shape")

    progress = previous_dist - current_dist
    return torch.where(advanced_mask, torch.zeros_like(progress), progress)


def update_success_streak(
    *,
    success_streak: torch.Tensor,
    in_success_state: torch.Tensor,
    max_steps: int,
) -> torch.Tensor:
    """Increment (capped) the consecutive-success counter, else reset to zero."""
    if success_streak.ndim != 1:
        raise ValueError("success_streak must have shape (num_envs,)")
    if in_success_state.shape != success_streak.shape:
        raise ValueError("in_success_state must match success_streak shape")

    cap = torch.full_like(success_streak, int(max_steps))
    incremented = torch.minimum(success_streak + 1, cap)
    return torch.where(in_success_state, incremented, torch.zeros_like(success_streak))


def compute_approach_alignment(
    *,
    position: torch.Tensor,
    goal: torch.Tensor,
    velocity: torch.Tensor,
    eps: float = 1.0e-8,
) -> torch.Tensor:
    """Cosine similarity of velocity and goal direction; zero at rest."""
    if position.ndim != 2:
        raise ValueError("position must have shape (num_envs, dim)")
    if goal.shape != position.shape or velocity.shape != position.shape:
        raise ValueError("goal and velocity must match position shape")
    if eps <= 0.0:
        raise ValueError("eps must be positive")

    direction = goal - position
    direction_unit = direction / (torch.norm(direction, dim=-1, keepdim=True) + eps)
    velocity_unit = velocity / (torch.norm(velocity, dim=-1, keepdim=True) + eps)
    return torch.sum(velocity_unit * direction_unit, dim=-1)


def compute_reset_pose_range(
    *,
    base_pose_range: dict,
    interior_origin: bool,
    first_waypoint: tuple[float, float, float] | None,
    spread: float = 0.5,
) -> dict:
    """Return reset ranges centered on the origin or first waypoint.

    Preserve orientation ranges and use base XYZ ranges when neither applies.
    """
    if spread <= 0.0:
        raise ValueError("spread must be positive")

    pose_range = dict(base_pose_range)
    if interior_origin:
        pose_range.update(
            {
                "x": (-spread, spread),
                "y": (-spread, spread),
                "z": (-spread, spread),
            }
        )
    elif first_waypoint is not None:
        pose_range.update(
            {
                "x": (first_waypoint[0] - spread, first_waypoint[0] + spread),
                "y": (first_waypoint[1] - spread, first_waypoint[1] + spread),
                "z": (first_waypoint[2] - spread, first_waypoint[2] + spread),
            }
        )
    return pose_range


def waypoint_path_to_handrail_segments(
    waypoint_path: tuple[tuple[float, float, float], ...],
    radius: float,
) -> tuple[tuple[tuple[float, float, float], tuple[float, float, float], float], ...]:
    """Convert consecutive waypoints into ``(start, end, radius)`` rails."""
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    return tuple(
        (waypoint_path[i], waypoint_path[i + 1], radius)
        for i in range(len(waypoint_path) - 1)
    )


def legacy_handrail_specs_to_segments(
    specs: tuple[tuple[tuple[float, float, float], float, float], ...],
) -> tuple[tuple[tuple[float, float, float], tuple[float, float, float], float], ...]:
    """Expand ``(center, length, radius)`` X-axis rails into endpoint segments."""
    return tuple(
        (
            (center[0] - 0.5 * length, center[1], center[2]),
            (center[0] + 0.5 * length, center[1], center[2]),
            radius,
        )
        for center, length, radius in specs
    )
