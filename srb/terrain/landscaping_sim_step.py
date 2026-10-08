"""Isaac-free tensor helpers for landscaping simulation steps.

Tasks provide Isaac tensors; this module handles pose conversion and small
reductions. The task assembles the oracle reward snapshot, while actor
observations come from the external-heightmap seam.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from srb.terrain.landscaping_pose import (
    quat_xyzw_to_rpy_torch,
)


@dataclass(frozen=True, slots=True)
class LandscapingBasePose:
    """Planar pose and full RPY derived from the sim root state."""

    base_pos_w: torch.Tensor
    base_rpy: torch.Tensor
    yaw: torch.Tensor
    base_pose_world_optitrack: torch.Tensor


def landscaping_last_step_env_ids(last_step: torch.Tensor) -> torch.Tensor:
    """Return env ids whose next step reaches the finite horizon."""
    return last_step.nonzero(as_tuple=False).squeeze(-1)


def landscaping_filtered_contact_force_norm(
    force_matrix_w: torch.Tensor,
    *,
    normalizing_force_n: float,
) -> torch.Tensor:
    """Reduce a filtered contact-force matrix to a bounded per-env score."""
    # Isaac Lab 6 exposes sensor buffers as ProxyArray. Its logical ``shape``
    # omits the vec3 element width, while the cached torch view restores the
    # conventional ``(..., 3)`` tensor shape used below.
    force_matrix_w = getattr(force_matrix_w, "torch", force_matrix_w)
    if normalizing_force_n <= 0.0:
        raise ValueError("normalizing_force_n must be positive")
    if force_matrix_w.ndim < 2:
        raise ValueError("force_matrix_w must include env and vector dimensions")
    if force_matrix_w.shape[-1] != 3:
        raise ValueError("force_matrix_w final dimension must be XYZ")
    if force_matrix_w.numel() == 0:
        return torch.zeros(
            force_matrix_w.shape[0],
            dtype=force_matrix_w.dtype,
            device=force_matrix_w.device,
        )
    max_force = force_matrix_w.norm(dim=-1).flatten(start_dim=1).amax(dim=1)
    return (max_force / normalizing_force_n).clamp(0.0, 1.0)


def landscaping_base_pose_from_root(
    *,
    base_pos_w: torch.Tensor,
    base_quat_w: torch.Tensor,
) -> LandscapingBasePose:
    """Build deploy-equivalent planar pose from sim root pose tensors."""
    roll, pitch, yaw = quat_xyzw_to_rpy_torch(base_quat_w)
    base_rpy = torch.stack([roll, pitch, yaw], dim=-1)
    base_pose_world_optitrack = torch.cat(
        [base_pos_w, yaw.unsqueeze(-1)],
        dim=-1,
    )
    return LandscapingBasePose(
        base_pos_w=base_pos_w,
        base_rpy=base_rpy,
        yaw=yaw,
        base_pose_world_optitrack=base_pose_world_optitrack,
    )


def landscaping_robot_state_finite(
    *tensors: torch.Tensor,
) -> torch.Tensor:
    """Per-environment mask: every supplied robot-state tensor is finite."""
    if not tensors:
        raise ValueError("at least one robot-state tensor is required")
    mask = torch.isfinite(tensors[0]).all(dim=-1)
    for tensor in tensors[1:]:
        mask = mask & torch.isfinite(tensor).all(dim=-1)
    return mask


def landscaping_poison_invalid_rewards(
    reward_terms: dict[str, torch.Tensor],
    robot_finite: torch.Tensor,
) -> dict[str, torch.Tensor]:
    """Mark rewards NaN for environments with non-finite robot state.

    Particle-map rewards and held observations can otherwise hide a physics
    failure until stale-frame abort. The repository-wide non-finite scrub then
    terminates and resets affected environments. Return the original mapping
    when all states are finite to avoid allocating on the common path.
    """
    if bool(robot_finite.all()):
        return reward_terms
    return {
        key: torch.where(robot_finite, value, torch.full_like(value, torch.nan))
        for key, value in reward_terms.items()
    }


__all__ = [
    "LandscapingBasePose",
    "landscaping_base_pose_from_root",
    "landscaping_filtered_contact_force_norm",
    "landscaping_last_step_env_ids",
    "landscaping_poison_invalid_rewards",
    "landscaping_robot_state_finite",
]
