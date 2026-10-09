"""Isaac-free PyTorch geometry and reward helpers for excavation."""

from __future__ import annotations

import math

import torch

# Default forward-tilt angle for the scoop ideal attitude.
DEFAULT_SCOOP_TILT_RAD: float = math.radians(25.0)


def compute_scoop_capacity(
    half_extents_m: tuple[float, float, float],
    particles_size_m: float,
    packing_efficiency: float = 0.64,
) -> int:
    """Estimate capacity from scoop volume and packing efficiency; round down."""
    hx, hy, hz = half_extents_m
    scoop_volume = (2.0 * hx) * (2.0 * hy) * (2.0 * hz)
    particle_radius = particles_size_m / 2.0
    particle_volume = (4.0 / 3.0) * math.pi * (particle_radius**3)
    if particle_volume <= 0.0:
        return 0
    return int(scoop_volume * packing_efficiency / particle_volume)


def rot6d_target_forward_tilt(
    target_rad: float = DEFAULT_SCOOP_TILT_RAD,
    device: "torch.device | str | None" = None,
    dtype: "torch.dtype | None" = None,
) -> torch.Tensor:
    """Return the target rot6d for a forward rotation about the base-frame y-axis.

    The values follow ``rotmat_to_rot6d`` row-major ordering of the first two
    rotation-matrix columns.
    """
    c = math.cos(target_rad)
    s = math.sin(target_rad)
    return torch.tensor([c, 0.0, 0.0, 1.0, -s, 0.0], device=device, dtype=dtype)


def _matrix_from_quat_xyzw(quat: torch.Tensor) -> torch.Tensor:
    """Convert a batch of (x, y, z, w) quaternions to rotation matrices."""
    qx = quat[..., 0]
    qy = quat[..., 1]
    qz = quat[..., 2]
    qw = quat[..., 3]

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)
    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)
    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)

    row0 = torch.stack([r00, r01, r02], dim=-1)
    row1 = torch.stack([r10, r11, r12], dim=-1)
    row2 = torch.stack([r20, r21, r22], dim=-1)
    return torch.stack([row0, row1, row2], dim=-2)


def particle_mean_pos_relative_base(
    particle_pos_w: torch.Tensor,
    ee_pos_w: torch.Tensor,
    base_quat_w: torch.Tensor,
) -> torch.Tensor:
    """Return the particle centroid relative to the end effector in the base frame.

    Returns zeros with shape ``(N, 3)`` when there are no particles."""
    n_envs = base_quat_w.shape[0]
    device = base_quat_w.device
    dtype = base_quat_w.dtype

    if particle_pos_w.numel() == 0 or particle_pos_w.shape[1] == 0:
        return torch.zeros((n_envs, 3), device=device, dtype=dtype)

    p_mean_w = particle_pos_w.mean(dim=1)  # (N, 3)
    delta_w = p_mean_w - ee_pos_w  # (N, 3)

    # R_base->world from quaternion; to get world->base we use the transpose
    # (rotation matrices are orthonormal, so R^T = R^{-1}).
    R = _matrix_from_quat_xyzw(base_quat_w)  # (N, 3, 3)
    delta_b = torch.einsum("nij,nj->ni", R.transpose(-1, -2), delta_w)
    return delta_b


def mask_lift_reward(r_lift: torch.Tensor, valid_mask: torch.Tensor) -> torch.Tensor:
    """Zero rewards for environments with no valid heightmap cells."""
    has_valid = valid_mask.to(torch.bool).any(dim=-1).any(dim=-1)
    return torch.where(has_valid, r_lift, torch.zeros_like(r_lift))
