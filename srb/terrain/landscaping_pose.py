"""Isaac- and ROS-free pose conversions shared by sim and deployment.

Quaternions use XYZW order, matching Isaac Lab asset state and ROS TF.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import torch


def quat_xyzw_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return ZYX-Euler yaw from a quaternion in ``(x, y, z, w)`` order."""
    x = float(x)
    y = float(y)
    z = float(z)
    w = float(w)
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq < 1e-12:
        return 0.0
    return math.atan2(
        2.0 * (w * z + x * y),
        norm_sq - 2.0 * (y * y + z * z),
    )


def quat_xyzw_to_rpy_torch(
    quat: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Return ZYX-Euler roll, pitch, yaw from ``(..., 4)`` XYZW quaternions."""
    qx = quat[..., 0]
    qy = quat[..., 1]
    qz = quat[..., 2]
    qw = quat[..., 3]
    return _quat_components_to_rpy_torch(qx=qx, qy=qy, qz=qz, qw=qw)


def _quat_components_to_rpy_torch(
    *,
    qx: torch.Tensor,
    qy: torch.Tensor,
    qz: torch.Tensor,
    qw: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Convert named quaternion components to ZYX-Euler angles."""
    import torch

    norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
    safe_norm_sq = norm_sq.clamp_min(torch.finfo(norm_sq.dtype).eps)
    roll = torch.atan2(
        2.0 * (qw * qx + qy * qz),
        norm_sq - 2.0 * (qx * qx + qy * qy),
    )
    pitch = torch.asin((2.0 * (qw * qy - qz * qx) / safe_norm_sq).clamp(-1.0, 1.0))
    yaw = torch.atan2(
        2.0 * (qw * qz + qx * qy),
        norm_sq - 2.0 * (qy * qy + qz * qz),
    )
    return roll, pitch, yaw


def wrap_angle_pi_torch(angle: torch.Tensor) -> torch.Tensor:
    """Wrap angles to ``[-pi, pi)`` (mission-yaw convention)."""
    import torch

    two_pi = 2.0 * math.pi
    return torch.remainder(angle + math.pi, two_pi) - math.pi


def mission_pose_from_world_torch(
    *,
    base_pos_w: torch.Tensor,
    base_yaw: torch.Tensor,
    env_origins: torch.Tensor,
) -> torch.Tensor:
    """Mission-local planar pose ``(x, y, yaw)`` per environment.

    Simulation-side world-to-mission calibration: subtract each environment's
    origin and wrap yaw to ``[-pi, pi)``. ``base_pos_w`` may be ``(N, 2)`` or
    ``(N, 3)``; only the planar components are used.
    """
    import torch

    xy = base_pos_w[..., :2] - env_origins[..., :2].to(
        device=base_pos_w.device, dtype=base_pos_w.dtype
    )
    yaw = wrap_angle_pi_torch(base_yaw.to(base_pos_w.dtype))
    return torch.cat([xy, yaw.unsqueeze(-1)], dim=-1)


def quat_xyzw_to_rotmat_np(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a float32 3x3 rotation matrix from an XYZW quaternion."""
    x = float(x)
    y = float(y)
    z = float(z)
    w = float(w)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s
    return np.array(
        (
            1.0 - (yy + zz),
            xy - wz,
            xz + wy,
            xy + wz,
            1.0 - (xx + zz),
            yz - wx,
            xz - wy,
            yz + wx,
            1.0 - (xx + yy),
        ),
        dtype=np.float32,
    ).reshape((3, 3))


def quat_xyzw_to_rotmat_torch(quat: torch.Tensor) -> torch.Tensor:
    """Return ``(..., 3, 3)`` rotation matrices from XYZW quaternions."""
    import torch

    if quat.shape[-1] != 4:
        raise ValueError("quat must have shape (..., 4)")
    if not quat.is_floating_point():
        raise TypeError("quat must use a floating-point dtype")

    qx, qy, qz, qw = quat.unbind(dim=-1)
    norm_sq = (quat * quat).sum(dim=-1)
    scale = 2.0 / norm_sq.clamp_min(torch.finfo(quat.dtype).eps)
    xx = qx * qx * scale
    yy = qy * qy * scale
    zz = qz * qz * scale
    xy = qx * qy * scale
    xz = qx * qz * scale
    yz = qy * qz * scale
    wx = qw * qx * scale
    wy = qw * qy * scale
    wz = qw * qz * scale
    matrix = torch.stack(
        (
            1.0 - (yy + zz),
            xy - wz,
            xz + wy,
            xy + wz,
            1.0 - (xx + zz),
            yz - wx,
            xz - wy,
            yz + wx,
            1.0 - (xx + yy),
        ),
        dim=-1,
    ).reshape(*quat.shape[:-1], 3, 3)

    zero_norm = norm_sq < 1e-12
    identity = torch.eye(3, device=quat.device, dtype=quat.dtype).expand_as(matrix)
    return torch.where(zero_norm[..., None, None], identity, matrix)


def world_points_to_frame_torch(
    *,
    points_w: torch.Tensor,
    frame_pos_w: torch.Tensor,
    frame_quat_xyzw: torch.Tensor,
) -> torch.Tensor:
    """Transform batched world points into batched local frames.

    ``points_w`` has shape ``(N, ..., 3)``; frame pose tensors have shapes
    ``(N, 3)`` and ``(N, 4)``.
    """
    import torch

    if points_w.ndim < 3 or points_w.shape[0] != frame_pos_w.shape[0]:
        raise ValueError("points_w must have shape (num_envs, ..., 3)")
    if points_w.shape[-1] != 3:
        raise ValueError("points_w must end in xyz")
    if tuple(frame_pos_w.shape) != (points_w.shape[0], 3):
        raise ValueError("frame_pos_w must have shape (num_envs, 3)")
    if tuple(frame_quat_xyzw.shape) != (points_w.shape[0], 4):
        raise ValueError("frame_quat_xyzw must have shape (num_envs, 4)")

    pos = frame_pos_w.to(device=points_w.device, dtype=points_w.dtype)
    quat = frame_quat_xyzw.to(device=points_w.device, dtype=points_w.dtype)
    relative_w = points_w - pos.view(points_w.shape[0], *([1] * (points_w.ndim - 2)), 3)
    world_from_frame = quat_xyzw_to_rotmat_torch(quat)
    return torch.einsum("n...i,nij->n...j", relative_w, world_from_frame)


__all__ = [
    "mission_pose_from_world_torch",
    "quat_xyzw_to_rotmat_np",
    "quat_xyzw_to_rotmat_torch",
    "quat_xyzw_to_rpy_torch",
    "quat_xyzw_to_yaw",
    "world_points_to_frame_torch",
    "wrap_angle_pi_torch",
]
