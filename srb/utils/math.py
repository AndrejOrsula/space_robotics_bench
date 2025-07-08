import math
from typing import Sequence, Tuple

import torch
import torch.nn.functional as F
from isaaclab.utils.math import *  # noqa: F403  # type: ignore
from isaaclab.utils.math import combine_frame_transforms, matrix_from_quat
from isaaclab.utils.math import subtract_frame_transforms as _subtract_frame_transforms
from isaaclab.utils.math import transform_points as _transform_points


@torch.jit.script
def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


@torch.jit.script
def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def rpy_to_quat(
    *rpy: float | Sequence[float], deg: bool = True
) -> Tuple[float, float, float, float]:
    """
    Returns wxyz quaternion from roll-pitch-yaw angles.
    Accepts either separate values or a sequence of 3 numbers.

    Args:
        rpy: Either (roll, pitch, yaw) as separate floats or a sequence of 3 numbers
        deg: If True, input is in degrees, otherwise in radians

    Returns:
        Tuple of (w, x, y, z) quaternion components

    Raises:
        ValueError: If input doesn't contain exactly 3 values
    """
    if len(rpy) == 3:
        roll, pitch, yaw = rpy
    elif len(rpy) == 1 and isinstance(rpy[0], Sequence) and len(rpy[0]) == 3:
        roll, pitch, yaw = rpy[0]
    else:
        raise ValueError(
            "Input must be either 3 separate values or a sequence of 3 values"
        )

    if roll == 0.0 and pitch == 0.0 and yaw == 0.0:
        return 1.0, 0.0, 0.0, 0.0

    if deg:
        roll, pitch, yaw = (deg_to_rad(angle) for angle in (roll, pitch, yaw))  # type: ignore

    # Compute half angles
    r2, p2, y2 = roll / 2.0, pitch / 2.0, yaw / 2.0  # type: ignore

    # Precompute trig functions
    cr, cp, cy = math.cos(r2), math.cos(p2), math.cos(y2)
    sr, sp, sy = math.sin(r2), math.sin(p2), math.sin(y2)

    return (
        cy * cr * cp + sy * sr * sp,  # w
        cy * sr * cp - sy * cr * sp,  # x
        cy * cr * sp + sy * sr * cp,  # y
        sy * cr * cp - cy * sr * sp,  # z
    )


@torch.jit.script
def rotmat_to_rot6d(rotmat: torch.Tensor) -> torch.Tensor:
    return rotmat[..., :, :2].reshape(rotmat.shape[:-2] + (6,))


@torch.jit.script
def quat_to_rot6d(quaternions: torch.Tensor) -> torch.Tensor:
    return rotmat_to_rot6d(matrix_from_quat(quaternions))


@torch.jit.script
def slerp(q1: torch.Tensor, q2: torch.Tensor, t: float) -> torch.Tensor:
    dot = torch.sum(q1 * q2, dim=-1)

    # If the dot product is negative, the quaternions have opposite handedness and
    # slerp won't take the shorter path. Fix by reversing one quaternion.
    q2_corrected = torch.where(dot.unsqueeze(-1) < 0, -q2, q2)
    dot_corrected = torch.where(dot < 0, -dot, dot)

    # If the inputs are too close for comfort, linearly interpolate
    # and normalize the result.
    close_mask = dot_corrected > 0.95

    # Normal slerp
    theta_0 = torch.acos(dot_corrected)  # angle between input vectors
    sin_theta_0 = torch.sin(theta_0)  # compute sine of angle
    theta = theta_0 * t  # angle between v0 and result
    sin_theta = torch.sin(theta)  # compute sine of new angle
    s0 = torch.cos(theta) - dot_corrected * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    # For very close quaternions, use linear interpolation
    s0 = torch.where(close_mask, 1.0 - t, s0)
    s1 = torch.where(close_mask, t, s1)

    res = (s0.unsqueeze(-1) * q1) + (s1.unsqueeze(-1) * q2_corrected)
    return F.normalize(res, p=2.0, dim=-1)


def combine_frame_transforms_tuple(
    t01: Tuple[float, float, float] | Sequence[float],
    q01: Tuple[float, float, float, float] | Sequence[float],
    t12: Tuple[float, float, float] | Sequence[float] | None = None,
    q12: Tuple[float, float, float, float] | Sequence[float] | None = None,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    _t01 = torch.tensor(t01)
    _q01 = torch.tensor(q01)
    _t12 = torch.tensor(t12) if t12 is not None else None
    _q12 = torch.tensor(q12) if q12 is not None else None
    t02, q02 = combine_frame_transforms(_t01, _q01, _t12, _q12)
    return tuple(t02.cpu().tolist()), tuple(q02.cpu().tolist())


@torch.jit.script
def subtract_frame_transforms(
    t01: torch.Tensor,
    q01: torch.Tensor,
    t02: torch.Tensor | None = None,
    q02: torch.Tensor | None = None,
) -> Tuple[torch.Tensor, torch.Tensor]:
    return _subtract_frame_transforms(t01, q01, t02, q02)


def subtract_frame_transforms_tuple(
    t01: Tuple[float, float, float],
    q01: Tuple[float, float, float, float],
    t02: Tuple[float, float, float] | None = None,
    q02: Tuple[float, float, float, float] | None = None,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    _t01 = torch.tensor(t01)
    _q01 = torch.tensor(q01)
    _t02 = torch.tensor(t02) if t02 is not None else None
    _q02 = torch.tensor(q02) if q02 is not None else None
    t12, q12 = subtract_frame_transforms(_t01, _q01, _t02, _q02)
    return tuple(t12.cpu().tolist()), tuple(q12.cpu().tolist())


@torch.jit.script
def transform_points(
    points: torch.Tensor,
    pos: torch.Tensor | None = None,
    quat: torch.Tensor | None = None,
) -> torch.Tensor:
    return _transform_points(points, pos, quat)
