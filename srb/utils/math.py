import functools
import inspect as _inspect
import math
from math import degrees, radians  # noqa: F401
from typing import Sequence

import torch
import torch.jit
import torch.nn.functional as F
from isaaclab.utils import math as _isaac_math
from isaaclab.utils.math import *  # noqa: F403  # type: ignore[import-untyped]


def _to_torch(x):
    # Isaac Lab v3 ProxyArray supports arithmetic but fails TorchScript checks.
    return x.torch if hasattr(x, "torch") else x


def _proxy_safe(_func):
    """Wrap an Isaac Lab function to unwrap ProxyArray inputs."""

    @functools.wraps(_func)
    def _wrapped(*args, **kwargs):
        coerced_args = tuple(_to_torch(a) for a in args)
        coerced_kwargs = {k: _to_torch(v) for k, v in kwargs.items()}
        return _func(*coerced_args, **coerced_kwargs)

    return _wrapped


# Wrap upstream functions so ProxyArray inputs reach TorchScript as tensors.

for _name in dir(_isaac_math):
    if _name.startswith("_"):
        continue
    _obj = getattr(_isaac_math, _name)
    _is_jit = isinstance(_obj, torch.jit.ScriptFunction)
    _is_py_fn = (
        _inspect.isfunction(_obj) or _inspect.isbuiltin(_obj) or _inspect.ismethod(_obj)
    )
    if _is_jit or _is_py_fn:
        globals()[_name] = _proxy_safe(_obj)
del _name, _obj, _is_jit, _is_py_fn, _inspect

# Keep unwrapped aliases for callers compiled by TorchScript.
_isaac_matrix_from_quat = _isaac_math.matrix_from_quat
_isaac_combine_frame_transforms = _isaac_math.combine_frame_transforms
_isaac_subtract_frame_transforms = _isaac_math.subtract_frame_transforms
_isaac_transform_points = _isaac_math.transform_points


def rpy_to_quat(
    *rpy: float | Sequence[float], deg: bool = True
) -> tuple[float, float, float, float]:
    """
    Returns quaternion from roll-pitch-yaw angles.
    Accepts either separate values or a sequence of 3 numbers.

    Args:
        rpy: Either (roll, pitch, yaw) as separate floats or a sequence of 3 numbers
        deg: If True, input is in degrees, otherwise in radians

    Returns:
        tuple of (x, y, z, w) quaternion components

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
        return 0.0, 0.0, 0.0, 1.0

    if deg:
        roll, pitch, yaw = (radians(angle) for angle in (roll, pitch, yaw))  # type: ignore[misc]
    # Compute half angles
    r2, p2, y2 = roll / 2.0, pitch / 2.0, yaw / 2.0  # type: ignore[misc]
    # Precompute trig functions
    cr, cp, cy = math.cos(r2), math.cos(p2), math.cos(y2)
    sr, sp, sy = math.sin(r2), math.sin(p2), math.sin(y2)

    return (
        cy * sr * cp - sy * cr * sp,  # x
        cy * cr * sp + sy * sr * cp,  # y
        sy * cr * cp - cy * sr * sp,  # z
        cy * cr * cp + sy * sr * sp,  # w
    )


@torch.jit.script
def rotmat_to_rot6d(rotmat: torch.Tensor) -> torch.Tensor:
    return rotmat[..., :, :2].reshape(rotmat.shape[:-2] + (6,))


@torch.jit.script
def slerp(q1: torch.Tensor, q2: torch.Tensor, t: float) -> torch.Tensor:
    dot = torch.sum(q1 * q2, dim=-1)

    # If the dot product is negative, the quaternions have opposite handedness and
    # slerp won't take the shorter path. Fix by reversing one quaternion.
    q2_corrected = torch.where(dot.unsqueeze(-1) < 0, -q2, q2)
    dot_corrected = torch.where(dot < 0, -dot, dot)

    # Clamp to [0, 1] to avoid NaN from acos due to floating point precision
    dot_corrected = torch.clamp(dot_corrected, max=1.0)

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


def normalize_joint_pos(joint_pos: torch.Tensor, limits: torch.Tensor) -> torch.Tensor:
    """Scale joints with finite limits to [-1, 1]; pass unbounded joints through."""
    lower, upper = limits[..., 0], limits[..., 1]
    finite = torch.isfinite(lower) & torch.isfinite(upper)
    # Substitute unit limits for unbounded joints so the masked branch stays
    # finite; one branch-free path avoids a per-step host sync.
    scaled = scale_transform(  # noqa: F405
        joint_pos,
        torch.where(finite, lower, -1.0),
        torch.where(finite, upper, 1.0),
    )
    return torch.where(finite, scaled, joint_pos)


def combine_frame_transforms_tuple(
    t01: tuple[float, float, float] | Sequence[float],
    q01: tuple[float, float, float, float] | Sequence[float],
    t12: tuple[float, float, float] | Sequence[float] | None = None,
    q12: tuple[float, float, float, float] | Sequence[float] | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    _t01 = torch.tensor(t01)
    _q01 = torch.tensor(q01)
    _t12 = torch.tensor(t12) if t12 is not None else None
    _q12 = torch.tensor(q12) if q12 is not None else None
    t02, q02 = _isaac_combine_frame_transforms(_t01, _q01, _t12, _q12)
    return tuple(t02.cpu().tolist()), tuple(q02.cpu().tolist())


def subtract_frame_transforms_tuple(
    t01: tuple[float, float, float],
    q01: tuple[float, float, float, float],
    t02: tuple[float, float, float] | None = None,
    q02: tuple[float, float, float, float] | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    _t01 = torch.tensor(t01)
    _q01 = torch.tensor(q01)
    _t02 = torch.tensor(t02) if t02 is not None else None
    _q02 = torch.tensor(q02) if q02 is not None else None
    t12, q12 = _isaac_subtract_frame_transforms(_t01, _q01, _t02, _q02)
    return tuple(t12.cpu().tolist()), tuple(q12.cpu().tolist())
