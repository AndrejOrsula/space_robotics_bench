"""Tool-point frame helpers for task-space controllers (pure torch, no Isaac imports).

Controllers act on ``body ∘ body_offset``. The offset must match ``robot_ee``:
``frame_flange.offset ∘ tool.frame_tool_centre_point.offset`` (or the flange
offset alone). Jacobians and task frames use the articulation root frame; for
lever ``r = R(root^-1 * body) @ offset_pos``,
``J_v(point) = J_v(body) - skew(r) @ J_w(body)`` and angular rows are unchanged.
Controller velocity uses the body's link-frame velocity. Isaac Lab handles
Jacobian transport and velocity; SRB supplies the offset and root-body check.
Quaternions use ``(x, y, z, w)``.
"""

from __future__ import annotations

import math
from typing import Any, Callable, Sequence

import torch

Pose = tuple[tuple[float, float, float], tuple[float, float, float, float]]
ComposeFn = Callable[
    [Sequence[float], Sequence[float], Sequence[float], Sequence[float]], Pose
]

_IDENTITY_TOL = 1e-6


def quat_mul(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor:
    """Hamilton product ``q1 * q2`` of ``(..., 4)`` ``xyzw`` quaternions."""
    x1, y1, z1, w1 = q1.unbind(-1)
    x2, y2, z2, w2 = q2.unbind(-1)
    return torch.stack(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ),
        dim=-1,
    )


def quat_apply(quat: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """Rotate ``(..., 3)`` vectors by ``(..., 4)`` unit ``xyzw`` quaternions."""
    xyz = quat[..., :3]
    t = 2.0 * torch.cross(xyz, vec, dim=-1)
    return vec + quat[..., 3:] * t + torch.cross(xyz, t, dim=-1)


def resolve_jacobian_base_body(asset: Any, base_name: str) -> int | None:
    """Resolve ``base_name``; only the articulation root matches the task frame.

    Raises ``ValueError`` for no/ambiguous matches or a non-root body, avoiding
    silent mixing of the root-frame task state with another Jacobian frame.
    """
    if not base_name:
        return None
    base_ids, base_names = asset.find_bodies(base_name)
    if len(base_ids) != 1:
        raise ValueError(
            f"Expected one match for the base name: {base_name}. "
            f"Found {len(base_ids)}: {base_names}."
        )
    if base_ids[0] != 0:
        raise ValueError(
            f"base_name {base_name!r} resolves to body {base_names[0]!r}, not the "
            f"articulation root {asset.body_names[0]!r}: the task-space terms "
            "express the end-effector pose and velocity in the root frame, so the "
            "Jacobian must use it too. Set base_name to the root body or leave it "
            "empty."
        )
    return int(base_ids[0])


def compose_transforms(
    pos_a: Sequence[float],
    rot_a: Sequence[float],
    pos_b: Sequence[float],
    rot_b: Sequence[float],
) -> Pose:
    """Compose ``(position, xyzw quaternion)`` poses as ``a ∘ b``."""
    qa = torch.tensor(rot_a, dtype=torch.float64)
    pos = torch.tensor(pos_a, dtype=torch.float64) + quat_apply(
        qa, torch.tensor(pos_b, dtype=torch.float64)
    )
    rot = quat_mul(qa, torch.tensor(rot_b, dtype=torch.float64))
    return (
        tuple(float(v) for v in pos),  # type: ignore[return-value]
        tuple(float(v) for v in rot),
    )


def end_effector_frame_offset(
    manipulator: Any, combine: ComposeFn = compose_transforms
) -> Pose | None:
    """Return the ``robot_ee`` offset from the flange, or ``None`` without a flange.

    Compose the flange offset with the tool-centre-point offset. The mount
    offset places the tool body and is not part of the tracked TCP frame.
    """
    flange = getattr(manipulator, "frame_flange", None)
    if flange is None:
        return None
    flange_pos, flange_rot = tuple(flange.offset.pos), tuple(flange.offset.rot)
    tool = getattr(manipulator, "end_effector", None)
    if tool is None:
        return flange_pos, flange_rot
    tcp = tool.frame_tool_centre_point.offset
    return combine(flange_pos, flange_rot, tuple(tcp.pos), tuple(tcp.rot))


def _same_pose(
    a_pos: Sequence[float],
    a_rot: Sequence[float],
    b_pos: Sequence[float],
    b_rot: Sequence[float],
) -> bool:
    if any(abs(float(x) - float(y)) > _IDENTITY_TOL for x, y in zip(a_pos, b_pos)):
        return False
    dot = sum(float(x) * float(y) for x, y in zip(a_rot, b_rot))
    # q and -q are the same rotation.
    return math.isclose(abs(dot), 1.0, abs_tol=_IDENTITY_TOL)


def bind_controller_to_end_effector_frame(
    action_term: Any, manipulator: Any, combine: ComposeFn = compose_transforms
) -> None:
    """Point a diff-IK/OSC term's ``body_offset`` at the tracked ``robot_ee`` frame.

    The term must act on the flange body (``body_name`` equal to the
    ``frame_flange`` prim) so ``body ∘ body_offset`` is the ``tf_end_effector``
    target. A declared ``body_offset`` must be ``None`` or the identity: the
    flange and tool-centre-point offsets are composed here, so a non-identity
    declaration would either be applied twice or silently discarded. Binding
    is idempotent.

    Raises ``ValueError`` if ``body_name`` is not the flange or a declared
    ``body_offset`` conflicts with the composed target.
    """
    target = end_effector_frame_offset(manipulator, combine)
    if target is None:
        return
    target_pos, target_rot = target
    flange_body = manipulator.frame_flange.prim_relpath.rstrip("/").rsplit("/", 1)[-1]
    if action_term.body_name != flange_body:
        raise ValueError(
            f"{type(manipulator).__name__}: task-space action term acts on body "
            f"{action_term.body_name!r}, but the tracked end-effector frame "
            f"(tf_end_effector) is attached to the flange body {flange_body!r}"
            f"; set body_name to {flange_body!r}"
        )
    declared = action_term.body_offset
    if declared is not None:
        declared_pos, declared_rot = tuple(declared.pos), tuple(declared.rot)
        identity = _same_pose(
            declared_pos, declared_rot, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        )
        if not identity and not _same_pose(
            declared_pos, declared_rot, target_pos, target_rot
        ):
            raise ValueError(
                f"{type(manipulator).__name__}: task-space action term declares "
                f"body_offset pos={declared_pos} rot={declared_rot}; the controller "
                "frame is composed from frame_flange and the tool centre point, so "
                "declare the offset there instead"
            )
    action_term.body_offset = action_term.__class__.OffsetCfg(
        pos=tuple(target_pos), rot=tuple(target_rot)
    )
