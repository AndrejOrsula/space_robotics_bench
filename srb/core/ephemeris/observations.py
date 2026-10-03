"""Ephemeris observation term.

Pure tensor math over the live ephemeris state (``_ephemeris_state`` /
``_ephemeris_occluder`` / ``_ephemeris_eclipse_factor`` written by
``srb.core.mdp.events._apply_sun_state``). Torch only -- NO isaaclab / omni /
pxr imports (this module must stay importable under the Isaac-free
unit-test stubs), and NO SPICE / USD work (it runs in the per-step
observation path).

Quaternions are xyzw (w-last), SRB's IsaacLab-fork convention. Rotating a
WORLD-frame vector into the robot BASE frame is rotation by the conjugate of
the base's world quaternion, written out explicitly below (the fork's
``isaaclab.utils.math.quat_apply_inverse`` is not importable Isaac-free).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from srb.core.ephemeris.state import BodyState, EphemerisState


def _rotate_world_to_base(q_xyzw: torch.Tensor, v_world: torch.Tensor) -> torch.Tensor:
    """Rotate world-frame vectors ``v_world`` (N, 3) into the base frame of
    ``q_xyzw`` (N, 4), the base->world orientation quaternion in xyzw order.

    Uses the conjugate quaternion (negated vector part) with the standard
    expansion ``v' = v + 2 * (w * (u x v) + u x (u x v))`` where ``u`` is the
    (conjugated) vector part and ``w`` the scalar part.
    """
    u = -q_xyzw[..., :3]
    w = q_xyzw[..., 3:4]
    uv = torch.linalg.cross(u, v_world, dim=-1)
    return v_world + 2.0 * (w * uv + torch.linalg.cross(u, uv, dim=-1))


def ephemeris_observation(
    state: "EphemerisState | None",
    occluder: "BodyState | None",
    factor: float,
    root_quat_w_xyzw: torch.Tensor,
) -> torch.Tensor:
    """Return base-frame sun and occluder directions plus eclipse factor.

    Output shape is (N, 7); missing occluder or state yields a zero direction.
    Dtype and device follow the input quaternion.
    """
    q = root_quat_w_xyzw
    if q.ndim != 2 or q.shape[-1] != 4:
        raise ValueError(f"root_quat_w_xyzw must be (N, 4) xyzw, got {tuple(q.shape)}")
    n = q.shape[0]
    kw = {"dtype": q.dtype, "device": q.device}

    if state is not None:
        sun_world = torch.tensor(state.bodies["sun"].direction_world, **kw).expand(n, 3)
        sun_base = _rotate_world_to_base(q, sun_world)
    else:
        sun_base = torch.zeros(n, 3, **kw)

    if occluder is not None:
        occ_world = torch.tensor(occluder.direction_world, **kw).expand(n, 3)
        occ_base = _rotate_world_to_base(q, occ_world)
    else:
        occ_base = torch.zeros(n, 3, **kw)

    fac = torch.full((n, 1), float(factor), **kw)
    return torch.cat((sun_base, occ_base, fac), dim=-1)
