"""USD/PhysX spawn fixups for joints and massless end-effector links.

The wrappers are idempotent and defer ``pxr`` imports until spawn, keeping
this module importable without Isaac Sim.
"""

from __future__ import annotations

import math

_SPAWN_WRAPPER_MARKERS = (
    "__srb_joint_disable_wrapped__",
    "__srb_ee_mass_wrapped__",
)


def _preserve_spawn_wrapper_markers(wrapped, original) -> None:
    for marker in _SPAWN_WRAPPER_MARKERS:
        if getattr(original, marker, False):
            setattr(wrapped, marker, True)


def wrap_spawn_to_disable_joints(asset_cfg) -> None:
    """Wrap an asset's spawn function so all USD joints under the spawned
    root are disabled immediately after spawn (before PhysX validates them)."""
    spawn = getattr(asset_cfg, "spawn", None)
    if spawn is None or not hasattr(spawn, "func"):
        return
    original = spawn.func
    if getattr(original, "__srb_joint_disable_wrapped__", False):
        return

    def wrapped(prim_path, cfg, *args, **kwargs):
        from pxr import Usd, UsdPhysics

        prim = original(prim_path, cfg, *args, **kwargs)
        if prim is not None and prim.IsValid():
            for sub in Usd.PrimRange(prim):
                if sub.IsA(UsdPhysics.Joint):
                    joint = UsdPhysics.Joint(sub)
                    if joint:
                        joint.GetJointEnabledAttr().Set(False)
        return prim

    _preserve_spawn_wrapper_markers(wrapped, original)
    wrapped.__srb_joint_disable_wrapped__ = True
    spawn.func = wrapped


def wrap_spawn_to_fix_end_effector_mass(asset_cfg) -> None:
    """Wrap a robot's spawn function so ``end_effector_link`` (and similar
    massless wrist frames) receive a small positive mass + inertia before
    PhysX validates the articulation."""
    spawn = getattr(asset_cfg, "spawn", None)
    if spawn is None or not hasattr(spawn, "func"):
        return
    original = spawn.func
    if getattr(original, "__srb_ee_mass_wrapped__", False):
        return

    def wrapped(prim_path, cfg, *args, **kwargs):
        from pxr import Gf, Usd, UsdPhysics

        prim = original(prim_path, cfg, *args, **kwargs)
        if prim is None or not prim.IsValid():
            return prim
        targets = ("end_effector_link", "tool0", "tcp")
        for sub in Usd.PrimRange(prim):
            if sub.GetName() not in targets:
                continue
            mass_api = UsdPhysics.MassAPI.Apply(sub)
            mass_attr = mass_api.GetMassAttr()
            current = mass_attr.Get() if mass_attr.HasAuthoredValue() else None
            if current is None or not (current and current > 0.0):
                mass_attr.Set(1e-3)
            inertia_attr = mass_api.GetDiagonalInertiaAttr()
            authored = inertia_attr.Get() if inertia_attr.HasAuthoredValue() else None
            if authored is None or any(
                not math.isfinite(c) or c <= 0.0 for c in tuple(authored)
            ):
                inertia_attr.Set(Gf.Vec3f(1e-6, 1e-6, 1e-6))
        return prim

    _preserve_spawn_wrapper_markers(wrapped, original)
    wrapped.__srb_ee_mass_wrapped__ = True
    spawn.func = wrapped
