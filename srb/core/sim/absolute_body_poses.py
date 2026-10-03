from __future__ import annotations

from typing import Any

__all__ = ["make_rigid_body_poses_absolute"]


def _is_pose_op(name: str) -> bool:
    # Pivots, matrices and suffixed ops cannot be rebuilt from a world pose.
    return name in ("xformOp:translate", "xformOp:orient", "xformOp:scale") or (
        name.startswith("xformOp:rotate") and ":" not in name.removeprefix("xformOp:")
    )


def make_rigid_body_poses_absolute(stage: Any, root_path: str = "/World") -> int:
    """Author world-space poses on dynamic bodies whose parents are transformed.

    Returns the number of bodies whose transform stack was reset.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0

    op = UsdGeom.XformOp
    vec3 = {
        op.PrecisionDouble: Gf.Vec3d,
        op.PrecisionFloat: Gf.Vec3f,
        op.PrecisionHalf: Gf.Vec3h,
    }
    quat = {
        op.PrecisionDouble: Gf.Quatd,
        op.PrecisionFloat: Gf.Quatf,
        op.PrecisionHalf: Gf.Quath,
    }

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    identity = Gf.Matrix4d(1.0)
    updated = 0
    for prim in Usd.PrimRange(root):
        if prim.IsInstanceProxy() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        # PhysX writes back only simulated dynamic bodies; USD drives the rest.
        body = UsdPhysics.RigidBodyAPI(prim)
        if body.GetKinematicEnabledAttr().Get() or (
            body.GetRigidBodyEnabledAttr().Get() is False
        ):
            continue
        xformable = UsdGeom.Xformable(prim)
        if xformable.GetResetXformStack():
            continue
        if Gf.IsClose(xform_cache.GetParentToWorldTransform(prim), identity, 1e-9):
            continue
        if not all(_is_pose_op(o.GetOpName()) for o in xformable.GetOrderedXformOps()):
            continue

        # World poses are unchanged, so cached parent transforms stay valid.
        world_matrix = xform_cache.GetLocalToWorldTransform(prim)
        world = Gf.Transform(world_matrix)
        # Translate/orient/scale cannot express shear (rotated non-uniform
        # parent scale); leave such bodies under their parents.
        trs = Gf.Transform()
        trs.SetScale(world.GetScale())
        trs.SetRotation(world.GetRotation())
        trs.SetTranslation(world.GetTranslation())
        if not Gf.IsClose(trs.GetMatrix(), world_matrix, 1e-6):
            continue
        xformable.ClearXformOpOrder()
        for op_type, name, value, cast in (
            (
                op.TypeTranslate,
                "xformOp:translate",
                world.GetTranslation(),
                vec3,
            ),
            (
                op.TypeOrient,
                "xformOp:orient",
                world.GetRotation().GetQuat(),
                quat,
            ),
            (op.TypeScale, "xformOp:scale", world.GetScale(), vec3),
        ):
            # Reuse an authored attribute's precision; a mismatch cannot be added.
            attr = prim.GetAttribute(name)
            op_precision = op(attr).GetPrecision() if attr else op.PrecisionDouble
            xformable.AddXformOp(op_type, op_precision).Set(cast[op_precision](value))
        xformable.SetResetXformStack(True)
        updated += 1
    return updated
