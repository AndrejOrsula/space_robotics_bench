"""Write a single rock as a Mesh prim with a default gray PBR material."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

_BODY_TINTS = {
    "moon": (0.45, 0.42, 0.40),
    "mars": (0.55, 0.30, 0.22),
    "earth": (0.38, 0.32, 0.20),
}


def write_rock_usd(
    out_path: Path, mesh: trimesh.Trimesh, *, body: str = "moon"
) -> None:
    """Write ``out_path`` containing /Rock/Mesh + a default UsdPreviewSurface."""
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    UsdGeom.Xform.Define(stage, "/Rock")
    geom = UsdGeom.Mesh.Define(stage, "/Rock/Mesh")
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    geom.CreatePointsAttr([Gf.Vec3f(*v) for v in mesh.vertices])
    geom.CreateFaceVertexCountsAttr([3] * mesh.faces.shape[0])
    geom.CreateFaceVertexIndicesAttr(
        np.asarray(mesh.faces, dtype=np.int32).flatten().tolist()
    )
    geom.CreateExtentAttr([Gf.Vec3f(*mesh.bounds[0]), Gf.Vec3f(*mesh.bounds[1])])

    # Default body-tinted PBR material; baked albedo/roughness can be supplied later.
    tint = _BODY_TINTS.get(body, _BODY_TINTS["moon"])
    mat = UsdShade.Material.Define(stage, "/Rock/Material")
    surf = UsdShade.Shader.Define(stage, "/Rock/Material/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")

    UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)

    stage.GetRootLayer().Save()
