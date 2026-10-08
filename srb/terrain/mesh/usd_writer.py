"""Assemble the terrain USD: mesh + material binding + physics collision.

The mesh is written inline; the material lives in a sibling ``material.usda``
referenced via composition. Physics: `UsdPhysics.CollisionAPI` plus a
physics-purpose binding to a `UsdShade.Material` carrying
`UsdPhysics.MaterialAPI` with body-specific friction/restitution.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any, Literal, Optional

import numpy as np
import trimesh
from pxr import (
    Sdf,
    Usd,
    UsdGeom,
    UsdPhysics,
    UsdShade,
    Vt,
)

from srb.terrain.materials.physics import get_physics_material

Body = Literal["moon", "mars", "earth"]


def _georef_to_usd_dict(georef: dict[str, Any]) -> dict[str, Any]:
    """Convert a georef block to USD-customData-compatible value types.

    USD customData holds a ``VtDictionary``; list-of-float values (the two
    coordinate pairs) must become ``Vt.DoubleArray``, while ``str``/``float``
    values pass through unchanged.
    """
    out: dict[str, Any] = {}
    for key, value in georef.items():
        if isinstance(value, (list, tuple)):
            out[key] = Vt.DoubleArray([float(x) for x in value])
        else:
            out[key] = value
    return out


# USD customData accepts nested dictionaries, but not dictionaries inside lists
# or ``None`` nested in a dictionary. Store layer lists as ordinal-keyed dicts
# and omit optional ``None`` values. Readers recover list order by sorting keys
# numerically; lexicographic ordering would place "10" before "2".

_SCALAR_TYPES = (str, int, float, bool)


def _optional_scalar_dict(d: dict[str, Any]) -> dict[str, Any]:
    """Validate scalar leaves and omit ``None`` values for USD customData."""
    for k, v in d.items():
        if v is not None and not isinstance(v, _SCALAR_TYPES):
            raise TypeError(
                f"_optional_scalar_dict: key {k!r} has non-scalar value "
                f"{v!r} of type {type(v).__name__}; this converter only "
                "handles flat str/int/float/bool-or-None leaves -- a "
                "list/dict-valued leaf needs its own Vt.*Array / "
                "nested-dict handling (see _layers_to_usd_dict / "
                "_fusion_to_usd_dict)"
            )
    return {k: v for k, v in d.items() if v is not None}


def _layer_to_usd_dict(layer: dict[str, Any]) -> dict[str, Any]:
    """One layer/contributor record converted to USD-safe value types."""
    out = _optional_scalar_dict({k: v for k, v in layer.items() if k != "alignment"})
    if "alignment" in layer:
        out["alignment"] = _optional_scalar_dict(layer["alignment"])
    return out


def _layers_to_usd_dict(layers: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Encode layer records as a dict keyed by their ordinal strings."""
    return {str(i): _layer_to_usd_dict(layer) for i, layer in enumerate(layers)}


def _fusion_to_usd_dict(fusion: dict[str, Any]) -> dict[str, Any]:
    """Convert fusion metadata to USD-safe customData values.

    Uses the source dict directly and omits optional ``None`` fields."""
    grid = fusion["grid"]
    out: dict[str, Any] = {
        "fusion_version": fusion["fusion_version"],
        "grid": {
            "crs": grid["crs"],
            "center_xy_m": Vt.DoubleArray([float(x) for x in grid["center_xy_m"]]),
            "size_m": grid["size_m"],
            "gsd_m": grid["gsd_m"],
            "pixels": grid["pixels"],
        },
        "dem_layers": _layers_to_usd_dict(fusion["dem_layers"]),
        "color_layers": _layers_to_usd_dict(fusion["color_layers"]),
        "contributors": _layers_to_usd_dict(fusion.get("contributors", [])),
        "warnings": Vt.StringArray([str(w) for w in fusion["warnings"]]),
    }
    if fusion.get("provenance_tif") is not None:
        out["provenance_tif"] = fusion["provenance_tif"]
    if fusion.get("uncertainty_tif") is not None:
        out["uncertainty_tif"] = fusion["uncertainty_tif"]
    return out


def write_terrain_usd(
    out_path: Path,
    *,
    mesh: trimesh.Trimesh,
    material_usda_relpath: str,
    material_name: str,
    body: Body,
    georef: Optional[dict[str, Any]] = None,
    fusion: Optional[dict[str, Any]] = None,
) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root_xform = UsdGeom.Xform.Define(stage, "/Terrain")
    stage.SetDefaultPrim(root_xform.GetPrim())

    # Georeference provenance (cross-module contract: consumers read these
    # exact keys). Only present when the bake could resolve site coords + CRS.
    if georef is not None:
        root_xform.GetPrim().SetCustomDataByKey("georef", _georef_to_usd_dict(georef))

    # Fusion provenance: the frozen
    # `meta.json["fusion"]` schema, mirrored verbatim onto the prim. Only
    # present on a v2 (fused) bake -- `fusion=None` (every v1 bake, and any
    # caller that predates this parameter) writes no `fusion` customData key
    # at all, exactly like `georef` above.
    if fusion is not None:
        root_xform.GetPrim().SetCustomDataByKey("fusion", _fusion_to_usd_dict(fusion))

    # Reference the sibling material.usda so the Material prim is composed in.
    stage.DefinePrim("/Terrain/Looks")
    mat_prim = stage.DefinePrim(f"/Terrain/Looks/{material_name}")
    mat_prim.GetReferences().AddReference(material_usda_relpath, f"/{material_name}")

    geom = UsdGeom.Mesh.Define(stage, "/Terrain/geom")

    pts = Vt.Vec3fArray.FromNumpy(mesh.vertices.astype(np.float32))
    counts = Vt.IntArray([3] * len(mesh.faces))
    indices = Vt.IntArray(mesh.faces.flatten().astype(np.int32).tolist())

    # Triangles are final geometry; the USD fallback scheme is catmullClark.
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    geom.GetPointsAttr().Set(pts)
    geom.GetFaceVertexCountsAttr().Set(counts)
    geom.GetFaceVertexIndicesAttr().Set(indices)
    geom.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(pts))

    if mesh.visual is not None and mesh.visual.uv is not None:
        st = UsdGeom.PrimvarsAPI(geom).CreatePrimvar(
            "st",
            Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying,
        )
        # face-varying UVs: one UV per face vertex
        face_uvs = mesh.visual.uv[mesh.faces].reshape(-1, 2).astype(np.float32)
        st.Set(Vt.Vec2fArray.FromNumpy(face_uvs))

    # Material binding
    UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(UsdShade.Material(mat_prim))

    # Collision + physics material
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim())
    mesh_coll.CreateApproximationAttr().Set("meshSimplification")

    # The physics material must be a `UsdShade.Material` prim: a
    # physics-purpose binding only resolves (`ComputeBoundMaterial("physics")`)
    # to a Material, and a non-Material target silently leaves the collider on
    # the scene's default material instead of the body friction below.
    pm = get_physics_material(body)
    pmat_material = UsdShade.Material.Define(stage, "/Terrain/PhysicsMaterial")
    pmat_prim = pmat_material.GetPrim()
    pmat = UsdPhysics.MaterialAPI.Apply(pmat_prim)
    pmat.CreateStaticFrictionAttr().Set(pm.static_friction)
    pmat.CreateDynamicFrictionAttr().Set(pm.dynamic_friction)
    pmat.CreateRestitutionAttr().Set(pm.restitution)
    pmat.CreateDensityAttr().Set(pm.density_kg_m3)
    UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(
        pmat_material,
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics",
    )

    stage.GetRootLayer().Save()
