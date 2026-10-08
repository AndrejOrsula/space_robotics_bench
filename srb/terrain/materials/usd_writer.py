"""Emit a UsdPreviewSurface material that references baked PNG textures.

The material is written as a standalone ``material.usda`` next to its
``textures/`` directory; ``mesh/usd_writer.py`` later references it from the
terrain stage via composition.
"""

from __future__ import annotations

from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from srb.terrain.materials.baker import BakedTextures


def write_material_usda(
    out_path: Path,
    *,
    textures: BakedTextures,
    material_name: str = "Regolith",
) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    mat = UsdShade.Material.Define(stage, f"/{material_name}")
    pbr = UsdShade.Shader.Define(stage, f"/{material_name}/PBR")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    pbr.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)

    primvar = UsdShade.Shader.Define(stage, f"/{material_name}/UVReader")
    primvar.CreateIdAttr("UsdPrimvarReader_float2")
    primvar.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    primvar_out = primvar.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def _texture(name: str, file: Path, color_space: str | None) -> UsdShade.Shader:
        s = UsdShade.Shader.Define(stage, f"/{material_name}/{name}")
        s.CreateIdAttr("UsdUVTexture")
        s.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            f"./{file.relative_to(out_path.parent).as_posix()}"
        )
        if color_space:
            s.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(color_space)
        s.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(primvar_out)
        return s

    albedo = _texture("Albedo", textures.albedo, "sRGB")
    roughness = _texture("Roughness", textures.roughness, "raw")
    normal = _texture("Normal", textures.normal, "raw")
    # UsdPreviewSurface expects tangent-space normals in [-1, 1]; the PNG
    # stores them as n * 0.5 + 0.5 in [0, 1]. Remap with scale 2 / bias -1 on
    # RGB (alpha untouched), as the UsdPreviewSurface spec prescribes for
    # 8-bit normal maps.
    normal.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(
        Gf.Vec4f(2.0, 2.0, 2.0, 1.0)
    )
    normal.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(
        Gf.Vec4f(-1.0, -1.0, -1.0, 0.0)
    )
    ao = _texture("AO", textures.ao, "raw")

    a_out = albedo.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    r_out = roughness.CreateOutput("r", Sdf.ValueTypeNames.Float)
    n_out = normal.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    o_out = ao.CreateOutput("r", Sdf.ValueTypeNames.Float)

    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(a_out)
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(r_out)
    pbr.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(n_out)
    pbr.CreateInput("occlusion", Sdf.ValueTypeNames.Float).ConnectToSource(o_out)

    mat.CreateSurfaceOutput().ConnectToSource(
        pbr.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    )

    stage.GetRootLayer().Save()
