"""Write a USD stage with a single UsdLuxDomeLight referencing a texture file."""

from __future__ import annotations

from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdLux


def write_dome_usd(out_path: Path, *, texture_path: Path, intensity: float) -> None:
    """Create ``out_path`` referencing ``texture_path`` (relative)."""
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    UsdGeom.Xform.Define(stage, "/Sky")
    dome = UsdLux.DomeLight.Define(stage, "/Sky/Dome")
    dome.GetIntensityAttr().Set(intensity)

    rel_tex = texture_path.name
    dome.GetTextureFileAttr().Set(Sdf.AssetPath(rel_tex))
    # texture:format = "automatic" lets Hydra infer EXR/PNG/HDR
    dome.GetTextureFormatAttr().Set(UsdLux.Tokens.automatic)

    stage.GetRootLayer().Save()
