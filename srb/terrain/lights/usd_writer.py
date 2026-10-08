"""Write a USD stage containing the DistantLight prims for a lighting preset."""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdLux

from srb.terrain.lights.schema import LightingPreset


def _orient_light(
    prim: UsdGeom.Xformable,
    *,
    elevation_deg: float,
    azimuth_deg: float,
    quat_xyzw: tuple[float, float, float, float] | None = None,
) -> None:
    """Set the prim's local-to-world rotation so its -Z axis (light direction)
    points at the surface from (elevation, azimuth).

    ``quat_xyzw``: optional precomputed orientation quaternion ``(x, y, z, w)``.
    When given, it is authored directly (the az/el math is skipped); when ``None``
    the legacy (+cos az, +sin az) horizontal convention runs byte-identically so
    static-preset output is unchanged.
    """
    if quat_xyzw is None:
        el = math.radians(elevation_deg)
        az = math.radians(azimuth_deg)
        # Surface direction (from sky toward ground) — we want -Z to align here.
        dx = math.cos(el) * math.cos(az)
        dy = math.cos(el) * math.sin(az)
        dz = -math.sin(el)
        target = Gf.Vec3d(dx, dy, dz).GetNormalized()
        z_axis_negated = -target  # the prim's local Z axis points opposite the light
        rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), z_axis_negated)
        xform_op = prim.AddOrientOp(opSuffix="lighting")
        xform_op.Set(Gf.Quatf(rot.GetQuat()))
        return

    # Precomputed orientation (xyzw) — USD quaternions are w-first.
    x, y, z, w = quat_xyzw
    xform_op = prim.AddOrientOp(opSuffix="lighting")
    xform_op.Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))


def write_lights_usd(
    out_path: Path,
    *,
    preset: LightingPreset,
    sun_orient_quat_xyzw: tuple[float, float, float, float] | None = None,
    earthshine_orient_quat_xyzw: tuple[float, float, float, float] | None = None,
) -> None:
    """Write ``out_path`` containing /Lights/Sun (+ optional /Lights/Earthshine).

    ``sun_orient_quat_xyzw`` / ``earthshine_orient_quat_xyzw``: optional
    precomputed ``(x, y, z, w)`` orientation quaternions. When ``None`` (default)
    the legacy az/el orientation is used and output is byte-identical to before.
    """
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    UsdGeom.Xform.Define(stage, "/Lights")

    sun_prim = UsdLux.DistantLight.Define(stage, "/Lights/Sun")
    sun_prim.GetIntensityAttr().Set(preset.sun_intensity_lux)
    sun_prim.GetColorTemperatureAttr().Set(preset.sun_color_temperature_k)
    sun_prim.GetEnableColorTemperatureAttr().Set(True)
    sun_prim.GetAngleAttr().Set(preset.sun_angular_diameter_deg)
    _orient_light(
        UsdGeom.Xformable(sun_prim.GetPrim()),
        elevation_deg=preset.sun_elevation_deg,
        azimuth_deg=preset.sun_azimuth_deg,
        quat_xyzw=sun_orient_quat_xyzw,
    )

    if preset.earthshine_intensity_lux:
        es_prim = UsdLux.DistantLight.Define(stage, "/Lights/Earthshine")
        es_prim.GetIntensityAttr().Set(preset.earthshine_intensity_lux)
        if preset.earthshine_color_temperature_k is not None:
            es_prim.GetColorTemperatureAttr().Set(preset.earthshine_color_temperature_k)
            es_prim.GetEnableColorTemperatureAttr().Set(True)
        es_prim.GetAngleAttr().Set(0.5)  # earth angular diameter from moon
        # Earthshine points opposite the sun (azimuth + 180°, low elevation).
        # `sun_elevation_deg` may be negative: clamp to [2, 90]
        # so a below-horizon sun doesn't push this fallback past the zenith.
        _orient_light(
            UsdGeom.Xformable(es_prim.GetPrim()),
            elevation_deg=max(2.0, min(90.0, 90.0 - preset.sun_elevation_deg)),
            azimuth_deg=(preset.sun_azimuth_deg + 180.0) % 360.0,
            quat_xyzw=earthshine_orient_quat_xyzw,
        )

    stage.GetRootLayer().Save()
