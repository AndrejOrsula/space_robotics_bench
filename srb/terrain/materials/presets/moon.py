from __future__ import annotations

from srb.terrain.materials.presets import MaterialPreset

LUNAR_MARE = MaterialPreset(
    name="lunar_mare",
    body="moon",
    base_rgb=(0.13, 0.13, 0.14),
    bedrock_rgb=(0.08, 0.08, 0.08),
    weathered_rgb=(0.20, 0.20, 0.21),
    shadow_rgb=(0.06, 0.06, 0.07),
    bedrock_slope_deg=25.0,
    albedo_grain_m=0.20,
    roughness=0.93,
    normal_strength=1.0,
    micro_normal_freq=20.0,
    smooth_px=1.5,
)

LUNAR_HIGHLAND = MaterialPreset(
    name="lunar_highland",
    body="moon",
    base_rgb=(0.34, 0.33, 0.31),
    bedrock_rgb=(0.20, 0.19, 0.18),
    weathered_rgb=(0.45, 0.43, 0.40),
    shadow_rgb=(0.18, 0.18, 0.17),
    bedrock_slope_deg=22.0,
    albedo_grain_m=0.18,
    roughness=0.92,
    normal_strength=1.1,
    micro_normal_freq=22.0,
    smooth_px=1.5,
)

MOON_DEFAULT = LUNAR_MARE
