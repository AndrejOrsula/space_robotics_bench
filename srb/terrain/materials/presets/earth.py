from __future__ import annotations

from srb.terrain.materials.presets import MaterialPreset

EARTH_DEFAULT = MaterialPreset(
    name="earth_default",
    body="earth",
    base_rgb=(0.36, 0.28, 0.18),
    bedrock_rgb=(0.22, 0.20, 0.17),
    weathered_rgb=(0.30, 0.34, 0.16),
    shadow_rgb=(0.14, 0.13, 0.10),
    bedrock_slope_deg=28.0,
    albedo_grain_m=0.14,
    roughness=0.75,
    normal_strength=0.9,
    micro_normal_freq=16.0,
    smooth_px=1.2,
)
