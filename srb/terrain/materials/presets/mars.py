from __future__ import annotations

from srb.terrain.materials.presets import MaterialPreset

MARS_BASALT = MaterialPreset(
    name="mars_basalt",
    body="mars",
    base_rgb=(0.32, 0.18, 0.10),
    bedrock_rgb=(0.20, 0.10, 0.05),
    weathered_rgb=(0.45, 0.28, 0.16),
    shadow_rgb=(0.18, 0.10, 0.06),
    bedrock_slope_deg=24.0,
    albedo_grain_m=0.16,
    roughness=0.86,
    normal_strength=1.0,
    micro_normal_freq=20.0,
    smooth_px=1.5,
)

MARS_DUST = MaterialPreset(
    name="mars_dust",
    body="mars",
    base_rgb=(0.55, 0.36, 0.22),
    bedrock_rgb=(0.30, 0.18, 0.10),
    weathered_rgb=(0.65, 0.45, 0.28),
    shadow_rgb=(0.32, 0.20, 0.12),
    bedrock_slope_deg=20.0,
    albedo_grain_m=0.22,
    roughness=0.92,
    normal_strength=0.85,
    micro_normal_freq=18.0,
    smooth_px=2.0,
)

MARS_CLAY = MaterialPreset(
    name="mars_clay",
    body="mars",
    base_rgb=(0.42, 0.30, 0.20),
    bedrock_rgb=(0.24, 0.16, 0.10),
    weathered_rgb=(0.52, 0.38, 0.26),
    shadow_rgb=(0.20, 0.14, 0.10),
    bedrock_slope_deg=22.0,
    albedo_grain_m=0.18,
    roughness=0.88,
    normal_strength=0.95,
    micro_normal_freq=20.0,
    smooth_px=1.8,
)

MARS_DEFAULT = MARS_BASALT
