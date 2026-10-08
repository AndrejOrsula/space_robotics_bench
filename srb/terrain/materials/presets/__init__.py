from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

Body = Literal["moon", "mars", "earth"]
RGB = tuple[float, float, float]


@dataclass(frozen=True)
class MaterialPreset:
    name: str
    body: Body
    base_rgb: RGB
    bedrock_rgb: RGB
    weathered_rgb: RGB
    shadow_rgb: RGB
    bedrock_slope_deg: float
    albedo_grain_m: float
    roughness: float
    normal_strength: float
    micro_normal_freq: float
    smooth_px: float


from srb.terrain.materials.presets.earth import EARTH_DEFAULT  # noqa: E402
from srb.terrain.materials.presets.mars import (  # noqa: E402
    MARS_BASALT,
    MARS_CLAY,
    MARS_DEFAULT,
    MARS_DUST,
)
from srb.terrain.materials.presets.moon import (  # noqa: E402
    LUNAR_HIGHLAND,
    LUNAR_MARE,
    MOON_DEFAULT,
)

PRESETS: dict[str, MaterialPreset] = {
    "lunar_default": MOON_DEFAULT,
    "lunar_mare": LUNAR_MARE,
    "lunar_highland": LUNAR_HIGHLAND,
    "mars_default": MARS_DEFAULT,
    "mars_basalt": MARS_BASALT,
    "mars_dust": MARS_DUST,
    "mars_clay": MARS_CLAY,
    "earth_default": EARTH_DEFAULT,
}

_DEFAULT_BY_BODY: dict[str, MaterialPreset] = {
    "moon": MOON_DEFAULT,
    "mars": MARS_DEFAULT,
    "earth": EARTH_DEFAULT,
}


def get_preset(name: str) -> MaterialPreset:
    if name not in PRESETS:
        raise KeyError(
            f"unknown material preset {name!r}; choose one of {', '.join(PRESETS)}"
        )
    return PRESETS[name]


def get_default_preset(body: Body) -> MaterialPreset:
    if body not in _DEFAULT_BY_BODY:
        raise KeyError(
            f"no default material preset is registered for body {body!r}; "
            f"choose one of {', '.join(_DEFAULT_BY_BODY)}"
        )
    return _DEFAULT_BY_BODY[body]


__all__ = [
    "MaterialPreset",
    "PRESETS",
    "get_preset",
    "get_default_preset",
]
