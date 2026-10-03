"""Map ephemeris state to lighting values and night policy."""

from __future__ import annotations

from dataclasses import dataclass

from srb.core.ephemeris.geometry import earthshine_fraction, sun_visibility_factor
from srb.core.ephemeris.state import EphemerisState

_MEAN_IRRADIANCE_SCALE = {
    "earth": 1.0,
    "moon": 1.0,
    "mars": 1.0 / (1.523679**2),
}


@dataclass(frozen=True)
class EphemerisLighting:
    """Lighting values matching LightingPreset fields."""

    sun_elevation_deg: float
    sun_azimuth_deg: float
    sun_intensity_lux: float
    sun_color_temperature_k: float
    sun_angular_diameter_deg: float
    earthshine_intensity_lux: float | None
    earthshine_color_temperature_k: float | None


def lighting_values(
    state: EphemerisState,
    *,
    base_intensity_lux: float,
    base_color_temperature_k: float,
    earthshine_base_lux: float = 200.0,
    earthshine_color_temperature_k: float = 7500.0,
) -> EphemerisLighting:
    sun = state.bodies["sun"]
    reference_scale = _MEAN_IRRADIANCE_SCALE[state.site.body]
    visibility = sun_visibility_factor(
        sun.elevation_deg, 0.5 * sun.angular_diameter_deg
    )
    earth = state.bodies.get("earth")
    earth_up = earth is not None and earth.elevation_deg > 0.0
    return EphemerisLighting(
        sun_elevation_deg=sun.elevation_deg,
        sun_azimuth_deg=sun.azimuth_deg % 360.0,
        sun_intensity_lux=(
            base_intensity_lux * state.irradiance_scale / reference_scale * visibility
        ),
        sun_color_temperature_k=base_color_temperature_k,
        sun_angular_diameter_deg=sun.angular_diameter_deg,
        earthshine_intensity_lux=(
            earthshine_base_lux * earthshine_fraction(sun, earth) if earth_up else None
        ),
        earthshine_color_temperature_k=earthshine_color_temperature_k
        if earth_up
        else None,
    )
