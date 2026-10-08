"""Augmentation presets: per-body parameters for the SimForge layer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True, slots=True)
class AugmentPreset:
    """SimForge augmentation parameters.

    Each preset records the per-body distribution parameters used by
    ``RealTerrainAugment``: rock density and size, dust displacement, and
    crater scattering.
    """

    body: Literal["moon", "mars", "earth"]
    rock_density_per_m2: float
    rock_size_range_m: tuple[float, float]
    dust_amp_m: float
    dust_scale_m: float
    crater_density_per_m2: float
    crater_radius_range_m: tuple[float, float]


LUNAR_DEFAULT = AugmentPreset(
    body="moon",
    rock_density_per_m2=0.05,
    rock_size_range_m=(0.05, 0.5),
    dust_amp_m=0.02,
    dust_scale_m=2.5,
    crater_density_per_m2=0.0,
    crater_radius_range_m=(0.5, 2.0),
)

MARS_DEFAULT = AugmentPreset(
    body="mars",
    rock_density_per_m2=0.02,
    rock_size_range_m=(0.04, 0.8),
    dust_amp_m=0.04,
    dust_scale_m=3.0,
    crater_density_per_m2=0.0,
    crater_radius_range_m=(0.3, 1.5),
)


EARTH_DEFAULT = AugmentPreset(
    body="earth",
    rock_density_per_m2=0.0,
    rock_size_range_m=(0.05, 0.5),
    dust_amp_m=0.0,
    dust_scale_m=1.0,
    crater_density_per_m2=0.0,
    crater_radius_range_m=(0.5, 2.0),
)


# Opt-in crater presets for bake-time stamping. Defaults keep crater density
# at zero; distinct names ensure crater settings affect the cache key.
LUNAR_CRATERED = AugmentPreset(
    body=LUNAR_DEFAULT.body,
    rock_density_per_m2=LUNAR_DEFAULT.rock_density_per_m2,
    rock_size_range_m=LUNAR_DEFAULT.rock_size_range_m,
    dust_amp_m=LUNAR_DEFAULT.dust_amp_m,
    dust_scale_m=LUNAR_DEFAULT.dust_scale_m,
    crater_density_per_m2=0.002,
    crater_radius_range_m=(1.0, 8.0),
)

MARS_CRATERED = AugmentPreset(
    body=MARS_DEFAULT.body,
    rock_density_per_m2=MARS_DEFAULT.rock_density_per_m2,
    rock_size_range_m=MARS_DEFAULT.rock_size_range_m,
    dust_amp_m=MARS_DEFAULT.dust_amp_m,
    dust_scale_m=MARS_DEFAULT.dust_scale_m,
    crater_density_per_m2=0.002,
    crater_radius_range_m=(1.0, 8.0),
)


_REGISTRY: dict[str, AugmentPreset] = {
    "lunar_default": LUNAR_DEFAULT,
    "mars_default": MARS_DEFAULT,
    "earth_default": EARTH_DEFAULT,
    "lunar_cratered": LUNAR_CRATERED,
    "mars_cratered": MARS_CRATERED,
}


def get_augment_preset(name: str) -> AugmentPreset:
    """Look up a preset by name; raises KeyError on miss."""
    if name not in _REGISTRY:
        raise KeyError(
            f"unknown augment preset {name!r}; available: {sorted(_REGISTRY)}"
        )
    return _REGISTRY[name]
