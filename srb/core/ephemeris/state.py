"""Frozen ephemeris result records."""

from __future__ import annotations

from dataclasses import dataclass

from srb.core.ephemeris.config import SiteSpec


@dataclass(frozen=True)
class BodyState:
    azimuth_deg: float  # compass az in local ENU (NaN in orbit mode)
    elevation_deg: float  # may be negative; NaN in orbit mode
    direction_world: tuple[float, float, float]  # unit vector, z-up world
    distance_m: float
    angular_diameter_deg: float


@dataclass(frozen=True)
class EphemerisState:
    epoch_utc: str
    epoch_et: float
    site: SiteSpec
    bodies: dict[str, BodyState]  # "sun" always present
    irradiance_scale: float  # (1 AU / sun distance)**2
    provenance: dict[str, str]
