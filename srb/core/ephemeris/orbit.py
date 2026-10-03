"""Pure-NumPy circular-orbit observer model for Domain.ORBIT ephemeris.

Elements use the central body's J2000 axes. The Keplerian model omits J2,
drag, and frame rotation; it suits lighting and eclipse cadence, not navigation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from srb.core.ephemeris.state import BodyState

GM_M3S2: dict[str, float] = {
    "earth": 3.986004418e14,
    "moon": 4.9048695e12,
    "mars": 4.282837e13,
}

BODY_RADIUS_M: dict[str, float] = {
    "earth": 6_371_000.0,
    "moon": 1_737_400.0,
    "mars": 3_389_500.0,
}


@dataclass(frozen=True, slots=True)
class OrbitSpec:
    """Circular orbit around the ephemeris site's central body."""

    altitude_m: float
    inclination_deg: float = 0.0
    raan_deg: float = 0.0
    arg_lat_deg: float = 0.0  # argument of latitude at the reference epoch

    def validate(self) -> None:
        for name in ("altitude_m", "inclination_deg", "raan_deg", "arg_lat_deg"):
            value = getattr(self, name)
            if not math.isfinite(float(value)):
                raise ValueError(f"OrbitSpec.{name} must be finite, got {value!r}")
        if self.altitude_m <= 0.0:
            raise ValueError(f"OrbitSpec.altitude_m {self.altitude_m} must be > 0")
        if not (0.0 <= self.inclination_deg <= 180.0):
            raise ValueError(
                f"OrbitSpec.inclination_deg {self.inclination_deg} outside [0, 180]"
            )
        for name in ("raan_deg", "arg_lat_deg"):
            value = float(getattr(self, name))
            if not (-360.0 <= value <= 360.0):
                raise ValueError(f"OrbitSpec.{name} {value} outside [-360, 360]")


def _gm_radius(body: str) -> tuple[float, float]:
    try:
        return GM_M3S2[body], BODY_RADIUS_M[body]
    except KeyError as exc:
        raise ValueError(
            f"orbit propagation: body {body!r} not in {sorted(GM_M3S2)}"
        ) from exc


def orbit_position_j2000(orbit: OrbitSpec, body: str, dt_s: float) -> np.ndarray:
    """Central-body-centered position [m] in J2000 axes at *dt_s* seconds past
    the reference epoch (where the argument of latitude is ``arg_lat_deg``)."""
    gm, radius = _gm_radius(body)
    a = radius + orbit.altitude_m
    n = np.sqrt(gm / a**3)
    u = np.radians(orbit.arg_lat_deg) + n * dt_s
    inc = np.radians(orbit.inclination_deg)
    raan = np.radians(orbit.raan_deg)
    # in-plane position, then rotate by R3(-raan) @ R1(-inc)
    x_p, y_p = a * np.cos(u), a * np.sin(u)
    ci, si = np.cos(inc), np.sin(inc)
    cO, sO = np.cos(raan), np.sin(raan)
    return np.array(
        [
            cO * x_p - sO * ci * y_p,
            sO * x_p + cO * ci * y_p,
            si * y_p,
        ]
    )


def central_body_state(pos_j2000_m: np.ndarray, body: str) -> BodyState:
    """Return the central body's state from the orbiting observer's position.

    Azimuth and elevation are NaN because orbit mode has no horizon.
    """
    _, radius = _gm_radius(body)
    dist = float(np.linalg.norm(pos_j2000_m))
    direction = tuple(float(v) for v in (-np.asarray(pos_j2000_m, dtype=float) / dist))
    ang = 2.0 * float(np.degrees(np.arcsin(min(1.0, radius / dist))))
    return BodyState(
        azimuth_deg=float("nan"),
        elevation_deg=float("nan"),
        direction_world=direction,
        distance_m=dist,
        angular_diameter_deg=ang,
    )
