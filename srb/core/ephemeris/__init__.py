"""Optional real-ephemeris subsystem.

Keep this module importable without Isaac Sim and without spiceypy; heavy
imports live in provider implementations.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from srb.core.ephemeris.config import EphemerisCfg, EpochSpec, SiteSpec  # noqa: F401
from srb.core.ephemeris.orbit import OrbitSpec  # noqa: F401
from srb.core.ephemeris.state import BodyState, EphemerisState  # noqa: F401

if TYPE_CHECKING:
    from srb.core.ephemeris.provider import HealthObserver


def query(
    site: SiteSpec,
    epoch_utc: str,
    bodies: tuple[str, ...] = ("sun",),
    provider: str = "spice",
    *,
    health_observer: HealthObserver | None = None,
    optional_bodies: tuple[str, ...] = (),
) -> EphemerisState:
    """Query body states; see :func:`srb.core.ephemeris.provider.query`.

    The provider module is imported lazily to keep this package light.
    """
    from srb.core.ephemeris.provider import query as _query

    return _query(
        site,
        epoch_utc,
        bodies,
        provider,
        health_observer=health_observer,
        optional_bodies=optional_bodies,
    )
