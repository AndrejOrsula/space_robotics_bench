"""Isaac-free, frozen configuration records for the ephemeris subsystem.

Records use ``slots=True`` so IsaacLab's ``@configclass`` post-init walker
does not descend into them and attempt to mutate frozen fields. This also
means its structural ``MISSING`` validation and ``class_to_dict`` conversion
skip nested records. Each record therefore validates itself, and environment
code must call ``self.ephemeris.validate()`` explicitly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from numbers import Real

_BODIES = ("moon", "mars", "earth")
# Bodies accepted by an ephemeris request. ``_BODIES`` remains the
# surface-site vocabulary; requests may additionally ask for the sun or a
# planetary body observed from that site.
SUPPORTED_BODIES = ("sun", "earth", "moon", "mars", "venus")
_PROVIDERS = ("spice", "table")  # "table" reserved for precomputed products
_EPOCH_MIN = datetime(1850, 1, 1)  # noqa: DTZ001 - naive UTC comparison bound
# Inclusive through the end of 2149-12-31, as documented and as products.py.
_EPOCH_MAX = datetime(2149, 12, 31, 23, 59, 59, 999999)  # noqa: DTZ001 - naive UTC comparison bound


def _finite_real(value: object, *, name: str) -> float:
    """Return a finite real config value without coercing string/bool inputs."""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return number


def _parse_epoch(utc: str) -> datetime:
    if not isinstance(utc, str):
        raise ValueError(f"ephemeris epoch is not ISO-8601: {utc!r}")
    iso_utc = f"{utc[:-1]}+00:00" if utc.endswith("Z") else utc
    try:
        parsed = datetime.fromisoformat(iso_utc)
    except ValueError as exc:
        raise ValueError(f"ephemeris epoch is not ISO-8601: {utc!r}") from exc
    if parsed.tzinfo is not None:
        parsed = parsed.astimezone(timezone.utc).replace(  # noqa: UP017
            tzinfo=None
        )
    if not (_EPOCH_MIN <= parsed <= _EPOCH_MAX):
        raise ValueError(
            f"ephemeris epoch {utc!r} outside de440s coverage "
            f"[{_EPOCH_MIN.date()}, {_EPOCH_MAX.date()}]"
        )
    return parsed


def _validate_body_sequence(bodies: object, *, context: str) -> tuple[str, ...]:
    """Validate and return an ordered body sequence without deduplication."""
    if isinstance(bodies, (str, bytes)):
        raise ValueError(f"{context} must be a sequence of body names")
    try:
        values: tuple[str, ...] = tuple(bodies)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{context} must be a sequence of body names") from exc
    if any(not isinstance(body, str) or not body for body in values):
        raise ValueError(f"{context} must contain non-empty string body names")
    duplicates = tuple(body for i, body in enumerate(values) if body in values[:i])
    if duplicates:
        raise ValueError(
            f"{context} contains duplicate body name(s): "
            f"{tuple(dict.fromkeys(duplicates))!r}"
        )
    unknown = tuple(body for body in values if body not in SUPPORTED_BODIES)
    if unknown:
        raise ValueError(
            f"{context} contains unsupported body name(s) {unknown!r}; "
            f"supported bodies: {SUPPORTED_BODIES}"
        )
    return values


@dataclass(frozen=True, slots=True)
class EpochSpec:
    """When. Exactly one of `utc` (fixed), `utc_range` (sampled per reset), or
    `utc_windows` (sampled per reset from a duration-weighted set of epoch
    windows, e.g. built from
    :func:`srb.core.ephemeris.products.sunlit_epoch_windows`)."""

    utc: str | None = None
    utc_range: tuple[str, str] | None = None
    utc_windows: tuple[tuple[str, str], ...] | None = None
    time_scale: float = 0.0
    update_interval_s: float = 600.0

    def validate(self) -> None:
        fields = {
            "utc": self.utc,
            "utc_range": self.utc_range,
            "utc_windows": self.utc_windows,
        }
        set_names = [name for name, value in fields.items() if value is not None]
        if len(set_names) != 1:
            raise ValueError(
                "EpochSpec needs exactly one of utc / utc_range / utc_windows "
                f"(got: {', '.join(set_names) if set_names else 'none'})"
            )
        if self.utc is not None:
            _parse_epoch(self.utc)
        elif self.utc_range is not None:
            start, end = (_parse_epoch(u) for u in self.utc_range)
            if start >= end:
                raise ValueError(f"utc_range start {start} not before end {end}")
        else:
            assert self.utc_windows is not None
            if len(self.utc_windows) == 0:
                raise ValueError("utc_windows needs at least one window")
            for i, (w_start, w_end) in enumerate(self.utc_windows):
                start, end = _parse_epoch(w_start), _parse_epoch(w_end)
                if start > end:
                    raise ValueError(
                        f"utc_windows window {i}: start {start} after end {end}"
                    )
        numeric_times = {
            name: _finite_real(getattr(self, name), name=name)
            for name in ("time_scale", "update_interval_s")
        }
        if (
            numeric_times["time_scale"] < 0.0
            or numeric_times["update_interval_s"] <= 0.0
        ):
            raise ValueError("time_scale must be >= 0 and update_interval_s > 0")


@dataclass(frozen=True, slots=True)
class SiteSpec:
    """Where. Surface site (lat+lon) or orbit-centric observer (lat/lon None)."""

    body: str
    lat_deg: float | None = None
    lon_deg: float | None = None
    alt_m: float = 0.0
    north_yaw_deg: float = 0.0

    @property
    def is_surface(self) -> bool:
        return self.lat_deg is not None

    def validate(self) -> None:
        if self.body not in _BODIES:
            raise ValueError(f"SiteSpec.body {self.body!r} not in {_BODIES}")
        if (self.lat_deg is None) != (self.lon_deg is None):
            raise ValueError("SiteSpec needs both or neither of lat_deg/lon_deg")
        numeric_site_fields: dict[str, float] = {}
        for name in ("lat_deg", "lon_deg", "alt_m", "north_yaw_deg"):
            value = getattr(self, name)
            if value is None and name in ("lat_deg", "lon_deg"):
                continue
            numeric_site_fields[name] = _finite_real(value, name=f"SiteSpec.{name}")
        if self.lat_deg is not None and not (
            -90.0 <= numeric_site_fields["lat_deg"] <= 90.0
        ):
            raise ValueError(f"lat_deg {self.lat_deg} outside [-90, 90]")
        if self.lon_deg is not None and not (
            -360.0 <= numeric_site_fields["lon_deg"] <= 360.0
        ):
            raise ValueError(f"lon_deg {self.lon_deg} outside [-360, 360]")


@dataclass(frozen=True, slots=True)
class EphemerisCfg:
    """Top-level opt-in configuration."""

    enabled: bool = False
    provider: str = "spice"
    epoch: EpochSpec = field(
        default_factory=lambda: EpochSpec(utc="2026-11-01T12:00:00")
    )
    site: SiteSpec | None = None
    orbit: "OrbitSpec | None" = None  # type: ignore[name-defined]  # noqa: F821
    bodies: tuple[str, ...] = ("sun",)
    drive_sunlight: bool = True
    drive_skydome: bool = False
    expose_observations: bool = False
    eclipse: bool = False
    on_missing: str = "fallback"  # "fallback" | "error"

    def validate(self) -> None:
        if self.provider not in _PROVIDERS:
            raise ValueError(
                f"EphemerisCfg.provider {self.provider!r} not in {_PROVIDERS}"
            )
        if self.on_missing not in ("fallback", "error"):
            raise ValueError(
                f"on_missing {self.on_missing!r} not in ('fallback', 'error')"
            )
        validated_bodies = _validate_body_sequence(
            self.bodies, context="EphemerisCfg.bodies"
        )
        if "sun" not in validated_bodies:
            raise ValueError(
                f"EphemerisCfg.bodies {self.bodies!r} must include 'sun' -- "
                "sun_light_params/orbit_sun_light_params always look up "
                "state.bodies['sun']"
            )
        self.epoch.validate()
        if self.site is not None:
            self.site.validate()
            if self.provider == "table" and not self.site.is_surface:
                raise ValueError(
                    'EphemerisCfg.provider="table" serves surface sites only; '
                    "set env.ephemeris.site lat_deg/lon_deg or use the spice "
                    "provider"
                )
        if self.orbit is not None:
            self.orbit.validate()
            if self.provider == "table":
                raise ValueError(
                    'EphemerisCfg.provider="table" cannot be combined with '
                    "orbit mode (env.ephemeris.orbit set) -- the table "
                    "provider (srb.core.ephemeris.table_provider.TableProvider) "
                    "serves surface sites only and always raises ValueError at "
                    "query time for a non-surface SiteSpec; this is a config-"
                    "time equivalent of that same rejection, raised before a "
                    "Kit boot rather than at the first reset/query"
                )
