"""Build epoch-keyed lighting assets from real ephemeris results.

Ephemeris generations use request identity, leaving the static catalog cache
untouched. Keep imports lazy so this module loads without Isaac Sim or ``pxr``.
"""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any, Callable, NoReturn, cast

from srb.terrain.lights.schema import LightingPreset
from srb.utils.digest import sha256_file

if TYPE_CHECKING:  # Isaac-only; never imported at module top.
    from pathlib import Path

    from srb.core.asset import AssetBaseCfg  # type: ignore[attr-defined]
    from srb.core.ephemeris.state import EphemerisState
    from srb.utils.failures import HealthEvent

EPHEMERIS_LIGHTING = "ephemeris"


def _ephemeris_bodies(body: str) -> tuple[str, ...]:
    """Sun always; Earth too on the Moon (earthshine secondary light)."""
    return ("sun", "earth") if body == "moon" else ("sun",)


_BODY_DEFAULT_PRESET_NAME: dict[str, str] = {
    "moon": "lunar_default",
    "mars": "mars_default",
    "earth": "earth_default",
}


def _body_default_preset_name(body: str) -> str:
    return _BODY_DEFAULT_PRESET_NAME[body]


def _body_default_preset(body: str) -> LightingPreset:
    from srb.terrain.lights.catalog_loader import load_lighting_catalog

    return load_lighting_catalog().presets[_body_default_preset_name(body)]


def _kernels_present() -> bool:
    """Check for the pinned NAIF bundle without importing SPICE or downloading."""
    from srb.core.ephemeris.kernels import kernel_bundle_present

    return kernel_bundle_present()


def _default_lighting_warn(message: str) -> None:
    from srb.terrain.log import get_logger

    get_logger("lighting").warning(message)


def _quat_tuple(values: Any) -> tuple[float, float, float, float]:
    x, y, z, w = (float(value) for value in values)
    return x, y, z, w


def _fallback_static_lighting_asset_cfg(
    *,
    body: str,
    site: str,
    epoch_utc: str,
    on_missing: str,
    warn: Callable[[str], None] | None,
    health: Callable[["HealthEvent"], None] | None,
    on_health: Callable[["HealthEvent"], None] | None,
) -> "AssetBaseCfg":
    """Warn once or raise, then return the body's static preset when kernels are absent."""
    from srb.core.ephemeris.kernels import KernelsMissingError, kernel_cache_root

    root = kernel_cache_root()
    reason = f"NAIF kernels missing under {root}"

    if on_missing == "error":
        raise KernelsMissingError(
            f"SPICE kernels required for ephemeris lighting of {body}/{site} "
            f"@ {epoch_utc} are missing under {root}; run: srb ephemeris download"
        )
    if on_missing != "fallback":
        raise ValueError(
            f"on_missing must be 'fallback' or 'error', got {on_missing!r}"
        )

    from srb.terrain.source_health import warn_ephemeris_lighting_fallback_once

    warn_ephemeris_lighting_fallback_once(
        site=site,
        body=body,
        epoch=epoch_utc,
        reason=reason,
        warn=warn if warn is not None else _default_lighting_warn,
        health=health,
        on_health=on_health,
    )

    from srb.terrain.lights.runtime import RealLighting

    return RealLighting.build_asset_cfg(_body_default_preset_name(body))


def _lighting_from_state(
    state: "EphemerisState",
    *,
    body: str,
    base_preset: LightingPreset,
    north_convergence_deg: float = 0.0,
) -> LightingPreset:
    """Map state lighting values into a preset, applying the grid-north correction."""
    from srb.core.ephemeris.lighting import lighting_values

    vals = lighting_values(
        state,
        base_intensity_lux=base_preset.sun_intensity_lux,
        base_color_temperature_k=base_preset.sun_color_temperature_k,
        earthshine_base_lux=(
            200.0
            if base_preset.earthshine_intensity_lux is None
            else base_preset.earthshine_intensity_lux
        ),
    )
    az_grid = (vals.sun_azimuth_deg + north_convergence_deg) % 360.0
    kernels = state.provenance.get("kernels", "")
    return LightingPreset(
        body=body,
        sun_elevation_deg=vals.sun_elevation_deg,
        sun_azimuth_deg=az_grid,
        sun_intensity_lux=vals.sun_intensity_lux,
        sun_color_temperature_k=vals.sun_color_temperature_k,
        sun_angular_diameter_deg=vals.sun_angular_diameter_deg,
        earthshine_intensity_lux=vals.earthshine_intensity_lux,
        earthshine_color_temperature_k=vals.earthshine_color_temperature_k,
        license=base_preset.license,
        attribution=f"SPICE ephemeris @ {state.epoch_utc} ({kernels})",
    )


def compute_ephemeris_lighting(
    *,
    body: str,
    lat_deg: float,
    lon_deg: float,
    epoch_utc: str,
    base_preset: LightingPreset,
    north_convergence_deg: float = 0.0,
) -> LightingPreset:
    from srb.core.ephemeris import query
    from srb.core.ephemeris.config import SiteSpec

    typed_query = cast(Callable[..., "EphemerisState"], query)
    state = typed_query(
        SiteSpec(body=body, lat_deg=lat_deg, lon_deg=lon_deg),
        epoch_utc,
        bodies=_ephemeris_bodies(body),
    )
    return _lighting_from_state(
        state,
        body=body,
        base_preset=base_preset,
        north_convergence_deg=north_convergence_deg,
    )


def _slug(value: str) -> str:
    """Filesystem-safe slug (epoch strings carry ``:`` which is path-hostile)."""
    return re.sub(r"[^0-9A-Za-z]+", "_", value).strip("_")


LIGHTS_WRITER_VERSION: int = 2
"""Bump when writer output bytes change; shared by static and ephemeris cache identities."""

_LIGHTING_IDENTITY_SCHEMA = "srb-ephemeris-lighting/2"
_LIGHTING_IDENTITY_FIELDS = (
    "schema",
    "writer_version",
    "body",
    "site",
    "site_coords",
    "epoch_utc",
    "base_preset",
    "computed_preset",
    "north_convergence_deg",
)
_LIGHTING_GENERATION_MEMBERS = ("lights.usd", "identity.json")


def _canonical_json(value: object) -> bytes:
    """Serialize one deterministic JSON identity payload."""
    return json.dumps(
        value,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _preset_identity(preset: LightingPreset) -> dict[str, object]:
    return cast(dict[str, object], preset.model_dump(mode="json"))


def _lighting_identity(
    *,
    body: str,
    site: str,
    lat_deg: float,
    lon_deg: float,
    epoch_utc: str,
    base_preset: LightingPreset,
    computed_preset: LightingPreset,
    north_convergence_deg: float,
) -> dict[str, object]:
    """Build the canonical request identity; every request and preset field is hashed."""
    core: dict[str, object] = {
        "schema": _LIGHTING_IDENTITY_SCHEMA,
        "writer_version": LIGHTS_WRITER_VERSION,
        "body": body,
        "site": site,
        "site_coords": {"lat_deg": lat_deg, "lon_deg": lon_deg},
        "epoch_utc": epoch_utc,
        "base_preset": _preset_identity(base_preset),
        "computed_preset": _preset_identity(computed_preset),
        "north_convergence_deg": north_convergence_deg,
    }
    return {
        **core,
        "sha256": hashlib.sha256(_canonical_json(core)).hexdigest(),
    }


def _validate_lighting_generation(
    generation: Any,
    *,
    expected_identity: dict[str, object],
) -> None:
    """Validate the request identity and the exact two-member generation."""
    from pathlib import Path

    from srb.utils.publication import (
        PublicationError,
        PublicationIdentityError,
    )

    member_hashes = generation.member_hashes
    if set(member_hashes) != set(_LIGHTING_GENERATION_MEMBERS):
        raise PublicationError(
            "ephemeris lighting generation must contain exactly "
            f"{_LIGHTING_GENERATION_MEMBERS!r}"
        )

    identity_path = Path(generation.path) / "identity.json"
    identity = generation.read_member("identity.json", _parse_json_object)
    if set(identity) != {
        *_LIGHTING_IDENTITY_FIELDS,
        "sha256",
        "lights_usd_sha256",
    }:
        raise PublicationError("ephemeris lighting identity has unexpected fields")

    expected_sha256 = expected_identity["sha256"]
    actual_sha256 = identity.get("sha256")
    if actual_sha256 != expected_sha256 or any(
        identity.get(field) != expected_identity.get(field)
        for field in _LIGHTING_IDENTITY_FIELDS
    ):
        raise PublicationIdentityError(
            path=identity_path,
            expected_sha256=str(expected_sha256),
            actual_sha256=str(actual_sha256),
        )

    core = {field: identity[field] for field in _LIGHTING_IDENTITY_FIELDS}
    canonical_sha256 = hashlib.sha256(_canonical_json(core)).hexdigest()
    if actual_sha256 != canonical_sha256:
        raise PublicationError("ephemeris lighting identity SHA-256 mismatch")

    lights_sha256 = identity.get("lights_usd_sha256")
    if (
        not isinstance(lights_sha256, str)
        or len(lights_sha256) != 64
        or any(char not in "0123456789abcdef" for char in lights_sha256)
        or lights_sha256 != member_hashes["lights.usd"]
    ):
        raise PublicationError("ephemeris lighting USD identity mismatch")


def _parse_json_object(payload: bytes) -> dict[str, object]:
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError("JSON identity must be an object")
    return cast(dict[str, object], value)


_LEAF_HASH_CHARS = 16


def _ephemeris_lights_out_path(
    *,
    body: str,
    site: str,
    epoch_utc: str,
    identity: Mapping[str, object],
) -> "Path":
    """Return an epoch-keyed slot, isolated from static presets and keyed by request
    identity.
    """
    from srb.utils.path import SRB_ASSETS_DIR_SRB_LIGHTS

    digest = str(identity["sha256"])[:_LEAF_HASH_CHARS]
    leaf = f"{body}_{_slug(site)}_{_slug(epoch_utc)}_{digest}"
    return SRB_ASSETS_DIR_SRB_LIGHTS / "ephemeris" / leaf / "lights.usd"


def _publish_ephemeris_lights(
    out_path: "Path",
    writer: Callable[["Path"], object],
    *,
    identity: dict[str, object],
) -> "Path":
    """Reuse or publish a complete immutable generation validated against the request
    identity.
    """
    from pathlib import Path

    from srb.utils.publication import (
        ArtifactPublication,
        PublicationError,
        PublishedGeneration,
    )

    out_path = Path(out_path)
    publication = ArtifactPublication.generation(out_path.parent)
    required = _LIGHTING_GENERATION_MEMBERS

    def matches(generation: PublishedGeneration) -> bool:
        try:
            _validate_lighting_generation(generation, expected_identity=identity)
        except (PublicationError, OSError, ValueError):
            return False
        return True

    try:
        generation: PublishedGeneration | None = cast(
            PublishedGeneration,
            publication.open(required_members=required),
        )
    except (PublicationError, OSError):
        generation = None
    if generation is None or not matches(generation):

        def build(staging: Path) -> None:
            lights_path = staging / out_path.name
            writer(lights_path)
            lights_sha256 = sha256_file(lights_path)
            identity_payload = {
                **identity,
                "lights_usd_sha256": lights_sha256,
            }
            (staging / "identity.json").write_bytes(_canonical_json(identity_payload))

        generation = publication.publish_or_reuse(
            build,
            required_members=required,
            reuse_if=matches,
            repair_invalid=True,
        )
    _validate_lighting_generation(generation, expected_identity=identity)
    return generation.path / out_path.name


def _read_north_convergence(baked_usd: str | None) -> float:
    """Read persisted grid convergence; missing legacy fields mean true north.

    Invalid or unreadable metadata raises ``IntegrityFailureError`` rather than
    silently changing the lighting frame. The persisted value is never recomputed.
    """
    if not baked_usd:
        return 0.0

    import json
    import math
    from pathlib import Path

    from srb.utils.publication import (
        ArtifactPublication,
        PublicationError,
        PublishedGeneration,
    )

    meta_path = Path(baked_usd).parent / "meta.json"

    def integrity_failure(reason: str, cause: BaseException | None = None) -> NoReturn:
        from srb.utils.failures import IntegrityFailureError

        details: dict[str, object] = {"path": str(meta_path), "reason": reason}
        if cause is not None:
            details["cause_type"] = (
                f"{type(cause).__module__}.{type(cause).__qualname__}"
            )
        error = IntegrityFailureError(
            code="terrain.lighting.ephemeris_georef_integrity",
            message=f"invalid ephemeris georef metadata at {meta_path}: {reason}",
            component="terrain.lighting.ephemeris",
            details=details,
            fallback_allowed=False,
            evidence_valid=False,
        )
        if cause is None:
            raise error
        raise error from cause

    def read_meta() -> object | None:
        # A v2 terrain member is covered by its generation manifest rather
        # than by a standalone file identity marker. Validate that generation
        # when the path has the canonical ``.../generations/<id>/`` shape;
        # older direct cache snapshots continue through the file contract.
        generation_dir = meta_path.parent
        generation: PublishedGeneration | None = None
        if generation_dir.parent.name == "generations":
            generation_root = generation_dir.parent.parent
            generation = cast(
                PublishedGeneration,
                ArtifactPublication.generation(generation_root).open(
                    required_members=("meta.json",)
                ),
            )
            if Path(generation.path) != generation_dir:
                raise PublicationError(
                    f"terrain metadata is not the current published generation: "
                    f"{meta_path}"
                )
        if generation is not None:
            return generation.read_member("meta.json", json.loads)
        return ArtifactPublication.file(meta_path).read(json.loads)

    try:
        meta = read_meta()
    except (OSError, PublicationError, TypeError, ValueError, UnicodeError) as exc:
        integrity_failure("publication or JSON read failed", exc)
    if meta is None:
        integrity_failure("metadata snapshot is missing")
    if not isinstance(meta, Mapping):
        integrity_failure("metadata root is not a mapping")
    if "georef" not in meta:
        return 0.0
    georef = meta["georef"]
    if not isinstance(georef, Mapping):
        integrity_failure("georef is not a mapping")
    if "north_convergence_deg" not in georef:
        return 0.0
    gamma = georef["north_convergence_deg"]
    # bool is an int subclass -- reject it so a stray JSON ``true`` is not 1.0.
    if isinstance(gamma, bool) or not isinstance(gamma, (int, float)):
        integrity_failure("north_convergence_deg is not a real number")
    value = float(gamma)
    # json.loads accepts ``NaN``/``Infinity``/``-Infinity`` literals; reject
    # them because a non-finite convergence is not scientific evidence.
    if not math.isfinite(value):
        integrity_failure("north_convergence_deg is not finite")
    return value


def build_ephemeris_lighting_asset_cfg(
    *,
    body: str,
    site: str,
    lat_deg: float,
    lon_deg: float,
    epoch_utc: str,
    base_preset: LightingPreset,
    baked_usd: str | None = None,
    north_convergence_deg: float | None = None,
    on_missing: str = "fallback",
    warn: Callable[[str], None] | None = None,
    health: Callable[["HealthEvent"], None] | None = None,
    on_health: Callable[["HealthEvent"], None] | None = None,
) -> "AssetBaseCfg":
    """Build and publish epoch lighting, using persisted grid convergence when
    available.

    ``north_convergence_deg`` overrides baked metadata. Without NAIF kernels, fallback
    warns once and returns static lighting without querying or downloading kernels;
    ``on_missing="error"`` raises.
    """
    if on_missing not in ("fallback", "error"):
        raise ValueError(
            f"on_missing must be 'fallback' or 'error', got {on_missing!r}"
        )

    if north_convergence_deg is None:
        gamma = _read_north_convergence(baked_usd)
    else:
        import math
        from numbers import Real

        from srb.utils.failures import InvalidInputError

        if isinstance(north_convergence_deg, bool) or not isinstance(
            north_convergence_deg, Real
        ):
            raise InvalidInputError(
                code="terrain.lighting.invalid_north_convergence",
                message="north_convergence_deg must be a finite real number",
                component="terrain.lighting.ephemeris",
                details={"value_type": type(north_convergence_deg).__name__},
                fallback_allowed=False,
                evidence_valid=False,
            )
        gamma = float(north_convergence_deg)
        if not math.isfinite(gamma):
            raise InvalidInputError(
                code="terrain.lighting.invalid_north_convergence",
                message="north_convergence_deg must be finite",
                component="terrain.lighting.ephemeris",
                details={"value": repr(north_convergence_deg)},
                fallback_allowed=False,
                evidence_valid=False,
            )

    if not _kernels_present():
        return _fallback_static_lighting_asset_cfg(
            body=body,
            site=site,
            epoch_utc=epoch_utc,
            on_missing=on_missing,
            warn=warn,
            health=health,
            on_health=on_health,
        )

    from srb.core.ephemeris import query
    from srb.core.ephemeris.config import SiteSpec
    from srb.core.ephemeris.geometry import light_quat_xyzw

    typed_query = cast(Callable[..., "EphemerisState"], query)
    state = typed_query(
        SiteSpec(body=body, lat_deg=lat_deg, lon_deg=lon_deg),
        epoch_utc,
        bodies=_ephemeris_bodies(body),
    )
    preset = _lighting_from_state(
        state,
        body=body,
        base_preset=base_preset,
        north_convergence_deg=gamma,
    )

    identity = _lighting_identity(
        body=body,
        site=site,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        epoch_utc=epoch_utc,
        base_preset=base_preset,
        computed_preset=preset,
        north_convergence_deg=gamma,
    )
    out_path = _ephemeris_lights_out_path(
        body=body,
        site=site,
        epoch_utc=epoch_utc,
        identity=identity,
    )

    # SUN: grid azimuth already folded into preset.sun_azimuth_deg (+gamma).
    sun_q = light_quat_xyzw(preset.sun_azimuth_deg, preset.sun_elevation_deg)

    # EARTHSHINE: real Earth direction (grid-corrected), not sun-az+180.
    earth = state.bodies.get("earth")
    es_q = None
    if preset.earthshine_intensity_lux and earth is not None:
        es_q = light_quat_xyzw(
            (earth.azimuth_deg + gamma) % 360.0,
            earth.elevation_deg,
        )

    from srb.terrain.lights.usd_writer import write_lights_usd

    out_path = _publish_ephemeris_lights(
        out_path,
        lambda staged: write_lights_usd(
            staged,
            preset=preset,
            sun_orient_quat_xyzw=_quat_tuple(sun_q),
            earthshine_orient_quat_xyzw=(None if es_q is None else _quat_tuple(es_q)),
        ),
        identity=identity,
    )

    from srb.core.asset import AssetBaseCfg  # type: ignore[attr-defined]
    from srb.core.sim import UsdFileCfg  # type: ignore[attr-defined]
    from srb.terrain.lights.runtime import RealLighting

    return AssetBaseCfg(
        prim_path=RealLighting.PRIM_PATH,
        spawn=UsdFileCfg(usd_path=str(out_path)),
    )


def make_ephemeris_build_lighting(
    *,
    body: str,
    site: str | None,
    baked_usd: str | None = None,
    load_manifest: Callable[[], Any] | None = None,
    on_missing: str = "fallback",
    health: Callable[["HealthEvent"], None] | None = None,
    on_health: Callable[["HealthEvent"], None] | None = None,
) -> Callable[[str], Any] | None:
    """Return a lighting closure for an ephemeris site, or ``None`` for static lighting.

    An ephemeris site without an epoch raises ``ValueError``.
    """
    if site is None:
        return None

    if load_manifest is None:
        from srb.terrain.manifest_loader import load_manifest as _load_manifest

        load_manifest = _load_manifest

    manifest = load_manifest()
    site_entry = manifest.sites.get(site)
    if (
        site_entry is None
        or getattr(site_entry, "lighting", None) != EPHEMERIS_LIGHTING
    ):
        return None

    epoch = getattr(site_entry, "epoch", None)
    if not epoch:
        raise ValueError(
            f"site {site!r} has lighting='{EPHEMERIS_LIGHTING}' but no 'epoch'; "
            "an ephemeris-driven site must declare its epoch"
        )

    coords = site_entry.coords
    lat_deg = coords.lat
    lon_deg = coords.lon

    def _build_lighting(_name: str) -> Any:
        return build_ephemeris_lighting_asset_cfg(
            body=body,
            site=site,
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            epoch_utc=epoch,
            base_preset=_body_default_preset(body),
            baked_usd=baked_usd,
            on_missing=on_missing,
            health=health,
            on_health=on_health,
        )

    return _build_lighting


def get_last_ephemeris_lighting_health_event() -> "HealthEvent | None":
    """Return the queryable state for the last static-lighting fallback."""
    from srb.terrain.source_health import (
        get_last_ephemeris_lighting_health_event as _get_last_event,
    )

    return _get_last_event()


__all__ = [
    "EPHEMERIS_LIGHTING",
    "build_ephemeris_lighting_asset_cfg",
    "compute_ephemeris_lighting",
    "get_last_ephemeris_lighting_health_event",
    "make_ephemeris_build_lighting",
]
