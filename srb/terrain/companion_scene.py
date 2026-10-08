"""Real-terrain companion scene policy.

Manifest defaults, overrides, asset construction, and scene attachment share
one Isaac-light policy seam.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Literal, Mapping

from srb.terrain.source_health import classify_terrain_source_exception
from srb.terrain.spec import Body
from srb.utils.failures import (
    FALLBACK_ALLOWED_CATEGORIES,
    FailureCategory,
    FailureError,
    FailureInfo,
    HealthEvent,
    UnsupportedCapabilityError,
)

TerrainAugmentMode = Literal["none", "simforge", "database"]
_COMPANION_HEALTH_EVENTS_KEY = "_terrain_health_events"


@dataclass(frozen=True, slots=True)
class TerrainCompanionDefaults:
    sky: str | None = None
    lighting: str | None = None
    rocks: str | None = None
    augment_preset: str | None = None
    health_event: HealthEvent | None = None

    def as_dict(self) -> dict[str, str | None]:
        return {
            "sky": self.sky,
            "lighting": self.lighting,
            "rocks": self.rocks,
            "augment_preset": self.augment_preset,
        }


@dataclass(frozen=True, slots=True)
class TerrainCompanionPlan:
    sky: str | None = None
    lighting: str | None = None
    rocks: str | None = None
    augment_preset: str | None = None
    health_event: HealthEvent | None = None


NO_TERRAIN_COMPANIONS = TerrainCompanionDefaults()


_EMITTED_SCENERY_COMPANION_ATTACH_FAILURES: set[tuple[str, str]] = set()
"""Site/error pairs already warned about during this process."""


def disabled_companion_kwargs() -> dict[str, None]:
    """Return RealTerrain kwargs that explicitly disable scene companions."""

    return {
        "sky": None,
        "lighting": None,
        "rocks": None,
    }


def resolve_companion_name(
    field_value: str | None,
    manifest_default: str | None,
) -> str | None:
    """Resolve ``auto``/``None``/explicit companion override semantics."""

    if field_value == "auto":
        return manifest_default
    return field_value


_DEFAULT_AUGMENT_PRESET_BY_BODY: dict[str, str] = {
    "moon": "lunar_default",
    "mars": "mars_default",
    "earth": "earth_default",
}


def default_augment_preset_for_body(body: Body) -> str:
    try:
        return _DEFAULT_AUGMENT_PRESET_BY_BODY[body]
    except KeyError:
        raise ValueError(
            f"body must be one of {sorted(_DEFAULT_AUGMENT_PRESET_BY_BODY)}, "
            f"got {body!r}"
        ) from None


def spec_augment_preset(spec: Any) -> str | None:
    """The augment preset name a bake of ``spec`` (a ``TerrainSpec``) uses.

    The request override wins; a manifest body default resolves to ``None``
    so existing cache keys remain unchanged.
    """
    if spec.augment_preset:
        return spec.augment_preset
    if spec.site is None:
        return None
    from srb.terrain.manifest_loader import get_site

    declared = get_site(spec.site).augment_preset
    if declared == default_augment_preset_for_body(spec.body):
        return None
    return declared


def resolve_effective_augment_preset(explicit: str | None, body: Body) -> Any:
    """Resolve a body-compatible preset shared by cache, bake, and scene paths.

    Unknown names raise ``KeyError``; an explicit preset for another body
    raises ``ValueError`` at construction-time validation.
    """
    from srb.terrain.augmentation.presets import _REGISTRY, get_augment_preset

    preset = get_augment_preset(explicit or default_augment_preset_for_body(body))
    if explicit is not None and preset.body != body:
        available = sorted(name for name, p in _REGISTRY.items() if p.body == body)
        raise ValueError(
            f"augment_preset={explicit!r} is a {preset.body!r} preset, "
            f"which does not match body={body!r}; available presets for "
            f"body={body!r}: {available}"
        )
    return preset


def _classify_companion_manifest_failure(exc: BaseException) -> FailureInfo:
    """Classify manifest-load failures without authorizing unknown fallbacks."""
    if isinstance(exc, FailureError):
        return exc.failure_info
    if isinstance(exc, (FileNotFoundError, ModuleNotFoundError)):
        return FailureInfo(
            category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
            code="terrain.companion.manifest_unavailable",
            message=str(exc) or type(exc).__name__,
            component="terrain.companion_scene",
            details={"exception_type": type(exc).__name__},
            fallback_allowed=True,
        )
    return classify_terrain_source_exception(exc).failure_info


def _companion_manifest_health_event(*, site: str, failure: FailureInfo) -> HealthEvent:
    """Build the observable event required for an allowed manifest fallback."""
    if (
        not failure.fallback_allowed
        or failure.category not in FALLBACK_ALLOWED_CATEGORIES
    ):
        raise ValueError("companion manifest fallback is not permitted")
    return HealthEvent.degraded_fallback(
        failure,
        component="terrain.companion_scene",
        code="terrain.companion_default_scene_fallback",
        message=(
            "real-terrain companion manifest unavailable; keeping the "
            "environment's default scene"
        ),
        details={"site": site},
    )


def _warn_scenery_companion_attach_failed_once(
    env_cfg: Any,
    scenery: Any,
    err: Exception,
) -> None:
    """Apply only an explicitly classified, observable companion fallback.

    Only classified optional, transient, or unsupported failures may fall back;
    integrity, safety, invalid-input, and unknown failures are re-raised.
    """
    from srb.terrain.source_health import (
        TerrainFallbackWarning,
        make_terrain_fallback_health_event,
    )

    if isinstance(err, FailureError):
        failure = err.failure_info
        if (
            not failure.fallback_allowed
            or failure.category not in FALLBACK_ALLOWED_CATEGORIES
        ):
            raise err
        health_event = HealthEvent.degraded_fallback(
            failure,
            component="terrain.companion_scene",
            code="terrain.companion_default_scene_fallback",
            message=(
                "real-terrain companion attachment unavailable; keeping the "
                "environment's default scene"
            ),
            details={
                "site": str(getattr(scenery, "site", None)),
                "body": str(getattr(scenery, "body", None)),
            },
        )
    else:
        issue = classify_terrain_source_exception(err)
        health_event = make_terrain_fallback_health_event(
            surface_name="terrain.companion_scene",
            body=str(getattr(scenery, "body", "unknown")),
            site=str(getattr(scenery, "site", "unknown")),
            issue=issue,
        )

    current_events = tuple(getattr(env_cfg, "terrain_health_events", ()))
    if health_event not in current_events:
        env_cfg.terrain_health_events = (*current_events, health_event)

    key = (
        str(getattr(scenery, "site", None)),
        f"{type(err).__name__}: {err}",
    )
    if key in _EMITTED_SCENERY_COMPANION_ATTACH_FAILURES:
        return
    _EMITTED_SCENERY_COMPANION_ATTACH_FAILURES.add(key)

    from srb.utils import logging

    logging.warning(
        TerrainFallbackWarning(
            "real-terrain companion attachment failed for site "
            f"{getattr(scenery, 'site', None)!r} -- keeping the env's default "
            "sunlight/skydome (no companion products attached). Set "
            "env.scenery.sky/lighting/rocks to None (or "
            "env.scenery.attach_companions_enabled=False) to silence. Cause: "
            f"{type(err).__name__}: {err}",
            health_event,
        )
    )


def handle_companion_attach_failure(
    env_cfg: Any,
    scenery: Any,
    err: Exception,
) -> None:
    """Handle one failed companion attach through terrain-owned policy."""
    _warn_scenery_companion_attach_failed_once(env_cfg, scenery, err)


def load_terrain_companion_defaults(
    site: str | None,
    *,
    load_manifest: Callable[[], Any] | None = None,
) -> TerrainCompanionDefaults:
    """Return manifest companion defaults for a curated terrain site."""

    if site is None:
        return NO_TERRAIN_COMPANIONS

    if load_manifest is None:
        from srb.terrain.manifest_loader import load_manifest as _load_manifest

        load_manifest = _load_manifest

    try:
        manifest = load_manifest()
    except Exception as exc:
        failure = _classify_companion_manifest_failure(exc)
        if (
            not failure.fallback_allowed
            or failure.category not in FALLBACK_ALLOWED_CATEGORIES
        ):
            raise
        return TerrainCompanionDefaults(
            health_event=_companion_manifest_health_event(site=site, failure=failure)
        )

    site_entry = manifest.sites.get(site)
    if site_entry is None:
        return NO_TERRAIN_COMPANIONS

    return TerrainCompanionDefaults(
        sky=site_entry.sky,
        lighting=site_entry.lighting,
        rocks=site_entry.rocks,
        augment_preset=site_entry.augment_preset,
    )


def plan_real_terrain_companions(
    *,
    body: Body,
    site: str | None,
    sky: str | None,
    lighting: str | None,
    rocks: str | None,
    augment: TerrainAugmentMode | str,
    augment_preset: str | None,
    load_manifest: Callable[[], Any] | None = None,
) -> TerrainCompanionPlan:
    """Resolve the companion names a real-terrain scene should build."""

    if augment not in ("none", "simforge", "database"):
        raise ValueError(
            f"augment must be 'none', 'simforge', or 'database', got {augment!r}"
        )

    defaults = load_terrain_companion_defaults(
        site,
        load_manifest=load_manifest,
    )
    return TerrainCompanionPlan(
        sky=resolve_companion_name(sky, defaults.sky),
        lighting=resolve_companion_name(lighting, defaults.lighting),
        rocks=resolve_companion_name(rocks, defaults.rocks),
        health_event=defaults.health_event,
        augment_preset=_resolve_augment_preset(
            body=body,
            augment=augment,
            explicit_preset=augment_preset,
            manifest_default=defaults.augment_preset,
        ),
    )


def build_real_terrain_companions(
    *,
    plan: TerrainCompanionPlan,
    body: Body,
    base_usd: str,
    seed: int,
    augment: TerrainAugmentMode | str = "simforge",
    augment_max_rocks: int = 256,
    build_skydome: Callable[[str], Any] | None = None,
    build_lighting: Callable[[str], Any] | None = None,
    build_rocks: Callable[[str], list[Any]] | None = None,
    get_preset: Callable[[str], Any] | None = None,
    build_augment: Callable[..., Any] | None = None,
    build_database_rocks: Callable[..., list[Any]] | None = None,
    load_augmentation_db: Callable[[Any], Any] | None = None,
    make_asset_cfg: Callable[..., Any] | None = None,
) -> dict[str, Any]:
    """Build planned companions; the selected augment mode controls rock scatter.

    ``simforge`` uses SimForge placement; ``database`` uses baked transforms.
    Missing database rocks warn and produce no rock companion.
    """

    out: dict[str, Any] = {}
    if plan.health_event is not None:
        out[_COMPANION_HEALTH_EVENTS_KEY] = (plan.health_event,)

    if plan.sky:
        if build_skydome is None:
            build_skydome = _build_skydome_asset_cfg
        out["sky"] = build_skydome(plan.sky)

    if plan.lighting:
        if build_lighting is None:
            build_lighting = _build_lighting_asset_cfg
        out["lighting"] = build_lighting(plan.lighting)

    if plan.rocks:
        if build_rocks is None:
            build_rocks = _build_rocks_asset_cfgs
        out["rocks"] = build_rocks(plan.rocks)

    if plan.augment_preset:
        if get_preset is None:
            from srb.terrain.augmentation.presets import get_augment_preset

            get_preset = get_augment_preset

        preset = get_preset(plan.augment_preset)
        # Injected presets may omit `.body`; production presets provide it.
        preset_body = getattr(preset, "body", None)
        if preset_body is not None and preset_body != body:
            raise ValueError(
                f"augment_preset={plan.augment_preset!r} is a "
                f"{preset_body!r} preset, which does not match this "
                f"scene's body={body!r}"
            )

        if augment == "database":
            if load_augmentation_db is None:
                from srb.terrain.augmentation.database import (
                    load_augmentation_db as _load_augmentation_db,
                )

                load_augmentation_db = _load_augmentation_db
            if build_database_rocks is None:
                from srb.terrain.augmentation.database_scatter import (
                    build_database_rock_cfgs,
                )

                build_database_rocks = build_database_rock_cfgs

            baked_dir = Path(base_usd).parent
            db = load_augmentation_db(baked_dir)
            if db is None:
                raise UnsupportedCapabilityError(
                    code="terrain.augmentation.database_absent",
                    message=(
                        'augment="database" requires a baked augmentation '
                        f"database at {baked_dir}"
                    ),
                    component="terrain.augmentation",
                    details={"baked_dir": str(baked_dir), "body": body},
                    fallback_allowed=False,
                    evidence_valid=False,
                )
            if db.rocks.shape[0] == 0:
                raise UnsupportedCapabilityError(
                    code="terrain.augmentation.database_has_no_rocks",
                    message=(
                        'augment="database" requires an augmentation database '
                        "containing at least one rock"
                    ),
                    component="terrain.augmentation",
                    details={"baked_dir": str(baked_dir), "body": body},
                    fallback_allowed=False,
                    evidence_valid=False,
                )
            db_rock_cfgs = build_database_rocks(
                db=db,
                base_usd=base_usd,
                body=body,
                preset=preset,
                max_rocks=augment_max_rocks,
            )
            if db_rock_cfgs:
                out["augment_rocks_db"] = db_rock_cfgs
        else:
            if build_augment is None:
                from srb.terrain.augmentation.simforge_ext import (
                    build_real_terrain_augment_cfg,
                )

                build_augment = build_real_terrain_augment_cfg
            if make_asset_cfg is None:
                make_asset_cfg = _make_asset_base_cfg

            rock_scatter_spawn = build_augment(
                base_usd=base_usd,
                body=body,
                preset=preset,
                seed_offset=seed,
            )
            # A zero-density preset (e.g. "earth_default") short-circuits
            # rock generator construction in
            # `build_real_terrain_augment_cfg`, which returns None -- no
            # `augment_rocks` companion to attach.
            if rock_scatter_spawn is not None:
                out["augment_rocks"] = make_asset_cfg(
                    prim_path="{ENV_REGEX_NS}/augment_rocks",
                    spawn=rock_scatter_spawn,
                )

    return out


def build_real_terrain_companion_assets(
    *,
    body: Body,
    site: str | None,
    sky: str | None,
    lighting: str | None,
    rocks: str | None,
    augment: TerrainAugmentMode | str,
    augment_preset: str | None,
    baked_usd: str,
    seed: int,
    augment_max_rocks: int = 256,
    get_preset: Callable[[str], Any] | None = None,
    build_augment: Callable[..., Any] | None = None,
    load_manifest: Callable[[], Any] | None = None,
    plan_companions: Callable[..., TerrainCompanionPlan] = plan_real_terrain_companions,
    build_companions: Callable[..., dict[str, Any]] = build_real_terrain_companions,
) -> dict[str, Any]:
    """Plan and build companion asset cfgs from real-terrain runtime fields."""

    plan = plan_companions(
        body=body,
        site=site,
        sky=sky,
        lighting=lighting,
        rocks=rocks,
        augment=augment,
        augment_preset=augment_preset,
        load_manifest=load_manifest,
    )

    # The ephemeris sentinel uses the site's manifest SPICE data.
    build_lighting: Callable[[str], Any] | None = None
    from srb.terrain.lights.ephemeris_bridge import (
        EPHEMERIS_LIGHTING,
        make_ephemeris_build_lighting,
    )

    if plan.lighting == EPHEMERIS_LIGHTING:
        build_lighting = make_ephemeris_build_lighting(
            body=body,
            site=site,
            baked_usd=baked_usd,
            load_manifest=load_manifest,
        )
        if build_lighting is None:
            # Fail clearly if an override requests ephemeris without site data.
            raise ValueError(
                f"lighting={EPHEMERIS_LIGHTING!r} was requested for site {site!r} "
                "but it is not ephemeris-configured (needs a manifest site with "
                "lighting='ephemeris' and an 'epoch'); cannot drive ephemeris "
                "lighting"
            )

    base_usd = baked_usd if plan.augment_preset else ""
    return build_companions(
        plan=plan,
        body=body,
        base_usd=base_usd,
        seed=seed,
        augment=augment,
        augment_max_rocks=augment_max_rocks,
        build_lighting=build_lighting,
        get_preset=get_preset,
        build_augment=build_augment,
    )


@dataclass(frozen=True, slots=True)
class CompanionSuppression:
    """One companion product suppressed by a winning environment knob."""

    companion: str
    """Companion key that was suppressed (``"lighting"`` or ``"sky"``)."""
    winner: str
    """The env knob that owns the contested scene element."""
    loser: str
    """The companion knob whose product was suppressed."""


@dataclass(frozen=True, slots=True)
class CompanionArbitration:
    """The full precedence decision for one companion set."""

    attach_keys: tuple[str, ...]
    null_scene_sunlight: bool
    null_skydome_randomizer: bool
    warn_sunlight_randomizer: bool
    suppressions: tuple[CompanionSuppression, ...]


def arbitrate_companion_scene(
    *,
    companion_keys: Any,
    ephemeris_enabled: bool = False,
    ephemeris_drive_sunlight: bool = False,
    ephemeris_drive_skydome: bool = False,
    env_skydome_override: str | None = None,
    env_skydome_disabled: bool = False,
    sunlight_randomizer_configured: bool = False,
) -> CompanionArbitration:
    """Pure precedence decision; reports every suppressed companion.

    Sun ownership requires enabled ephemeris and ``drive_sunlight``; sky requires
    ``drive_skydome``. Companion sky replaces the default dome unless an explicit
    override or disable wins. Other keys always attach.
    """

    ephemeris_owns_sun = ephemeris_enabled and ephemeris_drive_sunlight
    ephemeris_owns_dome = ephemeris_enabled and ephemeris_drive_skydome

    attach: list[str] = []
    suppressions: list[CompanionSuppression] = []
    null_scene_sunlight = False
    null_skydome_randomizer = False
    warn_sunlight_randomizer = False

    for key in companion_keys:
        if key == "lighting":
            if ephemeris_owns_sun:
                suppressions.append(
                    CompanionSuppression(
                        companion="lighting",
                        winner="env.ephemeris.drive_sunlight (enabled)",
                        loser="the real-terrain companion lighting product "
                        "(env.scenery.lighting)",
                    )
                )
                continue
            null_scene_sunlight = True
            warn_sunlight_randomizer = sunlight_randomizer_configured
        elif key == "sky":
            if ephemeris_owns_dome:
                suppressions.append(
                    CompanionSuppression(
                        companion="sky",
                        winner="env.ephemeris.drive_skydome (enabled)",
                        loser="the real-terrain companion sky product "
                        "(env.scenery.sky)",
                    )
                )
                continue
            if env_skydome_disabled:
                suppressions.append(
                    CompanionSuppression(
                        companion="sky",
                        winner="env.skydome=None/False (skydome explicitly disabled)",
                        loser="the real-terrain companion sky product "
                        "(env.scenery.sky)",
                    )
                )
                continue
            if env_skydome_override is not None:
                suppressions.append(
                    CompanionSuppression(
                        companion="sky",
                        winner=f"env.skydome={env_skydome_override!r} "
                        "(explicit non-default skydome)",
                        loser="the real-terrain companion sky product "
                        "(env.scenery.sky)",
                    )
                )
                continue
            null_skydome_randomizer = True
        attach.append(key)

    return CompanionArbitration(
        attach_keys=tuple(attach),
        null_scene_sunlight=null_scene_sunlight,
        null_skydome_randomizer=null_skydome_randomizer,
        warn_sunlight_randomizer=warn_sunlight_randomizer,
        suppressions=tuple(suppressions),
    )


_EMITTED_COMPANION_ARBITRATION_WARNINGS: set[tuple[str, str]] = set()
"""Suppression warnings already emitted during this process."""

_SUNLIGHT_RANDOMIZER_TERMS: tuple[str, ...] = (
    "randomize_sunlight_orientation",
    "randomize_sunlight_intensity",
    "randomize_sunlight_angular_diameter",
    "randomize_sunlight_color_temperature",
)


def _companion_context_label(context: str | None) -> str:
    return context if context else "<unnamed real-terrain scenery>"


def _warn_companion_suppressed_once(
    suppression: CompanionSuppression, context: str | None = None
) -> None:
    label = _companion_context_label(context)
    key = ("suppressed", f"{label}:{suppression.companion}:{suppression.winner}")
    if key in _EMITTED_COMPANION_ARBITRATION_WARNINGS:
        return
    _EMITTED_COMPANION_ARBITRATION_WARNINGS.add(key)

    from srb.utils import logging

    logging.warning(
        "[terrain %s] %s owns the scene %s -- suppressing %s. Disable the "
        "winning knob to use the companion product instead.",
        label,
        suppression.winner,
        "sun" if suppression.companion == "lighting" else "skydome",
        suppression.loser,
    )


def _warn_companion_owns_sunlight_once(context: str | None = None) -> None:
    label = _companion_context_label(context)
    key = ("owns_sunlight", label)
    if key in _EMITTED_COMPANION_ARBITRATION_WARNINGS:
        return
    _EMITTED_COMPANION_ARBITRATION_WARNINGS.add(key)

    from srb.utils import logging

    logging.warning(
        "[terrain %s] companion lighting (env.scenery.lighting) owns the sun: "
        "scene.sunlight=None and events.randomize_sunlight_* are skipped. "
        "Set env.scenery.lighting=None to keep the default sun and randomizers.",
        label,
    )


_SKYDOME_UNSET = object()
"""Distinguish an absent skydome setting from an explicit disable."""


def attach_terrain_companions_arbitrated(
    env_cfg: Any,
    companions: Mapping[str, Any],
    *,
    context: str | None = None,
) -> CompanionArbitration:
    """Attach surviving companions and apply scene ownership rules.

    Missing knobs are treated as unconfigured. Suppressions warn once per
    context; empty companion mappings are a silent no-op.
    """

    health_events_value = companions.get(_COMPANION_HEALTH_EVENTS_KEY, ())
    if not isinstance(health_events_value, tuple) or not all(
        isinstance(event, HealthEvent) for event in health_events_value
    ):
        raise TypeError(
            f"{_COMPANION_HEALTH_EVENTS_KEY} must be a tuple of HealthEvent"
        )
    for health_event in health_events_value:
        current_events = tuple(getattr(env_cfg, "terrain_health_events", ()))
        if health_event not in current_events:
            env_cfg.terrain_health_events = (*current_events, health_event)

    visible_companions = {
        key: value
        for key, value in companions.items()
        if key != _COMPANION_HEALTH_EVENTS_KEY
    }
    if not visible_companions:
        return CompanionArbitration(
            attach_keys=(),
            null_scene_sunlight=False,
            null_skydome_randomizer=False,
            warn_sunlight_randomizer=False,
            suppressions=(),
        )

    ephemeris = getattr(env_cfg, "ephemeris", None)
    events = getattr(env_cfg, "events", None)
    skydome_setting = getattr(env_cfg, "skydome", _SKYDOME_UNSET)

    env_skydome_override = (
        skydome_setting
        if isinstance(skydome_setting, str) and skydome_setting != "low_res"
        else None
    )
    # Missing means no operator intent; only an explicit falsy value disables.
    # The config cannot distinguish an explicit default from its own default.
    env_skydome_disabled = skydome_setting is not _SKYDOME_UNSET and not bool(
        skydome_setting
    )
    sunlight_randomizer_configured = events is not None and any(
        getattr(events, term, None) is not None for term in _SUNLIGHT_RANDOMIZER_TERMS
    )

    decision = arbitrate_companion_scene(
        companion_keys=visible_companions.keys(),
        ephemeris_enabled=bool(ephemeris is not None and ephemeris.enabled),
        ephemeris_drive_sunlight=bool(
            ephemeris is not None and ephemeris.drive_sunlight
        ),
        ephemeris_drive_skydome=bool(ephemeris is not None and ephemeris.drive_skydome),
        env_skydome_override=env_skydome_override,
        env_skydome_disabled=env_skydome_disabled,
        sunlight_randomizer_configured=sunlight_randomizer_configured,
    )

    for suppression in decision.suppressions:
        _warn_companion_suppressed_once(suppression, context)
    if decision.warn_sunlight_randomizer:
        _warn_companion_owns_sunlight_once(context)

    attach_terrain_companions(
        env_cfg, {key: visible_companions[key] for key in decision.attach_keys}
    )
    if decision.null_scene_sunlight:
        env_cfg.scene.sunlight = None
    if decision.null_skydome_randomizer and events is not None:
        events.randomize_skydome_orientation = None

    return decision


def attach_terrain_companions(
    env_cfg: Any,
    companions: Mapping[str, Any],
) -> None:
    """Register companion cfgs onto ``env_cfg.scene``."""

    if not companions:
        return None

    scene = env_cfg.scene
    if "sky" in companions:
        setattr(scene, "skydome", companions["sky"])
    if "lighting" in companions:
        setattr(scene, "lighting", companions["lighting"])
    if "rocks" in companions:
        for i, cfg in enumerate(companions["rocks"]):
            setattr(scene, f"rocks_{i:02d}", cfg)
    if "augment_rocks" in companions:
        setattr(scene, "augment_rocks", companions["augment_rocks"])
    if "augment_rocks_db" in companions:
        for i, cfg in enumerate(companions["augment_rocks_db"]):
            setattr(scene, f"augment_rocks_db_{i:03d}", cfg)

    return None


def _resolve_augment_preset(
    *,
    body: Body,
    augment: TerrainAugmentMode | str,
    explicit_preset: str | None,
    manifest_default: str | None,
) -> str | None:
    if augment not in ("simforge", "database"):
        return None
    if explicit_preset is not None:
        return explicit_preset
    return manifest_default or default_augment_preset_for_body(body)


def _build_skydome_asset_cfg(name: str) -> Any:
    from srb.terrain.hdri.runtime import RealSkydome

    return RealSkydome.build_asset_cfg(name)


def _build_lighting_asset_cfg(name: str) -> Any:
    from srb.terrain.lights.runtime import RealLighting

    return RealLighting.build_asset_cfg(name)


def _build_rocks_asset_cfgs(name: str) -> list[Any]:
    from srb.terrain.rocks.runtime import RealRockSet

    return RealRockSet.build_asset_cfgs(name)


def _make_asset_base_cfg(*, prim_path: str, spawn: Any) -> Any:
    from srb.core.asset import AssetBaseCfg

    return AssetBaseCfg(
        prim_path=prim_path,
        spawn=spawn,
    )


__all__ = [
    "NO_TERRAIN_COMPANIONS",
    "CompanionArbitration",
    "CompanionSuppression",
    "TerrainCompanionDefaults",
    "TerrainCompanionPlan",
    "arbitrate_companion_scene",
    "attach_terrain_companions",
    "attach_terrain_companions_arbitrated",
    "build_real_terrain_companion_assets",
    "build_real_terrain_companions",
    "default_augment_preset_for_body",
    "disabled_companion_kwargs",
    "handle_companion_attach_failure",
    "load_terrain_companion_defaults",
    "plan_real_terrain_companions",
    "resolve_companion_name",
    "resolve_effective_augment_preset",
]
