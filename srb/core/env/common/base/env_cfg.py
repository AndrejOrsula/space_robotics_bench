"""Shared configuration for direct, managed, and multi-agent environments."""

import math
from dataclasses import MISSING, field
from os import environ
from pathlib import Path
from types import MethodType
from typing import (
    Any,
    Callable,
    Iterable,
    Literal,
    Mapping,
    Sequence,
)

import torch
from simforge import BakeType

from srb.core.action import (
    ActionGroup,
    ActionTermCfg,
    DifferentialInverseKinematicsActionCfg,
    OperationalSpaceControllerActionCfg,
)
from srb.core.action.action_group import (
    _renamed_command_mapping_getter,
    _snapshot_action_group_leaf,
)
from srb.core.action.term import DummyActionCfg
from srb.core.action.term.manipulation._frame_jacobian import (
    bind_controller_to_end_effector_frame,
)
from srb.core.asset import (
    ActivePayload,
    ActiveTool,
    ArticulationCfg,
    Asset,
    AssetBaseCfg,
    AssetVariant,
    CombinedMobileManipulator,
    Manipulator,
    MobileRobot,
    RigidObjectCfg,
    RigidObjectCollectionCfg,
    Robot,
    Scenery,
    Tool,
    Transform,
)
from srb.core.asset.particle_system import ParticleSystemCfg
from srb.core.command_mapping import (
    CommandDestination,
    CommandMapping,
    EventSign,
)
from srb.core.domain import Domain
from srb.core.env.wrapper_cfg import ActionSmoothingCfg, ObservationStackingCfg
from srb.core.ephemeris import EphemerisCfg, SiteSpec
from srb.core.ephemeris.env_support import (
    DOMAIN_TO_BODY,  # noqa: F401
    TerrainEphemerisFacts,
    check_ephemeris_precedence,
    resolve_orbit_site,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import VisualizationMarkersCfg
from srb.core.mdp import (
    reset_xform_orientation_uniform,
    settle_and_reset_particle_systems,
)
from srb.core.particle_layout import particle_pitch_m
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.sensor import ImuCfg, SensorBaseCfg
from srb.core.sensor.category import SensorCategory
from srb.core.sim import (
    CraterPileParticlesSpawnerCfg,
    DistantLightCfg,  # noqa: F401
    DomeLightCfg,
    GridParticlesSpawnerCfg,
    MoundPileParticlesSpawnerCfg,
    MultiAssetSpawnerCfg,
    ParticlesSpawnerCfg,
    PyramidParticlesSpawnerCfg,
    RigidBodyMaterialCfg,
    SimulationCfg,
    SpawnerCfg,
    TerrainGeneratorCfg,
    TerrainImporterCfg,
    VisualMaterialCfg,
)
from srb.core.sim.robot_setup import RobotAssemblerCfg
from srb.core.visuals import VisualsCfg
from srb.utils._variant_resolver import (
    get_field_annotation,
    resolve_variant,
)
from srb.utils.cfg import configclass
from srb.utils.config_introspection import (
    iter_config_items_by_type,
    iter_config_values,
    iter_config_values_by_type,
)
from srb.utils.failures import HealthEvent, InvalidInputError
from srb.utils.math import combine_frame_transforms_tuple, rpy_to_quat  # noqa: F401
from srb.utils.path import (
    SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES,
    SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES,
    SRB_LOGS_DIR,
)
from srb.utils.rates import resolve_decimation
from srb.utils.str import sanitize_action_term_name

from .ephemeris_lighting import (
    _EMITTED_EPHEMERIS_BELOW_HORIZON_WARNINGS,  # noqa: F401
    _EMITTED_EPHEMERIS_KERNELS_MISSING_WARNINGS,  # noqa: F401
    _EMITTED_EPHEMERIS_TERRAIN_BODY_MISMATCH_WARNINGS,  # noqa: F401
    SiteResolvers,
    SunlightBuildRequest,
    SurfaceSiteResolver,
    _resolve_surface_site,
    _warn_ephemeris_terrain_body_mismatch_once,  # noqa: F401
)
from .ephemeris_lighting import (
    build_ephemeris_sunlight_asset as _build_ephemeris_sunlight_asset_impl,
)
from .ephemeris_lighting import (
    build_sunlight_asset as _build_sunlight_asset_impl,
)
from .ephemeris_lighting import (
    resolve_ephemeris_site as _resolve_ephemeris_site_impl,
)
from .event_cfg import BaseEventCfg
from .scene_cfg import BaseSceneCfg


def resolve_ephemeris_site(
    env_cfg: Any,
    *,
    orbit_mode: bool | None = None,
    resolve_orbit_site_fn: Callable[[SiteSpec | None], SiteSpec] | None = None,
    # Adapt the historical BaseEventCfg keyword callback to SiteResolvers.
    resolve_site_fn: Callable[..., SiteSpec] | None = None,
):
    is_orbit = str(env_cfg.domain) == "orbit" if orbit_mode is None else orbit_mode
    surface_resolver: SurfaceSiteResolver = _resolve_surface_site
    if resolve_site_fn is not None:

        def _legacy_surface_resolver(
            ephemeris_site: SiteSpec | None,
            domain_str: str,
            terrain_site_coords: tuple[float, float] | None,
            terrain_north_convergence_deg: float,
        ) -> SiteSpec:
            return resolve_site_fn(
                ephemeris_site=ephemeris_site,
                domain_str=domain_str,
                terrain_site_coords=terrain_site_coords,
                terrain_north_convergence_deg=terrain_north_convergence_deg,
            )

        surface_resolver = _legacy_surface_resolver

    return _resolve_ephemeris_site_impl(
        env_cfg,
        orbit_mode=orbit_mode,
        terrain_facts=None if is_orbit else _terrain_ephemeris_facts(env_cfg),
        site_resolvers=SiteResolvers(
            orbit=(
                resolve_orbit_site
                if resolve_orbit_site_fn is None
                else resolve_orbit_site_fn
            ),
            surface=surface_resolver,
        ),
    )


def _terrain_ephemeris_facts(env_cfg: Any) -> TerrainEphemerisFacts | None:
    """Return ephemeris facts exposed by the resolved scenery, if available."""
    scenery = getattr(env_cfg, "_scenery", None)
    provider = getattr(scenery, "ephemeris_site_facts", None)
    if provider is None:
        return None
    facts = provider() if callable(provider) else provider
    if facts is None:
        return None
    if not isinstance(facts, TerrainEphemerisFacts):
        raise TypeError(
            "scenery.ephemeris_site_facts must return TerrainEphemerisFacts"
        )
    return facts


def _build_sunlight_asset(
    env_cfg: Any, *, prim_path: str = "/World/sunlight", **kwargs: Any
):
    return _build_sunlight_asset_impl(
        env_cfg,
        request=SunlightBuildRequest(prim_path=prim_path, spawn_kwargs=kwargs),
    )


def _build_ephemeris_sunlight_asset(
    env_cfg: Any, *, prim_path: str = "/World/sunlight", **kwargs: Any
):
    return _build_ephemeris_sunlight_asset_impl(
        env_cfg,
        request=SunlightBuildRequest(prim_path=prim_path, spawn_kwargs=kwargs),
        terrain_facts=(
            None
            if str(env_cfg.domain) == "orbit"
            else _terrain_ephemeris_facts(env_cfg)
        ),
    )


@configclass
class BaseEnvCfg:
    seed: int = 0
    domain: Domain = Domain.MOON
    gravity: Domain | str | None = None
    skydome: Literal["low_res", "high_res"] | bool | None = "low_res"
    scenery_flat_area_size: float | None = None
    """Override the procedural scenery's central flattened-area size (SI meters).

    ``None`` (default) keeps the value derived from env spacing. For multi-robot
    renders set this large enough to cover the whole robot/particle footprint so
    everything rests on one flat surface (e.g. ``16``) instead of scattering over
    terrain relief. Only affects ``AssetVariant`` (procedural) scenery.
    """
    sunlight_rpy: tuple[float, float, float] | None = None
    """Fixed sun orientation as ``(roll, pitch, yaw)`` Euler angles in *degrees*.

    ``None`` (default) keeps the built-in orientation (``45, 30, 0``). When set,
    the distant sunlight uses this orientation and the per-interval sunlight
    orientation randomization is disabled, so the lighting is deterministic for
    renders. Useful to front-light robots for a snapshot (e.g. flip the azimuth
    180° from the camera-facing default so shadows fall away from the camera).
    """
    ephemeris: EphemerisCfg = field(default_factory=EphemerisCfg)
    """Opt-in real-ephemeris (SPICE) configuration for the environment sun.

    ``enabled=False`` (default) leaves the built-in domain-default sun/skydome
    behavior (including ``sunlight_rpy``) untouched. When ``enabled=True``,
    the distant sunlight is instead driven by a real SPICE ephemeris query at
    ``epoch`` (a fixed UTC datetime, or one sampled per reset from a
    ``utc_range``) -- mutually exclusive with an explicit ``sunlight_rpy``
    (setting both raises). ``on_missing`` controls the behavior when SPICE
    kernels are not cached locally: ``"fallback"`` (default) warns once and
    keeps the domain-default lighting, ``"error"`` raises. ``drive_skydome``
    (requires ``drive_sunlight``; inert in ``ORBIT``) yaws the skydome to follow
    the Sun and disables random skydome yaw.
    """
    terrain_health_events: tuple[HealthEvent, ...] = ()
    """Canonical degraded-health events emitted by terrain fallbacks."""

    scenery: Scenery | AssetVariant | None = AssetVariant.PROCEDURAL
    _scenery: Scenery | None = MISSING  # type: ignore[assignment]
    robot: Robot | AssetVariant = AssetVariant.DATASET
    _robot: Robot = MISSING  # type: ignore[assignment]
    extra_robots: dict[str, Robot | AssetVariant] = {}
    _robots: dict[str, Robot] = {}
    n_procgen_variants: int | None = None

    joint_assemblies: dict[str, RobotAssemblerCfg] = {}
    assemble_rigid_end_effector: bool = False

    scene: BaseSceneCfg = BaseSceneCfg()
    stack: bool = False
    num_envs: int | None = None
    spacing: float | None = None

    events: BaseEventCfg = BaseEventCfg()

    env_rate: float = MISSING  # type: ignore[assignment]
    agent_rate: float = MISSING  # type: ignore[assignment]
    sim = SimulationCfg(
        dt=MISSING,  # type: ignore[arg-type]
        render_interval=MISSING,  # type: ignore[arg-type]
        gravity=MISSING,  # type: ignore[arg-type]
        # CPU is the default; Fabric transform reads are unsafe after rendering.
        device="cpu",
        # PhysX tuning is applied after Kit boots; its module is unavailable here.
        physics_material=RigidBodyMaterialCfg(
            # Shared baseline; average combine preserves asset-specific values.
            static_friction=0.5,
            dynamic_friction=0.5,
            # Keep zero baseline but average with assets so their restitution
            # remains effective.
            restitution=0.0,
            friction_combine_mode="average",
            restitution_combine_mode="average",
        ),
    )
    malloc_scale: float = 1.0

    visuals: VisualsCfg = VisualsCfg()

    truncate_episodes: bool = True
    extras: bool = False
    debug_vis: bool = (
        environ.get("DEBUG_VIS") or environ.get("SRB_DEBUG_VIS", "false")
    ).lower() in ("true", "1")

    hud: bool = False
    hud_title: str = "Space Robotics Bench"
    hud_update_interval_s: float = 0.1
    hud_max_lines: int = 8
    hud_show_reward: bool = True
    hud_show_time: bool = True
    hud_show_reset_hint: bool = True
    hud_reset_hint: str = "SpaceMouse L / keyboard L"

    particles: bool = False
    particles_systems: int = 1
    particles_spawner: str | Literal["grid", "pyramid", "crater_pile", "mound_pile"] = (
        "grid"
    )
    particles_size: float | tuple[float, float] = (0.01, 0.005)
    particles_area: float | tuple[float, float] = (0.75, 0.5)
    particles_ratio: float | tuple[float, float] = (0.025, 0.1)
    particles_height: int | tuple[int, int] = (1, 10)
    particles_density: float | tuple[float, float] = 1600.0
    particles_friction: float | tuple[float, float] = 1.0
    particles_damping: float | tuple[float, float] = 0.2
    particles_cohesion: float | tuple[float, float] = 0.01
    particles_adhesion: float | tuple[float, float] = 0.05
    particles_material: VisualMaterialCfg | None = None
    particles_dim_xy: int | tuple[int, int] | None = None
    """Explicit ``(dim_x = dim_y)`` grid count per particle system.

    ``None`` (default) → derived from ``particles_area * spacing / particles_size``
    and, if a scenery is attached, optionally overridden by the scenery to match its
    footprint (facility/terrain). A scalar broadcasts to all systems; a 2-tuple
    ``(min, max)`` interpolates across ``particles_systems``.

    Setting a non-``None`` value also suppresses scenery-specific ``dim_{x,y}``
    overrides — useful when the task wants an absolute pile geometry independent
    of the surrounding scenery.
    """
    particles_init_pos: tuple[float, float, float] | None = None
    """Explicit ``(x, y, z)`` init position (SI meters) for the first particle system.

    ``None`` (default) → placeholder ``(0.0, 0.0, 1.0)`` that any attached scenery
    (facility/terrain) is expected to overwrite via ``setup_extras`` to match the
    scenery footprint. Multi-system stacking in Z still runs for ``i > 0`` on top
    of whichever base position is used.

    Setting a non-``None`` value suppresses scenery-specific ``init_state`` overrides,
    letting the task place the pile at an absolute world position.
    """
    particles_freeze: bool = False
    """Freeze particle systems after they settle (e.g. for clean renders).

    When ``True``, each particle system is disabled (its PhysX
    ``particleSystemEnabled`` flag is cleared and velocities are zeroed) right
    after the initial settle completes. The settled point geometry remains in
    the stage and keeps rendering, but the solver stops simulating it so the
    particles stay perfectly still. Env-independent; works for any env that
    spawns particles.
    """
    particles_conform_to_terrain: bool = False
    """Snap particles onto the terrain surface (mesh height field) instead of PBD settling.

    Deterministic alternative to letting the spawned grid settle under PBD
    physics. When ``True``, the terrain *mesh* vertices are binned into a 2D max-height grid and each
    particle's Z is snapped to the terrain height at its spawn ``(x, y)`` — so
    the regolith conforms to the terrain relief with no physics simulated. (A
    downward scene-query raycast is deliberately *not* used: the collider is
    absent from the query scene at reset-event time.) Pair with
    :attr:`particles_freeze` for a static render bed.
    """
    particles_conform_jitter: float = 0.0
    """Random XY scatter (SI meters) applied when conforming particles to terrain.

    Only used with :attr:`particles_conform_to_terrain`. ``0`` (default) keeps
    the regular spawn grid; a positive value randomly offsets each particle's
    ``(x, y)`` by up to this much before snapping it to the terrain height,
    breaking up the lattice for a more natural regolith scatter. Seeded by
    ``seed``.
    """
    particles_pile_center_jitter: float = 0.0
    """Per-reset random XY offset (SI meters) of the whole settled particle pile.

    Domain randomization of pile *location*: on every reset the cached settled
    pile is restored and rigidly translated in XY by an independent sample in
    ``[-jitter, +jitter]`` per environment (the pile keeps its settled shape, so
    the translation is physically valid without re-settling). ``0`` (default)
    disables it. Used e.g. by the excavation task's
    ``randomize_particle_pile_shape`` centre component. Height / cone-angle
    spawn-shape variation would additionally require re-settling; the settled
    angle of repose is instead varied through the PBD-material friction
    randomizer.
    """

    robot_spacing: float | None = None
    """Minimum center-to-center distance (SI meters) between robots.

    ``None`` (default) → no layout is applied (robots use their configured
    ``init_state`` / per-env randomization). When set to a positive value, the
    primary robot and every entry in :attr:`extra_robots` are scattered with
    Poisson-disk spacing (each pair at least ``robot_spacing`` apart) over the
    square XY region defined by :attr:`robot_spacing_area`. The sampled XY
    positions are baked into each robot's ``init_state`` (each robot's own
    spawn height is preserved), and any per-robot root-pose randomization is
    removed so the layout is not overwritten at reset. Env-independent and
    useful for multi-robot renders/scenes. Sampling is seeded by ``seed`` for
    reproducibility.
    """
    robot_spacing_area: float | None = None
    """Half-extent (SI meters) of the square XY region robots are scattered in.

    Only used for Poisson-disk sampling (i.e. when :attr:`robot_spacing` is set
    and :attr:`robot_positions` is unset). ``None`` (default) → derived from the
    robot count and ``robot_spacing``. If the region is too small to fit all
    robots at the requested spacing, it is grown automatically.
    """
    robot_positions: tuple[tuple[float, float], ...] | None = None
    """Explicit ``(x, y)`` placements for the robots, in layout order.

    ``None`` (default) → robots are scattered with Poisson-disk spacing (see
    :attr:`robot_spacing`). When set, the given XY positions are used verbatim
    instead of random sampling — order is ``["robot", *extra_robots]`` — so the
    scene can be composed by hand. Must provide exactly one entry per robot.
    Each robot's spawn height (plus :attr:`robot_spawn_height`) is preserved;
    combine with ``--snapshot-settle`` to drop them onto the terrain. Enables the
    layout pass even when :attr:`robot_spacing` is unset.
    """
    robot_spawn_height: float | None = None
    """Extra Z offset (SI meters) added to every laid-out robot's spawn height.

    Applied whenever the layout pass runs (i.e. :attr:`robot_spacing` *or*
    :attr:`robot_positions` is set). ``None``/``0`` (default) keeps each robot's
    configured spawn height. A positive value lifts every robot that high above
    its configured height so it spawns clear of uneven terrain; pair it with the
    ``--snapshot-settle`` CLI option to let the robots drop and settle onto the
    surface under gravity before a snapshot is taken (otherwise they remain
    hovering, since the layout is hard-placed at reset).
    """

    action_smoothing: ActionSmoothingCfg = ActionSmoothingCfg(enable=False)
    observation_stacking: ObservationStackingCfg = ObservationStackingCfg(enable=False)

    active_sensors: set[SensorCategory] = {
        SensorCategory.PROPRIOCEPTION,
        SensorCategory.CONTACT,
        SensorCategory.TRANSFORM,
    }

    env_sensor_cfgs: dict[str, SensorBaseCfg] = {}

    camera_resolution: tuple[int, int] | None = (64, 64)
    camera_update_period: float = -1.0
    camera_data_types: Sequence[str] | None = ("rgb", "depth")
    camera_clipping_range: tuple[float, float] | None = None
    """Near and far planes (SI meters) for VISUAL cameras without an authored range.

    Fills ``spawn.clipping_range`` only on cameras whose far plane is missing or
    a render default (``>= 1e3`` m, Isaac Lab's pinhole default is 1e6 m); a
    range authored on the camera is kept. ``None`` (default) leaves every camera
    unchanged. Size the far plane to the task: depth observations saturate
    beyond it and RGB does not render geometry past it. Orbital environments set
    task-sized values (``srb/core/env/mobile/orbital/env.py``).
    """
    camera_rerender_on_reset: bool = True
    """Re-render once after a reset when VISUAL or EVENT_CAMERA sensors are active.

    Without it the first camera row of every episode shows the pre-reset scene,
    and event cameras take that stale frame as their episode baseline. The extra
    render runs on every step on which
    any environment resets, so visual training with many environments may set
    ``False`` to trade first-frame freshness for render time. A positive
    ``num_rerenders_on_reset`` is kept either way.
    """

    # Task event cameras spawn with `events` in `active_sensors`.
    event_camera_preset: str = "imx636_nominal"
    event_camera_backend: str = "v2e"
    event_camera_sub_render_hz: float = 250.0
    event_camera_resolution: tuple[int, int] = (1280, 720)

    camera_record: bool = False
    camera_record_dir: Path | str = SRB_LOGS_DIR.joinpath("camera_videos")
    # Optional per-episode provenance for event-camera HDF5 and manifest output.
    event_provenance: dict[str, str] | None = None

    def __post_init__(self):
        """Apply configuration phases, then materialize derived scene entries."""
        self._coerce_scenario()
        self._coerce_active_sensors()
        self._configure_scene_extent()
        self._validate_ephemeris_precedence()
        self._build_scene_assets()
        self._configure_simulation()
        self._apply_asset_and_particle_extras()
        self.finalize()

    def finalize(self):
        """Materialize deferred, add-only scene entries from the final config."""
        # Rebuild bindings so repeated finalization reflects current sensors.
        setattr(self, "_event_camera_bindings", {})
        for robot_name, robot in getattr(self, "_robots", {}).items():
            self._add_sensors(robot, robot_name=robot_name)
        # Re-render after reset for active cameras; preserve positive overrides.
        if hasattr(self, "num_rerenders_on_reset"):
            from srb.core.sensor.camera_defaults import (
                num_rerenders_on_reset_for_cameras,
            )
            from srb.core.sensor.category import SensorCategory

            self.num_rerenders_on_reset = num_rerenders_on_reset_for_cameras(
                int(self.num_rerenders_on_reset),
                visual_active=self.camera_rerender_on_reset
                and bool(
                    {SensorCategory.VISUAL, SensorCategory.EVENT_CAMERA}
                    & set(self.active_sensors or ())
                ),
            )
        # Must follow the wiring above: it is what applies `debug_vis` and the
        # marker-scale cap to sensors, including any a subclass added late.
        self._update_debug_vis()

    def _coerce_scenario(self):
        if not isinstance(self.gravity, Domain) and isinstance(self.gravity, str):
            _gravity = Domain.from_str(self.gravity)
            if _gravity is None:
                raise InvalidInputError(
                    f"Gravity '{self.gravity}' must be a valid member of {{{Domain.__name__}: {' | '.join(Domain.__members__.keys())}}}"
                )
            self.gravity = _gravity

    def _coerce_active_sensors(self):
        if self.active_sensors:
            from srb.core.sensor.category import SensorCategory

            _coerced_sensors = set()
            for _cat in self.active_sensors:
                if isinstance(_cat, SensorCategory):
                    _coerced_sensors.add(_cat)
                    continue
                _cat_str = str(_cat)
                try:
                    _coerced_sensors.add(SensorCategory(_cat_str))
                except ValueError:
                    try:
                        _coerced_sensors.add(SensorCategory[_cat_str.upper()])
                    except KeyError as exc:
                        raise InvalidInputError(
                            f"Invalid sensor category {_cat!r}; valid values: "
                            f"{[c.value for c in SensorCategory]} (or `events`)",
                            component="srb.env_cfg",
                        ) from exc
            self.active_sensors = _coerced_sensors

            from srb.utils import logging

            for _required in (SensorCategory.PROPRIOCEPTION, SensorCategory.CONTACT):
                if _required not in self.active_sensors:
                    logging.warning(
                        "active_sensors omits %s; tasks reading it (IMU for "
                        "PROPRIOCEPTION, contact forces for CONTACT) fail at the "
                        "first step. Re-add it if needed.",
                        _required.value,
                    )

    def _configure_scene_extent(self):
        if self.num_envs is not None:
            self.scene.num_envs = self.num_envs
        if self.spacing is None:
            self.spacing = self.scene.env_spacing
        if self.spacing is None or isinstance(self.spacing, type(MISSING)):
            raise ValueError(
                "env spacing is unset: set env.spacing or scene.env_spacing to a "
                "positive value (leaving both unset lets MISSING flow into scene "
                "geometry arithmetic and fail far from the cause)"
            )
        if not self.spacing > 0.0:
            raise InvalidInputError(
                f"env spacing must be positive, got {self.spacing!r}",
                component="srb.env_cfg",
            )

        if self.stack and self.particles and self.scene.num_envs != 1:
            from srb.utils import logging

            logging.warning(
                "Particles are not supported in stacked multi-envs; forcing env.stack=False for num_envs=%d.",
                self.scene.num_envs,
            )
            self.stack = False

        self.scene.env_spacing = 0.0 if self.stack else self.spacing

    def _validate_ephemeris_precedence(self):
        if self.ephemeris.enabled:
            self.ephemeris.validate()
            check_ephemeris_precedence(
                ephemeris_enabled=True,
                sunlight_rpy_set=self.sunlight_rpy is not None,
                domain_str=str(self.domain),
                ephemeris_site=self.ephemeris.site,
                orbit_mode=self.ephemeris.orbit is not None,
                drive_skydome=self.ephemeris.drive_skydome,
                drive_sunlight=self.ephemeris.drive_sunlight,
                expose_observations=self.ephemeris.expose_observations,
                eclipse=self.ephemeris.eclipse,
                time_scale=self.ephemeris.epoch.time_scale,
            )

    def _build_scene_assets(self):
        self._add_sunlight()
        self._add_skydome()
        self._add_scenery()
        self._add_sunlight_from_ephemeris_if_enabled()
        self._add_robot()
        self._add_extra_robots()

        self._add_particles()

        self.events.update(self)

        self._layout_robots()

    def _configure_simulation(self):
        # Isaac-free seam so the rule is unit-testable without booting Kit; see
        # srb/utils/rates.py for why the ratio must be exact.
        self.decimation = resolve_decimation(self.agent_rate, self.env_rate)
        self.sim.dt = self.env_rate
        self.sim.render_interval = self.decimation
        self.sim.gravity = (
            0.0,
            0.0,
            -(
                self.gravity.gravity_magnitude
                if self.gravity is not None
                else self.domain.gravity_magnitude
            ),
        )
        # GPU buffer capacities keep the PhysX defaults; `malloc_scale` is not
        # applied (see the modelling envelope page). SRB defaults to a CPU
        # device, where these capacities are inert. The former table set rigid
        # contacts to 8,192 vs. 8,388,608 by default at one env and underallocated
        # 8/11 buffers.

        self._enable_tgs_external_force_iterations()
        self._apply_physx_tuning()
        self._maybe_disable_fabric_on_cpu()

    def _apply_asset_and_particle_extras(self):
        self._setup_asset_extras()

        self._settle_down_particles()
        self._update_procedural_assets()
        self._maybe_disable_fabric_for_particles()

    def _add_sunlight(self, *, prim_path: str = "/World/sunlight", **kwargs):
        if self.domain.light_intensity <= 0.0:
            return
        self.scene.sunlight = _build_sunlight_asset(
            self,
            prim_path=prim_path,
            **kwargs,
        )

    def _add_sunlight_from_ephemeris_if_enabled(
        self, *, prim_path: str = "/World/sunlight", **kwargs
    ):
        """Apply ephemeris sunlight after scenery resolves; no-op unless enabled.

        Keep ``prim_path`` default aligned with :meth:`_add_sunlight`.
        """
        if self.domain.light_intensity <= 0.0:
            return
        if not (self.ephemeris.enabled and self.ephemeris.drive_sunlight):
            return
        self._add_sunlight_from_ephemeris(prim_path=prim_path, **kwargs)

    def _add_sunlight_from_ephemeris(
        self, *, prim_path: str = "/World/sunlight", **kwargs
    ):
        self.scene.sunlight = _build_ephemeris_sunlight_asset(
            self,
            prim_path=prim_path,
            **kwargs,
        )

    def _add_skydome(self, *, prim_path: str = "/World/skydome", **kwargs):
        # ``ephemeris.drive_skydome`` is honored
        # at runtime by the sun event terms (``apply_skydome_from_ephemeris``
        # inside the shared events core); the dome's spawn orientation is NOT
        # authored here from the ephemeris -- the first reset applies it (a
        # reset applier is always registered when drive_skydome is on, see
        # BaseEventCfg._update_sunlight), and duplicating the query/yaw math
        # at config time would be a second implementation that could drift.

        if not self.skydome:
            self.scene.skydome = None
            self.events.randomize_skydome_orientation = None
            return
        elif isinstance(self.skydome, str):
            match self.skydome:
                case "low_res":
                    skydome_dir = SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES
                case "high_res":
                    skydome_dir = SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES
                case _:
                    raise InvalidInputError(
                        f"Invalid skydome option {self.skydome!r}; set skydome to "
                        "'low_res' or 'high_res'",
                        component="srb.env_cfg",
                    )
        else:
            skydome_dir = SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES

        match self.domain:
            case Domain.EARTH:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath(
                            # "cloudy_sky.exr",
                            # "spaceport_moon_lab.exr",
                            # "lakeside_dawn.exr"
                            "stars.exr"
                        ).as_posix(),
                        **kwargs,
                    ),
                )
                self.events.randomize_skydome_orientation = SrbEventTermCfg(
                    func=reset_xform_orientation_uniform,
                    mode="interval",
                    is_global_time=True,
                    interval_range_s=(10.0, 60.0),
                    category=RandomizationCategory.VISUAL,
                    params={
                        "asset_cfg": SceneEntityCfg("skydome"),
                        "orientation_distribution_params": {
                            "yaw": (-torch.pi, torch.pi),
                        },
                    },
                )
            case Domain.MOON | Domain.ASTEROID:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath(
                            "stars.exr",
                            # "milky_way.exr",
                        ).as_posix(),
                        **kwargs,
                    ),
                )
                self.events.randomize_skydome_orientation = SrbEventTermCfg(
                    func=reset_xform_orientation_uniform,
                    mode="interval",
                    is_global_time=True,
                    interval_range_s=(10.0, 60.0),
                    category=RandomizationCategory.VISUAL,
                    params={
                        "asset_cfg": SceneEntityCfg("skydome"),
                        "orientation_distribution_params": {
                            "roll": (-torch.pi, torch.pi),
                            "pitch": (-torch.pi, torch.pi),
                            "yaw": (-torch.pi, torch.pi),
                        },
                    },
                )
            case Domain.MARS:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath(
                            "mars_sky.exr",
                        ).as_posix(),
                        **kwargs,
                    ),
                )
                self.events.randomize_skydome_orientation = SrbEventTermCfg(
                    func=reset_xform_orientation_uniform,
                    mode="interval",
                    is_global_time=True,
                    interval_range_s=(10.0, 60.0),
                    category=RandomizationCategory.VISUAL,
                    params={
                        "asset_cfg": SceneEntityCfg("skydome"),
                        "orientation_distribution_params": {
                            "yaw": (-torch.pi, torch.pi),
                        },
                    },
                )
            case Domain.ORBIT:
                self.scene.skydome = AssetBaseCfg(
                    prim_path=prim_path,
                    spawn=DomeLightCfg(
                        intensity=0.25 * self.domain.light_intensity,
                        texture_file=skydome_dir.joinpath(
                            "low_earth_orbit.exr",
                            # "low_lunar_orbit.jpg",
                            # "stars.exr",
                            # "milky_way.exr",
                        ).as_posix(),
                        **kwargs,
                    ),
                )
                self.events.randomize_skydome_orientation = SrbEventTermCfg(
                    func=reset_xform_orientation_uniform,
                    mode="interval",
                    is_global_time=True,
                    interval_range_s=(10.0, 60.0),
                    category=RandomizationCategory.VISUAL,
                    params={
                        "asset_cfg": SceneEntityCfg("skydome"),
                        "orientation_distribution_params": {
                            "roll": (-torch.pi, torch.pi),
                            "pitch": (-torch.pi, torch.pi),
                            "yaw": (-torch.pi, torch.pi),
                        },
                    },
                )
            case _:
                self.scene.skydome = None
                self.events.randomize_skydome_orientation = None

    def _add_scenery(
        self,
        *,
        prim_path: str = "{ENV_REGEX_NS}/scenery",
        prim_path_stacked: str = "/World/scenery",
        **kwargs,
    ):
        if self.scenery is None:
            self._scenery = None
            return

        scenery = self.scenery
        if isinstance(scenery, AssetVariant):
            assert self.spacing is not None
            scale = (
                self.spacing,
                self.spacing,
                0.1 * self.spacing,
            )
            _dyn_res = max(
                1,  # Min multiplier
                min(
                    10,  # Max multiplier
                    2
                    * round(
                        min(
                            8.0,
                            math.pow(
                                self.spacing,
                                0.4,  # Exponential scaling
                            ),
                        )
                        / 2
                    ),
                ),
            )
            texture_resolution = {
                BakeType.ALBEDO: _dyn_res * 1024,
                BakeType.EMISSION: _dyn_res * 128,
                BakeType.METALLIC: _dyn_res * 256,
                BakeType.NORMAL: _dyn_res * 1024,
                BakeType.ROUGHNESS: _dyn_res * 512,
            }
            density = 0.02 * (_dyn_res**2)
            flat_area_size = 0.8 * (_dyn_res**1.2)
            if self.scenery_flat_area_size is not None:
                flat_area_size = float(self.scenery_flat_area_size)

            resolved = resolve_variant(
                scenery,
                type_hints=get_field_annotation(self, "scenery"),
                base_class=Scenery,
                candidates_for_type=lambda typ: typ.scenery_registry(),
                instantiate=lambda typ: typ(
                    scale=scale,
                    texture_resolution=texture_resolution,
                    density=density,
                    flat_area_size=flat_area_size,
                    **kwargs,
                ),
                current_domain=self.domain,
                label="scenery",
            )
            if resolved is not None:
                scenery = resolved
        if not isinstance(scenery, Scenery):
            raise InvalidInputError(
                f"Failed to instantiate scenery from {repr(scenery)}"
            )

        # Configure terrain generator
        if (
            isinstance(scenery.asset_cfg, TerrainImporterCfg)
            and scenery.asset_cfg.terrain_type == "generator"
        ):
            assert isinstance(scenery.asset_cfg.terrain_generator, TerrainGeneratorCfg)
            assert self.spacing is not None
            scenery.asset_cfg.terrain_generator.seed = self.seed
            scenery.asset_cfg.terrain_generator.num_rows = math.ceil(
                self.spacing / scenery.asset_cfg.terrain_generator.size[0]
            )
            scenery.asset_cfg.terrain_generator.num_cols = math.ceil(
                self.spacing / scenery.asset_cfg.terrain_generator.size[1]
            )

        # Update prim path
        _use_stacked = self.stack or isinstance(scenery.asset_cfg, TerrainImporterCfg)
        _base_prim_path = prim_path_stacked if _use_stacked else prim_path
        # The stacked path stays concrete. Isaac Lab `release/3.0.0` at
        # `fdc650c8` plans shared (global) assets too and copies each one to
        # its declared path; a `.*` leaf, which the earlier stage-time
        # `spawn_multi_asset` needed, becomes a literal Sdf path there and
        # every stacked task failed with "Cannot create prim at path ''".
        scenery.asset_cfg.prim_path = _base_prim_path

        # Add to the scene
        scenery.asset_cfg = scenery.as_asset_base_cfg()  # type: ignore[misc]
        self.scene.scenery = scenery.asset_cfg

        # Store the updated config in an internal state
        self._scenery = scenery

        # Resolve companions before ephemeris lighting and events; arbitration sees
        # final sun/dome ownership. Route failures through the scenery hook.
        if getattr(scenery, "attach_companions_enabled", False):
            try:
                scenery.attach_companions(self)  # type: ignore[attr-defined]
            except Exception as err:
                handle_failure = getattr(
                    scenery, "handle_companion_attach_failure", None
                )
                if handle_failure is None:
                    raise
                handle_failure(self, err)

    def _add_robot(
        self,
        *,
        robot: "Robot | AssetVariant | None" = None,
        prim_path: str = "{ENV_REGEX_NS}/robot",
        prim_path_manipulator: str = "{ENV_REGEX_NS}/manipulator",
        prim_path_payload: str = "{ENV_REGEX_NS}/payload",
        prim_path_end_effector: str = "{ENV_REGEX_NS}/end_effector",
        **kwargs,
    ):
        robot_name = prim_path.rstrip("/").rsplit("/", 1)[-1]
        manipulator_name = prim_path_manipulator.rstrip("/").rsplit("/", 1)[-1]
        payload_name = prim_path_payload.rstrip("/").rsplit("/", 1)[-1]
        end_effector_name = prim_path_end_effector.rstrip("/").rsplit("/", 1)[-1]

        if robot is None:
            robot = self.robot
        if isinstance(robot, AssetVariant):
            resolved = resolve_variant(
                robot,
                type_hints=get_field_annotation(self, "robot"),
                base_class=Robot,
                candidates_for_type=lambda typ: typ.robot_registry(),
                instantiate=lambda typ: typ(**kwargs),
                current_domain=self.domain,
                label="robot",
            )
            if resolved is not None:
                robot = resolved
        if not isinstance(robot, Robot):
            raise InvalidInputError(f"Failed to instantiate robot from {repr(robot)}")

        from srb.core.env.mobile_manipulation._helpers import (
            downgrade_free_floating_manipulator_jacobian_actions,
        )

        # Free-floating manipulators require joint-space actions; resolve variants
        # first.
        downgrade_free_floating_manipulator_jacobian_actions(robot)

        if hasattr(self, "actions") and isinstance(self.actions, ActionGroup):
            if hasattr(self.actions, "__srb_command_mapper_fns"):
                get_mapper_fns = getattr(self.actions, "__srb_command_mapper_fns")
                get_mapping_getters = getattr(
                    self.actions, "__srb_command_mapping_getters"
                )
                map_cmd_to_action_fns = list(get_mapper_fns())
                command_mapping_getters = list(get_mapping_getters())
            else:
                initial_actions = _snapshot_action_group_leaf(self.actions)
                map_cmd_to_action_fns = [initial_actions.map_cmd_to_action]
                command_mapping_getters = [initial_actions.get_command_mapping]
        else:
            self.actions = ActionGroup()
            map_cmd_to_action_fns = []
            command_mapping_getters = []

        robot.asset_cfg.prim_path = prim_path
        setattr(self.scene, robot_name, robot.asset_cfg)
        # Actions
        robot_action_names = {}
        for action_term_name, action_term in iter_config_items_by_type(
            robot.action_group,
            ActionTermCfg,
        ):
            # Ensure the actions terms are applied to the correct asset
            action_term.asset_name = robot_name
            # Add action terms to the action group
            action_name = f"{robot_name}/{sanitize_action_term_name(action_term_name)}"
            setattr(self.actions, action_name, action_term)
            robot_action_names[action_term_name] = action_name
        # Add the command mapping function to the action group
        map_cmd_to_action_fns.append(robot.action_group.map_cmd_to_action)
        command_mapping_getters.append(
            _renamed_command_mapping_getter(robot.action_group, robot_action_names)
        )

        if isinstance(robot, MobileRobot):
            if robot.payload is not None:
                if isinstance(
                    robot.payload.asset_cfg, (RigidObjectCfg, ArticulationCfg)
                ):
                    robot.payload.asset_cfg.prim_path = prim_path_payload
                    payload_mount_offset = robot.frame_payload_mount.offset
                    if isinstance(robot.payload, ActivePayload):
                        payload_mount_pos, payload_mount_rot = (
                            combine_frame_transforms_tuple(
                                payload_mount_offset.pos,
                                payload_mount_offset.rot,
                                robot.payload.frame_mount.offset.pos,
                                robot.payload.frame_mount.offset.rot,
                            )
                        )
                        payload_mount_offset = Transform(
                            pos=payload_mount_pos,
                            rot=payload_mount_rot,
                        )
                    (
                        robot.payload.asset_cfg.init_state.pos,
                        robot.payload.asset_cfg.init_state.rot,
                    ) = combine_frame_transforms_tuple(
                        robot.asset_cfg.init_state.pos,
                        robot.asset_cfg.init_state.rot,
                        payload_mount_offset.pos,
                        payload_mount_offset.rot,
                    )
                    self.joint_assemblies[payload_name] = RobotAssemblerCfg(
                        base_path=robot.asset_cfg.prim_path,
                        attach_path=robot.payload.asset_cfg.prim_path,
                        base_mount_frame=f"/{robot.frame_payload_mount.prim_relpath}"
                        if robot.frame_payload_mount.prim_relpath
                        else "",
                        # Every payload that names its mount prim binds the
                        # fixed joint to it, exactly like the manipulator and
                        # end-effector branches below. Restricting this to
                        # ActivePayload left plain payloads bound to the bare
                        # Xform the assembler fabricates, which carries no
                        # rigid body, so PhysX silently dropped the joint and
                        # the payload stayed behind while the robot drove off.
                        attach_mount_frame=(
                            f"/{robot.payload.frame_mount.prim_relpath}"
                            if robot.payload.frame_mount.prim_relpath
                            else ""
                        ),
                        fixed_joint_offset=payload_mount_offset.pos,
                        fixed_joint_orient=payload_mount_offset.rot,
                        mask_all_collisions=True,
                    )
                else:
                    self.joint_assemblies.pop(payload_name, None)
                    robot.payload.asset_cfg.prim_path = f"{robot.asset_cfg.prim_path}/{robot.frame_base.prim_relpath}/{payload_name}"
                    (
                        robot.payload.asset_cfg.init_state.pos,
                        robot.payload.asset_cfg.init_state.rot,
                    ) = (
                        robot.frame_payload_mount.offset.pos,
                        robot.frame_payload_mount.offset.rot,
                    )
                    # A plain AssetBaseCfg payload (e.g. HuskyPayloadLunalab)
                    # stays a single-variant spawn: the pinned InteractiveScene
                    # replicates an env-scoped plain AssetBaseCfg to every
                    # environment (one clone-plan row, full env mask); a
                    # num_envs-copy wrapper only multiplied the clone plan's
                    # cartesian product by num_envs.
                setattr(self.scene, payload_name, robot.payload.asset_cfg)
                # Actions — handle active payloads.
                if isinstance(robot.payload, ActivePayload):
                    payload_action_names = {}
                    for (
                        action_term_name,
                        action_term,
                    ) in iter_config_items_by_type(
                        robot.payload.action_group,
                        ActionTermCfg,
                    ):
                        # Ensure the actions terms are applied to the correct asset
                        action_term.asset_name = payload_name
                        # Add action terms to the action group
                        action_name = f"{payload_name}/{sanitize_action_term_name(action_term_name)}"
                        setattr(self.actions, action_name, action_term)
                        payload_action_names[action_term_name] = action_name
                    # Add the command mapping function to the action group
                    map_cmd_to_action_fns.append(
                        robot.payload.action_group.map_cmd_to_action
                    )
                    command_mapping_getters.append(
                        _renamed_command_mapping_getter(
                            robot.payload.action_group, payload_action_names
                        )
                    )
            else:
                setattr(self.scene, payload_name, None)
                self.joint_assemblies.pop(payload_name, None)

        if isinstance(robot, CombinedMobileManipulator):
            robot.manipulator.asset_cfg.prim_path = prim_path_manipulator
            setattr(self.scene, manipulator_name, robot.manipulator.asset_cfg)
            manipulator_needs_jacobian = bool(
                next(
                    (
                        action_term
                        for action_term in iter_config_values_by_type(
                            robot.manipulator.action_group,
                            (
                                DifferentialInverseKinematicsActionCfg,
                                OperationalSpaceControllerActionCfg,
                            ),
                        )
                    ),
                    None,
                )
            )
            (
                robot.manipulator.asset_cfg.init_state.pos,
                robot.manipulator.asset_cfg.init_state.rot,
            ) = combine_frame_transforms_tuple(
                robot.asset_cfg.init_state.pos,
                robot.asset_cfg.init_state.rot,
                robot.frame_manipulator_mount.offset.pos,
                robot.frame_manipulator_mount.offset.rot,
            )
            self.joint_assemblies[manipulator_name] = RobotAssemblerCfg(
                base_path=prim_path,
                attach_path=prim_path_manipulator,
                base_mount_frame=f"/{robot.frame_base.prim_relpath}"
                if robot.frame_base.prim_relpath
                else "",
                attach_mount_frame=f"/{robot.manipulator.frame_base.prim_relpath}"
                if robot.manipulator.frame_base.prim_relpath
                else "",
                fixed_joint_offset=robot.frame_manipulator_mount.offset.pos,
                fixed_joint_orient=robot.frame_manipulator_mount.offset.rot,
                mask_all_collisions=True,
                disable_root_joints=not manipulator_needs_jacobian,
            )
            # Actions
            manipulator_action_names = {}
            for (
                action_term_name,
                action_term,
            ) in iter_config_items_by_type(
                robot.manipulator.action_group,
                ActionTermCfg,
            ):
                # Ensure the actions terms are applied to the correct asset
                action_term.asset_name = manipulator_name
                # Add action terms to the action group
                action_name = (
                    f"{manipulator_name}/{sanitize_action_term_name(action_term_name)}"
                )
                setattr(self.actions, action_name, action_term)
                manipulator_action_names[action_term_name] = action_name
            # Add the command mapping function to the action group
            map_cmd_to_action_fns.append(
                robot.manipulator.action_group.map_cmd_to_action
            )
            command_mapping_getters.append(
                _renamed_command_mapping_getter(
                    robot.manipulator.action_group, manipulator_action_names
                )
            )
        else:
            self.joint_assemblies.pop(manipulator_name, None)

        if isinstance(robot, (Manipulator, CombinedMobileManipulator)):
            manipulator = robot if isinstance(robot, Manipulator) else robot.manipulator
            if isinstance(manipulator.end_effector, Tool):
                if isinstance(
                    manipulator.end_effector.asset_cfg,
                    (
                        (RigidObjectCfg, ArticulationCfg)
                        if self.assemble_rigid_end_effector
                        else ArticulationCfg
                    ),
                ):
                    manipulator.end_effector.asset_cfg.prim_path = (
                        prim_path_end_effector
                    )
                    (_fixed_joint_offset, _fixed_joint_orient) = (
                        combine_frame_transforms_tuple(
                            manipulator.frame_flange.offset.pos,
                            manipulator.frame_flange.offset.rot,
                            manipulator.end_effector.frame_mount.offset.pos,
                            manipulator.end_effector.frame_mount.offset.rot,
                        )
                    )
                    (
                        manipulator.end_effector.asset_cfg.init_state.pos,
                        manipulator.end_effector.asset_cfg.init_state.rot,
                    ) = (_fixed_joint_offset, _fixed_joint_orient)
                    self.joint_assemblies[end_effector_name] = RobotAssemblerCfg(
                        base_path=manipulator.asset_cfg.prim_path,
                        attach_path=manipulator.end_effector.asset_cfg.prim_path,
                        base_mount_frame=f"/{manipulator.frame_flange.prim_relpath}"
                        if manipulator.frame_flange.prim_relpath
                        else "",
                        attach_mount_frame=f"/{manipulator.end_effector.frame_mount.prim_relpath}"
                        if manipulator.end_effector.frame_mount.prim_relpath
                        else "",
                        fixed_joint_offset=_fixed_joint_offset,
                        fixed_joint_orient=_fixed_joint_orient,
                        mask_all_collisions=True,
                    )
                else:
                    self.joint_assemblies.pop(end_effector_name, None)
                    manipulator.end_effector.asset_cfg = (  # type: ignore[misc]
                        manipulator.end_effector.as_asset_base_cfg(
                            remove_rigid_props=True
                        )
                    )
                    manipulator.end_effector.asset_cfg.prim_path = f"{manipulator.asset_cfg.prim_path}/{manipulator.frame_flange.prim_relpath}/{end_effector_name}"
                    (
                        manipulator.end_effector.asset_cfg.init_state.pos,
                        manipulator.end_effector.asset_cfg.init_state.rot,
                    ) = (
                        manipulator.frame_flange.offset.pos,
                        manipulator.frame_flange.offset.rot,
                    )
                setattr(
                    self.scene, end_effector_name, manipulator.end_effector.asset_cfg
                )
                # Actions
                if isinstance(manipulator.end_effector, ActiveTool):
                    end_effector_action_names = {}
                    for (
                        action_term_name,
                        action_term,
                    ) in iter_config_items_by_type(
                        manipulator.end_effector.action_group,
                        ActionTermCfg,
                    ):
                        # Ensure the actions terms are applied to the correct asset
                        action_term.asset_name = end_effector_name
                        # Add action terms to the action group
                        action_name = f"{end_effector_name}/{sanitize_action_term_name(action_term_name)}"
                        setattr(self.actions, action_name, action_term)
                        end_effector_action_names[action_term_name] = action_name
                    # Add the command mapping function to the action group
                    map_cmd_to_action_fns.append(
                        manipulator.end_effector.action_group.map_cmd_to_action
                    )
                    command_mapping_getters.append(
                        _renamed_command_mapping_getter(
                            manipulator.end_effector.action_group,
                            end_effector_action_names,
                        )
                    )
            else:
                setattr(self.scene, end_effector_name, None)
                self.joint_assemblies.pop(end_effector_name, None)
            # Task-space (IK/OSC) terms act on the tracked ``robot_ee`` frame:
            # ``frame_flange ∘ TCP`` (bare flange without a tool), the same
            # composition as the ``tf_end_effector`` target.
            for action_term in iter_config_values_by_type(
                manipulator.action_group,
                (
                    DifferentialInverseKinematicsActionCfg,
                    OperationalSpaceControllerActionCfg,
                ),
            ):
                bind_controller_to_end_effector_frame(
                    action_term, manipulator, combine=combine_frame_transforms_tuple
                )

        if isinstance(robot, CombinedMobileManipulator) and manipulator_needs_jacobian:
            # Add action terms to the action group
            setattr(
                self.actions,
                f"{robot_name}_{manipulator_name}_dummy_switch",
                DummyActionCfg(),
            )
            # Add the command mapping function to the action group
            map_cmd_to_action_fns.append(
                lambda twist, event: torch.Tensor((-1.0 if event else 1.0,)).to(
                    device=twist.device
                )
            )
            dummy_action_name = f"{robot_name}_{manipulator_name}_dummy_switch"
            dummy_mapping = CommandMapping(
                operation=EventSign(),
                destinations=(
                    CommandDestination(name=dummy_action_name, start=0, stop=1),
                ),
            )
            command_mapping_getters.append(lambda mapping=dummy_mapping: mapping)

        def composed_mapper(twist, event):
            return torch.cat([func(twist, event) for func in map_cmd_to_action_fns])

        self.actions.map_cmd_to_action = composed_mapper

        mapper_fns = tuple(map_cmd_to_action_fns)
        mapping_getters = tuple(command_mapping_getters)
        setattr(
            self.actions,
            "__srb_command_mapper_fns",
            lambda functions=mapper_fns: functions,
        )
        setattr(self.actions, "__srb_composed_command_mapper", composed_mapper)
        setattr(
            self.actions,
            "__srb_command_mapping_getters",
            lambda getters=mapping_getters: getters,
        )
        self.actions.get_command_mapping = MethodType(
            ActionGroup.get_command_mapping, self.actions
        )

        # Store the updated config in an internal state
        self._robots[robot_name] = robot
        if robot_name == "robot":
            self._robot = robot

        # Configure sensors from the robot (merges sub-assets like manipulator/payload)
        if hasattr(robot, "configure_sensors") and callable(robot.configure_sensors):
            robot.configure_sensors()

        # finalize() wires sensors after subclass __post_init__ methods finish.

    def _add_sensors(
        self,
        robot: Robot,
        *,
        robot_name: str = "robot",
    ):
        """Add active robot and environment sensors to the scene."""
        import copy
        import re

        from srb.core.sensor import CameraCfg, FrameTransformerCfg, SensorBaseCfg
        from srb.core.sensor.camera_defaults import camera_clipping_range_fill
        from srb.core.sensor.category import SensorCategory, get_sensor_category
        from srb.core.sensor.event_camera.config import EventCameraCfg
        from srb.core.sensor.event_camera.materialization import (
            materialize_event_camera_cfg,
        )

        _env_prefix_re = re.compile(r"^\{ENV_REGEX_NS\}/([^/]+)(.*)")

        # Preserve sibling sub-asset roots; rewrite only asset-authored roots.
        if robot_name == "robot":
            _sibling_prims = ("manipulator", "payload", "end_effector")
        else:
            _sibling_prims = (
                f"{robot_name}_manipulator",
                f"{robot_name}_payload",
                f"{robot_name}_end_effector",
            )

        def _fix_prim_path(path: str) -> tuple[str, bool]:
            m = _env_prefix_re.match(path)
            if m and robot_name != "robot" and m.group(1) == "manipulator":
                return (
                    f"{{ENV_REGEX_NS}}/{robot_name}_manipulator{m.group(2)}",
                    True,
                )
            if m and m.group(1) != robot_name and m.group(1) not in _sibling_prims:
                return f"{{ENV_REGEX_NS}}/{robot_name}{m.group(2)}", True
            return path, False

        # Track origin to rewrite robot sensor paths only.
        all_sensors: dict[str, tuple[Any, bool]] = {}
        for name, cfg in robot.sensor_cfgs.items():
            all_sensors[name] = (cfg, True)  # True = from robot
        # Environment sensors are shared; add them only on the primary pass.
        if robot_name == "robot":
            for name, cfg in self.env_sensor_cfgs.items():
                all_sensors[name] = (cfg, False)  # False = from environment

        event_camera_bindings = getattr(self, "_event_camera_bindings", None)
        if event_camera_bindings is None:
            event_camera_bindings = {}
            setattr(self, "_event_camera_bindings", event_camera_bindings)

        for sensor_name, (sensor_cfg, is_robot_sensor) in all_sensors.items():
            # Filter by active categories
            category = get_sensor_category(sensor_cfg)
            if category not in self.active_sensors:
                continue

            # Keep the original Isaac-free config in the binding; materialize a copy
            # only when prim rewriting requires it.
            if isinstance(sensor_cfg, EventCameraCfg):
                original_cfg = sensor_cfg
                source_cfg = original_cfg
                if is_robot_sensor:
                    new_prim, prim_changed = _fix_prim_path(original_cfg.prim_path)
                    if prim_changed:
                        source_cfg = copy.deepcopy(original_cfg)
                        source_cfg.prim_path = new_prim

                if robot_name != "robot":
                    new_name = sensor_name.replace("robot", robot_name)
                    if new_name == sensor_name:
                        new_name = f"{sensor_name}_{robot_name}"
                    sensor_name = new_name

                setattr(
                    self.scene,
                    sensor_name,
                    materialize_event_camera_cfg(source_cfg),
                )
                event_camera_bindings[sensor_name] = original_cfg
                continue

            # Only Isaac Lab sensor configs can enter InteractiveScene.
            if not isinstance(sensor_cfg, SensorBaseCfg):
                from srb.utils import logging

                logging.debug(
                    "Sensor %r (%s) is not an Isaac Lab SensorBaseCfg; "
                    "skipping scene materialisation.",
                    sensor_name,
                    type(sensor_cfg).__name__,
                )
                continue

            # Check if prim path needs rewriting (robot sensors only)
            prim_changed = False
            new_prim = sensor_cfg.prim_path
            if is_robot_sensor:
                new_prim, prim_changed = _fix_prim_path(sensor_cfg.prim_path)
            frame_needs_fix = False
            if is_robot_sensor and isinstance(sensor_cfg, FrameTransformerCfg):
                for frame in sensor_cfg.target_frames:
                    if hasattr(frame, "prim_path"):
                        _, fc = _fix_prim_path(frame.prim_path)
                        if fc:
                            frame_needs_fix = True
                            break

            # Resolve raycaster sentinels to terrain/scenery, excluding robot
            # meshes from the target set.
            resolved_mesh_paths: list[str] | None = None
            _mesh_paths = getattr(sensor_cfg, "mesh_prim_paths", None)
            if _mesh_paths is not None and ".*" in list(_mesh_paths):
                resolved_mesh_paths = []
                for _path in _mesh_paths:
                    if _path != ".*":
                        resolved_mesh_paths.append(_path)
                        continue
                    for _attr in ("terrain", "scenery"):
                        _target_path = getattr(
                            getattr(self.scene, _attr, None), "prim_path", None
                        )
                        if (
                            isinstance(_target_path, str)
                            and _target_path
                            and _target_path not in resolved_mesh_paths
                        ):
                            resolved_mesh_paths.append(_target_path)
                if not resolved_mesh_paths:
                    from srb.utils import logging

                    logging.warning(
                        "Sensor %r: cannot resolve the '.*' mesh-target "
                        "sentinel (scene has no terrain/scenery prim); "
                        "dropping the sensor.",
                        sensor_name,
                    )
                    continue

            needs_copy = (
                prim_changed
                or frame_needs_fix
                or resolved_mesh_paths is not None
                or robot_name != "robot"
                or (
                    category == SensorCategory.VISUAL
                    and isinstance(sensor_cfg, CameraCfg)
                )
            )
            if needs_copy:
                sensor_cfg = copy.deepcopy(sensor_cfg)

            if prim_changed:
                sensor_cfg.prim_path = new_prim

            if resolved_mesh_paths is not None:
                sensor_cfg.mesh_prim_paths = resolved_mesh_paths

            # Fix target frame prim paths for FrameTransformerCfg
            if frame_needs_fix and isinstance(sensor_cfg, FrameTransformerCfg):
                for frame in sensor_cfg.target_frames:
                    if hasattr(frame, "prim_path"):
                        frame.prim_path, _ = _fix_prim_path(frame.prim_path)

            # Apply camera-specific overrides for VISUAL-category sensors
            if category == SensorCategory.VISUAL and isinstance(sensor_cfg, CameraCfg):
                if self.camera_resolution is not None:
                    # Fill dimensions independently to preserve valid overrides.
                    if not isinstance(sensor_cfg.width, int) or sensor_cfg.width <= 0:
                        sensor_cfg.width = self.camera_resolution[0]
                    if not isinstance(sensor_cfg.height, int) or sensor_cfg.height <= 0:
                        sensor_cfg.height = self.camera_resolution[1]
                if self.camera_update_period is not None:
                    if sensor_cfg.update_period == 0.0:
                        if self.camera_update_period < 0.0:
                            sensor_cfg.update_period = self.agent_rate
                        else:
                            sensor_cfg.update_period = self.camera_update_period
                if self.camera_data_types is not None:
                    if (
                        len(sensor_cfg.data_types) == 1
                        and sensor_cfg.data_types[0] == "rgb"
                    ):
                        sensor_cfg.data_types = self.camera_data_types
                # Apply task depth range to the copied spawner when unspecified.
                _spawn = getattr(sensor_cfg, "spawn", None)
                if _spawn is not None and hasattr(_spawn, "clipping_range"):
                    _clipping = camera_clipping_range_fill(
                        _spawn.clipping_range, self.camera_clipping_range
                    )
                    if _clipping is not None:
                        _spawn.clipping_range = _clipping

            # Namespace for extra robots
            if robot_name != "robot":
                new_name = sensor_name.replace("robot", robot_name)
                if new_name == sensor_name:
                    new_name = f"{sensor_name}_{robot_name}"
                sensor_name = new_name

            setattr(self.scene, sensor_name, sensor_cfg)

        # Ensure Isaac Lab validation sees defaults on inactive configs too.
        if self.camera_resolution is not None:
            for orig_cfg in robot.sensor_cfgs.values():
                if isinstance(orig_cfg, CameraCfg):
                    if not isinstance(orig_cfg.width, int) or orig_cfg.width <= 0:
                        orig_cfg.width = self.camera_resolution[0]
                    if not isinstance(orig_cfg.height, int) or orig_cfg.height <= 0:
                        orig_cfg.height = self.camera_resolution[1]

    def _add_extra_robots(self):
        """Add extra robots to the scene and auto-duplicate robot-targeting events."""
        if not self.extra_robots:
            return

        for extra_name, extra_robot_cfg in self.extra_robots.items():
            self._add_robot(
                robot=extra_robot_cfg,
                prim_path=f"{{ENV_REGEX_NS}}/{extra_name}",
                prim_path_manipulator=f"{{ENV_REGEX_NS}}/{extra_name}_manipulator",
                prim_path_payload=f"{{ENV_REGEX_NS}}/{extra_name}_payload",
                prim_path_end_effector=f"{{ENV_REGEX_NS}}/{extra_name}_end_effector",
            )

        # Clone event terms that target the primary robot for each extra robot
        self._duplicate_robot_events()

    def _duplicate_robot_events(self):
        """Clone robot-targeting event terms for each extra robot."""
        import copy

        events_to_add: dict[str, EventTermCfg] = {}
        for event_name, event_term in iter_config_items_by_type(
            self.events,
            EventTermCfg,
        ):
            if not isinstance(event_term.params, dict):
                continue
            asset_cfg = event_term.params.get("asset_cfg")
            if isinstance(asset_cfg, SceneEntityCfg) and asset_cfg.name == "robot":
                for extra_name in self.extra_robots:
                    cloned = copy.deepcopy(event_term)
                    cloned.params["asset_cfg"] = SceneEntityCfg(extra_name)
                    events_to_add[f"{event_name}_{extra_name}"] = cloned

        for name, term in events_to_add.items():
            setattr(self.events, name, term)

    def _add_particles(self):
        # Initialize particle system cfg registry
        if not hasattr(self, "_particle_system_cfgs"):
            self._particle_system_cfgs: dict[str, ParticleSystemCfg] = {}

        if not self.particles:
            for particle_system_id, asset_cfg in iter_config_items_by_type(
                self.scene,
                AssetBaseCfg,
            ):
                if not isinstance(asset_cfg.spawn, ParticlesSpawnerCfg):
                    continue
                setattr(self.scene, particle_system_id, None)
            return

        # Deterministic particle-system ownership: propagate the
        # environment's physics scene into every predeclared particle spawner
        # discovered from the scene (e.g. a task's own regolith asset). The
        # managed spawners created below receive the same owner explicitly.
        for _particle_system_id, asset_cfg in iter_config_items_by_type(
            self.scene,
            AssetBaseCfg,
        ):
            if isinstance(asset_cfg.spawn, ParticlesSpawnerCfg):
                asset_cfg.spawn.simulation_owner = self.sim.physics_prim_path

        if self.particles_systems <= 0:
            return

        if self.scene.num_envs != 1 and self.stack:
            raise InvalidInputError(
                "Particles are not supported in multiple stacked environments; "
                "set env.stack=false or env.num_envs=1"
            )
        if self.spacing is None or self.spacing <= 0.0:
            raise InvalidInputError(f"Invalid particle spacing: {self.spacing!r}")

        _particles_spawner_key = self.particles_spawner.lower()
        _particles_spawners = {
            "crater_pile": CraterPileParticlesSpawnerCfg,
            "grid": GridParticlesSpawnerCfg,
            "mound_pile": MoundPileParticlesSpawnerCfg,
            "pyramid": PyramidParticlesSpawnerCfg,
        }
        if _particles_spawner_key not in _particles_spawners:
            raise InvalidInputError(
                f"Invalid particles_spawner {self.particles_spawner!r}; "
                f"valid values: {sorted(_particles_spawners)}",
                component="srb.env_cfg",
            )
        particles_spawner_cls = _particles_spawners[_particles_spawner_key]
        particles_size = (
            self.particles_size
            if isinstance(self.particles_size, (tuple, list))
            else (self.particles_size,) * 2
        )
        particles_area = (
            self.particles_area
            if isinstance(self.particles_area, (tuple, list))
            else (self.particles_area,) * 2
        )
        particles_ratio = (
            self.particles_ratio
            if isinstance(self.particles_ratio, (tuple, list))
            else (self.particles_ratio,) * 2
        )
        particles_height = (
            self.particles_height
            if isinstance(self.particles_height, (tuple, list))
            else (self.particles_height,) * 2
        )
        particles_density = (
            self.particles_density
            if isinstance(self.particles_density, (tuple, list))
            else (self.particles_density,) * 2
        )
        particles_friction = (
            self.particles_friction
            if isinstance(self.particles_friction, (tuple, list))
            else (self.particles_friction,) * 2
        )
        particles_damping = (
            self.particles_damping
            if isinstance(self.particles_damping, (tuple, list))
            else (self.particles_damping,) * 2
        )
        particles_cohesion = (
            self.particles_cohesion
            if isinstance(self.particles_cohesion, (tuple, list))
            else (self.particles_cohesion,) * 2
        )
        particles_adhesion = (
            self.particles_adhesion
            if isinstance(self.particles_adhesion, (tuple, list))
            else (self.particles_adhesion,) * 2
        )

        min_id = 0 if particles_size[0] <= particles_size[1] else 1
        max_id = (min_id + 1) % 2

        # Override hooks (task-level explicit dims / init pos)
        if self.particles_dim_xy is not None:
            particles_dim_xy = (
                self.particles_dim_xy
                if isinstance(self.particles_dim_xy, (tuple, list))
                else (self.particles_dim_xy,) * 2
            )
            if len(particles_dim_xy) != 2:
                raise InvalidInputError(
                    f"`particles_dim_xy` tuple must be length 2 (dim_x, dim_y); "
                    f"got length {len(particles_dim_xy)}: {particles_dim_xy!r}"
                )
        else:
            particles_dim_xy = None
        if self.particles_init_pos is not None:
            if len(self.particles_init_pos) != 3:
                raise InvalidInputError(
                    f"`particles_init_pos` tuple must be length 3 (x, y, z); "
                    f"got length {len(self.particles_init_pos)}: "
                    f"{self.particles_init_pos!r}"
                )
            init_pos_base = self.particles_init_pos
        else:
            init_pos_base = (0.0, 0.0, 1.0)

        total_particle_z_offset_local = 0.0
        for i in range(self.particles_systems):
            _f = (
                0 if self.particles_systems == 1 else (i / (self.particles_systems - 1))
            )
            p_size = particles_size[min_id] + _f * (
                particles_size[max_id] - particles_size[min_id]
            )
            p_area = particles_area[min_id] + _f * (
                particles_area[max_id] - particles_area[min_id]
            )
            p_ratio = particles_ratio[min_id] + _f * (
                particles_ratio[max_id] - particles_ratio[min_id]
            )
            p_height = math.floor(
                particles_height[min_id]
                + _f * (particles_height[max_id] - particles_height[min_id])
            )
            p_density = particles_density[min_id] + _f * (
                particles_density[max_id] - particles_density[min_id]
            )
            p_friction = particles_friction[min_id] + _f * (
                particles_friction[max_id] - particles_friction[min_id]
            )
            p_damping = particles_damping[min_id] + _f * (
                particles_damping[max_id] - particles_damping[min_id]
            )
            p_cohesion = particles_cohesion[min_id] + _f * (
                particles_cohesion[max_id] - particles_cohesion[min_id]
            )
            p_adhesion = particles_adhesion[min_id] + _f * (
                particles_adhesion[max_id] - particles_adhesion[min_id]
            )

            if particles_dim_xy is not None:
                p_dim_xy = round(
                    particles_dim_xy[min_id]
                    + _f * (particles_dim_xy[max_id] - particles_dim_xy[min_id])
                )
            else:
                p_dim_xy = round(p_area * self.spacing / p_size)  # type: ignore[arg-type]
            p_init_pos = (
                init_pos_base[0],
                init_pos_base[1],
                init_pos_base[2] + total_particle_z_offset_local,
            )

            particles_cfg = AssetBaseCfg(
                prim_path=f"{{ENV_REGEX_NS}}/particles{i}",
                spawn=particles_spawner_cls(
                    simulation_owner=self.sim.physics_prim_path,
                    ratio=p_ratio,
                    particle_size=p_size,
                    dim_x=p_dim_xy,
                    dim_y=p_dim_xy,
                    dim_z=p_height,
                    density=p_density,
                    friction=p_friction,
                    damping=p_damping,
                    cohesion=p_cohesion,
                    adhesion=p_adhesion,
                    velocity=((-0.5, 0.5), (-0.5, 0.5), (-0.25, 0.0)),
                    fluid=False,
                    visual_material=self.particles_material,
                ),
                init_state=AssetBaseCfg.InitialStateCfg(pos=p_init_pos),
            )
            setattr(self.scene, f"particles{i}", particles_cfg)
            total_particle_z_offset_local += p_height * particle_pitch_m(
                p_size, particles_cfg.spawn.particle_spacing, p_ratio
            )

            # Also register a ParticleSystemCfg for the new managed asset
            self._particle_system_cfgs[f"particles{i}"] = ParticleSystemCfg(
                prim_path=f"{{ENV_REGEX_NS}}/particles{i}",
                spawner=particles_spawner_cls(
                    simulation_owner=self.sim.physics_prim_path,
                    ratio=p_ratio,
                    particle_size=p_size,
                    dim_x=p_dim_xy,
                    dim_y=p_dim_xy,
                    dim_z=p_height,
                    density=p_density,
                    friction=p_friction,
                    damping=p_damping,
                    cohesion=p_cohesion,
                    adhesion=p_adhesion,
                    velocity=((-0.5, 0.5), (-0.5, 0.5), (-0.25, 0.0)),
                    fluid=False,
                ),
            )

    def _layout_robots(self):
        """Apply opt-in XY layout to all robots and preserve it across resets."""
        _has_spacing = self.robot_spacing is not None and self.robot_spacing > 0.0
        _has_explicit = self.robot_positions is not None
        if not _has_spacing and not _has_explicit:
            return

        robot_names = ["robot", *self.extra_robots.keys()]

        # Remove robot root-pose reset events (the original randomize_robot_state
        # plus any per-robot clones from _duplicate_robot_events) so the baked
        # init positions are not overwritten at reset.
        for event_name, event_term in list(
            iter_config_items_by_type(self.events, EventTermCfg)
        ):
            params = getattr(event_term, "params", None)
            if not isinstance(params, dict):
                continue
            asset_cfg = params.get("asset_cfg")
            if (
                isinstance(asset_cfg, SceneEntityCfg)
                and asset_cfg.name in robot_names
                and "pose_range" in params
            ):
                setattr(self.events, event_name, None)

        if _has_explicit:
            positions = [(float(x), float(y)) for x, y in self.robot_positions]
            if len(positions) != len(robot_names):
                raise ValueError(
                    f"robot_positions has {len(positions)} entries but there are "
                    f"{len(robot_names)} robots ({robot_names}); provide one (x, y) each."
                )
        else:
            positions = self._sample_spaced_positions(len(robot_names))

        for name, (pos_x, pos_y) in zip(robot_names, positions, strict=True):
            asset_cfg = getattr(self.scene, name, None)
            init_state = getattr(asset_cfg, "init_state", None)
            if init_state is None:
                from srb.utils import logging

                logging.warning(
                    "Robot '%s' has no init_state; cannot apply layout position.",
                    name,
                )
                continue
            prev_pos = getattr(init_state, "pos", None) or (0.0, 0.0, 0.0)
            z = float(prev_pos[2])
            if self.robot_spawn_height:
                z += float(self.robot_spawn_height)
            init_state.pos = (float(pos_x), float(pos_y), z)

    def _sample_spaced_positions(self, count: int) -> list[tuple[float, float]]:
        """Sample seeded XY positions at least ``robot_spacing`` apart."""
        from srb.utils.spatial import poisson_disk_positions

        return poisson_disk_positions(
            count,
            spacing=float(self.robot_spacing),  # type: ignore[arg-type]
            region_half=(
                None
                if self.robot_spacing_area is None
                else float(self.robot_spacing_area)
            ),
            seed=self.seed if isinstance(self.seed, int) else 0,
        )

    def _settle_down_particles(self):
        # Ensure the registry exists
        if not hasattr(self, "_particle_system_cfgs"):
            self._particle_system_cfgs: dict[str, ParticleSystemCfg] = {}

        # Auto-discover particle system entries from the scene that aren't
        # already registered (e.g. excavation task's regolith)
        for attr_name, asset_cfg in iter_config_items_by_type(
            self.scene,
            AssetBaseCfg,
        ):
            if (
                isinstance(asset_cfg.spawn, ParticlesSpawnerCfg)
                and attr_name not in self._particle_system_cfgs
            ):
                self._particle_system_cfgs[attr_name] = ParticleSystemCfg(
                    prim_path=asset_cfg.prim_path,
                    spawner=asset_cfg.spawn,
                )

        particle_system_names = list(self._particle_system_cfgs.keys())
        if particle_system_names:
            self.events.settle_and_reset_particles = (  # type: ignore[misc]
                SrbEventTermCfg(
                    func=settle_and_reset_particle_systems,
                    mode="reset",
                    category=RandomizationCategory.LIFECYCLE,
                    params={
                        "particle_system_names": particle_system_names,
                        "freeze": self.particles_freeze,
                        "conform_to_terrain": self.particles_conform_to_terrain,
                        "conform_surface_offset": 0.5
                        * (
                            min(self.particles_size)
                            if isinstance(self.particles_size, (tuple, list))
                            else self.particles_size
                        ),
                        "conform_xy_jitter": self.particles_conform_jitter,
                        "conform_jitter_seed": self.seed
                        if isinstance(self.seed, int)
                        else 0,
                        "pile_center_offset_range_m": self.particles_pile_center_jitter,
                    },
                )
            )
        else:
            self.events.settle_and_reset_particles = None  # type: ignore[misc]

    def _maybe_disable_fabric_for_particles(self):
        # Disable Fabric for particles
        for asset_cfg in iter_config_values_by_type(self.scene, AssetBaseCfg):
            if isinstance(asset_cfg.spawn, ParticlesSpawnerCfg):
                self.sim.use_fabric = False
                return

    def _maybe_disable_fabric_on_cpu(self):
        # USD reads become stale after rendering switches PhysX writes to Fabric.
        from srb.utils.fabric import (
            read_physx_fabric_transform_updates,
            resolve_use_fabric,
        )

        self.sim.use_fabric = resolve_use_fabric(
            use_fabric=bool(getattr(self.sim, "use_fabric", False)),
            device=str(getattr(self.sim, "device", "")),
            physx_writes_transforms_to_fabric=read_physx_fabric_transform_updates(),
        )

    def _apply_physx_tuning(self):
        """Apply SRB solver and contact tolerances; these settings affect results."""
        if getattr(self.sim, "physics", None) is None:
            from isaaclab_physx.physics import PhysxCfg

            self.sim.physics = PhysxCfg()

        # Contact-rich scenes need at least one velocity iteration.
        self.sim.physics.min_position_iteration_count = 2
        self.sim.physics.min_velocity_iteration_count = 1
        # Prevent small fast bodies from tunneling through thin geometry.
        self.sim.physics.enable_ccd = True
        self.sim.physics.enable_stabilization = False
        # Apply restitution at every impact speed.
        self.sim.physics.bounce_threshold_velocity = 0.0
        # Contact features are centimetre-scale; keep tolerances below that.
        self.sim.physics.friction_offset_threshold = 0.01
        self.sim.physics.friction_correlation_distance = 0.005

    def _enable_tgs_external_force_iterations(self):
        # Apply external forces each iteration to avoid TGS noisy-velocity warnings.
        if getattr(self.sim, "physics", None) is None:
            from isaaclab_physx.physics import PhysxCfg

            self.sim.physics = PhysxCfg()

        self.sim.physics.enable_external_forces_every_iteration = True

    def _setup_asset_extras(self):
        def _recursive_impl(attr: Any):
            if isinstance(attr, Asset):
                attr.setup_extras(self)
            elif isinstance(attr, Mapping):
                for item in attr.values():
                    _recursive_impl(item)
            elif isinstance(attr, Iterable) and not isinstance(attr, (str, bytes)):
                for item in attr:
                    _recursive_impl(item)

        _recursive_impl(iter_config_values(self))

    def _update_procedural_assets(self):
        def _recursive_impl(attr: Any, prim_path: str = ""):
            if isinstance(attr, SpawnerCfg):
                # Upstream defaults are the only unset signal; explicit 0/1
                # values are indistinguishable and may be inherited here.
                if hasattr(attr, "seed"):
                    if attr.seed == 0:  # type: ignore[misc]
                        attr.seed = self.seed  # type: ignore[misc]
                if hasattr(attr, "num_assets"):
                    if not prim_path:
                        raise ValueError(
                            f"{type(attr).__name__} declares num_assets but was "
                            "reached without a prim path, so the per-environment "
                            "variant count cannot be resolved; give the owning "
                            "scene entry a prim_path"
                        )
                    if attr.num_assets == 1 and (  # type: ignore[misc]
                        prim_path.startswith("{ENV_REGEX_NS}")
                        or prim_path.startswith("/World/envs/env_.*")
                    ):
                        attr.num_assets = self.n_procgen_variants or self.scene.num_envs  # type: ignore[misc]
            elif isinstance(attr, AssetBaseCfg):
                if isinstance(attr.spawn, MultiAssetSpawnerCfg):
                    for item in attr.spawn.assets_cfg:
                        _recursive_impl(item, prim_path=attr.prim_path)
                else:
                    _recursive_impl(attr.spawn, prim_path=attr.prim_path)
            elif isinstance(attr, RigidObjectCollectionCfg):
                _recursive_impl(attr.rigid_objects)
            elif isinstance(attr, Mapping):
                for item in attr.values():
                    _recursive_impl(item)
            elif isinstance(attr, Iterable) and not isinstance(attr, (str, bytes)):
                for item in attr:
                    _recursive_impl(item)

        _recursive_impl(iter_config_values(self.scene))

    def _update_debug_vis(self):
        for action_term in iter_config_values_by_type(self.actions, ActionTermCfg):
            action_term.debug_vis = self.debug_vis

        def _recursive_asset_impl(attr: Any):
            if isinstance(attr, SensorBaseCfg):
                # Note: Ignore debug visualization for IMUs
                if not isinstance(attr, ImuCfg):
                    attr.debug_vis = self.debug_vis
                # Cap oversized marker scale on sensors with visualizer configs
                if hasattr(attr, "visualizer_cfg") and isinstance(
                    attr.visualizer_cfg, VisualizationMarkersCfg
                ):
                    for marker in attr.visualizer_cfg.markers.values():
                        if isinstance(marker, dict):
                            if "scale" in marker and marker["scale"] is not None:
                                marker["scale"] = tuple(
                                    min(s, 0.1) for s in marker["scale"]
                                )
                        elif hasattr(marker, "scale") and marker.scale is not None:
                            marker.scale = tuple(min(s, 0.1) for s in marker.scale)
            elif isinstance(attr, Mapping):
                for item in attr.values():
                    _recursive_asset_impl(item)
            elif isinstance(attr, Iterable) and not isinstance(attr, (str, bytes)):
                for item in attr:
                    _recursive_asset_impl(item)

        _recursive_asset_impl(iter_config_values(self.scene))

        def _recursive_marker_impl(attr: Any):
            if isinstance(attr, VisualizationMarkersCfg):
                # Hydra may leave `markers` as MISSING until the task fills it.
                if not isinstance(attr.markers, Mapping):
                    return
                for marker in attr.markers.values():
                    if isinstance(marker, dict):
                        marker["visible"] = self.debug_vis
                    elif hasattr(marker, "visible"):
                        marker.visible = self.debug_vis
                    # Skip MISSING sentinels and Hydra-resolved "???" strings;
                    # the task __post_init__ fills them in after super() runs.
            elif isinstance(attr, Mapping):
                for item in attr.values():
                    _recursive_marker_impl(item)
            elif isinstance(attr, Iterable) and not isinstance(attr, (str, bytes)):
                for item in attr:
                    _recursive_marker_impl(item)

        _recursive_marker_impl(iter_config_values(self))
