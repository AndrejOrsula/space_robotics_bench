"""Isaac-free IO contract for both Raph landscaping tasks.

Defines action and observation schemas, map geometry, policies, serialization,
and target manifests. Simulator limits come from assets, not verified hardware
limits. Gymnasium and task-spec projections load lazily.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any

import numpy as np

from srb.core.particle_layout import build_flat_particle_bed_layout
from srb.utils.digest import sha256_bytes

if TYPE_CHECKING:  # pragma: no cover - typing only, keeps runtime pure
    import gymnasium

    from srb.core.spec.models import TermSpec

CONTRACT_SEMVER = "1.0.0"
CONTRACT_ID = "srb/terrain_landscaping/raph_external_heightmap"

TARGET_ENCODING_ANALYTIC_FLAT_PARTICLE_BED_V1 = "analytic_flat_particle_bed_v1"
RESET_LAYOUT_VERSION_CRATER_PILE_V1 = "crater_pile_v1"
RESET_LAYOUT_VERSION_MOUND_PILE_V1 = "mound_pile_v1"
SURFACE_RULE_ID_SUPPORT_DISC_P90_V1 = "support_disc_p90_v1"

# Anchored on the top-level `srb` package: importing `srb.tasks` (or deeper)
# would pull gymnasium/Isaac task modules into the pure contract path.
_MANIFEST_ANCHOR_PACKAGE = "srb"
_MANIFEST_RELATIVE_PATH = "tasks/mobile/terrain_landscaping/crater_target_manifest.json"
_MOUND_MANIFEST_RELATIVE_PATH = (
    "tasks/mobile/terrain_landscaping/mound_target_manifest.json"
)


def canonical_json(payload: Any) -> str:
    """Canonical JSON with sorted mapping keys; arrays retain semantic order."""
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
        ensure_ascii=False,
    )


def canonical_json_bytes(payload: Any) -> bytes:
    return canonical_json(payload).encode("utf-8")


def _sha256_hex(data: bytes) -> str:
    return sha256_bytes(data)


@dataclass(frozen=True, slots=True)
class ActionComponent:
    name: str
    unit: str
    scale: float
    saturation: float
    normalized_low: float = -1.0
    normalized_high: float = 1.0

    def physical_bounds(self) -> tuple[float, float]:
        bound = abs(self.saturation)
        first, second = (
            max(-bound, min(bound, endpoint * self.scale))
            for endpoint in (self.normalized_low, self.normalized_high)
        )
        return min(first, second), max(first, second)


@dataclass(frozen=True, slots=True)
class ActionTerm:
    key: str
    start: int
    stop: int
    frame: str
    actuation: str
    components: tuple[ActionComponent, ...]

    @property
    def dim(self) -> int:
        return self.stop - self.start


@dataclass(frozen=True, slots=True)
class DriveFilterPolicy:
    accel_lin_max_m_s2: float = 0.8
    accel_ang_max_rad_s2: float = 2.0
    filter_tau_s: float = 0.15
    wheelbase_m: tuple[float, float] = (0.381845, 0.390)
    wheel_radius_m: float = 0.0885


@dataclass(frozen=True, slots=True)
class RaphActionContract:
    robot_kind: str = "raph_rover"
    payload_kind: str = "raph_shovel"
    payload_joint_kind: str = "prismatic"
    shovel_joint_name: str = "shovel_joint"
    shovel_position_range_m: tuple[float, float] = (0.0, 0.2)
    shovel_stowed_position_m: float = 0.2
    control_rate_hz: float = 10.0
    hold: str = "zero_order_hold"
    drive_filter: DriveFilterPolicy = field(default_factory=DriveFilterPolicy)
    terms: tuple[ActionTerm, ...] = (
        ActionTerm(
            key="robot/cmd_vel",
            start=0,
            stop=2,
            frame="rover_body",
            actuation="velocity",
            components=(
                ActionComponent(
                    name="linear",
                    unit="m/s",
                    scale=0.4,
                    saturation=0.4,
                ),
                ActionComponent(
                    name="angular",
                    unit="rad/s",
                    scale=math.radians(60.0),
                    saturation=1.0,
                ),
            ),
        ),
        ActionTerm(
            key="payload/joint_vel",
            start=2,
            stop=3,
            frame="shovel_prismatic_axis",
            actuation="velocity",
            components=(
                ActionComponent(
                    name="prismatic",
                    unit="m/s",
                    scale=0.04,
                    saturation=0.04,
                ),
            ),
        ),
    )

    @property
    def action_dim(self) -> int:
        return self.terms[-1].stop

    @property
    def keys(self) -> tuple[str, ...]:
        return tuple(term.key for term in self.terms)

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "robot_kind": self.robot_kind,
            "payload_kind": self.payload_kind,
            "payload_joint_kind": self.payload_joint_kind,
            "shovel_joint_name": self.shovel_joint_name,
            "shovel_position_range_m": list(self.shovel_position_range_m),
            "shovel_stowed_position_m": self.shovel_stowed_position_m,
            "control_rate_hz": self.control_rate_hz,
            "hold": self.hold,
            "normalized_space": {
                "low": -1.0,
                "high": 1.0,
                "shape": [self.action_dim],
                "dtype": "float32",
            },
            "drive_filter": {
                "accel_lin_max_m_s2": self.drive_filter.accel_lin_max_m_s2,
                "accel_ang_max_rad_s2": self.drive_filter.accel_ang_max_rad_s2,
                "filter_tau_s": self.drive_filter.filter_tau_s,
                "wheelbase_m": list(self.drive_filter.wheelbase_m),
                "wheel_radius_m": self.drive_filter.wheel_radius_m,
            },
            # Explicit array: preserves action order under sorted mapping keys.
            "terms": [
                {
                    "key": term.key,
                    "slice": [term.start, term.stop],
                    "frame": term.frame,
                    "actuation": term.actuation,
                    "components": [
                        {
                            "name": component.name,
                            "unit": component.unit,
                            "normalized_low": component.normalized_low,
                            "normalized_high": component.normalized_high,
                            "scale": component.scale,
                            "saturation": component.saturation,
                            "physical_bounds": list(component.physical_bounds()),
                        }
                        for component in term.components
                    ],
                }
                for term in self.terms
            ],
        }


@dataclass(frozen=True, slots=True)
class SourceGridGeometry:
    cell_size_m: float = 0.05
    source_shape: tuple[int, int] = (146, 146)
    roi_shape: tuple[int, int] = (80, 80)
    guard_cells: int = 33
    lower_edge_m: tuple[float, float] = (-3.65, -3.65)
    axis0: str = "mission_x"
    axis1: str = "mission_y"
    yaw_sign: str = "ccw"
    global_shape: tuple[int, int] = (16, 16)
    global_block: tuple[int, int] = (5, 5)
    global_downsample: str = "area_mean"
    local_shape: tuple[int, int] = (24, 16)
    local_axis0: str = "rover_x_forward"
    local_axis1: str = "rover_y_left"
    local_x_centres_m: tuple[float, float] = (0.425, 1.575)
    local_y_centres_m: tuple[float, float] = (-0.375, 0.375)
    interpolation: str = "bilinear"
    extrapolation_allowed: bool = False

    def __post_init__(self) -> None:
        expected = tuple(roi + 2 * self.guard_cells for roi in self.roi_shape)
        if expected != self.source_shape:
            raise ValueError(
                "source shape must equal ROI plus symmetric guard: "
                f"{expected} != {self.source_shape}"
            )
        for axis in (0, 1):
            if (
                self.roi_shape[axis]
                != self.global_shape[axis] * self.global_block[axis]
            ):
                raise ValueError(
                    "global map must tile the ROI exactly with area-mean blocks"
                )

    @property
    def roi_index_start(self) -> tuple[int, int]:
        return (self.guard_cells, self.guard_cells)

    @property
    def roi_index_stop(self) -> tuple[int, int]:
        return (
            self.guard_cells + self.roi_shape[0],
            self.guard_cells + self.roi_shape[1],
        )

    def cell_centres(self, axis: int) -> np.ndarray:
        n = self.source_shape[axis]
        centres = self.lower_edge_m[axis] + (np.arange(n) + 0.5) * self.cell_size_m
        centres.setflags(write=False)
        return centres

    def local_cell_centres(self) -> tuple[np.ndarray, np.ndarray]:
        nx, ny = self.local_shape
        xs = np.linspace(self.local_x_centres_m[0], self.local_x_centres_m[1], nx)
        ys = np.linspace(self.local_y_centres_m[0], self.local_y_centres_m[1], ny)
        xs.setflags(write=False)
        ys.setflags(write=False)
        return xs, ys

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "cell_size_m": self.cell_size_m,
            "source_shape": list(self.source_shape),
            "roi_shape": list(self.roi_shape),
            "guard_cells": self.guard_cells,
            "lower_edge_m": list(self.lower_edge_m),
            "axis0": self.axis0,
            "axis1": self.axis1,
            "yaw_sign": self.yaw_sign,
            "cell_centre_rule": "lower_edge + (index + 0.5) * cell_size",
            "global_shape": list(self.global_shape),
            "global_block": list(self.global_block),
            "global_downsample": self.global_downsample,
            "local_shape": list(self.local_shape),
            "local_axis0": self.local_axis0,
            "local_axis1": self.local_axis1,
            "local_x_centres_m": list(self.local_x_centres_m),
            "local_y_centres_m": list(self.local_y_centres_m),
            "interpolation": self.interpolation,
            "extrapolation_allowed": self.extrapolation_allowed,
        }


@dataclass(frozen=True, slots=True)
class SurfaceAggregationRule:
    rule_id: str = SURFACE_RULE_ID_SUPPORT_DISC_P90_V1
    particle_radius_m: float = 0.025
    support_radius_m: float = 0.025 + math.sqrt(2.0) * 0.025
    percentile: float = 0.90
    percentile_convention: str = "nearest_rank_ascending_ceil_minus_one"
    radius_offset_m: float = 0.025
    empty_support_fill: str = "base_plane"
    base_plane_elevation_m: float = 0.0
    surface_layer: str = "terrain_work_surface"
    excluded_bodies: tuple[str, ...] = (
        "rover",
        "shovel",
        "containment_wall",
        "transient_occluders",
    )

    def percentile_rank(self, count: int) -> int:
        if count < 1:
            raise ValueError("percentile rank requires a non-empty support set")
        if not 0.0 < self.percentile <= 1.0:
            raise ValueError(f"percentile must be in (0, 1], got {self.percentile}")
        return math.ceil(self.percentile * count) - 1

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "rule_id": self.rule_id,
            "particle_radius_m": self.particle_radius_m,
            "support_radius_m": self.support_radius_m,
            "percentile": self.percentile,
            "percentile_convention": self.percentile_convention,
            "radius_offset_m": self.radius_offset_m,
            "empty_support_fill": self.empty_support_fill,
            "base_plane_elevation_m": self.base_plane_elevation_m,
            "surface_layer": self.surface_layer,
            "excluded_bodies": list(self.excluded_bodies),
        }


@dataclass(frozen=True, slots=True)
class MissionFramePolicy:
    """Same geometry, datum, and transform bind both tasks to this frame.
    Crater name is historical; renaming changes both fingerprints."""

    mission_frame_id: str = "srb/terrain_landscaping_crater/mission_v1"
    elevation_datum_id: str = "containment_base_plane_z0"
    world_to_mission: str = "sim_subtract_env_origin"
    calibration_version: str = "sim_env_origin_v1"

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "mission_frame_id": self.mission_frame_id,
            "elevation_datum_id": self.elevation_datum_id,
            "world_to_mission": self.world_to_mission,
            "calibration_version": self.calibration_version,
        }


@dataclass(frozen=True, slots=True)
class FreshnessPolicy:
    update_rate_hz: float = 10.0
    stale_cutoff_s: float = 0.5
    readiness_timeout_s: float = 2.0
    clock: str = "monotonic_sim_time"
    full_coverage_required: bool = True
    hold_last_on_dropout: bool = True
    reject_nonmonotonic_timestamps: bool = True
    stale_handling: str = "out_of_mdp_pause_or_abort"

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "update_rate_hz": self.update_rate_hz,
            "stale_cutoff_s": self.stale_cutoff_s,
            "readiness_timeout_s": self.readiness_timeout_s,
            "clock": self.clock,
            "full_coverage_required": self.full_coverage_required,
            "hold_last_on_dropout": self.hold_last_on_dropout,
            "reject_nonmonotonic_timestamps": self.reject_nonmonotonic_timestamps,
            "stale_handling": self.stale_handling,
        }


@dataclass(frozen=True, slots=True)
class ObservationLeaf:
    key: str
    shape: tuple[int, ...]
    unit: str
    frame: str
    low: tuple[float, ...]
    high: tuple[float, ...]
    dtype: str = "float32"

    def __post_init__(self) -> None:
        n = len(self.low)
        if len(self.high) != n:
            raise ValueError(f"{self.key}: low/high bound lengths differ")
        if n not in (1, self.size):
            raise ValueError(
                f"{self.key}: bounds must be scalar or per-element, got {n}"
            )
        for lo, hi in zip(self.low, self.high):
            if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
                raise ValueError(
                    f"{self.key}: bounds must be finite and ordered: ({lo}, {hi})"
                )

    @property
    def size(self) -> int:
        return int(np.prod(self.shape))

    def low_array(self) -> np.ndarray:
        arr = np.broadcast_to(
            np.asarray(self.low, dtype=np.float32).reshape(-1)
            if len(self.low) > 1
            else np.float32(self.low[0]),
            (self.size,),
        ).astype(np.float32)
        arr.setflags(write=False)
        return arr

    def high_array(self) -> np.ndarray:
        arr = np.broadcast_to(
            np.asarray(self.high, dtype=np.float32).reshape(-1)
            if len(self.high) > 1
            else np.float32(self.high[0]),
            (self.size,),
        ).astype(np.float32)
        arr.setflags(write=False)
        return arr

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "shape": list(self.shape),
            "dtype": self.dtype,
            "unit": self.unit,
            "frame": self.frame,
            "low": list(self.low),
            "high": list(self.high),
        }


_ELEVATION_LOW_M = -0.30
_ELEVATION_HIGH_M = 0.50

_MISSION_FRAME = MissionFramePolicy()

_MAP_FRAME = _MISSION_FRAME.mission_frame_id
_DATUM = _MISSION_FRAME.elevation_datum_id


def _map_leaf(key: str, shape: tuple[int, int], frame: str) -> ObservationLeaf:
    return ObservationLeaf(
        key=key,
        shape=shape,
        unit="m",
        frame=frame,
        low=(_ELEVATION_LOW_M,),
        high=(_ELEVATION_HIGH_M,),
    )


OBSERVATION_LEAVES: tuple[ObservationLeaf, ...] = (
    _map_leaf("proprio_dyn/heightmap_current_global", (16, 16), _MAP_FRAME),
    _map_leaf("proprio_dyn/heightmap_target_global", (16, 16), _MAP_FRAME),
    _map_leaf("proprio_dyn/heightmap_current_local", (24, 16), "rover_body"),
    _map_leaf("proprio_dyn/heightmap_target_local", (24, 16), "rover_body"),
    ObservationLeaf(
        key="proprio/mission_pose",
        shape=(3,),
        unit="m,m,rad",
        frame=_MAP_FRAME,
        low=(-2.0, -2.0, -math.pi),
        high=(2.0, 2.0, math.pi),
    ),
    ObservationLeaf(
        key="proprio/base_velocity",
        shape=(3,),
        unit="m/s,m/s,rad/s",
        frame="rover_body",
        low=(-1.5, -1.5, -4.0),
        high=(1.5, 1.5, 4.0),
    ),
    ObservationLeaf(
        key="proprio_dyn/heightmap_age_s",
        shape=(1,),
        unit="s",
        frame="none",
        low=(0.0,),
        high=(0.5,),
    ),
)

# Leaves deliberately absent from the actor observation. Kept as
# data so there is one source of truth.
EXCLUDED_ACTOR_LEAVES: tuple[str, ...] = (
    "cell_validity_mask",
    "shovel_extension",
    "previous_action",
    "episode_time_remaining",
    "particle_positions",
    "simulator_world_pose",
    "target_minus_current",
    "imu",
)


@dataclass(frozen=True, slots=True)
class LandscapingIOContract:
    semver: str = CONTRACT_SEMVER
    contract_id: str = CONTRACT_ID
    action: RaphActionContract = field(default_factory=RaphActionContract)
    observation_leaves: tuple[ObservationLeaf, ...] = OBSERVATION_LEAVES
    geometry: SourceGridGeometry = field(default_factory=SourceGridGeometry)
    surface_rule: SurfaceAggregationRule = field(default_factory=SurfaceAggregationRule)
    mission_frame: MissionFramePolicy = field(default_factory=MissionFramePolicy)
    freshness: FreshnessPolicy = field(default_factory=FreshnessPolicy)

    @property
    def observation_keys(self) -> tuple[str, ...]:
        return tuple(leaf.key for leaf in self.observation_leaves)

    @property
    def flat_observation_size(self) -> int:
        return sum(leaf.size for leaf in self.observation_leaves)

    def leaf(self, key: str) -> ObservationLeaf:
        for candidate in self.observation_leaves:
            if candidate.key == key:
                return candidate
        raise KeyError(f"unknown observation leaf: {key!r}")

    def flattened_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        low = np.concatenate(
            [leaf.low_array() for leaf in self.observation_leaves]
        ).astype(np.float32)
        high = np.concatenate(
            [leaf.high_array() for leaf in self.observation_leaves]
        ).astype(np.float32)
        low.setflags(write=False)
        high.setflags(write=False)
        return low, high

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "contract_id": self.contract_id,
            "semver": self.semver,
            "action": self.action.canonical_dict(),
            # Explicit array preserves leaf order under sorted-key JSON.
            "observation_leaves": [
                leaf.canonical_dict() for leaf in self.observation_leaves
            ],
            "excluded_actor_leaves": list(EXCLUDED_ACTOR_LEAVES),
            "flat_observation_size": self.flat_observation_size,
            "geometry": self.geometry.canonical_dict(),
            "surface_rule": self.surface_rule.canonical_dict(),
            "mission_frame": self.mission_frame.canonical_dict(),
            "freshness": self.freshness.canonical_dict(),
        }

    def canonical_json(self) -> str:
        return canonical_json(self.canonical_dict())

    def fingerprint(self) -> str:
        return _sha256_hex(canonical_json_bytes(self.canonical_dict()))

    # -- Projections (lazy imports keep the module Isaac/Torch/ROS/JAX-free) --

    def to_observation_term_specs(self) -> tuple[tuple[str, "TermSpec"], ...]:
        from srb.core.spec.models import TermSpec

        specs: list[tuple[str, TermSpec]] = []
        for leaf in self.observation_leaves:
            low = list(leaf.low)
            high = list(leaf.high)
            specs.append(
                (
                    leaf.key,
                    TermSpec(
                        name=leaf.key.split("/")[-1],
                        shape=leaf.shape,
                        dtype="float32",
                        units=leaf.unit,
                        frame=leaf.frame,
                        min=low[0] if len(low) == 1 else low,
                        max=high[0] if len(high) == 1 else high,
                        privileged=False,
                        provenance=self.contract_id,
                    ),
                )
            )
        return tuple(specs)

    def to_action_term_specs(self) -> tuple[tuple[str, "TermSpec"], ...]:
        from srb.core.spec.models import TermSpec

        specs: list[tuple[str, TermSpec]] = []
        for term in self.action.terms:
            specs.append(
                (
                    term.key,
                    TermSpec(
                        name=term.key.split("/")[-1],
                        shape=(term.dim,),
                        dtype="float32",
                        units=",".join(c.unit for c in term.components),
                        frame=term.frame,
                        min=-1.0,
                        max=1.0,
                        privileged=False,
                        provenance=self.contract_id,
                    ),
                )
            )
        return tuple(specs)

    def to_gym_observation_spaces(
        self,
    ) -> tuple[tuple[str, "gymnasium.spaces.Box"], ...]:
        import gymnasium

        spaces: list[tuple[str, gymnasium.spaces.Box]] = []
        for leaf in self.observation_leaves:
            spaces.append(
                (
                    leaf.key,
                    gymnasium.spaces.Box(
                        low=leaf.low_array().reshape(leaf.shape),
                        high=leaf.high_array().reshape(leaf.shape),
                        shape=leaf.shape,
                        dtype=np.float32,
                    ),
                )
            )
        return tuple(spaces)

    def to_gym_flattened_observation_space(self) -> "gymnasium.spaces.Box":
        import gymnasium

        low, high = self.flattened_bounds()
        return gymnasium.spaces.Box(
            low=low.copy(),
            high=high.copy(),
            shape=(self.flat_observation_size,),
            dtype=np.float32,
        )

    def to_gym_action_space(self) -> "gymnasium.spaces.Box":
        import gymnasium

        return gymnasium.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.action.action_dim,),
            dtype=np.float32,
        )


LANDSCAPING_IO_CONTRACT = LandscapingIOContract()


def landscaping_deployment_recipe(robot: Any) -> dict[str, str]:
    """Validate effective bound assets without changing the frozen IO schema.
    Callers pass the bound robot; missing selections are rejected."""
    payload = getattr(robot, "payload", None)
    recipe: dict[str, str] = {}
    for role, asset, expected in (
        ("robot", robot, LANDSCAPING_IO_CONTRACT.action.robot_kind),
        ("payload", payload, LANDSCAPING_IO_CONTRACT.action.payload_kind),
    ):
        name_fn = getattr(asset, "name", None)
        name = name_fn() if callable(name_fn) else None
        if not isinstance(name, str) or name != expected:
            raise ValueError(
                f"canonical landscaping {role}_kind requires {expected!r}, got {name!r}"
            )
        recipe[f"{role}_kind"] = name
    for role, asset, expected in (
        ("robot", robot, "drive_filtered"),
        ("payload", payload, "joint_vel"),
    ):
        mode = getattr(asset, "action_mode", None)
        if not isinstance(mode, str) or mode != expected:
            raise ValueError(
                f"canonical landscaping {role}_action_mode requires {expected!r}, "
                f"got {mode!r}"
            )
        recipe[f"{role}_action_mode"] = mode
    return recipe


def downsample_global_from_source(
    source: np.ndarray,
    *,
    geometry: SourceGridGeometry = LANDSCAPING_IO_CONTRACT.geometry,
) -> np.ndarray:
    source = np.asarray(source, dtype=np.float32)
    if source.shape != geometry.source_shape:
        raise ValueError(f"source shape {source.shape} != {geometry.source_shape}")
    r0, c0 = geometry.roi_index_start
    r1, c1 = geometry.roi_index_stop
    roi = source[r0:r1, c0:c1]
    g0, g1 = geometry.global_shape
    b0, b1 = geometry.global_block
    out = roi.reshape(g0, b0, g1, b1).mean(axis=(1, 3)).astype(np.float32)
    out.setflags(write=False)
    return out


def sample_local_from_source(
    source: np.ndarray,
    *,
    mission_pose: tuple[float, float, float],
    geometry: SourceGridGeometry = LANDSCAPING_IO_CONTRACT.geometry,
) -> np.ndarray:
    """Sample in rover coordinates: +x forward, +y left; yaw is CCW.
    Raises if bilinear sampling would extrapolate."""
    source = np.asarray(source, dtype=np.float32)
    if source.shape != geometry.source_shape:
        raise ValueError(f"source shape {source.shape} != {geometry.source_shape}")
    x, y, yaw = (float(v) for v in mission_pose)
    xs, ys = geometry.local_cell_centres()
    lx, ly = np.meshgrid(xs, ys, indexing="ij")
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    wx = x + lx * cos_yaw - ly * sin_yaw
    wy = y + lx * sin_yaw + ly * cos_yaw

    cell = geometry.cell_size_m
    fx = (wx - geometry.lower_edge_m[0]) / cell - 0.5
    fy = (wy - geometry.lower_edge_m[1]) / cell - 0.5
    n0, n1 = geometry.source_shape
    if fx.min() < 0.0 or fy.min() < 0.0 or fx.max() > n0 - 1 or fy.max() > n1 - 1:
        raise ValueError(
            "local crop requires extrapolation outside the source grid; "
            "the guard band no longer covers the requested pose"
        )
    x0 = np.floor(fx).astype(np.int64)
    y0 = np.floor(fy).astype(np.int64)
    x1 = np.minimum(x0 + 1, n0 - 1)
    y1 = np.minimum(y0 + 1, n1 - 1)
    tx = (fx - x0).astype(np.float32)
    ty = (fy - y0).astype(np.float32)
    v00 = source[x0, y0]
    v10 = source[x1, y0]
    v01 = source[x0, y1]
    v11 = source[x1, y1]
    out = (
        v00 * (1.0 - tx) * (1.0 - ty)
        + v10 * tx * (1.0 - ty)
        + v01 * (1.0 - tx) * ty
        + v11 * tx * ty
    ).astype(np.float32)
    out.setflags(write=False)
    return out


def materialize_surface_raster(
    particle_centres_xyz: np.ndarray,
    *,
    geometry: SourceGridGeometry = LANDSCAPING_IO_CONTRACT.geometry,
    rule: SurfaceAggregationRule = LANDSCAPING_IO_CONTRACT.surface_rule,
) -> tuple[np.ndarray, np.ndarray]:
    """Return float32 surface elevations and int64 support counts; zero means base-plane
    fill.
    """
    centres = np.asarray(particle_centres_xyz, dtype=np.float64)
    if centres.ndim != 2 or centres.shape[1] != 3:
        raise ValueError("particle centres must have shape (N, 3)")
    xs = geometry.cell_centres(0)
    ys = geometry.cell_centres(1)
    surface = np.full(
        geometry.source_shape, rule.base_plane_elevation_m, dtype=np.float32
    )
    support_count = np.zeros(geometry.source_shape, dtype=np.int64)
    px = centres[:, 0]
    py = centres[:, 1]
    pz = centres[:, 2]
    radius = rule.support_radius_m
    radius_sq = radius * radius
    order = np.argsort(px, kind="stable")
    px_sorted = px[order]
    py_sorted = py[order]
    pz_sorted = pz[order]
    for i, cx in enumerate(xs):
        lo = np.searchsorted(px_sorted, cx - radius, side="left")
        hi = np.searchsorted(px_sorted, cx + radius, side="right")
        if lo == hi:
            continue
        dx = px_sorted[lo:hi] - cx
        row_py = py_sorted[lo:hi]
        row_pz = pz_sorted[lo:hi]
        dx_sq = dx * dx
        for j, cy in enumerate(ys):
            dy = row_py - cy
            mask = dx_sq + dy * dy <= radius_sq
            count = int(np.count_nonzero(mask))
            if count == 0:
                continue
            z_support = np.sort(row_pz[mask])
            rank = rule.percentile_rank(count)
            surface[i, j] = np.float32(z_support[rank] + rule.radius_offset_m)
            support_count[i, j] = count
    surface.setflags(write=False)
    support_count.setflags(write=False)
    return surface, support_count


@dataclass(frozen=True, slots=True)
class FlatParticleBedParams:
    dim_x: int
    dim_y: int
    dim_z: int
    particle_size_m: float
    particle_spacing_m: float
    ratio: float
    translation_m: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "dim_x": self.dim_x,
            "dim_y": self.dim_y,
            "dim_z": self.dim_z,
            "particle_size_m": self.particle_size_m,
            "particle_spacing_m": self.particle_spacing_m,
            "ratio": self.ratio,
            "translation_m": list(self.translation_m),
            "orientation_wxyz": list(self.orientation_wxyz),
        }

    def world_particle_centres(self) -> np.ndarray:
        if tuple(self.orientation_wxyz) != (1.0, 0.0, 0.0, 0.0):
            raise ValueError(
                "analytic flat-bed materialization supports identity orientation only"
            )
        layout = build_flat_particle_bed_layout(
            dim_x=self.dim_x,
            dim_y=self.dim_y,
            dim_z=self.dim_z,
            particle_size=self.particle_size_m,
            particle_spacing=self.particle_spacing_m,
            ratio=self.ratio,
        )
        centres = layout.positions.astype(np.float64) + np.asarray(
            self.translation_m, dtype=np.float64
        )
        centres.setflags(write=False)
        return centres


@dataclass(frozen=True, slots=True)
class ResetLayoutParams:
    version: str
    spawner: str
    dim_x: int
    dim_y: int
    dim_z: int
    particle_size_m: float
    particle_spacing_m: float
    ratio: float
    crater_center_xy_m: tuple[float, float]
    crater_radius_m: float
    crater_depth_m: float
    pile_center_xy_m: tuple[float, float]
    pile_radius_m: float
    pile_height_m: float
    translation_m: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    velocity_m_s: tuple[float, float, float]

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "spawner": self.spawner,
            "dim_x": self.dim_x,
            "dim_y": self.dim_y,
            "dim_z": self.dim_z,
            "particle_size_m": self.particle_size_m,
            "particle_spacing_m": self.particle_spacing_m,
            "ratio": self.ratio,
            "crater_center_xy_m": list(self.crater_center_xy_m),
            "crater_radius_m": self.crater_radius_m,
            "crater_depth_m": self.crater_depth_m,
            "pile_center_xy_m": list(self.pile_center_xy_m),
            "pile_radius_m": self.pile_radius_m,
            "pile_height_m": self.pile_height_m,
            "translation_m": list(self.translation_m),
            "orientation_wxyz": list(self.orientation_wxyz),
            "velocity_m_s": list(self.velocity_m_s),
        }


@dataclass(frozen=True, slots=True)
class SuccessParams:
    success_tolerance_m: float = 0.02
    fraction_within_tolerance: float = 0.90
    sustained_frames: int = 20

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "success_tolerance_m": self.success_tolerance_m,
            "fraction_within_tolerance": self.fraction_within_tolerance,
            "sustained_frames": self.sustained_frames,
        }


@dataclass(frozen=True, slots=True)
class TargetMapMaterialization:
    target: np.ndarray
    support_count: np.ndarray
    grading_indices: np.ndarray


TARGET_MANIFEST_SCHEMA_VERSION = 1


@dataclass(frozen=True, slots=True)
class TargetMapManifest:
    schema_version: int
    manifest_id: str
    task_id: str
    mission_frame: MissionFramePolicy
    geometry: SourceGridGeometry
    surface_rule: SurfaceAggregationRule
    target_encoding: str
    flat_bed: FlatParticleBedParams
    reset_layout: ResetLayoutParams
    success: SuccessParams
    elevation_bounds_m: tuple[float, float]
    grading_cell_count: int
    target_map_sha256: str
    reset_layout_sha256: str
    io_schema_fingerprint: str
    manifest_sha256: str

    def canonical_dict(self, *, include_manifest_hash: bool = True) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": self.schema_version,
            "manifest_id": self.manifest_id,
            "task_id": self.task_id,
            "mission_frame": self.mission_frame.canonical_dict(),
            "grid_geometry": self.geometry.canonical_dict(),
            "surface_rule": self.surface_rule.canonical_dict(),
            "target": {
                "encoding": self.target_encoding,
                "flat_bed": self.flat_bed.canonical_dict(),
                "grading_cell_count": self.grading_cell_count,
                "target_map_sha256": self.target_map_sha256,
            },
            "reset_layout": {
                **self.reset_layout.canonical_dict(),
                "reset_layout_sha256": self.reset_layout_sha256,
            },
            "success": self.success.canonical_dict(),
            "elevation_bounds_m": list(self.elevation_bounds_m),
            "io_schema_fingerprint": self.io_schema_fingerprint,
        }
        if include_manifest_hash:
            payload["manifest_sha256"] = self.manifest_sha256
        return payload

    def canonical_json(self) -> str:
        return canonical_json(self.canonical_dict())

    def materialize(self) -> TargetMapMaterialization:
        """Build from the versioned flat lattice and surface rule, never a runtime
        reset. Grading indices are supported cells in the central ROI.
        """
        centres = self.flat_bed.world_particle_centres()
        target, support_count = materialize_surface_raster(
            centres,
            geometry=self.geometry,
            rule=self.surface_rule,
        )
        r0, c0 = self.geometry.roi_index_start
        r1, c1 = self.geometry.roi_index_stop
        roi_mask = np.zeros(self.geometry.source_shape, dtype=bool)
        roi_mask[r0:r1, c0:c1] = True
        grading = np.argwhere((support_count > 0) & roi_mask).astype(np.int64)
        grading.setflags(write=False)
        return TargetMapMaterialization(
            target=target,
            support_count=support_count,
            grading_indices=grading,
        )

    def verify(
        self,
        *,
        contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
    ) -> TargetMapMaterialization:
        """Common verification entry point for static and per-episode manifests."""
        return verify_target_manifest(self, contract=contract)


def compute_target_map_sha256(geometry: SourceGridGeometry, target: np.ndarray) -> str:
    """Hash canonical geometry, NUL, then target bytes as little-endian, C-contiguous
    float32.
    """
    target32 = np.ascontiguousarray(np.asarray(target, dtype="<f4"))
    return _sha256_hex(
        canonical_json_bytes(geometry.canonical_dict())
        + b"\x00"
        + target32.tobytes(order="C")
    )


def compute_reset_layout_sha256(reset_layout: ResetLayoutParams) -> str:
    return _sha256_hex(canonical_json_bytes(reset_layout.canonical_dict()))


def compute_manifest_sha256(manifest: TargetMapManifest) -> str:
    return _sha256_hex(
        canonical_json_bytes(manifest.canonical_dict(include_manifest_hash=False))
    )


CRATER_RESET_LAYOUT = ResetLayoutParams(
    version=RESET_LAYOUT_VERSION_CRATER_PILE_V1,
    spawner="crater_pile",
    dim_x=48,
    dim_y=48,
    dim_z=3,
    particle_size_m=0.05,
    particle_spacing_m=0.0,
    ratio=0.75,
    crater_center_xy_m=(0.45, 0.20),
    crater_radius_m=0.36,
    crater_depth_m=0.11,
    pile_center_xy_m=(-0.25, -0.20),
    pile_radius_m=0.32,
    pile_height_m=0.28,
    translation_m=(0.0, 0.0, 0.025),
    orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    velocity_m_s=(0.0, 0.0, 0.0),
)

CRATER_FLAT_BED = FlatParticleBedParams(
    dim_x=CRATER_RESET_LAYOUT.dim_x,
    dim_y=CRATER_RESET_LAYOUT.dim_y,
    dim_z=CRATER_RESET_LAYOUT.dim_z,
    particle_size_m=CRATER_RESET_LAYOUT.particle_size_m,
    particle_spacing_m=CRATER_RESET_LAYOUT.particle_spacing_m,
    ratio=CRATER_RESET_LAYOUT.ratio,
    translation_m=CRATER_RESET_LAYOUT.translation_m,
    orientation_wxyz=CRATER_RESET_LAYOUT.orientation_wxyz,
)


# The zeroed crater fields are sentinels; version selects the layout.
MOUND_RESET_LAYOUT = ResetLayoutParams(
    version=RESET_LAYOUT_VERSION_MOUND_PILE_V1,
    spawner="mound_pile",
    dim_x=48,
    dim_y=48,
    dim_z=3,
    particle_size_m=0.05,
    particle_spacing_m=0.0,
    ratio=0.75,
    crater_center_xy_m=(0.0, 0.0),
    crater_radius_m=0.0,
    crater_depth_m=0.0,
    pile_center_xy_m=(-0.25, -0.20),
    pile_radius_m=0.32,
    # 0.28 m transient reached 0.507 m, beyond [-0.3, 0.5]; 0.24 m uses four layers.
    pile_height_m=0.24,
    translation_m=(0.0, 0.0, 0.025),
    orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    velocity_m_s=(0.0, 0.0, 0.0),
)

MOUND_FLAT_BED = FlatParticleBedParams(
    dim_x=MOUND_RESET_LAYOUT.dim_x,
    dim_y=MOUND_RESET_LAYOUT.dim_y,
    dim_z=MOUND_RESET_LAYOUT.dim_z,
    particle_size_m=MOUND_RESET_LAYOUT.particle_size_m,
    particle_spacing_m=MOUND_RESET_LAYOUT.particle_spacing_m,
    ratio=MOUND_RESET_LAYOUT.ratio,
    translation_m=MOUND_RESET_LAYOUT.translation_m,
    orientation_wxyz=MOUND_RESET_LAYOUT.orientation_wxyz,
)


def build_mound_target_manifest(
    *,
    contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
) -> TargetMapManifest:
    provisional = TargetMapManifest(
        schema_version=TARGET_MANIFEST_SCHEMA_VERSION,
        manifest_id="srb/terrain_landscaping_mound/target_manifest_v1",
        task_id="terrain_landscaping_mound",
        mission_frame=contract.mission_frame,
        geometry=contract.geometry,
        surface_rule=contract.surface_rule,
        target_encoding=TARGET_ENCODING_ANALYTIC_FLAT_PARTICLE_BED_V1,
        flat_bed=MOUND_FLAT_BED,
        reset_layout=MOUND_RESET_LAYOUT,
        success=SuccessParams(),
        elevation_bounds_m=(_ELEVATION_LOW_M, _ELEVATION_HIGH_M),
        grading_cell_count=0,
        target_map_sha256="",
        reset_layout_sha256="",
        io_schema_fingerprint=contract.fingerprint(),
        manifest_sha256="",
    )
    materialized = provisional.materialize()
    hashed = replace(
        provisional,
        grading_cell_count=int(materialized.grading_indices.shape[0]),
        target_map_sha256=compute_target_map_sha256(
            provisional.geometry, materialized.target
        ),
        reset_layout_sha256=compute_reset_layout_sha256(provisional.reset_layout),
    )
    return replace(hashed, manifest_sha256=compute_manifest_sha256(hashed))


def analytic_mound_initial_excess_volume_m3(manifest: TargetMapManifest) -> float:
    """Compute analytic, unsettled excess volume for the bounded-reward normalizer.
    Settling changes the bed, so this is not a runtime measurement."""
    from srb.core.particle_layout import build_mound_pile_particle_layout

    if manifest.reset_layout.version != RESET_LAYOUT_VERSION_MOUND_PILE_V1:
        raise ValueError(
            "analytic mound excess volume requires a "
            f"{RESET_LAYOUT_VERSION_MOUND_PILE_V1!r} reset layout, got "
            f"{manifest.reset_layout.version!r}"
        )
    layout = manifest.reset_layout
    built = build_mound_pile_particle_layout(
        dim_x=layout.dim_x,
        dim_y=layout.dim_y,
        dim_z=layout.dim_z,
        particle_size=layout.particle_size_m,
        particle_spacing=layout.particle_spacing_m,
        ratio=layout.ratio,
        pile_center_xy_m=layout.pile_center_xy_m,
        pile_radius_m=layout.pile_radius_m,
        pile_height_m=layout.pile_height_m,
    )
    world = built.positions.astype(np.float64) + np.asarray(
        layout.translation_m, dtype=np.float64
    )
    surface, _support = materialize_surface_raster(
        world, geometry=manifest.geometry, rule=manifest.surface_rule
    )
    materialized = manifest.materialize()
    grading = materialized.grading_indices
    signed = (
        surface[grading[:, 0], grading[:, 1]]
        - materialized.target[grading[:, 0], grading[:, 1]]
    )
    cell_area = float(manifest.geometry.cell_size_m) ** 2
    excess = float(np.clip(signed, 0.0, None).sum() * cell_area)
    if not excess > 0.0:
        raise ValueError("analytic mound excess volume must be positive")
    return excess


def build_crater_target_manifest(
    *,
    contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
) -> TargetMapManifest:
    provisional = TargetMapManifest(
        schema_version=TARGET_MANIFEST_SCHEMA_VERSION,
        manifest_id="srb/terrain_landscaping_crater/target_manifest_v1",
        task_id="terrain_landscaping_crater",
        mission_frame=contract.mission_frame,
        geometry=contract.geometry,
        surface_rule=contract.surface_rule,
        target_encoding=TARGET_ENCODING_ANALYTIC_FLAT_PARTICLE_BED_V1,
        flat_bed=CRATER_FLAT_BED,
        reset_layout=CRATER_RESET_LAYOUT,
        success=SuccessParams(),
        elevation_bounds_m=(_ELEVATION_LOW_M, _ELEVATION_HIGH_M),
        grading_cell_count=0,
        target_map_sha256="",
        reset_layout_sha256="",
        io_schema_fingerprint=contract.fingerprint(),
        manifest_sha256="",
    )
    materialized = provisional.materialize()
    hashed = replace(
        provisional,
        grading_cell_count=int(materialized.grading_indices.shape[0]),
        target_map_sha256=compute_target_map_sha256(
            provisional.geometry, materialized.target
        ),
        reset_layout_sha256=compute_reset_layout_sha256(provisional.reset_layout),
    )
    return replace(hashed, manifest_sha256=compute_manifest_sha256(hashed))


def manifest_from_payload(payload: dict[str, Any]) -> TargetMapManifest:
    mission = payload["mission_frame"]
    geometry = payload["grid_geometry"]
    rule = payload["surface_rule"]
    target = payload["target"]
    flat_bed = target["flat_bed"]
    reset_layout = payload["reset_layout"]
    success = payload["success"]
    return TargetMapManifest(
        schema_version=int(payload["schema_version"]),
        manifest_id=str(payload["manifest_id"]),
        task_id=str(payload["task_id"]),
        mission_frame=MissionFramePolicy(
            mission_frame_id=mission["mission_frame_id"],
            elevation_datum_id=mission["elevation_datum_id"],
            world_to_mission=mission["world_to_mission"],
            calibration_version=mission["calibration_version"],
        ),
        geometry=SourceGridGeometry(
            cell_size_m=float(geometry["cell_size_m"]),
            source_shape=tuple(geometry["source_shape"]),
            roi_shape=tuple(geometry["roi_shape"]),
            guard_cells=int(geometry["guard_cells"]),
            lower_edge_m=tuple(geometry["lower_edge_m"]),
            axis0=geometry["axis0"],
            axis1=geometry["axis1"],
            yaw_sign=geometry["yaw_sign"],
            global_shape=tuple(geometry["global_shape"]),
            global_block=tuple(geometry["global_block"]),
            global_downsample=geometry["global_downsample"],
            local_shape=tuple(geometry["local_shape"]),
            local_axis0=geometry["local_axis0"],
            local_axis1=geometry["local_axis1"],
            local_x_centres_m=tuple(geometry["local_x_centres_m"]),
            local_y_centres_m=tuple(geometry["local_y_centres_m"]),
            interpolation=geometry["interpolation"],
            extrapolation_allowed=bool(geometry["extrapolation_allowed"]),
        ),
        surface_rule=SurfaceAggregationRule(
            rule_id=rule["rule_id"],
            particle_radius_m=float(rule["particle_radius_m"]),
            support_radius_m=float(rule["support_radius_m"]),
            percentile=float(rule["percentile"]),
            percentile_convention=rule["percentile_convention"],
            radius_offset_m=float(rule["radius_offset_m"]),
            empty_support_fill=rule["empty_support_fill"],
            base_plane_elevation_m=float(rule["base_plane_elevation_m"]),
            surface_layer=rule["surface_layer"],
            excluded_bodies=tuple(rule["excluded_bodies"]),
        ),
        target_encoding=str(target["encoding"]),
        flat_bed=FlatParticleBedParams(
            dim_x=int(flat_bed["dim_x"]),
            dim_y=int(flat_bed["dim_y"]),
            dim_z=int(flat_bed["dim_z"]),
            particle_size_m=float(flat_bed["particle_size_m"]),
            particle_spacing_m=float(flat_bed["particle_spacing_m"]),
            ratio=float(flat_bed["ratio"]),
            translation_m=tuple(flat_bed["translation_m"]),
            orientation_wxyz=tuple(flat_bed["orientation_wxyz"]),
        ),
        reset_layout=ResetLayoutParams(
            version=reset_layout["version"],
            spawner=reset_layout["spawner"],
            dim_x=int(reset_layout["dim_x"]),
            dim_y=int(reset_layout["dim_y"]),
            dim_z=int(reset_layout["dim_z"]),
            particle_size_m=float(reset_layout["particle_size_m"]),
            particle_spacing_m=float(reset_layout["particle_spacing_m"]),
            ratio=float(reset_layout["ratio"]),
            crater_center_xy_m=tuple(reset_layout["crater_center_xy_m"]),
            crater_radius_m=float(reset_layout["crater_radius_m"]),
            crater_depth_m=float(reset_layout["crater_depth_m"]),
            pile_center_xy_m=tuple(reset_layout["pile_center_xy_m"]),
            pile_radius_m=float(reset_layout["pile_radius_m"]),
            pile_height_m=float(reset_layout["pile_height_m"]),
            translation_m=tuple(reset_layout["translation_m"]),
            orientation_wxyz=tuple(reset_layout["orientation_wxyz"]),
            velocity_m_s=tuple(reset_layout["velocity_m_s"]),
        ),
        success=SuccessParams(
            success_tolerance_m=float(success["success_tolerance_m"]),
            fraction_within_tolerance=float(success["fraction_within_tolerance"]),
            sustained_frames=int(success["sustained_frames"]),
        ),
        elevation_bounds_m=tuple(payload["elevation_bounds_m"]),
        grading_cell_count=int(target["grading_cell_count"]),
        target_map_sha256=str(target["target_map_sha256"]),
        reset_layout_sha256=str(reset_layout["reset_layout_sha256"]),
        io_schema_fingerprint=str(payload["io_schema_fingerprint"]),
        manifest_sha256=str(payload["manifest_sha256"]),
    )


def _load_checked_in_manifest(relative_path: str) -> TargetMapManifest:
    """Load via top-level ``srb`` resources without importing task modules.
    Fall back to this module’s path for spec-less test namespace stubs."""
    from importlib import resources

    try:
        resource: Any = resources.files(_MANIFEST_ANCHOR_PACKAGE).joinpath(
            relative_path
        )
    except (AttributeError, TypeError, ValueError):
        from pathlib import Path

        resource = Path(__file__).resolve().parents[1] / relative_path
    payload = json.loads(resource.read_text(encoding="utf-8"))
    return manifest_from_payload(payload)


def load_crater_target_manifest() -> TargetMapManifest:
    return _load_checked_in_manifest(_MANIFEST_RELATIVE_PATH)


def load_mound_target_manifest() -> TargetMapManifest:
    return _load_checked_in_manifest(_MOUND_MANIFEST_RELATIVE_PATH)


def verify_target_manifest(
    manifest: TargetMapManifest,
    *,
    contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
) -> TargetMapMaterialization:
    """Materialize the target and verify all hashes and contract fields.
    Raises ``ValueError`` on mismatch."""
    if manifest.schema_version != TARGET_MANIFEST_SCHEMA_VERSION:
        # Unknown versions may encode targets differently; never materialize them.
        raise ValueError(
            f"unsupported target manifest schema_version {manifest.schema_version!r}; "
            f"expected {TARGET_MANIFEST_SCHEMA_VERSION}"
        )
    errors: list[str] = []
    if manifest.geometry != contract.geometry:
        errors.append("manifest grid geometry differs from the IO contract")
    if manifest.surface_rule != contract.surface_rule:
        errors.append("manifest surface rule differs from the IO contract")
    if manifest.mission_frame != contract.mission_frame:
        errors.append("manifest mission frame differs from the IO contract")
    if manifest.io_schema_fingerprint != contract.fingerprint():
        errors.append("io_schema_fingerprint does not match the IO contract")
    if manifest.target_encoding != TARGET_ENCODING_ANALYTIC_FLAT_PARTICLE_BED_V1:
        errors.append(f"unsupported target encoding {manifest.target_encoding!r}")

    materialized = manifest.materialize()
    expected_target_hash = compute_target_map_sha256(
        manifest.geometry, materialized.target
    )
    if manifest.target_map_sha256 != expected_target_hash:
        errors.append("target_map_sha256 does not match the analytic target")
    expected_reset_hash = compute_reset_layout_sha256(manifest.reset_layout)
    if manifest.reset_layout_sha256 != expected_reset_hash:
        errors.append("reset_layout_sha256 does not match the reset layout")
    expected_manifest_hash = compute_manifest_sha256(manifest)
    if manifest.manifest_sha256 != expected_manifest_hash:
        errors.append("manifest_sha256 does not match the canonical manifest")
    if manifest.grading_cell_count != int(materialized.grading_indices.shape[0]):
        errors.append("grading_cell_count does not match materialized indices")
    lo, hi = manifest.elevation_bounds_m
    if float(materialized.target.min()) < lo or float(materialized.target.max()) > hi:
        errors.append("materialized target violates the declared elevation bounds")
    if errors:
        raise ValueError("target manifest verification failed: " + "; ".join(errors))
    return materialized


__all__ = [
    "CONTRACT_ID",
    "CONTRACT_SEMVER",
    "CRATER_FLAT_BED",
    "CRATER_RESET_LAYOUT",
    "MOUND_FLAT_BED",
    "MOUND_RESET_LAYOUT",
    "EXCLUDED_ACTOR_LEAVES",
    "LANDSCAPING_IO_CONTRACT",
    "OBSERVATION_LEAVES",
    "RESET_LAYOUT_VERSION_CRATER_PILE_V1",
    "RESET_LAYOUT_VERSION_MOUND_PILE_V1",
    "SURFACE_RULE_ID_SUPPORT_DISC_P90_V1",
    "TARGET_ENCODING_ANALYTIC_FLAT_PARTICLE_BED_V1",
    "TARGET_MANIFEST_SCHEMA_VERSION",
    "ActionComponent",
    "ActionTerm",
    "DriveFilterPolicy",
    "FlatParticleBedParams",
    "FreshnessPolicy",
    "LandscapingIOContract",
    "MissionFramePolicy",
    "ObservationLeaf",
    "RaphActionContract",
    "ResetLayoutParams",
    "SourceGridGeometry",
    "SuccessParams",
    "SurfaceAggregationRule",
    "TargetMapManifest",
    "TargetMapMaterialization",
    "analytic_mound_initial_excess_volume_m3",
    "build_crater_target_manifest",
    "build_mound_target_manifest",
    "canonical_json",
    "canonical_json_bytes",
    "compute_manifest_sha256",
    "compute_reset_layout_sha256",
    "compute_target_map_sha256",
    "downsample_global_from_source",
    "landscaping_deployment_recipe",
    "load_crater_target_manifest",
    "load_mound_target_manifest",
    "manifest_from_payload",
    "materialize_surface_raster",
    "sample_local_from_source",
    "verify_target_manifest",
]
