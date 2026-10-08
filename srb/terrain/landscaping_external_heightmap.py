"""Isaac-free terrain source seam for the seven-leaf actor observation.

Reset waits for a complete frame per selected environment; reset frames
bypass simulated delay/drop but retain measurement noise and registration.
Observe returns an accepted-or-held frame at the 10 Hz source rate. The seam
transfers owned arrays through :data:`SourceFrameProvider`; geometry, frames,
and freshness follow :mod:`srb.terrain.landscaping_contract`.
"""

from __future__ import annotations

import hashlib
import math
import time
from collections import OrderedDict
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from srb.terrain.landscaping_contract import (
    LANDSCAPING_IO_CONTRACT,
    FreshnessPolicy,
    LandscapingIOContract,
    MissionFramePolicy,
    SourceGridGeometry,
    SurfaceAggregationRule,
    TargetMapMaterialization,
    canonical_json_bytes,
    downsample_global_from_source,
    sample_local_from_source,
)


class TargetManifest(Protocol):
    """Manifest interface used to verify and materialize target maps.

    Optional ``canonical_json()`` enables content-verified caching.
    """

    @property
    def manifest_sha256(self) -> str: ...

    def verify(
        self, *, contract: LandscapingIOContract = ...
    ) -> TargetMapMaterialization: ...


PROFILE_IDEAL = "ideal"
PROFILE_RANDOMIZED = "randomized"
PROFILE_FAILURE_STALE = "failure_stale"
PROFILE_NAMES = (PROFILE_IDEAL, PROFILE_RANDOMIZED, PROFILE_FAILURE_STALE)


class ExternalHeightmapError(RuntimeError):
    """Base error for the external-heightmap seam."""


class ExternalHeightmapReadinessError(ExternalHeightmapError):
    """Reset readiness timeout: some environments never produced a frame."""

    def __init__(self, failed_env_ids: Sequence[int], timeout_s: float) -> None:
        self.failed_env_ids = tuple(int(i) for i in failed_env_ids)
        self.timeout_s = float(timeout_s)
        super().__init__(
            "external heightmap readiness timeout after "
            f"{self.timeout_s:.3f} s for env ids {list(self.failed_env_ids)}; "
            "no zero startup frame is ever substituted"
        )


class ExternalHeightmapFrameError(ExternalHeightmapError):
    """A produced frame violated the frozen frame contract."""


class ExternalHeightmapStaleError(ExternalHeightmapError):
    """A frame crossed the hard stale cutoff (out-of-MDP pause/abort)."""

    def __init__(self, env_ids: Sequence[int], ages_s: Sequence[float]) -> None:
        self.env_ids = tuple(int(i) for i in env_ids)
        self.ages_s = tuple(float(a) for a in ages_s)
        super().__init__(
            f"external heightmap stale for env ids {list(self.env_ids)} "
            f"(ages {list(self.ages_s)} s exceed the hard cutoff); handle as "
            "an out-of-MDP pause or abort"
        )


@dataclass(frozen=True, slots=True)
class ExternalHeightmapProfile:
    """Immutable measurement profile for delay, dropout, and sensor error.

    Registration is sampled once per episode; pose jitter uses the same
    bounds per frame. Stale holds apply after every accepted frame.
    """

    name: str
    delay_range_s: tuple[float, float] = (0.0, 0.0)
    drop_probability: float = 0.0
    max_consecutive_drops: int = 0
    elevation_noise_std_m: float = 0.0
    elevation_noise_clip_m: float = 0.0
    registration_translation_std_m: float = 0.0
    registration_translation_clip_m: float = 0.0
    registration_yaw_std_rad: float = 0.0
    registration_yaw_clip_rad: float = 0.0
    stale_hold_updates: int = 0
    seed: int = 0

    def __post_init__(self) -> None:
        if self.name not in PROFILE_NAMES:
            raise ValueError(
                f"unknown external-heightmap profile {self.name!r}; "
                f"expected one of {PROFILE_NAMES}"
            )
        lo, hi = self.delay_range_s
        if not (math.isfinite(lo) and math.isfinite(hi)) or lo < 0.0 or hi < lo:
            raise ValueError(f"invalid delay range {self.delay_range_s}")
        if not 0.0 <= self.drop_probability < 1.0:
            raise ValueError(f"invalid drop probability {self.drop_probability}")
        # Negative deviations would reach the RNG; negative clips or counts
        # would silently disable noise clipping, drops, or stale holds.
        for name in (
            "elevation_noise_std_m",
            "elevation_noise_clip_m",
            "registration_translation_std_m",
            "registration_translation_clip_m",
            "registration_yaw_std_rad",
            "registration_yaw_clip_rad",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0, got {value!r}")
        for name in ("max_consecutive_drops", "stale_hold_updates"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be an integer >= 0, got {value!r}")
        if self.name == PROFILE_FAILURE_STALE and self.stale_hold_updates < 6:
            raise ValueError("failure_stale requires at least six consecutive holds")

    @property
    def is_ideal_content(self) -> bool:
        """True when frames pass through bitwise unmodified."""
        return (
            self.elevation_noise_std_m == 0.0
            and self.registration_translation_std_m == 0.0
            and self.registration_yaw_std_rad == 0.0
        )

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "delay_range_s": list(self.delay_range_s),
            "drop_probability": self.drop_probability,
            "max_consecutive_drops": self.max_consecutive_drops,
            "elevation_noise_std_m": self.elevation_noise_std_m,
            "elevation_noise_clip_m": self.elevation_noise_clip_m,
            "registration_translation_std_m": self.registration_translation_std_m,
            "registration_translation_clip_m": self.registration_translation_clip_m,
            "registration_yaw_std_rad": self.registration_yaw_std_rad,
            "registration_yaw_clip_rad": self.registration_yaw_clip_rad,
            "stale_hold_updates": self.stale_hold_updates,
            "seed": self.seed,
        }

    def profile_hash(self) -> str:
        """Return SHA-256 of the canonical profile parameters."""
        return hashlib.sha256(canonical_json_bytes(self.canonical_dict())).hexdigest()


def ideal_profile(*, seed: int = 0) -> ExternalHeightmapProfile:
    """``ideal``: zero delay, dropout, noise, registration, pose jitter."""
    return ExternalHeightmapProfile(name=PROFILE_IDEAL, seed=seed)


def randomized_profile(*, seed: int = 0) -> ExternalHeightmapProfile:
    """``randomized``: the exact conservative seeded defaults."""
    return ExternalHeightmapProfile(
        name=PROFILE_RANDOMIZED,
        delay_range_s=(0.0, 0.1),
        drop_probability=0.05,
        max_consecutive_drops=2,
        elevation_noise_std_m=0.002,
        elevation_noise_clip_m=0.006,
        registration_translation_std_m=0.005,
        registration_translation_clip_m=0.015,
        registration_yaw_std_rad=math.radians(0.25),
        registration_yaw_clip_rad=math.radians(0.75),
        seed=seed,
    )


def failure_stale_profile(
    *,
    seed: int = 0,
    stale_hold_updates: int = 6,
) -> ExternalHeightmapProfile:
    """``failure_stale``: >= 6 consecutive held updates (failure path)."""
    if stale_hold_updates < 6:
        raise ValueError("failure_stale requires at least six consecutive holds")
    return ExternalHeightmapProfile(
        name=PROFILE_FAILURE_STALE,
        stale_hold_updates=stale_hold_updates,
        seed=seed,
    )


@dataclass(frozen=True, slots=True)
class SourceFrame:
    """One synchronized truth frame for one environment.

    Elevation is the full mission-frame terrain-work surface; pose is
    ``(x, y, yaw)`` in mission coordinates and velocity ``(vx, vy, wz)`` in
    rover coordinates, synchronized to ``acquired_time_s`` on the source clock.
    """

    elevation: np.ndarray
    mission_pose: np.ndarray
    body_velocity: np.ndarray
    acquired_time_s: float


SourceFrameProvider = Callable[[np.ndarray], Sequence["SourceFrame | None"]]
"""Adapter callable: ordered env-id vector -> per-env frame or not-ready.

Providers must return promptly. Reset owns a monotonic wall-clock deadline and
polls at ``FreshnessPolicy.update_rate_hz``; a provider must not sleep to model
the update interval. A synchronous provider cannot be preempted, so one call
that blocks longer than the remaining deadline can overrun reset before the
source reports the timeout.
"""


def _owned_readonly(array: np.ndarray, dtype: Any) -> np.ndarray:
    out = np.array(array, dtype=dtype, copy=True)
    out.setflags(write=False)
    return out


def _owned_numeric_readonly(value: Any, *, name: str, dtype: Any) -> np.ndarray:
    try:
        raw = np.asarray(value)
        out = np.array(raw, dtype=dtype, copy=True)
    except (TypeError, ValueError, OverflowError) as exc:
        raise TypeError(f"{name} must be a numeric array") from exc
    if raw.dtype.kind not in "fiu":
        raise TypeError(f"{name} must be a numeric array")
    out.setflags(write=False)
    return out


def _validated_integer_array(
    value: Any,
    *,
    name: str,
    dtype: Any,
    nonnegative: bool = True,
) -> np.ndarray:
    """Copy an integer vector without lossy or wrapping conversion."""
    raw = np.asarray(value)
    if raw.dtype.kind == "b":
        raise ValueError(f"{name} must contain integral values, not booleans")
    if raw.dtype.kind in "iu":
        values = raw.astype(object, copy=False).reshape(-1).tolist()
    elif raw.dtype.kind == "f":
        if not np.isfinite(raw).all() or not np.equal(raw, np.floor(raw)).all():
            raise ValueError(f"{name} must contain integral values")
        values = raw.reshape(-1).tolist()
    elif raw.dtype.kind == "O":
        values = raw.reshape(-1).tolist()
    else:
        raise ValueError(f"{name} must contain integral values")

    info = np.iinfo(dtype)
    converted: list[int] = []
    for item in values:
        if isinstance(item, (bool, np.bool_)):
            raise ValueError(f"{name} must contain integral values, not booleans")
        if isinstance(item, (float, np.floating)):
            if not np.isfinite(item) or not float(item).is_integer():
                raise ValueError(f"{name} must contain integral values")
            integer = int(item)
        elif isinstance(item, (int, np.integer)):
            integer = int(item)
        else:
            raise ValueError(f"{name} must contain integral values")
        if nonnegative and integer < 0:
            raise ValueError(f"{name} must be nonnegative")
        if integer < int(info.min) or integer > int(info.max):
            raise ValueError(f"{name} values must fit {np.dtype(dtype).name}")
        converted.append(integer)
    return np.asarray(converted, dtype=dtype).reshape(raw.shape)


def _nonempty_string(value: Any, *, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _validated_manifest_hashes(value: Any, *, batch_size: int) -> tuple[str, ...]:
    if isinstance(value, (str, bytes)):
        raise ValueError("manifest_hashes must be a sequence of nonempty strings")
    try:
        hashes = tuple(value)
    except TypeError as exc:
        raise ValueError(
            "manifest_hashes must be a sequence of nonempty strings"
        ) from exc
    if len(hashes) != batch_size:
        raise ValueError("manifest_hashes must have one entry per row")
    for index, manifest_hash in enumerate(hashes):
        _nonempty_string(manifest_hash, name=f"manifest_hashes[{index}]")
    return hashes


def _contract_physical_bounds(
    contract: LandscapingIOContract,
) -> tuple[tuple[float, float], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    elevation_leaf = contract.leaf("proprio_dyn/heightmap_current_global")
    pose_leaf = contract.leaf("proprio/mission_pose")
    velocity_leaf = contract.leaf("proprio/base_velocity")
    return (
        (float(elevation_leaf.low[0]), float(elevation_leaf.high[0])),
        np.asarray(pose_leaf.low, dtype=np.float64),
        np.asarray(pose_leaf.high, dtype=np.float64),
        np.asarray(velocity_leaf.low, dtype=np.float64),
        np.asarray(velocity_leaf.high, dtype=np.float64),
    )


def _validate_frame_arrays(
    *,
    elevation: np.ndarray,
    mission_pose: np.ndarray,
    body_velocity: np.ndarray,
    geometry: SourceGridGeometry,
    contract: LandscapingIOContract,
    error_type: type[Exception] = ValueError,
) -> None:
    """Validate finite frame values and the frozen physical envelopes."""

    def fail(message: str) -> None:
        raise error_type(message)

    if elevation.shape != geometry.source_shape:
        fail(f"frame shape {elevation.shape} != {geometry.source_shape}")
    if mission_pose.shape != (3,):
        fail(f"mission_pose shape {mission_pose.shape} != (3,)")
    if body_velocity.shape != (3,):
        fail(f"body_velocity shape {body_velocity.shape} != (3,)")
    if not np.isfinite(elevation).all():
        fail("frame elevation must be fully finite")
    if not np.isfinite(mission_pose).all():
        fail("mission_pose must be fully finite")
    if not np.isfinite(body_velocity).all():
        fail("body_velocity must be fully finite")
    (elevation_low, elevation_high), pose_low, pose_high, vel_low, vel_high = (
        _contract_physical_bounds(contract)
    )
    # Cast bounds to the transport dtype: this preserves exact endpoint and
    # nextafter admission without adding tolerance to the frozen envelope.
    elevation_low_dtype = np.asarray(elevation_low, dtype=elevation.dtype)
    elevation_high_dtype = np.asarray(elevation_high, dtype=elevation.dtype)
    if (
        np.less(elevation, elevation_low_dtype).any()
        or np.greater(elevation, elevation_high_dtype).any()
    ):
        fail(
            "elevation values outside the declared physical envelope "
            f"[{elevation_low}, {elevation_high}]"
        )
    pose_low_dtype = np.asarray(pose_low, dtype=mission_pose.dtype)
    pose_high_dtype = np.asarray(pose_high, dtype=mission_pose.dtype)
    if (
        np.less(mission_pose, pose_low_dtype).any()
        or np.greater(mission_pose, pose_high_dtype).any()
    ):
        fail("mission_pose values outside the operating envelope")
    velocity_low_dtype = np.asarray(vel_low, dtype=body_velocity.dtype)
    velocity_high_dtype = np.asarray(vel_high, dtype=body_velocity.dtype)
    if (
        np.less(body_velocity, velocity_low_dtype).any()
        or np.greater(body_velocity, velocity_high_dtype).any()
    ):
        fail("body_velocity values outside the safety envelope")


@dataclass(frozen=True, slots=True)
class ExternalHeightmapBatch:
    """Immutable batch of accepted external-heightmap frames.

    Arrays are owned, read-only copies; rows follow ordered ``env_ids``.
    """

    env_ids: np.ndarray
    elevation: np.ndarray
    timestamp_s: np.ndarray
    sequence: np.ndarray
    mission_pose: np.ndarray
    body_velocity: np.ndarray
    manifest_hashes: tuple[str, ...]
    clock: str
    mission_frame_id: str
    elevation_datum_id: str
    geometry: SourceGridGeometry = field(default_factory=SourceGridGeometry)

    def __post_init__(self) -> None:
        env_ids = _validated_integer_array(self.env_ids, name="env_ids", dtype=np.int64)
        if env_ids.ndim != 1:
            raise ValueError("env_ids must be a 1-D integral vector")
        if env_ids.size == 0:
            raise ValueError("env_ids must be non-empty")
        if np.unique(env_ids).size != env_ids.size:
            raise ValueError("env_ids must be unique")
        if not isinstance(self.geometry, SourceGridGeometry):
            raise TypeError("geometry must be a SourceGridGeometry")
        env_ids = _owned_readonly(env_ids, np.int64)
        b = env_ids.shape[0]
        elevation = _owned_numeric_readonly(
            self.elevation, name="elevation", dtype=np.float32
        )
        if elevation.shape != (b, *self.geometry.source_shape):
            raise ValueError(
                f"elevation shape {elevation.shape} != "
                f"{(b, *self.geometry.source_shape)}"
            )
        timestamp = _owned_numeric_readonly(
            self.timestamp_s, name="timestamp_s", dtype=np.float64
        )
        sequence = _validated_integer_array(
            self.sequence, name="sequence", dtype=np.uint64
        )
        if sequence.size and int(sequence.max()) > np.iinfo(np.int64).max:
            raise ValueError("sequence values must fit signed int64 consumers")
        sequence = _owned_readonly(sequence, np.uint64)
        mission_pose = _owned_numeric_readonly(
            self.mission_pose, name="mission_pose", dtype=np.float32
        )
        body_velocity = _owned_numeric_readonly(
            self.body_velocity, name="body_velocity", dtype=np.float32
        )
        for name, arr, shape in (
            ("timestamp_s", timestamp, (b,)),
            ("sequence", sequence, (b,)),
            ("mission_pose", mission_pose, (b, 3)),
            ("body_velocity", body_velocity, (b, 3)),
        ):
            if arr.shape != shape:
                raise ValueError(f"{name} shape {arr.shape} != {shape}")
        if not np.isfinite(timestamp).all():
            raise ValueError("timestamp_s must be finite")
        hashes = _validated_manifest_hashes(self.manifest_hashes, batch_size=b)
        _nonempty_string(self.clock, name="clock")
        _nonempty_string(self.mission_frame_id, name="mission_frame_id")
        _nonempty_string(self.elevation_datum_id, name="elevation_datum_id")
        for row in range(b):
            _validate_frame_arrays(
                elevation=elevation[row],
                mission_pose=mission_pose[row],
                body_velocity=body_velocity[row],
                geometry=self.geometry,
                contract=LANDSCAPING_IO_CONTRACT,
            )
        object.__setattr__(self, "env_ids", env_ids)
        object.__setattr__(self, "elevation", elevation)
        object.__setattr__(self, "timestamp_s", timestamp)
        object.__setattr__(self, "sequence", sequence)
        object.__setattr__(self, "mission_pose", mission_pose)
        object.__setattr__(self, "body_velocity", body_velocity)
        object.__setattr__(self, "manifest_hashes", hashes)

    @property
    def batch_size(self) -> int:
        return int(self.env_ids.shape[0])


def validate_external_heightmap_batch(
    batch: ExternalHeightmapBatch,
    *,
    contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
    require_canonical_geometry: bool = False,
    require_contract_metadata: bool = False,
) -> None:
    """Validate an accepted batch at a policy/evaluation consumer seam."""
    if not isinstance(batch, ExternalHeightmapBatch):
        raise ExternalHeightmapFrameError(
            "expected an ExternalHeightmapBatch at the actor/evaluator seam"
        )
    if not isinstance(batch.geometry, SourceGridGeometry):
        raise ExternalHeightmapFrameError("batch geometry is not a SourceGridGeometry")
    if require_canonical_geometry and batch.geometry != contract.geometry:
        raise ExternalHeightmapFrameError(
            "batch geometry is not the canonical contract geometry"
        )
    if require_contract_metadata:
        mission_frame = contract.mission_frame
        if batch.clock != contract.freshness.clock:
            raise ExternalHeightmapFrameError(
                f"batch clock {batch.clock!r} does not match the contract clock"
            )
        if batch.mission_frame_id != mission_frame.mission_frame_id:
            raise ExternalHeightmapFrameError(
                "batch mission_frame_id does not match the contract"
            )
        if batch.elevation_datum_id != mission_frame.elevation_datum_id:
            raise ExternalHeightmapFrameError(
                "batch elevation_datum_id does not match the contract"
            )
    try:
        env_ids = _validated_integer_array(
            batch.env_ids, name="env_ids", dtype=np.int64
        )
        sequence = _validated_integer_array(
            batch.sequence, name="sequence", dtype=np.uint64
        )
    except (TypeError, ValueError, OverflowError) as exc:
        raise ExternalHeightmapFrameError(str(exc)) from exc
    if env_ids.ndim != 1 or sequence.ndim != 1:
        raise ExternalHeightmapFrameError("batch integer metadata must be 1-D")
    if env_ids.size == 0 or np.unique(env_ids).size != env_ids.size:
        raise ExternalHeightmapFrameError("batch env_ids must be nonempty and unique")
    if sequence.shape != env_ids.shape:
        raise ExternalHeightmapFrameError("batch sequence shape does not match env_ids")
    if sequence.size and int(sequence.max()) > np.iinfo(np.int64).max:
        raise ExternalHeightmapFrameError(
            "sequence values must fit signed int64 consumers"
        )
    timestamp = np.asarray(batch.timestamp_s, dtype=np.float64)
    if timestamp.shape != env_ids.shape or not np.isfinite(timestamp).all():
        raise ExternalHeightmapFrameError("batch timestamp_s must be finite")
    elevation = np.asarray(batch.elevation, dtype=np.float32)
    mission_pose = np.asarray(batch.mission_pose, dtype=np.float32)
    body_velocity = np.asarray(batch.body_velocity, dtype=np.float32)
    if elevation.shape != (env_ids.size, *batch.geometry.source_shape):
        raise ExternalHeightmapFrameError(
            "batch elevation shape does not match geometry"
        )
    if mission_pose.shape != (env_ids.size, 3):
        raise ExternalHeightmapFrameError("batch mission_pose shape is invalid")
    if body_velocity.shape != (env_ids.size, 3):
        raise ExternalHeightmapFrameError("batch body_velocity shape is invalid")
    try:
        hashes = _validated_manifest_hashes(
            batch.manifest_hashes, batch_size=int(env_ids.size)
        )
        _nonempty_string(batch.clock, name="clock")
        _nonempty_string(batch.mission_frame_id, name="mission_frame_id")
        _nonempty_string(batch.elevation_datum_id, name="elevation_datum_id")
        for row in range(int(env_ids.size)):
            _validate_frame_arrays(
                elevation=elevation[row],
                mission_pose=mission_pose[row],
                body_velocity=body_velocity[row],
                geometry=batch.geometry,
                contract=contract,
                error_type=ExternalHeightmapFrameError,
            )
    except (TypeError, ValueError) as exc:
        if isinstance(exc, ExternalHeightmapFrameError):
            raise
        raise ExternalHeightmapFrameError(str(exc)) from exc
    # Keep this binding so malformed forged batches fail before consumer state.
    del hashes


def validate_policy_time(
    batch: ExternalHeightmapBatch,
    policy_time_s: float,
    *,
    contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
    require_canonical_geometry: bool = False,
    require_contract_metadata: bool = False,
) -> np.ndarray:
    """Validate policy time and return immutable per-row transport ages."""
    validate_external_heightmap_batch(
        batch,
        contract=contract,
        require_canonical_geometry=require_canonical_geometry,
        require_contract_metadata=require_contract_metadata,
    )
    try:
        policy_time = float(policy_time_s)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ExternalHeightmapFrameError("policy time must be finite") from exc
    if not np.isfinite(policy_time):
        raise ExternalHeightmapFrameError("policy time must be finite")
    ages = policy_time - np.asarray(batch.timestamp_s, dtype=np.float64)
    negative = ages < 0.0
    if bool(negative.any()):
        raise ExternalHeightmapFrameError(
            "negative frame age (batch timestamp ahead of policy time) for "
            f"env ids {batch.env_ids[negative].tolist()}: "
            f"{ages[negative].tolist()}"
        )
    stale = ages > contract.freshness.stale_cutoff_s
    if bool(stale.any()):
        raise ExternalHeightmapStaleError(
            batch.env_ids[stale].tolist(), ages[stale].tolist()
        )
    out = ages.astype(np.float64)
    out.setflags(write=False)
    return out


def validate_external_heightmap_target(
    target_sources: Any,
    *,
    contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
    error_type: type[Exception] = ValueError,
) -> np.ndarray:
    """Validate a target map against the frozen external-heightmap envelope."""

    def fail(message: str) -> None:
        raise error_type(message)

    try:
        raw = np.asarray(target_sources)
    except (TypeError, ValueError) as exc:
        raise error_type("target sources must be numeric") from exc
    if raw.dtype.kind not in "fiu":
        fail("target sources must be numeric")
    try:
        targets = np.asarray(raw, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise error_type("target sources must be numeric") from exc
    if targets.ndim < 2 or tuple(targets.shape[-2:]) != contract.geometry.source_shape:
        fail("target sources shape does not match the contract geometry")
    if not np.isfinite(targets).all():
        fail("target sources must be fully finite")
    (target_low, target_high), _pose_low, _pose_high, _vel_low, _vel_high = (
        _contract_physical_bounds(contract)
    )
    target_low_dtype = np.asarray(target_low, dtype=targets.dtype)
    target_high_dtype = np.asarray(target_high, dtype=targets.dtype)
    if (
        np.less(targets, target_low_dtype).any()
        or np.greater(targets, target_high_dtype).any()
    ):
        fail(
            "target sources outside the declared physical envelope "
            f"[{target_low}, {target_high}]"
        )
    return targets


def materialize_surface_raster_fast(
    particle_centres_xyz: np.ndarray,
    *,
    geometry: SourceGridGeometry = LANDSCAPING_IO_CONTRACT.geometry,
    rule: SurfaceAggregationRule = LANDSCAPING_IO_CONTRACT.surface_rule,
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized implementation bitwise equal to the contract surface rule.

    It uses the same support membership, nearest-rank percentile, offset,
    base-plane fill, and shared rule parameters.
    """
    centres = np.asarray(particle_centres_xyz, dtype=np.float64)
    if centres.ndim != 2 or centres.shape[1] != 3:
        raise ValueError("particle centres must have shape (N, 3)")
    n0, n1 = geometry.source_shape
    surface = np.full((n0, n1), rule.base_plane_elevation_m, dtype=np.float32)
    support_count = np.zeros((n0, n1), dtype=np.int64)
    if centres.shape[0] == 0:
        surface.setflags(write=False)
        support_count.setflags(write=False)
        return surface, support_count

    cell = geometry.cell_size_m
    radius = rule.support_radius_m
    radius_sq = radius * radius
    xs = geometry.cell_centres(0)
    ys = geometry.cell_centres(1)

    px = centres[:, 0]
    py = centres[:, 1]
    pz = centres[:, 2]

    # This window includes every cell centre within the support radius.
    reach = int(math.ceil(radius / cell))
    base_i = np.floor((px - geometry.lower_edge_m[0]) / cell - 0.5).astype(np.int64)
    base_j = np.floor((py - geometry.lower_edge_m[1]) / cell - 0.5).astype(np.int64)
    offsets = np.arange(-reach, reach + 1, dtype=np.int64)

    cand_i = base_i[:, None] + offsets[None, :]  # (P, S)
    cand_j = base_j[:, None] + offsets[None, :]  # (P, S)
    in_i = (cand_i >= 0) & (cand_i < n0)
    in_j = (cand_j >= 0) & (cand_j < n1)
    ci = np.clip(cand_i, 0, n0 - 1)
    cj = np.clip(cand_j, 0, n1 - 1)
    # Match the contract's float64 operand order for bitwise-equal membership.
    dx = xs[ci] - px[:, None]  # (P, S)
    dy = ys[cj] - py[:, None]  # (P, S)
    dx_sq = dx * dx
    dy_sq = dy * dy
    mask = (dx_sq[:, :, None] + dy_sq[:, None, :] <= radius_sq) & (
        in_i[:, :, None] & in_j[:, None, :]
    )  # (P, S, S)

    p_idx, i_off, j_off = np.nonzero(mask)
    if p_idx.size == 0:
        surface.setflags(write=False)
        support_count.setflags(write=False)
        return surface, support_count
    flat_cell = ci[p_idx, i_off] * n1 + cj[p_idx, j_off]
    z_vals = pz[p_idx]

    order = np.lexsort((z_vals, flat_cell))
    flat_sorted = flat_cell[order]
    z_sorted = z_vals[order]
    seg_start = np.flatnonzero(
        np.concatenate(([True], flat_sorted[1:] != flat_sorted[:-1]))
    )
    seg_cell = flat_sorted[seg_start]
    seg_count = np.diff(np.concatenate((seg_start, [flat_sorted.size])))
    rank = np.ceil(rule.percentile * seg_count).astype(np.int64) - 1
    picked = z_sorted[seg_start + rank]

    rows = seg_cell // n1
    cols = seg_cell % n1
    surface[rows, cols] = (picked + rule.radius_offset_m).astype(np.float32)
    support_count[rows, cols] = seg_count
    surface.setflags(write=False)
    support_count.setflags(write=False)
    return surface, support_count


@dataclass(frozen=True, slots=True)
class Se2Registration:
    """Small mission-frame registration error ``(dx, dy, dyaw)``."""

    dx_m: float = 0.0
    dy_m: float = 0.0
    dyaw_rad: float = 0.0

    @property
    def is_identity(self) -> bool:
        return self.dx_m == 0.0 and self.dy_m == 0.0 and self.dyaw_rad == 0.0


def wrap_angle_pi(angle: float) -> float:
    """Wrap to the mission-yaw interval ``[-pi, pi)``.

    This matches the training transform; disagreement at the seam creates a
    maximal actor-input jump for equivalent headings ``-pi`` and ``+pi``.
    """
    two_pi = 2.0 * math.pi
    return float((float(angle) + math.pi) % two_pi - math.pi)


def apply_se2_registration_to_map(
    source: np.ndarray,
    registration: Se2Registration,
    *,
    geometry: SourceGridGeometry = LANDSCAPING_IO_CONTRACT.geometry,
) -> np.ndarray:
    """Resample a source raster in the misregistered mapper frame.

    Cell ``(i, j)`` samples the true surface at ``R(dyaw) @ centre + (dx, dy)``;
    edge clamping is limited to the outer guard ring.
    """
    if registration.is_identity:
        return np.asarray(source, dtype=np.float32)
    src = np.asarray(source, dtype=np.float32)
    xs = geometry.cell_centres(0)
    ys = geometry.cell_centres(1)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    cos_a = math.cos(registration.dyaw_rad)
    sin_a = math.sin(registration.dyaw_rad)
    tx = gx * cos_a - gy * sin_a + registration.dx_m
    ty = gx * sin_a + gy * cos_a + registration.dy_m

    cell = geometry.cell_size_m
    n0, n1 = geometry.source_shape
    fx = np.clip((tx - geometry.lower_edge_m[0]) / cell - 0.5, 0.0, n0 - 1.0)
    fy = np.clip((ty - geometry.lower_edge_m[1]) / cell - 0.5, 0.0, n1 - 1.0)
    x0 = np.floor(fx).astype(np.int64)
    y0 = np.floor(fy).astype(np.int64)
    x1 = np.minimum(x0 + 1, n0 - 1)
    y1 = np.minimum(y0 + 1, n1 - 1)
    wx = (fx - x0).astype(np.float32)
    wy = (fy - y0).astype(np.float32)
    v00 = src[x0, y0]
    v10 = src[x1, y0]
    v01 = src[x0, y1]
    v11 = src[x1, y1]
    return np.asarray(
        v00 * (1.0 - wx) * (1.0 - wy)
        + v10 * wx * (1.0 - wy)
        + v01 * (1.0 - wx) * wy
        + v11 * wx * wy
    ).astype(np.float32)


def apply_se2_registration_to_pose(
    mission_pose: np.ndarray,
    registration: Se2Registration,
) -> np.ndarray:
    """Express a true mission pose in the misregistered mapper frame.

    This inverse keeps the synchronized local crop aligned while shifting
    global map content.
    """
    if registration.is_identity:
        return np.asarray(mission_pose, dtype=np.float32)
    x, y, yaw = (float(v) for v in mission_pose)
    cos_a = math.cos(-registration.dyaw_rad)
    sin_a = math.sin(-registration.dyaw_rad)
    rx = x - registration.dx_m
    ry = y - registration.dy_m
    return np.asarray(
        (
            rx * cos_a - ry * sin_a,
            rx * sin_a + ry * cos_a,
            wrap_angle_pi(yaw - registration.dyaw_rad),
        ),
        dtype=np.float32,
    )


@dataclass(slots=True)
class _PendingFrame:
    acquired_s: float
    publish_s: float
    elevation: np.ndarray
    mission_pose: np.ndarray
    body_velocity: np.ndarray


@dataclass(slots=True)
class _EnvState:
    rng: np.random.Generator
    registration: Se2Registration = field(default_factory=Se2Registration)
    manifest_hash: str = ""
    target: np.ndarray | None = None
    pending: list[_PendingFrame] = field(default_factory=list)
    consecutive_drops: int = 0
    holds_remaining: int = 0
    last_acquired_s: float = -math.inf
    last_policy_time_s: float = -math.inf
    elevation: np.ndarray | None = None
    mission_pose: np.ndarray | None = None
    body_velocity: np.ndarray | None = None
    timestamp_s: float = math.nan
    sequence: int = 0
    has_frame: bool = False


_MANIFEST_TARGET_CACHE: OrderedDict[tuple[str, str, str], np.ndarray] = OrderedDict()

# Bound memory when manifests change every episode.
_MANIFEST_TARGET_CACHE_MAX_ENTRIES = 32


def _verified_target_for_manifest(
    manifest: TargetManifest,
    contract: LandscapingIOContract,
) -> np.ndarray:
    """Verify a manifest against the contract and return its target map.

    Verification checks hashes, geometry, mission frame and datum, and the
    target envelope. Cache hits require canonical content, declared hash, and
    contract fingerprint to match; manifests without canonical serialization
    reverify.
    """
    serialize = getattr(manifest, "canonical_json", None)
    key: tuple[str, str, str] | None = None
    if callable(serialize):
        serialized = serialize()
        if not isinstance(serialized, str):
            raise ValueError("manifest canonical_json() must return a string")
        key = (
            manifest.manifest_sha256,
            hashlib.sha256(serialized.encode("utf-8")).hexdigest(),
            contract.fingerprint(),
        )
        cached = _MANIFEST_TARGET_CACHE.get(key)
        if cached is not None:
            _MANIFEST_TARGET_CACHE.move_to_end(key)
            return cached
    materialized = manifest.verify(contract=contract)
    target = _owned_readonly(
        validate_external_heightmap_target(
            materialized.target,
            contract=contract,
            error_type=ExternalHeightmapFrameError,
        ),
        np.float32,
    )
    if key is not None:
        _MANIFEST_TARGET_CACHE[key] = target
        while len(_MANIFEST_TARGET_CACHE) > _MANIFEST_TARGET_CACHE_MAX_ENTRIES:
            _MANIFEST_TARGET_CACHE.popitem(last=False)
    return target


class MeasuredExternalHeightmapSource:
    """Batch source that wraps a truth provider with a measurement profile.

    Simulator and Isaac-free adapters supply different
    :data:`SourceFrameProvider` implementations.
    """

    def __init__(
        self,
        *,
        num_envs: int,
        provider: SourceFrameProvider,
        profile: ExternalHeightmapProfile,
        contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
    ) -> None:
        if num_envs < 1:
            raise ValueError("num_envs must be positive")
        self._num_envs = int(num_envs)
        self._provider = provider
        self._profile = profile
        self._contract = contract
        self._geometry: SourceGridGeometry = contract.geometry
        self._freshness: FreshnessPolicy = contract.freshness
        self._mission_frame: MissionFramePolicy = contract.mission_frame
        self._states = [
            _EnvState(
                rng=np.random.default_rng(
                    np.random.SeedSequence(entropy=(profile.seed, env_id))
                )
            )
            for env_id in range(self._num_envs)
        ]

    @property
    def profile(self) -> ExternalHeightmapProfile:
        return self._profile

    @property
    def clock(self) -> str:
        return self._freshness.clock

    def reset(
        self,
        env_ids: Sequence[int] | np.ndarray,
        target_manifests: Sequence[TargetManifest],
        timeout_s: float | None = None,
    ) -> ExternalHeightmapBatch:
        """Reset selected environments and wait for their first complete frames.

        Only selected rows change. Reset frames bypass simulated delay/drop,
        so age starts at zero; measurement errors still apply. Raises
        :class:`ExternalHeightmapReadinessError` on readiness timeout.
        """
        ids = self._coerce_env_ids(env_ids)
        if len(target_manifests) != ids.shape[0]:
            raise ValueError("target_manifests must have one entry per env id")
        try:
            timeout = float(
                self._freshness.readiness_timeout_s if timeout_s is None else timeout_s
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("timeout_s must be finite and nonnegative") from exc
        if not math.isfinite(timeout) or timeout < 0.0:
            raise ValueError("timeout_s must be finite and nonnegative")

        prior_sequences = {
            env_id: self._states[env_id].sequence for env_id in ids.tolist()
        }
        completed = False
        try:
            self.invalidate(ids)
            targets = [
                _verified_target_for_manifest(manifest, self._contract)
                for manifest in target_manifests
            ]
            for env_id, manifest, target in zip(
                ids.tolist(), target_manifests, targets, strict=True
            ):
                state = self._states[env_id]
                state.target = target
                state.manifest_hash = manifest.manifest_sha256
                state.registration = self._sample_registration(state.rng)

            # Use wall time and always capture once so timeout_s=0 stays useful
            # with synchronous providers.
            deadline = time.monotonic() + timeout
            poll_period_s = 1.0 / self._freshness.update_rate_hz
            waiting = list(ids.tolist())
            first_poll = True
            while waiting:
                if not first_poll:
                    remaining_s = deadline - time.monotonic()
                    if remaining_s <= 0.0:
                        break
                    time.sleep(min(poll_period_s, remaining_s))
                    if time.monotonic() >= deadline:
                        break
                first_poll = False
                frames = self._provider_frames(waiting)
                if timeout > 0.0 and time.monotonic() >= deadline:
                    break
                still_waiting: list[int] = []
                for env_id, frame in zip(waiting, frames, strict=True):
                    if frame is None:
                        still_waiting.append(env_id)
                        continue
                    acquired = self._validate_acquired_timestamp(frame)
                    self._accept_capture(
                        self._states[env_id],
                        frame,
                        publish_delay_s=0.0,
                    )
                    self._publish_pending(self._states[env_id], acquired)
                waiting = still_waiting
                if timeout == 0.0:
                    break
            if waiting:
                raise ExternalHeightmapReadinessError(waiting, timeout)
            batch = self._batch_for(ids)
            completed = True
            return batch
        finally:
            if not completed:
                self.invalidate(ids)
                for env_id in ids.tolist():
                    self._states[env_id].sequence = prior_sequences[env_id]

    def invalidate(self, env_ids: Sequence[int] | np.ndarray) -> None:
        """Clear selected episode frames while preserving identity and RNG state."""
        ids = self._coerce_env_ids(env_ids)
        for env_id in ids.tolist():
            self._clear_reset_state(self._states[env_id])

    @staticmethod
    def _clear_reset_state(state: _EnvState) -> None:
        """Leave a failed reset row unable to serve any prior or partial frame."""
        state.target = None
        state.manifest_hash = ""
        state.pending.clear()
        state.consecutive_drops = 0
        state.holds_remaining = 0
        state.has_frame = False
        state.elevation = None
        state.mission_pose = None
        state.body_velocity = None
        state.timestamp_s = math.nan
        state.last_acquired_s = -math.inf
        state.last_policy_time_s = -math.inf

    def observe(
        self,
        env_ids: Sequence[int] | np.ndarray,
        policy_time_s: float,
    ) -> ExternalHeightmapBatch:
        """Publish one new-or-held complete frame per selected environment."""
        ids = self._coerce_env_ids(env_ids)
        try:
            policy_time = float(policy_time_s)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ExternalHeightmapFrameError("policy time must be finite") from exc
        if not np.isfinite(policy_time):
            raise ExternalHeightmapFrameError("policy time must be finite")
        needs_capture: list[int] = []
        for env_id in ids.tolist():
            state = self._states[env_id]
            if not state.has_frame:
                raise ExternalHeightmapError(
                    f"observe before reset for env id {env_id}; reset must "
                    "provide the first complete frame"
                )
            if policy_time < state.last_policy_time_s:
                raise ExternalHeightmapFrameError(
                    f"non-monotonic policy time {policy_time} < "
                    f"{state.last_policy_time_s} for env id {env_id}"
                )
            if policy_time > state.last_acquired_s:
                needs_capture.append(env_id)

        if needs_capture:
            frames = self._provider_frames(needs_capture)
            captures: list[tuple[int, SourceFrame, float]] = []
            for env_id, frame in zip(needs_capture, frames):
                if frame is None:
                    continue  # transient provider gap -> hold-last
                state = self._states[env_id]
                acquired = self._validate_acquired_timestamp(frame)
                if acquired > policy_time:
                    raise ExternalHeightmapFrameError(
                        f"future source timestamp {acquired} > policy time "
                        f"{policy_time} for env id {env_id}"
                    )
                if acquired < state.last_acquired_s:
                    raise ExternalHeightmapFrameError(
                        f"non-monotonic source timestamp {acquired} < "
                        f"{state.last_acquired_s} for env id {env_id}"
                    )
                if acquired == state.last_acquired_s:
                    continue  # no newer measurement exists yet
                elevation, pose, velocity = self._coerce_frame_arrays(frame)
                self._validate_frame(elevation, pose, velocity)
                captures.append((env_id, frame, acquired))
            for env_id, frame, acquired in captures:
                state = self._states[env_id]
                if self._should_drop(state):
                    state.last_acquired_s = acquired
                    continue
                delay = self._sample_delay(state.rng)
                self._accept_capture(state, frame, publish_delay_s=delay)

        for env_id in ids.tolist():
            self._states[env_id].last_policy_time_s = policy_time
            self._publish_pending(self._states[env_id], policy_time)
        return self._batch_for(ids)

    def target_sources(self, env_ids: Sequence[int] | np.ndarray) -> np.ndarray:
        """Verified full-resolution target maps for the ordered env ids."""
        ids = self._coerce_env_ids(env_ids)
        rows = []
        for env_id in ids.tolist():
            target = self._states[env_id].target
            if target is None:
                raise ExternalHeightmapError(
                    f"no verified target manifest for env id {env_id}"
                )
            rows.append(target)
        out = np.stack(rows).astype(np.float32)
        out.setflags(write=False)
        return out

    def check_freshness(
        self,
        batch: ExternalHeightmapBatch,
        policy_time_s: float,
    ) -> np.ndarray:
        """Return per-row ages; raise on negative ages or hard-stale rows."""
        return validate_policy_time(
            batch,
            policy_time_s,
            contract=self._contract,
            require_canonical_geometry=False,
            require_contract_metadata=False,
        )

    def _coerce_env_ids(self, env_ids: Sequence[int] | np.ndarray) -> np.ndarray:
        try:
            ids = _validated_integer_array(
                env_ids, name="env_ids", dtype=np.int64
            ).reshape(-1)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(str(exc)) from exc
        if ids.size == 0:
            raise ValueError("env_ids must be non-empty")
        if ids.min() < 0 or ids.max() >= self._num_envs:
            raise ValueError(f"env ids {ids.tolist()} outside [0, {self._num_envs})")
        if np.unique(ids).size != ids.size:
            raise ValueError("env_ids must be unique")
        return ids

    def _validate_acquired_timestamp(self, frame: SourceFrame) -> float:
        try:
            acquired = float(frame.acquired_time_s)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ExternalHeightmapFrameError(
                "source acquired timestamp must be finite"
            ) from exc
        if not np.isfinite(acquired):
            raise ExternalHeightmapFrameError(
                "source acquired timestamp must be finite"
            )
        return acquired

    def _sample_registration(self, rng: np.random.Generator) -> Se2Registration:
        profile = self._profile
        if profile.registration_translation_std_m == 0.0 and (
            profile.registration_yaw_std_rad == 0.0
        ):
            return Se2Registration()
        clip_t = profile.registration_translation_clip_m
        clip_y = profile.registration_yaw_clip_rad
        dx, dy = np.clip(
            rng.normal(0.0, profile.registration_translation_std_m, size=2),
            -clip_t,
            clip_t,
        )
        dyaw = float(
            np.clip(rng.normal(0.0, profile.registration_yaw_std_rad), -clip_y, clip_y)
        )
        return Se2Registration(dx_m=float(dx), dy_m=float(dy), dyaw_rad=dyaw)

    def _sample_delay(self, rng: np.random.Generator) -> float:
        lo, hi = self._profile.delay_range_s
        if hi <= 0.0:
            return 0.0
        return float(rng.uniform(lo, hi))

    def _should_drop(self, state: _EnvState) -> bool:
        profile = self._profile
        if profile.stale_hold_updates > 0:
            # Hold the declared number of updates after each accepted frame.
            if state.holds_remaining > 0:
                state.holds_remaining -= 1
                return True
            return False
        if profile.drop_probability <= 0.0:
            return False
        if state.consecutive_drops >= profile.max_consecutive_drops:
            state.consecutive_drops = 0
            return False
        if bool(state.rng.random() < profile.drop_probability):
            state.consecutive_drops += 1
            return True
        state.consecutive_drops = 0
        return False

    def _accept_capture(
        self,
        state: _EnvState,
        frame: SourceFrame,
        *,
        publish_delay_s: float,
    ) -> None:
        # Validate first so malformed captures cannot partially update state.
        acquired = self._validate_acquired_timestamp(frame)
        elevation, pose, velocity = self._coerce_frame_arrays(frame)
        self._validate_frame(elevation, pose, velocity)
        elevation, pose, velocity = self._process_frame(
            state, elevation, pose, velocity
        )
        self._validate_frame(elevation, pose, velocity)
        state.last_acquired_s = acquired
        state.pending.append(
            _PendingFrame(
                acquired_s=acquired,
                publish_s=acquired + float(publish_delay_s),
                elevation=elevation,
                mission_pose=pose,
                body_velocity=velocity,
            )
        )

    def _provider_frames(
        self, env_ids: Sequence[int]
    ) -> tuple[SourceFrame | None, ...]:
        """Return one ordered provider result for every requested environment."""
        requested = np.asarray(env_ids, dtype=np.int64)
        result = self._provider(requested)
        try:
            frames = tuple(result)
        except TypeError as exc:
            raise ExternalHeightmapFrameError(
                "source provider must return one frame per requested environment"
            ) from exc
        if len(frames) != len(env_ids):
            raise ExternalHeightmapFrameError(
                "source provider must return one frame per requested environment "
                f"(requested {len(env_ids)}, received {len(frames)})"
            )
        return frames

    def _coerce_frame_arrays(
        self, frame: SourceFrame
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        try:
            return (
                np.asarray(frame.elevation, dtype=np.float32),
                np.asarray(frame.mission_pose, dtype=np.float32),
                np.asarray(frame.body_velocity, dtype=np.float32),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise ExternalHeightmapFrameError(
                "source frame arrays must be numeric"
            ) from exc

    def _process_frame(
        self,
        state: _EnvState,
        elevation: np.ndarray,
        pose: np.ndarray,
        velocity: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        profile = self._profile
        if profile.is_ideal_content:
            return elevation.copy(), pose.copy(), velocity.copy()

        elevation = apply_se2_registration_to_map(
            elevation, state.registration, geometry=self._geometry
        )
        if profile.elevation_noise_std_m > 0.0:
            noise = np.clip(
                state.rng.normal(
                    0.0, profile.elevation_noise_std_m, size=elevation.shape
                ),
                -profile.elevation_noise_clip_m,
                profile.elevation_noise_clip_m,
            ).astype(np.float32)
            elevation = elevation + noise
        pose = apply_se2_registration_to_pose(pose, state.registration)
        # Per-frame external pose jitter reuses the registration bounds.
        if profile.registration_translation_std_m > 0.0 or (
            profile.registration_yaw_std_rad > 0.0
        ):
            jitter_xy = np.clip(
                state.rng.normal(0.0, profile.registration_translation_std_m, size=2),
                -profile.registration_translation_clip_m,
                profile.registration_translation_clip_m,
            )
            jitter_yaw = float(
                np.clip(
                    state.rng.normal(0.0, profile.registration_yaw_std_rad),
                    -profile.registration_yaw_clip_rad,
                    profile.registration_yaw_clip_rad,
                )
            )
            pose = np.asarray(
                (
                    pose[0] + jitter_xy[0],
                    pose[1] + jitter_xy[1],
                    wrap_angle_pi(float(pose[2]) + jitter_yaw),
                ),
                dtype=np.float32,
            )
        return elevation.astype(np.float32), pose, velocity.copy()

    def _validate_frame(
        self,
        elevation: np.ndarray,
        pose: np.ndarray,
        velocity: np.ndarray,
    ) -> None:
        try:
            _validate_frame_arrays(
                elevation=elevation,
                mission_pose=pose,
                body_velocity=velocity,
                geometry=self._geometry,
                contract=self._contract,
                error_type=ExternalHeightmapFrameError,
            )
        except ExternalHeightmapFrameError as exc:
            # Preserve the established source-facing diagnostics for coverage
            # and physical-envelope failures.
            message = str(exc)
            if "elevation must be fully finite" in message:
                message = (
                    "incomplete source coverage: non-finite cells present "
                    "(masks and sentinel fills are unsupported)"
                )
            raise ExternalHeightmapFrameError(message) from exc

    def _publish_pending(self, state: _EnvState, policy_time_s: float) -> None:
        if state.has_frame:
            state.pending = [
                frame for frame in state.pending if frame.acquired_s > state.timestamp_s
            ]
        published = [f for f in state.pending if f.publish_s <= policy_time_s]
        if not published:
            return
        newest = max(published, key=lambda f: f.acquired_s)
        state.pending = [
            frame
            for frame in state.pending
            if frame.acquired_s > newest.acquired_s and frame.publish_s > policy_time_s
        ]
        if state.has_frame and newest.acquired_s <= state.timestamp_s:
            return
        state.elevation = newest.elevation
        state.mission_pose = newest.mission_pose
        state.body_velocity = newest.body_velocity
        state.timestamp_s = newest.acquired_s
        state.sequence += 1
        state.has_frame = True
        if self._profile.stale_hold_updates > 0:
            state.holds_remaining = self._profile.stale_hold_updates

    def _batch_for(self, ids: np.ndarray) -> ExternalHeightmapBatch:
        elevations = []
        poses = []
        velocities = []
        timestamps = []
        sequences = []
        hashes = []
        for env_id in ids.tolist():
            state = self._states[env_id]
            if not state.has_frame or state.elevation is None:
                raise ExternalHeightmapError(f"no accepted frame for env id {env_id}")
            elevations.append(state.elevation)
            poses.append(state.mission_pose)
            velocities.append(state.body_velocity)
            timestamps.append(state.timestamp_s)
            sequences.append(state.sequence)
            hashes.append(state.manifest_hash)
        return ExternalHeightmapBatch(
            env_ids=ids,
            elevation=np.stack(elevations),
            timestamp_s=np.asarray(timestamps, dtype=np.float64),
            sequence=np.asarray(sequences, dtype=np.uint64),
            mission_pose=np.stack(poses),
            body_velocity=np.stack(velocities),
            manifest_hashes=tuple(hashes),
            clock=self._freshness.clock,
            mission_frame_id=self._mission_frame.mission_frame_id,
            elevation_datum_id=self._mission_frame.elevation_datum_id,
            geometry=self._geometry,
        )


# ---------------------------------------------------------------------------
# Actor projection (one projector for current AND target)
# ---------------------------------------------------------------------------


def project_actor_observation(
    batch: ExternalHeightmapBatch,
    target_sources: np.ndarray,
    *,
    policy_time_s: float,
) -> dict[str, np.ndarray]:
    """Project one accepted batch into the seven actor leaves.

    Local crops use each row's synchronized frame pose (never a newer
    policy-read-time pose); global views are exact area means of the 4 m ROI;
    the same projector serves current and target maps. Returns float32 arrays
    keyed by the contract leaf keys, batched on axis 0.
    """
    geometry = LANDSCAPING_IO_CONTRACT.geometry
    if batch.geometry != geometry:
        raise ValueError("batch geometry is not the canonical contract geometry")
    ages = validate_policy_time(
        batch,
        policy_time_s,
        contract=LANDSCAPING_IO_CONTRACT,
        require_canonical_geometry=True,
        require_contract_metadata=True,
    )
    try:
        targets = np.asarray(target_sources, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ExternalHeightmapFrameError("target sources must be numeric") from exc
    if targets.shape != batch.elevation.shape:
        raise ValueError(
            f"target sources shape {targets.shape} != {batch.elevation.shape}"
        )
    targets = validate_external_heightmap_target(
        targets,
        contract=LANDSCAPING_IO_CONTRACT,
        error_type=ExternalHeightmapFrameError,
    )
    b = batch.batch_size
    g0, g1 = geometry.global_shape
    l0, l1 = geometry.local_shape
    current_global = np.empty((b, g0, g1), dtype=np.float32)
    target_global = np.empty((b, g0, g1), dtype=np.float32)
    current_local = np.empty((b, l0, l1), dtype=np.float32)
    target_local = np.empty((b, l0, l1), dtype=np.float32)
    for row in range(b):
        pose = (
            float(batch.mission_pose[row, 0]),
            float(batch.mission_pose[row, 1]),
            float(batch.mission_pose[row, 2]),
        )
        current_global[row] = downsample_global_from_source(
            batch.elevation[row], geometry=geometry
        )
        target_global[row] = downsample_global_from_source(
            targets[row], geometry=geometry
        )
        current_local[row] = sample_local_from_source(
            batch.elevation[row], mission_pose=pose, geometry=geometry
        )
        target_local[row] = sample_local_from_source(
            targets[row], mission_pose=pose, geometry=geometry
        )
    age = ages.astype(np.float32)
    leaves = {
        "proprio_dyn/heightmap_current_global": current_global,
        "proprio_dyn/heightmap_target_global": target_global,
        "proprio_dyn/heightmap_current_local": current_local,
        "proprio_dyn/heightmap_target_local": target_local,
        "proprio/mission_pose": batch.mission_pose.astype(np.float32),
        "proprio/base_velocity": batch.body_velocity.astype(np.float32),
        "proprio_dyn/heightmap_age_s": age.reshape(b, 1),
    }
    for value in leaves.values():
        value.setflags(write=False)
    return leaves


__all__ = [
    "PROFILE_FAILURE_STALE",
    "PROFILE_IDEAL",
    "PROFILE_NAMES",
    "PROFILE_RANDOMIZED",
    "ExternalHeightmapBatch",
    "ExternalHeightmapError",
    "ExternalHeightmapFrameError",
    "ExternalHeightmapProfile",
    "ExternalHeightmapReadinessError",
    "ExternalHeightmapStaleError",
    "MeasuredExternalHeightmapSource",
    "Se2Registration",
    "SourceFrame",
    "SourceFrameProvider",
    "TargetManifest",
    "apply_se2_registration_to_map",
    "apply_se2_registration_to_pose",
    "failure_stale_profile",
    "ideal_profile",
    "materialize_surface_raster_fast",
    "project_actor_observation",
    "randomized_profile",
    "validate_external_heightmap_batch",
    "validate_external_heightmap_target",
    "validate_policy_time",
    "wrap_angle_pi",
]
