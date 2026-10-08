"""Terrain-map helpers and general-task mission generation.

Provides particle-binned diagnostic maps and deterministic, hashed,
zero-net-volume targets for ``terrain_landscaping``.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np
import torch

from srb.terrain.landscaping_contract import (
    LANDSCAPING_IO_CONTRACT,
    FlatParticleBedParams,
    LandscapingIOContract,
    MissionFramePolicy,
    SourceGridGeometry,
    SuccessParams,
    SurfaceAggregationRule,
    TargetMapMaterialization,
    canonical_json,
    canonical_json_bytes,
    compute_target_map_sha256,
    materialize_surface_raster,
)
from srb.utils.digest import sha256_bytes

# Bump these when the corresponding serialized semantics change.
GENERAL_TARGET_GENERATOR_VERSION = "mass_balanced_features_v2_float32_headroom"
GENERAL_TARGET_ENCODING = "mass_balanced_delta_on_flat_particle_bed_v1"
GENERAL_RESET_LAYOUT_VERSION = "flat_grid_bed_v1"
GENERAL_TASK_ID = "terrain_landscaping"
GENERAL_MANIFEST_SCHEMA_VERSION = 1

# Keep general-task seeds independent from other RNG streams.
_GENERATOR_ENTROPY_SALT = 0xA10_1A5D


def _sha256_hex(data: bytes) -> str:
    return sha256_bytes(data)


def compute_elevation_map(
    particle_pos_w: torch.Tensor,
    env_origins: torch.Tensor,
    resolution: int,
    extent: float = 4.0,
    percentile: float = 0.9,
) -> torch.Tensor:
    """Bin world particle positions into an env-local elevation map.

    Returns ``(n_envs, resolution, resolution)`` in local Z; empty cells are 0.
    Uses the nearest-rank percentile within each occupied cell.
    """
    grid_n = int(resolution)
    if not 0.0 < percentile <= 1.0:
        raise ValueError("percentile must be in (0, 1]")
    n_envs = particle_pos_w.shape[0]
    device = particle_pos_w.device
    half = extent * 0.5
    cell = extent / grid_n

    local = particle_pos_w - env_origins.unsqueeze(1)
    lx, ly, lz = local[:, :, 0], local[:, :, 1], local[:, :, 2]

    # Floor keeps particles just beyond the negative edge outside the grid.
    ix = torch.floor((lx + half) / cell).long()
    iy = torch.floor((ly + half) / cell).long()
    valid = (ix >= 0) & (ix < grid_n) & (iy >= 0) & (iy < grid_n)
    # A diverged particle must not poison its cell's height.
    valid &= torch.isfinite(lz)

    # Stable grouping preserves height order, allowing one nearest-rank gather
    # without per-cell Python loops and their GPU synchronization.
    env_idx = torch.arange(n_envs, device=device).unsqueeze(1).expand_as(ix)
    flat_cell = (env_idx * grid_n * grid_n + ix * grid_n + iy)[valid]
    heights = lz[valid]

    height_order = torch.argsort(heights, stable=True)
    cells_height_sorted = flat_cell[height_order]
    heights_sorted = heights[height_order]
    cell_order = torch.argsort(cells_height_sorted, stable=True)
    cells_grouped = cells_height_sorted[cell_order]
    heights_grouped = heights_sorted[cell_order]

    occupied_cells, counts = torch.unique_consecutive(
        cells_grouped,
        return_counts=True,
    )
    starts = counts.cumsum(dim=0) - counts
    ranks = torch.ceil(counts.to(torch.float64) * percentile).to(torch.long) - 1
    selected_heights = heights_grouped[starts + ranks]

    elev = torch.zeros(
        n_envs * grid_n * grid_n,
        device=device,
        dtype=torch.float32,
    )
    elev[occupied_cells] = selected_heights
    return elev.reshape(n_envs, grid_n, grid_n)


@dataclass(frozen=True, slots=True)
class GeneralMissionBounds:
    """Declared feasibility limits, serialized in and covered by the manifest hash."""

    volume_balance_tolerance_m3: float = 1.0e-4
    """Maximum absolute cut/fill imbalance over graded cells, in m³."""

    min_target_elevation_m: float = 0.02
    """Lowest graded elevation; one success tolerance above the empty-cell base plane."""

    max_target_elevation_m: float = 0.28
    """Highest graded elevation, kept below the 0.30 m containment wall."""

    work_zone_half_extent_m: float = 1.60
    """Half-extent of the reachable work zone in the mission frame."""

    significant_elevation_m: float = 0.01
    """Deformation magnitude treated as significant for reach containment."""

    min_grading_mae_m: float = 0.01
    """Minimum mean absolute deviation from the flat reset."""

    max_initial_fraction_within_tolerance: float = 0.85
    """Maximum within-tolerance fraction already scored by the flat reset."""

    def __post_init__(self) -> None:
        if self.volume_balance_tolerance_m3 <= 0.0:
            raise ValueError("volume_balance_tolerance_m3 must be positive")
        if self.max_target_elevation_m <= self.min_target_elevation_m:
            raise ValueError("target elevation bounds must be increasing")
        if self.work_zone_half_extent_m <= 0.0:
            raise ValueError("work_zone_half_extent_m must be positive")
        if self.significant_elevation_m <= 0.0:
            raise ValueError("significant_elevation_m must be positive")
        if self.min_grading_mae_m <= 0.0:
            raise ValueError("min_grading_mae_m must be positive")
        if not 0.0 < self.max_initial_fraction_within_tolerance < 1.0:
            raise ValueError("max_initial_fraction_within_tolerance must be in (0, 1)")

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "volume_balance_tolerance_m3": self.volume_balance_tolerance_m3,
            "min_target_elevation_m": self.min_target_elevation_m,
            "max_target_elevation_m": self.max_target_elevation_m,
            "work_zone_half_extent_m": self.work_zone_half_extent_m,
            "significant_elevation_m": self.significant_elevation_m,
            "min_grading_mae_m": self.min_grading_mae_m,
            "max_initial_fraction_within_tolerance": (
                self.max_initial_fraction_within_tolerance
            ),
        }


@dataclass(frozen=True, slots=True)
class GeneralMissionProfile:
    """Sampling ranges and feasibility limits for general-task missions."""

    cut_feature_count_range: tuple[int, int] = (1, 3)
    fill_feature_count_range: tuple[int, int] = (1, 3)
    amplitude_range_m: tuple[float, float] = (0.06, 0.10)
    sigma_range_m: tuple[float, float] = (0.28, 0.50)
    bounds: GeneralMissionBounds = field(default_factory=GeneralMissionBounds)
    max_attempts: int = 64

    def __post_init__(self) -> None:
        for name, rng_ in (
            ("cut_feature_count_range", self.cut_feature_count_range),
            ("fill_feature_count_range", self.fill_feature_count_range),
        ):
            lo, hi = int(rng_[0]), int(rng_[1])
            if lo < 1 or hi < lo:
                raise ValueError(f"{name} must be an increasing range with min >= 1")
        amp_lo, amp_hi = self.amplitude_range_m
        if amp_lo <= 0.0 or amp_hi < amp_lo:
            raise ValueError("amplitude_range_m must be a positive increasing range")
        sig_lo, sig_hi = self.sigma_range_m
        if sig_lo <= 0.0 or sig_hi < sig_lo:
            raise ValueError("sigma_range_m must be a positive increasing range")
        if self.max_attempts < 1:
            raise ValueError("max_attempts must be positive")

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "cut_feature_count_range": list(self.cut_feature_count_range),
            "fill_feature_count_range": list(self.fill_feature_count_range),
            "amplitude_range_m": list(self.amplitude_range_m),
            "sigma_range_m": list(self.sigma_range_m),
            "max_attempts": self.max_attempts,
            "bounds": self.bounds.canonical_dict(),
        }


@dataclass(frozen=True, slots=True)
class MassBalancedFeature:
    """One radial elevation feature; amplitude is positive for fill and negative for
    cut.
    """

    centre_xy_m: tuple[float, float]
    sigma_m: float
    amplitude_m: float

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "centre_xy_m": list(self.centre_xy_m),
            "sigma_m": self.sigma_m,
            "amplitude_m": self.amplitude_m,
        }

    def significant_radius_m(self, significant_elevation_m: float) -> float:
        """Radius beyond which this feature's magnitude is insignificant."""
        magnitude = abs(float(self.amplitude_m))
        if magnitude <= significant_elevation_m:
            return 0.0
        return float(
            self.sigma_m * np.sqrt(2.0 * np.log(magnitude / significant_elevation_m))
        )


@dataclass(frozen=True, slots=True)
class GeneralResetLayoutParams:
    """Versioned deterministic flat-bed reset layout of the general task."""

    version: str
    spawner: str
    dim_x: int
    dim_y: int
    dim_z: int
    particle_size_m: float
    particle_spacing_m: float
    ratio: float
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
            "translation_m": list(self.translation_m),
            "orientation_wxyz": list(self.orientation_wxyz),
            "velocity_m_s": list(self.velocity_m_s),
        }

    def flat_bed(self) -> FlatParticleBedParams:
        """The analytic flat lattice this reset layout spawns."""
        return FlatParticleBedParams(
            dim_x=self.dim_x,
            dim_y=self.dim_y,
            dim_z=self.dim_z,
            particle_size_m=self.particle_size_m,
            particle_spacing_m=self.particle_spacing_m,
            ratio=self.ratio,
            translation_m=self.translation_m,
            orientation_wxyz=self.orientation_wxyz,
        )


def compute_general_reset_layout_sha256(layout: GeneralResetLayoutParams) -> str:
    """SHA-256 over the canonical general flat-bed reset-layout parameters."""
    return _sha256_hex(canonical_json_bytes(layout.canonical_dict()))


# This fixed lattice stays inside the 4 m ROI, leaving the guard at the
# containment base plane rather than extending the regolith plateau.
GENERAL_RESET_LAYOUT = GeneralResetLayoutParams(
    version=GENERAL_RESET_LAYOUT_VERSION,
    spawner="grid",
    dim_x=78,
    dim_y=78,
    dim_z=3,
    particle_size_m=0.05,
    particle_spacing_m=0.0,
    ratio=0.75,
    translation_m=(0.0, 0.0, 0.025),
    orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    velocity_m_s=(0.0, 0.0, 0.0),
)


@dataclass(frozen=True, slots=True)
class FlatBedRaster:
    """Cached analytic flat-bed surface, support counts, and grading cells."""

    surface: np.ndarray
    support_count: np.ndarray
    grading_indices: np.ndarray
    grading_mask: np.ndarray
    cell_centres_xy: np.ndarray

    @property
    def grading_cell_count(self) -> int:
        return int(self.grading_indices.shape[0])


_FLAT_BED_RASTER_CACHE: dict[str, FlatBedRaster] = {}


def flat_bed_raster(
    flat_bed: FlatParticleBedParams,
    *,
    geometry: SourceGridGeometry,
    rule: SurfaceAggregationRule,
) -> FlatBedRaster:
    """Rasterize and cache the analytic flat bed and its grading cells."""
    key = canonical_json(
        {
            "flat_bed": flat_bed.canonical_dict(),
            "geometry": geometry.canonical_dict(),
            "surface_rule": rule.canonical_dict(),
        }
    )
    cached = _FLAT_BED_RASTER_CACHE.get(key)
    if cached is not None:
        return cached

    surface, support_count = materialize_surface_raster(
        flat_bed.world_particle_centres(), geometry=geometry, rule=rule
    )
    r0, c0 = geometry.roi_index_start
    r1, c1 = geometry.roi_index_stop
    roi_mask = np.zeros(geometry.source_shape, dtype=bool)
    roi_mask[r0:r1, c0:c1] = True
    grading_mask = (support_count > 0) & roi_mask
    grading_indices = np.argwhere(grading_mask).astype(np.int64)
    if grading_indices.shape[0] < 1:
        raise ValueError("the flat bed supports no grading cell inside the ROI")
    centres_x = geometry.cell_centres(0)
    centres_y = geometry.cell_centres(1)
    grid_x, grid_y = np.meshgrid(centres_x, centres_y, indexing="ij")
    cell_centres_xy = np.stack([grid_x, grid_y], axis=-1)

    for array in (surface, support_count, grading_indices, grading_mask):
        array.setflags(write=False)
    cell_centres_xy.setflags(write=False)
    raster = FlatBedRaster(
        surface=surface,
        support_count=support_count,
        grading_indices=grading_indices,
        grading_mask=grading_mask,
        cell_centres_xy=cell_centres_xy,
    )
    _FLAT_BED_RASTER_CACHE[key] = raster
    return raster


@dataclass(frozen=True, slots=True)
class GeneralMissionFeasibility:
    """Measured feasibility of one general mission against its declared bounds."""

    grading_cell_count: int
    cut_volume_m3: float
    fill_volume_m3: float
    net_volume_m3: float
    available_material_m3: float
    """Coarse available material above the per-cell floor, summed over graded cells."""

    max_cut_depth_m: float
    max_fill_height_m: float
    min_graded_elevation_m: float
    max_graded_elevation_m: float
    max_significant_radius_m: float
    """Largest max-norm radius of materialized cells above the significance threshold."""

    grading_mae_from_reset_m: float
    fraction_within_tolerance_from_reset: float
    coverage_complete: bool
    violations: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.violations

    def canonical_dict(self) -> dict[str, Any]:
        return {
            "grading_cell_count": self.grading_cell_count,
            "cut_volume_m3": self.cut_volume_m3,
            "fill_volume_m3": self.fill_volume_m3,
            "net_volume_m3": self.net_volume_m3,
            "available_material_m3": self.available_material_m3,
            "max_cut_depth_m": self.max_cut_depth_m,
            "max_fill_height_m": self.max_fill_height_m,
            "min_graded_elevation_m": self.min_graded_elevation_m,
            "max_graded_elevation_m": self.max_graded_elevation_m,
            "max_significant_radius_m": self.max_significant_radius_m,
            "grading_mae_from_reset_m": self.grading_mae_from_reset_m,
            "fraction_within_tolerance_from_reset": (
                self.fraction_within_tolerance_from_reset
            ),
            "coverage_complete": self.coverage_complete,
            "violations": list(self.violations),
        }


class GeneralMissionInfeasibleError(ValueError):
    """Raised when a general mission violates its declared feasibility bounds."""


@dataclass(frozen=True, slots=True)
class GeneralTargetManifest:
    """Hashed per-episode manifest for a general landscaping target."""

    schema_version: int
    manifest_id: str
    task_id: str
    generator_version: str
    seed: int
    mission_frame: MissionFramePolicy
    geometry: SourceGridGeometry
    surface_rule: SurfaceAggregationRule
    target_encoding: str
    reset_layout: GeneralResetLayoutParams
    profile: GeneralMissionProfile
    features: tuple[MassBalancedFeature, ...]
    success: SuccessParams
    elevation_bounds_m: tuple[float, float]
    grading_cell_count: int
    target_map_sha256: str
    reset_layout_sha256: str
    io_schema_fingerprint: str
    manifest_sha256: str

    @property
    def bounds(self) -> GeneralMissionBounds:
        return self.profile.bounds

    def canonical_dict(self, *, include_manifest_hash: bool = True) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": self.schema_version,
            "manifest_id": self.manifest_id,
            "task_id": self.task_id,
            "generator": {
                "version": self.generator_version,
                "seed": self.seed,
                "profile": self.profile.canonical_dict(),
            },
            "mission_frame": self.mission_frame.canonical_dict(),
            "grid_geometry": self.geometry.canonical_dict(),
            "surface_rule": self.surface_rule.canonical_dict(),
            "target": {
                "encoding": self.target_encoding,
                "features": [f.canonical_dict() for f in self.features],
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

    # -- materialization -------------------------------------------------

    def _raster(self) -> FlatBedRaster:
        return flat_bed_raster(
            self.reset_layout.flat_bed(),
            geometry=self.geometry,
            rule=self.surface_rule,
        )

    def delta(self) -> np.ndarray:
        """Signed requested deformation of the reset surface, masked to G."""
        raster = self._raster()
        return _feature_field(
            self.features,
            cell_centres_xy=raster.cell_centres_xy,
            grading_mask=raster.grading_mask,
        )

    def materialize(self) -> TargetMapMaterialization:
        """Materialize the target and static grading indices from the flat bed and
        features.
        """
        raster = self._raster()
        target = (raster.surface.astype(np.float64) + self.delta()).astype(np.float32)
        target.setflags(write=False)
        return TargetMapMaterialization(
            target=target,
            support_count=raster.support_count,
            grading_indices=raster.grading_indices,
        )

    def feasibility(self) -> GeneralMissionFeasibility:
        """Measure this mission against its declared feasibility envelope."""
        raster = self._raster()
        return _measure_feasibility(
            reset_surface=raster.surface,
            target=self.materialize().target,
            grading_mask=raster.grading_mask,
            grading_cell_count=raster.grading_cell_count,
            bounds=self.bounds,
            geometry=self.geometry,
            success_tolerance_m=float(self.success.success_tolerance_m),
            elevation_bounds_m=self.elevation_bounds_m,
        )

    # -- verification ----------------------------------------------------

    def verify(
        self,
        *,
        contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
    ) -> TargetMapMaterialization:
        """Verify hashes, contract identity, and feasibility; raise on mismatch."""
        errors: list[str] = []
        if self.task_id != GENERAL_TASK_ID:
            errors.append(f"manifest identifies task {self.task_id!r}")
        if self.generator_version != GENERAL_TARGET_GENERATOR_VERSION:
            errors.append(f"unsupported generator {self.generator_version!r}")
        if self.target_encoding != GENERAL_TARGET_ENCODING:
            errors.append(f"unsupported target encoding {self.target_encoding!r}")
        if self.geometry != contract.geometry:
            errors.append("manifest grid geometry differs from the IO contract")
        if self.surface_rule != contract.surface_rule:
            errors.append("manifest surface rule differs from the IO contract")
        if self.mission_frame != contract.mission_frame:
            errors.append("manifest mission frame differs from the IO contract")
        if self.io_schema_fingerprint != contract.fingerprint():
            errors.append("io_schema_fingerprint does not match the IO contract")

        materialized = self.materialize()
        if self.target_map_sha256 != compute_target_map_sha256(
            self.geometry, materialized.target
        ):
            errors.append("target_map_sha256 does not match the materialized target")
        if self.reset_layout_sha256 != compute_general_reset_layout_sha256(
            self.reset_layout
        ):
            errors.append("reset_layout_sha256 does not match the reset layout")
        if self.manifest_sha256 != compute_general_manifest_sha256(self):
            errors.append("manifest_sha256 does not match the canonical manifest")
        if self.grading_cell_count != int(materialized.grading_indices.shape[0]):
            errors.append("grading_cell_count does not match materialized indices")

        feasibility = self.feasibility()
        errors.extend(feasibility.violations)

        if errors:
            raise GeneralMissionInfeasibleError(
                f"general target manifest {self.manifest_id!r} verification "
                "failed: " + "; ".join(errors)
            )
        return materialized


def compute_general_manifest_sha256(manifest: GeneralTargetManifest) -> str:
    """SHA-256 over canonical general-manifest JSON excluding its own hash."""
    return _sha256_hex(
        canonical_json_bytes(manifest.canonical_dict(include_manifest_hash=False))
    )


def _feature_field(
    features: tuple[MassBalancedFeature, ...] | list[MassBalancedFeature],
    *,
    cell_centres_xy: np.ndarray,
    grading_mask: np.ndarray,
) -> np.ndarray:
    """Sum the signed feature field over the source grid, masked to G."""
    grid_x = cell_centres_xy[..., 0]
    grid_y = cell_centres_xy[..., 1]
    field_sum = np.zeros(grid_x.shape, dtype=np.float64)
    for feature in features:
        cx, cy = feature.centre_xy_m
        sigma = float(feature.sigma_m)
        dist_sq = (grid_x - cx) ** 2 + (grid_y - cy) ** 2
        field_sum += float(feature.amplitude_m) * np.exp(
            -dist_sq / (2.0 * sigma * sigma)
        )
    field_sum *= grading_mask
    return field_sum


def _measure_feasibility(
    *,
    reset_surface: np.ndarray,
    target: np.ndarray,
    grading_mask: np.ndarray,
    grading_cell_count: int,
    bounds: GeneralMissionBounds,
    geometry: SourceGridGeometry,
    success_tolerance_m: float,
    elevation_bounds_m: tuple[float, float],
) -> GeneralMissionFeasibility:
    """Measure declared feasibility invariants on the materialized target."""
    cell_area = float(geometry.cell_size_m) ** 2
    graded_reset = reset_surface.astype(np.float64)[grading_mask]
    graded_target = target.astype(np.float64)[grading_mask]
    delta = graded_target - graded_reset

    fill = delta[delta > 0.0]
    cut = -delta[delta < 0.0]
    fill_volume = float(fill.sum() * cell_area)
    cut_volume = float(cut.sum() * cell_area)
    net_volume = float(delta.sum() * cell_area)
    # This aggregate is a coarse sanity bound; the per-cell elevation floor
    # below is the operative cut limit because cells cannot borrow depth.
    available = float((graded_reset - bounds.min_target_elevation_m).sum() * cell_area)

    max_cut_depth = float(cut.max()) if cut.size else 0.0
    max_fill_height = float(fill.max()) if fill.size else 0.0
    min_graded = float(graded_target.min())
    max_graded = float(graded_target.max())
    abs_delta = np.abs(delta)
    grading_mae = float(abs_delta.mean())
    fraction_within = float((abs_delta <= success_tolerance_m).mean())

    # Overlapping features can exceed the threshold beyond their individual
    # analytic radii, so measure reach on the materialized field.
    significant_cells = (
        np.abs(target.astype(np.float64) - reset_surface.astype(np.float64))
        > bounds.significant_elevation_m
    )
    if significant_cells.any():
        rows, cols = np.nonzero(significant_cells)
        centres_x = np.asarray(geometry.cell_centres(0))
        centres_y = np.asarray(geometry.cell_centres(1))
        max_radius = float(
            max(np.abs(centres_x[rows]).max(), np.abs(centres_y[cols]).max())
        )
    else:
        max_radius = 0.0

    coverage_complete = bool(
        np.isfinite(target).all() and target.shape == tuple(geometry.source_shape)
    )
    guard_mask = np.ones(target.shape, dtype=bool)
    r0, c0 = geometry.roi_index_start
    r1, c1 = geometry.roi_index_stop
    guard_mask[r0:r1, c0:c1] = False

    violations: list[str] = []
    if abs(net_volume) > bounds.volume_balance_tolerance_m3:
        violations.append(
            f"cut/fill imbalance {net_volume:+.6g} m^3 exceeds the declared "
            f"tolerance {bounds.volume_balance_tolerance_m3:g} m^3"
        )
    if cut_volume > available:
        violations.append(
            f"requested cut {cut_volume:.6g} m^3 exceeds the available "
            f"material {available:.6g} m^3"
        )
    if min_graded < bounds.min_target_elevation_m - 1e-9:
        violations.append(
            f"graded target minimum {min_graded:.6g} m is below the declared "
            f"floor {bounds.min_target_elevation_m:g} m"
        )
    if max_graded > bounds.max_target_elevation_m + 1e-9:
        violations.append(
            f"graded target maximum {max_graded:.6g} m is above the declared "
            f"containment ceiling {bounds.max_target_elevation_m:g} m"
        )
    low, high = elevation_bounds_m
    if float(target.min()) < low or float(target.max()) > high:
        violations.append("target violates the contract elevation envelope")
    if max_radius > bounds.work_zone_half_extent_m + 1e-9:
        violations.append(
            f"significant deformation reaches {max_radius:.6g} m, outside the "
            f"declared work zone {bounds.work_zone_half_extent_m:g} m"
        )
    if not coverage_complete:
        violations.append("target is not a complete finite source-grid map")
    base_plane = float(reset_surface[guard_mask].max())
    if not np.array_equal(target[guard_mask], reset_surface[guard_mask]):
        violations.append("guard cells differ from the containment base plane")
    if base_plane != 0.0:
        violations.append(
            f"guard base-plane elevation {base_plane:g} m is not the datum"
        )
    if grading_mae < bounds.min_grading_mae_m:
        violations.append(
            f"mission magnitude {grading_mae:.6g} m is below the declared "
            f"minimum {bounds.min_grading_mae_m:g} m"
        )
    if fraction_within >= bounds.max_initial_fraction_within_tolerance:
        violations.append(
            f"the reset state already scores {fraction_within:.4f} within "
            f"tolerance, at or above the declared maximum "
            f"{bounds.max_initial_fraction_within_tolerance:g}"
        )

    return GeneralMissionFeasibility(
        grading_cell_count=int(grading_cell_count),
        cut_volume_m3=cut_volume,
        fill_volume_m3=fill_volume,
        net_volume_m3=net_volume,
        available_material_m3=available,
        max_cut_depth_m=max_cut_depth,
        max_fill_height_m=max_fill_height,
        min_graded_elevation_m=min_graded,
        max_graded_elevation_m=max_graded,
        max_significant_radius_m=max_radius,
        grading_mae_from_reset_m=grading_mae,
        fraction_within_tolerance_from_reset=fraction_within,
        coverage_complete=coverage_complete,
        violations=tuple(violations),
    )


def _sample_features(
    rng: np.random.Generator,
    *,
    profile: GeneralMissionProfile,
    bed_half_extent_m: float,
) -> list[MassBalancedFeature]:
    """Draw one cut set and one fill set inside the reachable work zone."""
    bounds = profile.bounds
    half_limit = min(bed_half_extent_m, bounds.work_zone_half_extent_m)
    features: list[MassBalancedFeature] = []
    for sign, count_range in (
        (1.0, profile.fill_feature_count_range),
        (-1.0, profile.cut_feature_count_range),
    ):
        count = int(rng.integers(count_range[0], count_range[1] + 1))
        for _ in range(count):
            amplitude = float(rng.uniform(*profile.amplitude_range_m))
            sigma = float(rng.uniform(*profile.sigma_range_m))
            # Keep the significant footprint inside the work zone when it fits;
            # materialized-field feasibility remains authoritative.
            radius = float(
                sigma
                * np.sqrt(
                    2.0
                    * np.log(
                        max(amplitude, bounds.significant_elevation_m * 1.01)
                        / bounds.significant_elevation_m
                    )
                )
            )
            centre_half = half_limit - radius
            if centre_half <= 0.0:
                centre_half = 0.0
            features.append(
                MassBalancedFeature(
                    centre_xy_m=(
                        float(rng.uniform(-centre_half, centre_half)),
                        float(rng.uniform(-centre_half, centre_half)),
                    ),
                    sigma_m=sigma,
                    amplitude_m=sign * amplitude,
                )
            )
    return features


def _balance_and_bound(
    features: list[MassBalancedFeature],
    *,
    raster: FlatBedRaster,
    bounds: GeneralMissionBounds,
) -> tuple[MassBalancedFeature, ...] | None:
    """Balance cut and fill, then scale features to fit elevation bounds.

    Return ``None`` when masking leaves only cuts or only fills.
    """
    fill = [f for f in features if f.amplitude_m > 0.0]
    cut = [f for f in features if f.amplitude_m < 0.0]
    if not fill or not cut:
        return None

    fill_field = _feature_field(
        fill, cell_centres_xy=raster.cell_centres_xy, grading_mask=raster.grading_mask
    )
    cut_field = _feature_field(
        cut, cell_centres_xy=raster.cell_centres_xy, grading_mask=raster.grading_mask
    )
    fill_sum = float(fill_field.sum())
    cut_sum = float(-cut_field.sum())
    if fill_sum <= 0.0 or cut_sum <= 0.0:
        return None

    balance_scale = fill_sum / cut_sum
    delta = fill_field + cut_field * balance_scale

    graded_reset = raster.surface.astype(np.float64)[raster.grading_mask]
    graded_delta = delta[raster.grading_mask]
    # Targets are published as float32. Leave two representable float32 ULPs
    # below the declared ceiling: one for the final cast and one for rounding
    # differences between this pre-scale sum and feature rematerialization.
    ceiling_f32 = np.float32(bounds.max_target_elevation_m)
    float32_ulp = abs(float(np.spacing(ceiling_f32)))
    safe_max_target = bounds.max_target_elevation_m - 2.0 * float32_ulp
    head_up = safe_max_target - graded_reset
    head_down = graded_reset - bounds.min_target_elevation_m
    if float(head_up.min()) < 0.0 or float(head_down.min()) < 0.0:
        raise GeneralMissionInfeasibleError(
            "the reset surface itself violates the declared elevation bounds"
        )
    up = np.where(
        graded_delta > 1e-12, head_up / np.maximum(graded_delta, 1e-12), np.inf
    )
    down = np.where(
        graded_delta < -1e-12, head_down / np.maximum(-graded_delta, 1e-12), np.inf
    )
    bound_scale = min(1.0, float(up.min()), float(down.min()))
    if bound_scale <= 0.0:
        return None

    return tuple(
        replace(
            feature,
            amplitude_m=float(
                feature.amplitude_m
                * bound_scale
                * (balance_scale if feature.amplitude_m < 0.0 else 1.0)
            ),
        )
        for feature in features
    )


def general_mission_seed(*, base_seed: int, env_id: int, episode_index: int) -> int:
    """Return a reproducible mission seed for one environment episode."""
    sequence = np.random.SeedSequence(
        entropy=(
            _GENERATOR_ENTROPY_SALT,
            int(base_seed),
            int(env_id),
            int(episode_index),
        )
    )
    return int(sequence.generate_state(1, dtype=np.uint32)[0])


def generate_general_target_manifest(
    *,
    seed: int,
    success: SuccessParams,
    reset_layout: GeneralResetLayoutParams = GENERAL_RESET_LAYOUT,
    profile: GeneralMissionProfile | None = None,
    contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
) -> GeneralTargetManifest:
    """Generate a feasible, mass-balanced, hashed mission from the flat reset bed.

    Infeasible draws are retried; exhaustion raises
    ``GeneralMissionInfeasibleError``.
    """
    profile = GeneralMissionProfile() if profile is None else profile
    bounds = profile.bounds
    flat_bed = reset_layout.flat_bed()
    raster = flat_bed_raster(
        flat_bed, geometry=contract.geometry, rule=contract.surface_rule
    )
    bed_half_extent = float(np.abs(flat_bed.world_particle_centres()[:, :2]).max())
    elevation_leaf = contract.leaf("proprio_dyn/heightmap_current_global")
    elevation_bounds = (float(elevation_leaf.low[0]), float(elevation_leaf.high[0]))

    rng = np.random.default_rng(
        np.random.SeedSequence(entropy=(_GENERATOR_ENTROPY_SALT, int(seed)))
    )
    rejections: list[str] = []
    for _attempt in range(profile.max_attempts):
        drawn = _sample_features(
            rng, profile=profile, bed_half_extent_m=bed_half_extent
        )
        features = _balance_and_bound(drawn, raster=raster, bounds=bounds)
        if features is None:
            rejections.append("unbalanceable draw")
            continue
        delta = _feature_field(
            features,
            cell_centres_xy=raster.cell_centres_xy,
            grading_mask=raster.grading_mask,
        )
        target = (raster.surface.astype(np.float64) + delta).astype(np.float32)
        feasibility = _measure_feasibility(
            reset_surface=raster.surface,
            target=target,
            grading_mask=raster.grading_mask,
            grading_cell_count=raster.grading_cell_count,
            bounds=bounds,
            geometry=contract.geometry,
            success_tolerance_m=float(success.success_tolerance_m),
            elevation_bounds_m=elevation_bounds,
        )
        if not feasibility.ok:
            rejections.append("; ".join(feasibility.violations))
            continue

        provisional = GeneralTargetManifest(
            schema_version=GENERAL_MANIFEST_SCHEMA_VERSION,
            manifest_id=(
                f"srb/{GENERAL_TASK_ID}/{GENERAL_TARGET_GENERATOR_VERSION}"
                f"/seed_{int(seed):010d}"
            ),
            task_id=GENERAL_TASK_ID,
            generator_version=GENERAL_TARGET_GENERATOR_VERSION,
            seed=int(seed),
            mission_frame=contract.mission_frame,
            geometry=contract.geometry,
            surface_rule=contract.surface_rule,
            target_encoding=GENERAL_TARGET_ENCODING,
            reset_layout=reset_layout,
            profile=profile,
            features=features,
            success=success,
            elevation_bounds_m=elevation_bounds,
            grading_cell_count=raster.grading_cell_count,
            target_map_sha256=compute_target_map_sha256(contract.geometry, target),
            reset_layout_sha256=compute_general_reset_layout_sha256(reset_layout),
            io_schema_fingerprint=contract.fingerprint(),
            manifest_sha256="",
        )
        return replace(
            provisional,
            manifest_sha256=compute_general_manifest_sha256(provisional),
        )

    raise GeneralMissionInfeasibleError(
        f"no feasible general mission for seed {seed} within "
        f"{profile.max_attempts} attempts; last rejections: "
        + " | ".join(rejections[-3:])
    )


__all__ = [
    "GENERAL_MANIFEST_SCHEMA_VERSION",
    "GENERAL_RESET_LAYOUT",
    "GENERAL_RESET_LAYOUT_VERSION",
    "GENERAL_TARGET_ENCODING",
    "GENERAL_TARGET_GENERATOR_VERSION",
    "GENERAL_TASK_ID",
    "FlatBedRaster",
    "GeneralMissionBounds",
    "GeneralMissionFeasibility",
    "GeneralMissionInfeasibleError",
    "GeneralMissionProfile",
    "GeneralResetLayoutParams",
    "GeneralTargetManifest",
    "MassBalancedFeature",
    "compute_elevation_map",
    "compute_general_manifest_sha256",
    "compute_general_reset_layout_sha256",
    "flat_bed_raster",
    "general_mission_seed",
    "generate_general_target_manifest",
]
