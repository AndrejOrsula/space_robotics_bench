"""Isaac-free particle layout generators."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _as_finite_float32(positions: np.ndarray) -> np.ndarray:
    """Convert positions only when every coordinate is representable in float32."""
    max_float32 = np.finfo(np.float32).max
    if not np.all(np.isfinite(positions)) or np.any(np.abs(positions) > max_float32):
        raise ValueError(
            "particle positions must be finite and representable as float32"
        )
    return positions.astype(np.float32, copy=False)


def particle_pitch_m(
    particle_size: float, particle_spacing: float, ratio: float
) -> float:
    """Return the lattice pitch; ``ratio < 1`` thins particles by widening it."""
    ratio_scale = ratio ** (1.0 / 3.0) if ratio < 1.0 else 1.0
    return particle_spacing + particle_size / ratio_scale


@dataclass(frozen=True, slots=True)
class CraterPileParticleLayout:
    """Particle positions plus geometry metadata for a crater-and-pile bed."""

    positions: np.ndarray
    spacing_m: float
    bed_surface_z_m: float
    crater_particle_count: int
    pile_particle_count: int
    effective_dim_xy: tuple[int, int]


@dataclass(frozen=True, slots=True)
class MoundPileParticleLayout:
    """Particle positions plus geometry metadata for a mound-on-bed layout.

    The full analytic pile cone is stacked on an intact flat bed (layout
    version ``mound_pile_v1``): unlike the
    crater-and-pile layout there is no excavation, so the particle count is
    the bed count plus the whole cone.
    """

    positions: np.ndarray
    spacing_m: float
    bed_surface_z_m: float
    pile_particle_count: int
    effective_dim_xy: tuple[int, int]


@dataclass(frozen=True, slots=True)
class FlatParticleBedLayout:
    """Undeformed flat particle-bed lattice shared with the crater builder.

    This is the versioned analytic flat lattice referenced by the landscaping
    target manifest (``encoding="analytic_flat_particle_bed_v1"``): the exact
    layered bed that :func:`build_crater_pile_particle_layout` starts from
    before any crater particles are moved into the pile.
    """

    positions: np.ndarray
    spacing_m: float
    bed_surface_z_m: float
    effective_dim_xy: tuple[int, int]


def build_flat_particle_bed_layout(
    *,
    dim_x: int,
    dim_y: int,
    dim_z: int,
    particle_size: float,
    particle_spacing: float,
    ratio: float,
) -> FlatParticleBedLayout:
    """Build the undeformed flat layered bed lattice.

    Coordinates are local to the particle-set prim; the spawner applies its
    configured world translation and rotation afterwards. The lattice math is
    byte-identical to the bed that seeds the crater-and-pile layout.
    """
    if min(dim_x, dim_y) < 1 or dim_z < 1:
        raise ValueError("flat particle bed requires dim_x/dim_y >= 1 and dim_z >= 1")
    if not math.isfinite(particle_size) or particle_size <= 0.0:
        raise ValueError("particle_size must be positive")
    if not math.isfinite(particle_spacing) or particle_spacing < 0.0:
        raise ValueError("particle_spacing must be finite and non-negative")
    if not 0.0 < ratio <= 1.0:
        raise ValueError("ratio must be in (0, 1]")

    bed, _x, _y, spacing_m, bed_surface_z_m, effective_dim_xy = _flat_bed_lattice(
        dim_x=dim_x,
        dim_y=dim_y,
        dim_z=dim_z,
        particle_size=particle_size,
        particle_spacing=particle_spacing,
        ratio=ratio,
    )
    return FlatParticleBedLayout(
        positions=_as_finite_float32(bed),
        spacing_m=spacing_m,
        bed_surface_z_m=bed_surface_z_m,
        effective_dim_xy=effective_dim_xy,
    )


def _flat_bed_lattice(
    *,
    dim_x: int,
    dim_y: int,
    dim_z: int,
    particle_size: float,
    particle_spacing: float,
    ratio: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float, tuple[int, int]]:
    """Return the shared layered-bed lattice and its geometry metadata."""
    ratio_scale = ratio ** (1.0 / 3.0) if ratio < 1.0 else 1.0
    effective_dim_x = math.floor(ratio_scale * dim_x)
    effective_dim_y = math.floor(ratio_scale * dim_y)
    if effective_dim_x < 1 or effective_dim_y < 1:
        raise ValueError("ratio leaves no particles along X or Y")

    spacing_m = particle_pitch_m(particle_size, particle_spacing, ratio)
    x = -spacing_m * effective_dim_x / 2.0 + np.arange(effective_dim_x) * spacing_m
    y = -spacing_m * effective_dim_y / 2.0 + np.arange(effective_dim_y) * spacing_m
    z = np.arange(dim_z) * spacing_m
    gx, gy, gz = np.meshgrid(x, y, z, indexing="ij")
    bed = np.stack((gx, gy, gz), axis=-1).reshape(-1, 3)
    bed_surface_z_m = float(z[-1])
    return bed, x, y, spacing_m, bed_surface_z_m, (effective_dim_x, effective_dim_y)


def build_crater_pile_particle_layout(
    *,
    dim_x: int,
    dim_y: int,
    dim_z: int,
    particle_size: float,
    particle_spacing: float,
    ratio: float,
    crater_center_xy_m: tuple[float, float],
    crater_radius_m: float,
    crater_depth_m: float,
    pile_center_xy_m: tuple[float, float],
    pile_radius_m: float,
    pile_height_m: float,
) -> CraterPileParticleLayout:
    """Build one shallow bowl crater and one nearby pile from a layered bed.

    Particles removed from the crater are reused in the pile, preserving mass
    and particle count. Coordinates are local to the particle-set prim; the
    spawner applies its configured world translation and rotation afterwards.
    """
    _validate_layout_inputs(
        dim_x=dim_x,
        dim_y=dim_y,
        dim_z=dim_z,
        particle_size=particle_size,
        particle_spacing=particle_spacing,
        ratio=ratio,
        crater_center_xy_m=crater_center_xy_m,
        crater_radius_m=crater_radius_m,
        crater_depth_m=crater_depth_m,
        pile_center_xy_m=pile_center_xy_m,
        pile_radius_m=pile_radius_m,
        pile_height_m=pile_height_m,
    )

    bed, x, y, spacing_m, bed_surface_z_m, effective_dim_xy = _flat_bed_lattice(
        dim_x=dim_x,
        dim_y=dim_y,
        dim_z=dim_z,
        particle_size=particle_size,
        particle_spacing=particle_spacing,
        ratio=ratio,
    )
    effective_dim_x, effective_dim_y = effective_dim_xy

    _validate_feature_bounds(
        x=x,
        y=y,
        spacing_m=spacing_m,
        crater_center_xy_m=crater_center_xy_m,
        crater_radius_m=crater_radius_m,
        pile_center_xy_m=pile_center_xy_m,
        pile_radius_m=pile_radius_m,
    )

    crater_dx = bed[:, 0] - crater_center_xy_m[0]
    crater_dy = bed[:, 1] - crater_center_xy_m[1]
    crater_distance = np.hypot(crater_dx, crater_dy)
    crater_shape = np.clip(
        1.0 - (crater_distance / crater_radius_m) ** 2,
        0.0,
        1.0,
    )
    crater_surface = np.maximum(
        0.0,
        bed_surface_z_m - crater_depth_m * crater_shape,
    )
    removed_mask = (crater_distance < crater_radius_m) & (
        bed[:, 2] > crater_surface + np.finfo(np.float64).eps
    )
    crater_particle_count = int(np.count_nonzero(removed_mask))
    if crater_particle_count < 1:
        raise ValueError(
            "crater geometry removes no particles; increase crater depth/radius"
        )

    pile_candidates = _pile_candidates(
        spacing_m=spacing_m,
        bed_surface_z_m=bed_surface_z_m,
        pile_center_xy_m=pile_center_xy_m,
        pile_radius_m=pile_radius_m,
        pile_height_m=pile_height_m,
    )
    if pile_candidates.shape[0] < crater_particle_count:
        raise ValueError(
            "pile geometry has insufficient capacity for excavated particles: "
            f"capacity={pile_candidates.shape[0]}, required={crater_particle_count}"
        )

    pile = pile_candidates[:crater_particle_count]
    retained_bed = bed[~removed_mask]
    positions = _as_finite_float32(np.concatenate((retained_bed, pile), axis=0))
    if positions.shape[0] != bed.shape[0]:
        raise RuntimeError("crater-pile layout did not conserve particle count")

    return CraterPileParticleLayout(
        positions=positions,
        spacing_m=spacing_m,
        bed_surface_z_m=bed_surface_z_m,
        crater_particle_count=crater_particle_count,
        pile_particle_count=pile.shape[0],
        effective_dim_xy=(effective_dim_x, effective_dim_y),
    )


def build_mound_pile_particle_layout(
    *,
    dim_x: int,
    dim_y: int,
    dim_z: int,
    particle_size: float,
    particle_spacing: float,
    ratio: float,
    pile_center_xy_m: tuple[float, float],
    pile_radius_m: float,
    pile_height_m: float,
) -> MoundPileParticleLayout:
    """Build one regolith mound (the full analytic pile cone) on a flat bed.

    Layout version ``mound_pile_v1``: the bed lattice is byte-identical to
    :func:`build_flat_particle_bed_layout` (and therefore to the manifest's
    analytic flat target), and the entire :func:`_pile_candidates` cone is
    stacked on top of it — no crater, no excavation, deterministic count.
    """
    if min(dim_x, dim_y) < 1 or dim_z < 2:
        raise ValueError("mound-pile bed requires dim_x/dim_y >= 1 and dim_z >= 2")
    if not math.isfinite(particle_size) or particle_size <= 0.0:
        raise ValueError("particle_size must be positive")
    if not math.isfinite(particle_spacing) or particle_spacing < 0.0:
        raise ValueError("particle_spacing must be finite and non-negative")
    if not 0.0 < ratio <= 1.0:
        raise ValueError("ratio must be in (0, 1]")
    if pile_radius_m <= 0.0 or pile_height_m <= 0.0:
        raise ValueError("pile radius and height must be positive")
    values = (*pile_center_xy_m, pile_radius_m, pile_height_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("mound-pile geometry values must be finite")

    bed, x, y, spacing_m, bed_surface_z_m, effective_dim_xy = _flat_bed_lattice(
        dim_x=dim_x,
        dim_y=dim_y,
        dim_z=dim_z,
        particle_size=particle_size,
        particle_spacing=particle_spacing,
        ratio=ratio,
    )

    x_limit = min(abs(float(x[0])), abs(float(x[-1])))
    y_limit = min(abs(float(y[0])), abs(float(y[-1])))
    if abs(pile_center_xy_m[0]) + pile_radius_m > x_limit:
        raise ValueError("pile exceeds particle bed along X")
    if abs(pile_center_xy_m[1]) + pile_radius_m > y_limit:
        raise ValueError("pile exceeds particle bed along Y")

    pile = _pile_candidates(
        spacing_m=spacing_m,
        bed_surface_z_m=bed_surface_z_m,
        pile_center_xy_m=pile_center_xy_m,
        pile_radius_m=pile_radius_m,
        pile_height_m=pile_height_m,
    )
    positions = _as_finite_float32(np.concatenate((bed, pile), axis=0))
    return MoundPileParticleLayout(
        positions=positions,
        spacing_m=spacing_m,
        bed_surface_z_m=bed_surface_z_m,
        pile_particle_count=int(pile.shape[0]),
        effective_dim_xy=effective_dim_xy,
    )


def _pile_candidates(
    *,
    spacing_m: float,
    bed_surface_z_m: float,
    pile_center_xy_m: tuple[float, float],
    pile_radius_m: float,
    pile_height_m: float,
) -> np.ndarray:
    layer_count = math.floor(pile_height_m / spacing_m)
    if layer_count < 1:
        raise ValueError("pile_height_m must fit at least one particle layer")

    extent_cells = math.ceil(pile_radius_m / spacing_m)
    offsets = np.arange(-extent_cells, extent_cells + 1) * spacing_m
    offset_x, offset_y = np.meshgrid(offsets, offsets, indexing="ij")
    radial_distance = np.hypot(offset_x, offset_y)
    candidate_order = np.lexsort(
        (offset_y.reshape(-1), offset_x.reshape(-1), radial_distance.reshape(-1))
    )

    layers: list[np.ndarray] = []
    for layer_idx in range(layer_count):
        layer_radius = pile_radius_m * (layer_count - layer_idx) / layer_count
        valid = radial_distance.reshape(-1)[candidate_order] <= (
            layer_radius + np.finfo(np.float64).eps
        )
        ordered_x = offset_x.reshape(-1)[candidate_order][valid]
        ordered_y = offset_y.reshape(-1)[candidate_order][valid]
        layer_z = bed_surface_z_m + (layer_idx + 1) * spacing_m
        layers.append(
            np.column_stack(
                (
                    ordered_x + pile_center_xy_m[0],
                    ordered_y + pile_center_xy_m[1],
                    np.full(ordered_x.shape, layer_z),
                )
            )
        )
    return np.concatenate(layers, axis=0)


def _validate_layout_inputs(
    *,
    dim_x: int,
    dim_y: int,
    dim_z: int,
    particle_size: float,
    particle_spacing: float,
    ratio: float,
    crater_center_xy_m: tuple[float, float],
    crater_radius_m: float,
    crater_depth_m: float,
    pile_center_xy_m: tuple[float, float],
    pile_radius_m: float,
    pile_height_m: float,
) -> None:
    if min(dim_x, dim_y) < 1 or dim_z < 2:
        raise ValueError("crater-pile bed requires dim_x/dim_y >= 1 and dim_z >= 2")
    if not math.isfinite(particle_size) or particle_size <= 0.0:
        raise ValueError("particle_size must be positive")
    if not math.isfinite(particle_spacing) or particle_spacing < 0.0:
        raise ValueError("particle_spacing must be finite and non-negative")
    if not 0.0 < ratio <= 1.0:
        raise ValueError("ratio must be in (0, 1]")
    if crater_radius_m <= 0.0 or crater_depth_m <= 0.0:
        raise ValueError("crater radius and depth must be positive")
    if pile_radius_m <= 0.0 or pile_height_m <= 0.0:
        raise ValueError("pile radius and height must be positive")

    values = (
        *crater_center_xy_m,
        crater_radius_m,
        crater_depth_m,
        *pile_center_xy_m,
        pile_radius_m,
        pile_height_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("crater-pile geometry values must be finite")


def _validate_feature_bounds(
    *,
    x: np.ndarray,
    y: np.ndarray,
    spacing_m: float,
    crater_center_xy_m: tuple[float, float],
    crater_radius_m: float,
    pile_center_xy_m: tuple[float, float],
    pile_radius_m: float,
) -> None:
    x_limit = min(abs(float(x[0])), abs(float(x[-1])))
    y_limit = min(abs(float(y[0])), abs(float(y[-1])))
    features = (
        ("crater", crater_center_xy_m, crater_radius_m),
        ("pile", pile_center_xy_m, pile_radius_m),
    )
    for name, center, radius in features:
        if abs(center[0]) + radius > x_limit:
            raise ValueError(f"{name} exceeds particle bed along X")
        if abs(center[1]) + radius > y_limit:
            raise ValueError(f"{name} exceeds particle bed along Y")

    center_distance = math.dist(crater_center_xy_m, pile_center_xy_m)
    required_distance = crater_radius_m + pile_radius_m + spacing_m
    if center_distance < required_distance:
        raise ValueError(
            "crater and pile footprints overlap: "
            f"distance={center_distance:.3f}, required={required_distance:.3f}"
        )


__all__ = [
    "CraterPileParticleLayout",
    "FlatParticleBedLayout",
    "MoundPileParticleLayout",
    "build_crater_pile_particle_layout",
    "build_flat_particle_bed_layout",
    "build_mound_pile_particle_layout",
]
