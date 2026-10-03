from dataclasses import MISSING
from typing import Callable

from pxr import Usd

from srb.core.sim import SpawnerCfg, VisualMaterialCfg
from srb.utils.cfg import configclass

from .impl import (
    spawn_particles_crater_pile,
    spawn_particles_grid,
    spawn_particles_mound_pile,
    spawn_particles_pyramid,
)


@configclass
class ParticlesSpawnerCfg(SpawnerCfg):
    # Distribution
    dim_x: int = MISSING  # type: ignore[assignment]
    dim_y: int = MISSING  # type: ignore[assignment]
    dim_z: int = MISSING  # type: ignore[assignment]
    # Distribution
    particle_size: float = 0.01
    particle_spacing: float = 0.0
    ratio: float = 1.0

    # Initial velocity
    velocity: tuple[
        float | tuple[float, float],
        float | tuple[float, float],
        float | tuple[float, float],
    ] = (0.0, 0.0, 0.0)

    ## Particle system
    simulation_owner: str = "/physicsScene"
    """Prim path of the ``UsdPhysics.Scene`` that owns this particle system.

    Authored as the ``physxParticle:simulationOwner`` relationship so ownership
    is deterministic instead of relying on the PhysX fallback (first traversed
    physics scene). Spawning fails fast when the prim is absent or is not a
    ``UsdPhysics.Scene``. ``BaseEnvCfg._add_particles()`` propagates
    ``sim.physics_prim_path`` here; the default matches Isaac Lab's
    ``SimulationCfg.physics_prim_path`` default.
    """
    particle_system_enabled: bool = True
    particle_contact_offset: float | None = None
    max_velocity: float | None = None
    enable_ccd: bool = False
    wind: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Advanced
    contact_offset: float | None = None
    rest_offset: float | None = None
    solid_rest_offset: float | None = None
    fluid_rest_offset: float | None = None
    solver_position_iterations: int = 16
    max_depenetration_velocity: float | None = None
    max_neighborhood: int = 96
    neighborhood_scale: float = 1.01
    global_self_collision_enabled: bool | None = None
    non_particle_collision_enabled: bool | None = None

    ## Particle set
    self_collision: bool = True
    fluid: bool = True
    # Advanced
    particle_group: int = 0

    ## PBD Material
    density: float = 0.0
    friction: float = 0.2
    damping: float = 0.0
    viscosity: float = 0.0
    cohesion: float = 0.0
    surface_tension: float = 0.0
    # NOTE: the `drag` / `lift` PBD-fluid params were removed from omni.physx's
    # ``add_pbd_particle_material`` in Isaac Sim 6.0.1 (omni.physx 110.x). Do not
    # re-add them here: passing them crashes particle spawning with
    # ``TypeError: add_pbd_particle_material() got an unexpected keyword argument``.
    # Advanced
    adhesion: float = 0.0
    particle_adhesion_scale: float = 1.0
    adhesion_offset_scale: float = 0.0
    particle_friction_scale: float = 1.0
    vorticity_confinement: float = 0.0
    cfl_coefficient: float = 1.0
    gravity_scale: float = 1.0

    ## Visuals
    cast_shadows: bool = False
    visible: bool = True
    visual_material: VisualMaterialCfg | None = None

    ## Particle anisotropy (fluid only)
    particle_anisotropy_enabled: bool = True
    anisotropy_scale: float = 1.0
    anisotropy_min: float = 0.2
    anisotropy_max: float = 2.0

    ## Particle smoothing (fluid only)
    particle_smoothing_enabled: bool = True
    smoothing_strength: float = 0.8

    ## Particle isosurface (fluid only)
    isosurface_enabled: bool = False
    isosurface_max_vertices: int = 2**20
    isosurface_max_triangles: int = 2**21
    isosurface_max_subgrids: int = 2**11
    isosurface_grid_spacing: float | None = None
    isosurface_surface_distance: float | None = None
    isosurface_mesh_smoothing_passes: int = 4
    isosurface_mesh_normal_smoothing_passes: int = 4
    isosurface_grid_filtering_passes: str = "GSRS"
    isosurface_grid_smoothing_radius: float | None = None


@configclass
class GridParticlesSpawnerCfg(ParticlesSpawnerCfg):
    func: Callable[..., Usd.Prim] = spawn_particles_grid


@configclass
class PyramidParticlesSpawnerCfg(ParticlesSpawnerCfg):
    func: Callable[..., Usd.Prim] = spawn_particles_pyramid


@configclass
class CraterPileParticlesSpawnerCfg(ParticlesSpawnerCfg):
    """Layered particle bed with one bowl crater and one mass-conserving pile."""

    crater_center_xy_m: tuple[float, float] = (0.45, 0.20)
    crater_radius_m: float = 0.36
    crater_depth_m: float = 0.11
    pile_center_xy_m: tuple[float, float] = (-0.25, -0.20)
    pile_radius_m: float = 0.32
    pile_height_m: float = 0.28
    func: Callable[..., Usd.Prim] = spawn_particles_crater_pile


@configclass
class MoundPileParticlesSpawnerCfg(ParticlesSpawnerCfg):
    """Layered particle bed with one full-cone regolith mound and no crater.

    Layout version ``mound_pile_v1`` for the mound task: the intact
    flat bed plus the entire analytic pile cone stacked on top.
    """

    pile_center_xy_m: tuple[float, float] = (-0.25, -0.20)
    pile_radius_m: float = 0.32
    pile_height_m: float = 0.24
    func: Callable[..., Usd.Prim] = spawn_particles_mound_pile
