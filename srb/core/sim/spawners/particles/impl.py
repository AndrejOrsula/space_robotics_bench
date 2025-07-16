import math
from itertools import count
from typing import TYPE_CHECKING, List, Sequence, Tuple

import isaacsim.core.utils.stage as stage_utils
import numpy
import omni.kit.commands
import omni.timeline
from isaaclab.sim import clone
from omni.physx.scripts import particleUtils as particle_utils
from omni.physx.scripts import physicsUtils as physics_utils
from pxr import Gf, Sdf, Usd, UsdGeom, Vt

from pxr import PhysxSchema  # isort: skip

if TYPE_CHECKING:
    from .cfg import GridParticlesSpawnerCfg, ParticlesSpawnerCfg


@clone
def spawn_particles_grid(
    prim_path: str,
    cfg: "GridParticlesSpawnerCfg",
    translation: Tuple[float, float, float] | None = None,
    orientation: Tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    # Ratio
    if cfg.ratio < 1.0:
        ratio_cbrt = math.pow(cfg.ratio, 1.0 / 3.0)
        dim_x, dim_y, dim_z = (
            math.floor(ratio_cbrt * cfg.dim_x),
            math.floor(ratio_cbrt * cfg.dim_y),
            cfg.dim_z,
        )
        particle_spacing = (
            cfg.particle_spacing + (1.0 / ratio_cbrt - 1.0) * cfg.particle_size
        )
    else:
        dim_x, dim_y, dim_z = cfg.dim_x, cfg.dim_y, cfg.dim_z
        particle_spacing = cfg.particle_spacing

    # Distribution
    positions = _create_particles_grid(
        lower=Gf.Vec3f(
            -(cfg.particle_size + particle_spacing) * dim_x / 2.0,
            -(cfg.particle_size + particle_spacing) * dim_y / 2.0,
            0.0,
        )
        + (
            Gf.Vec3f(
                translation[0],
                translation[1],
                translation[2],
            )
            if translation is not None
            else Gf.Vec3f(0.0, 0.0, 0.0)
        ),
        particle_spacing=cfg.particle_size + particle_spacing,
        dim_x=dim_x,
        dim_y=dim_y,
        dim_z=dim_z,
    )

    # Randomize velocities
    velocities = [
        Gf.Vec3f(
            numpy.random.uniform(cfg.velocity[0][0], cfg.velocity[0][1])
            if isinstance(cfg.velocity[0], Sequence)
            else cfg.velocity[0],
            numpy.random.uniform(cfg.velocity[1][0], cfg.velocity[1][1])
            if isinstance(cfg.velocity[1], Sequence)
            else cfg.velocity[1],
            numpy.random.uniform(cfg.velocity[2][0], cfg.velocity[2][1])
            if isinstance(cfg.velocity[2], Sequence)
            else cfg.velocity[2],
        )
        for _ in range(len(positions))
    ]

    return _spawn_particles(prim_path, cfg, positions, velocities)


@clone
def spawn_particles_pyramid(
    prim_path: str,
    cfg: "GridParticlesSpawnerCfg",
    translation: Tuple[float, float, float] | None = None,
    orientation: Tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    # Ratio
    if cfg.ratio < 1.0:
        ratio_cbrt = math.pow(cfg.ratio, 1.0 / 3.0)
        dim_x, dim_y, dim_z = (
            math.floor(ratio_cbrt * cfg.dim_x),
            math.floor(ratio_cbrt * cfg.dim_y),
            cfg.dim_z,
        )
        particle_spacing = (
            cfg.particle_spacing + (1.0 / ratio_cbrt - 1.0) * cfg.particle_size
        )
    else:
        dim_x, dim_y, dim_z = cfg.dim_x, cfg.dim_y, cfg.dim_z
        particle_spacing = cfg.particle_spacing

    # Distribution
    positions = []
    slope_x = math.floor(dim_x / dim_z)
    slope_y = math.floor(dim_y / dim_z)
    for z in range(dim_z):
        lower = Gf.Vec3f(
            -(cfg.particle_size + particle_spacing) * (dim_x - z * slope_x) / 2.0,
            -(cfg.particle_size + particle_spacing) * (dim_y - z * slope_y) / 2.0,
            z * (cfg.particle_size + particle_spacing),
        ) + (
            Gf.Vec3f(
                translation[0],
                translation[1],
                translation[2],
            )
            if translation is not None
            else Gf.Vec3f(0.0, 0.0, 0.0)
        )
        positions.extend(
            _create_particles_grid(
                lower=lower,
                particle_spacing=cfg.particle_size + particle_spacing,
                dim_x=max(1, dim_x - z * slope_x),
                dim_y=max(1, dim_y - z * slope_y),
                dim_z=1,
            )
        )

    # Randomize velocities
    velocities = [
        Gf.Vec3f(
            numpy.random.uniform(cfg.velocity[0][0], cfg.velocity[0][1])
            if isinstance(cfg.velocity[0], Sequence)
            else cfg.velocity[0],
            numpy.random.uniform(cfg.velocity[1][0], cfg.velocity[1][1])
            if isinstance(cfg.velocity[1], Sequence)
            else cfg.velocity[1],
            numpy.random.uniform(cfg.velocity[2][0], cfg.velocity[2][1])
            if isinstance(cfg.velocity[2], Sequence)
            else cfg.velocity[2],
        )
        for _ in range(len(positions))
    ]

    return _spawn_particles(prim_path, cfg, positions, velocities)


def _spawn_particles(
    prim_path: str,
    cfg: "ParticlesSpawnerCfg",
    positions: Sequence[Gf.Vec3f],
    velocities: Sequence[Gf.Vec3f],
) -> Usd.Prim:
    ## Extract stage
    stage = stage_utils.get_current_stage()

    ## Particle system
    # Find a unique name for the particle system (append digits to the name)
    for i in count(1):
        particle_system_path = Sdf.Path(
            f"{str(particle_utils.get_default_particle_system_path(stage))}{i}"
        )
        if not stage.GetPrimAtPath(particle_system_path):
            break
    # Determine the appropriate offsets based on the particle size
    particle_contact_offset = (
        cfg.particle_contact_offset
        if cfg.particle_contact_offset is not None
        else 0.8 * cfg.particle_size
    )
    # Create the particle system
    particle_system = particle_utils.add_physx_particle_system(
        stage=stage,
        particle_system_path=particle_system_path,
        particle_system_enabled=cfg.particle_system_enabled,
        contact_offset=(
            cfg.contact_offset
            if cfg.contact_offset is not None
            else particle_contact_offset
        ),
        rest_offset=(
            cfg.rest_offset
            if cfg.rest_offset is not None
            else 0.99 * particle_contact_offset
        ),
        particle_contact_offset=(
            cfg.particle_contact_offset
            if cfg.particle_contact_offset is not None
            else particle_contact_offset
        ),
        solid_rest_offset=(
            cfg.solid_rest_offset
            if cfg.solid_rest_offset is not None
            else 0.99 * 0.6 * particle_contact_offset
        ),
        fluid_rest_offset=(
            cfg.fluid_rest_offset
            if cfg.fluid_rest_offset is not None
            else 0.99 * 0.6 * particle_contact_offset
        ),
        enable_ccd=cfg.enable_ccd,
        solver_position_iterations=cfg.solver_position_iterations,
        max_depenetration_velocity=cfg.max_depenetration_velocity,
        wind=cfg.wind,
        max_neighborhood=cfg.max_neighborhood,
        neighborhood_scale=cfg.neighborhood_scale,
        max_velocity=cfg.max_velocity,
        global_self_collision_enabled=cfg.global_self_collision_enabled,
        non_particle_collision_enabled=cfg.non_particle_collision_enabled,
    )

    ## Material
    mtl_created_list = []
    omni.kit.commands.execute(
        "CreateAndBindMdlMaterialFromLibrary",
        mdl_name="OmniSurfacePresets.mdl",
        mtl_created_list=mtl_created_list,
    )
    mtl_path = mtl_created_list[0]
    omni.kit.commands.execute(
        "BindMaterial", prim_path=particle_system_path, material_path=mtl_path
    )
    particle_utils.add_pbd_particle_material(
        stage=stage,
        path=mtl_path,
        friction=cfg.friction,
        particle_friction_scale=cfg.particle_friction_scale,
        damping=cfg.damping,
        viscosity=cfg.viscosity,
        vorticity_confinement=cfg.vorticity_confinement,
        surface_tension=cfg.surface_tension,
        cohesion=cfg.cohesion,
        adhesion=cfg.adhesion,
        particle_adhesion_scale=cfg.particle_adhesion_scale,
        adhesion_offset_scale=cfg.adhesion_offset_scale,
        gravity_scale=cfg.gravity_scale,
        lift=cfg.lift,
        drag=cfg.drag,
        density=cfg.density,
        cfl_coefficient=cfg.cfl_coefficient,
    )
    physics_utils.add_physics_material_to_prim(
        stage, particle_system.GetPrim(), mtl_path
    )

    ## Visuals
    # Shadows
    if not cfg.cast_shadows:
        primvars_api = UsdGeom.PrimvarsAPI(particle_system)
        primvars_api.CreatePrimvar(  # type:ignore
            "doNotCastShadows", Sdf.ValueTypeNames.Bool
        ).Set(False)
    if cfg.fluid:
        # Particle anisotropy
        if cfg.particle_anisotropy_enabled:
            anisotropy_api = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(
                particle_system.GetPrim()
            )
            anisotropy_api.CreateParticleAnisotropyEnabledAttr().Set(True)
            anisotropy_api.CreateScaleAttr().Set(cfg.anisotropy_scale)
            anisotropy_api.CreateMinAttr().Set(cfg.anisotropy_min)
            anisotropy_api.CreateMaxAttr().Set(cfg.anisotropy_max)

        # Particle smoothing
        if cfg.particle_smoothing_enabled:
            smoothing_api = PhysxSchema.PhysxParticleSmoothingAPI.Apply(
                particle_system.GetPrim()
            )
            smoothing_api.CreateParticleSmoothingEnabledAttr().Set(True)
            smoothing_api.CreateStrengthAttr().Set(cfg.smoothing_strength)

        # Particle isosurface
        if cfg.isosurface_enabled:
            isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(
                particle_system.GetPrim()
            )
            isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
            isosurfaceAPI.CreateMaxVerticesAttr().Set(cfg.isosurface_max_vertices)
            isosurfaceAPI.CreateMaxTrianglesAttr().Set(cfg.isosurface_max_triangles)
            isosurfaceAPI.CreateMaxSubgridsAttr().Set(cfg.isosurface_max_subgrids)
            if cfg.isosurface_grid_spacing is not None:
                isosurfaceAPI.CreateGridSpacingAttr().Set(cfg.isosurface_grid_spacing)
            if cfg.isosurface_surface_distance is not None:
                isosurfaceAPI.CreateSurfaceDistanceAttr().Set(
                    cfg.isosurface_surface_distance
                )
            isosurfaceAPI.CreateNumMeshSmoothingPassesAttr().Set(
                cfg.isosurface_mesh_smoothing_passes
            )
            isosurfaceAPI.CreateNumMeshNormalSmoothingPassesAttr().Set(
                cfg.isosurface_mesh_normal_smoothing_passes
            )
            isosurfaceAPI.CreateGridFilteringPassesAttr().Set(
                cfg.isosurface_grid_filtering_passes
            )
            if cfg.isosurface_grid_smoothing_radius is not None:
                isosurfaceAPI.CreateGridSmoothingRadiusAttr().Set(
                    cfg.isosurface_grid_smoothing_radius
                )

    ## Particle set
    prim = particle_utils.add_physx_particleset_points(
        stage=stage,
        path=Sdf.Path(prim_path),
        positions_list=Vt.Vec3fArray(positions),
        velocities_list=Vt.Vec3fArray(velocities),
        widths_list=[cfg.particle_size] * len(positions),
        particle_system_path=particle_system_path,
        self_collision=cfg.self_collision,
        fluid=cfg.fluid,
        particle_group=cfg.particle_group,
        particle_mass=0.0,
        density=cfg.density,
    )

    ## Visibility
    if not cfg.visible:
        visibility_attribute = prim.GetVisibilityAttr()
        visibility_attribute.Set("invisible")

    return prim


def _create_particles_grid(
    lower: Gf.Vec3f,
    particle_spacing: float,
    dim_x: int,
    dim_y: int,
    dim_z: int,
) -> List[Gf.Vec3f]:
    x = lower[0]
    y = lower[1]
    z = lower[2]
    positions = [Gf.Vec3f(0.0)] * dim_x * dim_y * dim_z
    index = 0
    for _ in range(dim_x):
        for _ in range(dim_y):
            for _ in range(dim_z):
                positions[index] = Gf.Vec3f(x, y, z)
                index += 1
                z = z + particle_spacing
            z = lower[2]
            y = y + particle_spacing
        y = lower[1]
        x = x + particle_spacing
    return positions
