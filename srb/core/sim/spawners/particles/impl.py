import math
import re
from collections.abc import Sequence
from itertools import count
from typing import TYPE_CHECKING

import carb
import numpy as np
import omni.kit.commands
import omni.timeline
from isaaclab.sim import bind_visual_material
from isaaclab.sim import utils as sim_utils
from omni.physx.scripts import particleUtils as particle_utils
from omni.physx.scripts import physicsUtils as physics_utils
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from srb.core.particle_layout import (
    build_crater_pile_particle_layout,
    build_mound_pile_particle_layout,
)
from srb.core.particle_state_registry import register_authored_particle_states
from srb.core.sim.prim_utils import get_current_stage
from srb.utils.logging import logger

from pxr import PhysxSchema  # isort: skip

if TYPE_CHECKING:
    from .cfg import (
        CraterPileParticlesSpawnerCfg,
        GridParticlesSpawnerCfg,
        MoundPileParticlesSpawnerCfg,
        ParticlesSpawnerCfg,
        PyramidParticlesSpawnerCfg,
    )


# Kit persists this app-global setting in user.config.json. 0 forces CPU PhysX
# regardless of per-scene ``enableGPUDynamics``; PBD particles are GPU-only,
# so the override rejects them at parse time. -1 restores per-scene settings.
_PHYSX_OVERRIDE_GPU_SETTING = "/persistent/physics/overrideGPUSettings"
_PHYSX_OVERRIDE_FORCE_CPU = 0
_PHYSX_OVERRIDE_PER_SCENE = -1


def _clear_force_cpu_physics_override() -> None:
    """Clear a Force-CPU override before PhysX parses authored particle prims."""
    settings = carb.settings.get_settings()
    override = settings.get(_PHYSX_OVERRIDE_GPU_SETTING)
    if override == _PHYSX_OVERRIDE_FORCE_CPU:
        logger.warning(
            "Persisted Kit setting %s=0 forces CPU PhysX, which rejects GPU-only "
            "particles at parse time; restoring per-scene GPU settings (-1).",
            _PHYSX_OVERRIDE_GPU_SETTING,
        )
        settings.set(_PHYSX_OVERRIDE_GPU_SETTING, _PHYSX_OVERRIDE_PER_SCENE)


def _resolve_simulation_owner(stage: Usd.Stage, cfg: "ParticlesSpawnerCfg") -> Sdf.Path:
    """Require the configured scene to match SimulationCfg.physics_prim_path; no
    fallback.
    """
    owner_path = Sdf.Path(str(cfg.simulation_owner))
    owner_prim = stage.GetPrimAtPath(owner_path)
    if not owner_prim or not owner_prim.IsValid():
        raise RuntimeError(
            f"Particle simulation owner prim '{owner_path}' does not exist. "
            "Set ParticlesSpawnerCfg.simulation_owner to the environment's "
            "SimulationCfg.physics_prim_path (the UsdPhysics.Scene prim)."
        )
    if not owner_prim.IsA(UsdPhysics.Scene):
        raise RuntimeError(
            f"Particle simulation owner prim '{owner_path}' is a "
            f"'{owner_prim.GetTypeName()}', not a UsdPhysics.Scene. Set "
            "ParticlesSpawnerCfg.simulation_owner to the environment's "
            "SimulationCfg.physics_prim_path."
        )
    return owner_path


def _enable_gpu_particle_dynamics(stage: Usd.Stage, owner_path: Sdf.Path) -> None:
    """Enable GPU particle solving and CPU-facing USD state readback.

    Managed state uses ``UsdGeom.Points`` and
    ``/physics/updateParticlesToUsd``. Isaac Sim 6 suppresses this readback
    with the CUDA direct-data pipeline, so particle tasks use a CPU simulation
    device with GPU scene dynamics. Particles still run on the GPU solver.
    """
    physics_scene_prim = stage.GetPrimAtPath(owner_path)
    physics_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
    physics_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
    physics_scene_api.CreateBroadphaseTypeAttr().Set("GPU")


def _sample_velocity_component(value: float | Sequence[float]) -> float:
    if isinstance(value, Sequence):
        return np.random.uniform(value[0], value[1])
    return value


def spawn_particles_grid(
    prim_path: str,
    cfg: "GridParticlesSpawnerCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    # Ratio
    if cfg.ratio < 1.0:
        ratio_cbrt = math.pow(cfg.ratio, 1.0 / 3.0)
        dim_x, dim_y, dim_z = (
            max(1, math.floor(ratio_cbrt * cfg.dim_x)) if cfg.dim_x else 0,
            max(1, math.floor(ratio_cbrt * cfg.dim_y)) if cfg.dim_y else 0,
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
        ),
        particle_spacing=cfg.particle_size + particle_spacing,
        dim_x=dim_x,
        dim_y=dim_y,
        dim_z=dim_z,
    )

    # Randomize velocities
    velocities = [
        Gf.Vec3f(*(_sample_velocity_component(component) for component in cfg.velocity))
        for _ in range(len(positions))
    ]

    # Transform particles
    positions, velocities = _transform_particles(
        positions, velocities, translation, orientation
    )

    return _spawn_particles(prim_path, cfg, positions, velocities)


def spawn_particles_pyramid(
    prim_path: str,
    cfg: "PyramidParticlesSpawnerCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    # Ratio
    if cfg.ratio < 1.0:
        ratio_cbrt = math.pow(cfg.ratio, 1.0 / 3.0)
        dim_x, dim_y, dim_z = (
            max(1, math.floor(ratio_cbrt * cfg.dim_x)) if cfg.dim_x else 0,
            max(1, math.floor(ratio_cbrt * cfg.dim_y)) if cfg.dim_y else 0,
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
        Gf.Vec3f(*(_sample_velocity_component(component) for component in cfg.velocity))
        for _ in range(len(positions))
    ]

    # Transform particles
    positions, velocities = _transform_particles(
        positions, velocities, translation, orientation
    )

    return _spawn_particles(prim_path, cfg, positions, velocities)


def spawn_particles_crater_pile(
    prim_path: str,
    cfg: "CraterPileParticlesSpawnerCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    """Spawn a layered bed containing one crater and its excavated-material pile."""
    layout = build_crater_pile_particle_layout(
        dim_x=cfg.dim_x,
        dim_y=cfg.dim_y,
        dim_z=cfg.dim_z,
        particle_size=cfg.particle_size,
        particle_spacing=cfg.particle_spacing,
        ratio=cfg.ratio,
        crater_center_xy_m=cfg.crater_center_xy_m,
        crater_radius_m=cfg.crater_radius_m,
        crater_depth_m=cfg.crater_depth_m,
        pile_center_xy_m=cfg.pile_center_xy_m,
        pile_radius_m=cfg.pile_radius_m,
        pile_height_m=cfg.pile_height_m,
    )
    positions = [
        Gf.Vec3f(float(position[0]), float(position[1]), float(position[2]))
        for position in layout.positions
    ]
    velocities = [
        Gf.Vec3f(*(_sample_velocity_component(component) for component in cfg.velocity))
        for _ in range(len(positions))
    ]
    positions, velocities = _transform_particles(
        positions,
        velocities,
        translation,
        orientation,
    )
    return _spawn_particles(prim_path, cfg, positions, velocities)


def spawn_particles_mound_pile(
    prim_path: str,
    cfg: "MoundPileParticlesSpawnerCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    """Spawn a flat layered bed with one full-cone regolith mound on top."""
    layout = build_mound_pile_particle_layout(
        dim_x=cfg.dim_x,
        dim_y=cfg.dim_y,
        dim_z=cfg.dim_z,
        particle_size=cfg.particle_size,
        particle_spacing=cfg.particle_spacing,
        ratio=cfg.ratio,
        pile_center_xy_m=cfg.pile_center_xy_m,
        pile_radius_m=cfg.pile_radius_m,
        pile_height_m=cfg.pile_height_m,
    )
    positions = [
        Gf.Vec3f(float(position[0]), float(position[1]), float(position[2]))
        for position in layout.positions
    ]
    velocities = [
        Gf.Vec3f(*(_sample_velocity_component(component) for component in cfg.velocity))
        for _ in range(len(positions))
    ]
    positions, velocities = _transform_particles(
        positions,
        velocities,
        translation,
        orientation,
    )
    return _spawn_particles(prim_path, cfg, positions, velocities)


def _particle_system_path_for_set(stage: Usd.Stage, set_path: str) -> Sdf.Path:
    """Return a unique particle-system sibling inside the set's env namespace."""
    set_sdf_path = Sdf.Path(set_path)
    parent = set_sdf_path.GetParentPath()
    stem = f"{set_sdf_path.name}System"
    for index in count(1):
        suffix = "" if index == 1 else str(index)
        candidate = parent.AppendChild(f"{stem}{suffix}")
        if not stage.GetPrimAtPath(candidate):
            return candidate
    raise RuntimeError("unreachable particle-system path search")


def _add_particle_system(
    stage: Usd.Stage,
    cfg: "ParticlesSpawnerCfg",
    system_path: Sdf.Path,
    simulation_owner: Sdf.Path,
) -> PhysxSchema.PhysxParticleSystem:
    """Create one env-local solver and validate its shared scene owner."""
    iterations = cfg.solver_position_iterations
    if type(iterations) is not int or iterations <= 0:
        raise ValueError(
            "solver_position_iterations must be a positive int, "
            f"got {iterations!r} ({type(iterations).__name__})"
        )
    particle_contact_offset = (
        cfg.particle_contact_offset
        if cfg.particle_contact_offset is not None
        else 0.8 * cfg.particle_size
    )
    system = particle_utils.add_physx_particle_system(
        stage=stage,
        particle_system_path=system_path,
        particle_system_enabled=cfg.particle_system_enabled,
        simulation_owner=simulation_owner,
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
        particle_contact_offset=particle_contact_offset,
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
        solver_position_iterations=iterations,
        max_depenetration_velocity=cfg.max_depenetration_velocity,
        wind=cfg.wind,
        max_neighborhood=cfg.max_neighborhood,
        neighborhood_scale=cfg.neighborhood_scale,
        max_velocity=cfg.max_velocity,
        global_self_collision_enabled=cfg.global_self_collision_enabled,
        non_particle_collision_enabled=cfg.non_particle_collision_enabled,
    )
    authored_owners = list(system.GetSimulationOwnerRel().GetTargets())
    if authored_owners != [simulation_owner]:
        raise RuntimeError(
            f"Particle system '{system_path}' authored simulation owner(s) "
            f"{authored_owners} != configured [{simulation_owner}]."
        )
    return system


def _create_particle_material(
    stage: Usd.Stage, cfg: "ParticlesSpawnerCfg"
) -> tuple[Sdf.Path, bool]:
    """Create one immutable PBD material shared by env-local systems."""
    created: list[str] = []
    try:
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniSurfacePresets.mdl",
            mtl_name="OmniSurface_Default",
            mtl_created_list=created,
            select_new_prim=False,
        )
    except Exception as e:  # noqa: BLE001
        logger.warning(
            "OmniSurfacePresets.mdl material creation failed (%s: %s); "
            "falling back to blank Material prim for PBD physics binding.",
            type(e).__name__,
            e,
        )
        created = []

    if created:
        material_path = Sdf.Path(created[0])
        bind_render_material = True
    else:
        material_root = Sdf.Path("/World/srbParticleMaterials")
        UsdGeom.Scope.Define(stage, material_root)
        for index in count(1):
            material_path = material_root.AppendChild(f"particleMaterial{index}")
            if not stage.GetPrimAtPath(material_path):
                break
        UsdShade.Material.Define(stage, material_path)
        bind_render_material = False

    particle_utils.add_pbd_particle_material(
        stage=stage,
        path=material_path,
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
        density=cfg.density,
        cfl_coefficient=cfg.cfl_coefficient,
    )
    return material_path, bind_render_material


def _configure_particle_system(
    stage: Usd.Stage,
    system: PhysxSchema.PhysxParticleSystem,
    cfg: "ParticlesSpawnerCfg",
    material_path: Sdf.Path,
    *,
    bind_render_material: bool,
) -> None:
    """Apply shared material and per-system render/fluid schemas."""
    system_prim = system.GetPrim()
    if bind_render_material:
        omni.kit.commands.execute(
            "BindMaterial",
            prim_path=system_prim.GetPath(),
            material_path=material_path,
        )
    physics_utils.add_physics_material_to_prim(stage, system_prim, material_path)

    if not cfg.cast_shadows:
        primvars_api = UsdGeom.PrimvarsAPI(system)
        primvars_api.CreatePrimvar(  # type: ignore[misc]
            "doNotCastShadows", Sdf.ValueTypeNames.Bool
        ).Set(True)
    if not cfg.fluid:
        return
    if cfg.particle_anisotropy_enabled:
        anisotropy_api = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(system_prim)
        anisotropy_api.CreateParticleAnisotropyEnabledAttr().Set(True)
        anisotropy_api.CreateScaleAttr().Set(cfg.anisotropy_scale)
        anisotropy_api.CreateMinAttr().Set(cfg.anisotropy_min)
        anisotropy_api.CreateMaxAttr().Set(cfg.anisotropy_max)
    if cfg.particle_smoothing_enabled:
        smoothing_api = PhysxSchema.PhysxParticleSmoothingAPI.Apply(system_prim)
        smoothing_api.CreateParticleSmoothingEnabledAttr().Set(True)
        smoothing_api.CreateStrengthAttr().Set(cfg.smoothing_strength)
    if cfg.isosurface_enabled:
        isosurface_api = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(system_prim)
        isosurface_api.CreateIsosurfaceEnabledAttr().Set(True)
        isosurface_api.CreateMaxVerticesAttr().Set(cfg.isosurface_max_vertices)
        isosurface_api.CreateMaxTrianglesAttr().Set(cfg.isosurface_max_triangles)
        isosurface_api.CreateMaxSubgridsAttr().Set(cfg.isosurface_max_subgrids)
        if cfg.isosurface_grid_spacing is not None:
            isosurface_api.CreateGridSpacingAttr().Set(cfg.isosurface_grid_spacing)
        if cfg.isosurface_surface_distance is not None:
            isosurface_api.CreateSurfaceDistanceAttr().Set(
                cfg.isosurface_surface_distance
            )
        isosurface_api.CreateNumMeshSmoothingPassesAttr().Set(
            cfg.isosurface_mesh_smoothing_passes
        )
        isosurface_api.CreateNumMeshNormalSmoothingPassesAttr().Set(
            cfg.isosurface_mesh_normal_smoothing_passes
        )
        isosurface_api.CreateGridFilteringPassesAttr().Set(
            cfg.isosurface_grid_filtering_passes
        )
        if cfg.isosurface_grid_smoothing_radius is not None:
            isosurface_api.CreateGridSmoothingRadiusAttr().Set(
                cfg.isosurface_grid_smoothing_radius
            )


def _particle_set_paths(stage: Usd.Stage, prim_path: str) -> list[str]:
    """Resolve source plus sibling environment particle-set paths."""
    paths = [prim_path]
    env_dir, leaf_name = prim_path.rsplit("/", 1)
    if "/" not in env_dir:
        return paths
    envs_root, env_name = env_dir.rsplit("/", 1)
    if not envs_root or not re.fullmatch(r"env_\d+", env_name):
        return paths
    envs_parent = stage.GetPrimAtPath(envs_root)
    if not envs_parent or not envs_parent.IsValid():
        return paths
    for env_prim in envs_parent.GetChildren():
        sibling = env_prim.GetName()
        if sibling != env_name and re.fullmatch(r"env_\d+", sibling):
            paths.append(f"{envs_root}/{sibling}/{leaf_name}")
    return paths


def _resolve_particle_source_path(stage: Usd.Stage, prim_path: str) -> str:
    """Resolve an IsaacLab env-regex path without its CopySpec clone decorator."""
    parent_path, leaf_name = prim_path.rsplit("/", 1)
    parent_prim = stage.GetPrimAtPath(parent_path)
    if parent_prim and parent_prim.IsValid():
        return prim_path
    if "/" not in parent_path:
        raise RuntimeError(f"Particle parent prim does not exist: '{parent_path}'")

    parent_root, parent_expr = parent_path.rsplit("/", 1)
    root_prim = stage.GetPrimAtPath(parent_root)
    if not root_prim or not root_prim.IsValid():
        raise RuntimeError(f"Particle parent root prim does not exist: '{parent_root}'")
    matches = sorted(
        child.GetName()
        for child in root_prim.GetChildren()
        if re.fullmatch(parent_expr, child.GetName())
    )
    if not matches:
        raise RuntimeError(
            f"Particle parent expression '{parent_path}' matched no stage prims"
        )
    return f"{parent_root}/{matches[0]}/{leaf_name.replace('.*', '0')}"


def _spawn_particles(
    prim_path: str,
    cfg: "ParticlesSpawnerCfg",
    positions: Sequence[Gf.Vec3f],
    velocities: Sequence[Gf.Vec3f],
) -> Usd.Prim:
    """Spawn one env-local system/set pair per environment."""
    stage = get_current_stage()
    prim_path = _resolve_particle_source_path(stage, prim_path)
    _clear_force_cpu_physics_override()
    simulation_owner = _resolve_simulation_owner(stage, cfg)
    _enable_gpu_particle_dynamics(stage, simulation_owner)
    local_positions = list(positions)
    local_velocities = list(velocities)
    set_paths = _particle_set_paths(stage, prim_path)

    systems = [
        _add_particle_system(
            stage,
            cfg,
            _particle_system_path_for_set(stage, set_path),
            simulation_owner,
        )
        for set_path in set_paths
    ]
    material_path, bind_render_material = _create_particle_material(stage, cfg)
    for system in systems:
        _configure_particle_system(
            stage,
            system,
            cfg,
            material_path,
            bind_render_material=bind_render_material,
        )

    particle_sets: list[UsdGeom.Points] = []
    for set_path, system in zip(set_paths, systems, strict=True):
        particle_set = particle_utils.add_physx_particleset_points(
            stage=stage,
            path=Sdf.Path(set_path),
            positions_list=Vt.Vec3fArray(local_positions),
            velocities_list=Vt.Vec3fArray(local_velocities),
            widths_list=[cfg.particle_size] * len(local_positions),
            particle_system_path=system.GetPrim().GetPath(),
            self_collision=cfg.self_collision,
            fluid=cfg.fluid,
            particle_group=cfg.particle_group,
            particle_mass=0.0,
            density=cfg.density,
        )
        if not cfg.visible:
            particle_set.GetVisibilityAttr().Set("invisible")
        if cfg.visual_material is not None:
            import dataclasses

            visual_material_path = f"{set_path}/visualMaterial"
            if dataclasses.is_dataclass(cfg.visual_material):
                for field in dataclasses.fields(cfg.visual_material):
                    value = getattr(cfg.visual_material, field.name)
                    if isinstance(value, list) or type(value).__name__ == "ListConfig":
                        setattr(cfg.visual_material, field.name, tuple(value))
            cfg.visual_material.func(visual_material_path, cfg.visual_material)
            bind_visual_material(set_path, visual_material_path)
        sim_utils.standardize_xform_ops(particle_set.GetPrim())
        particle_sets.append(particle_set)

    # DirectRLEnv starts physics before managed wrappers exist. Preserve the
    # exact parent-local arrays now; first reset projects and rewinds each row.
    register_authored_particle_states(set_paths, local_positions, local_velocities)
    return particle_sets[0]


def _create_particles_grid(
    lower: Gf.Vec3f,
    particle_spacing: float,
    dim_x: int,
    dim_y: int,
    dim_z: int,
) -> list[Gf.Vec3f]:
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


def _transform_particles(
    positions: list[Gf.Vec3f],
    velocities: list[Gf.Vec3f],
    translation: tuple[float, float, float] | None,
    orientation: tuple[float, float, float, float] | None,
) -> tuple[list[Gf.Vec3f], list[Gf.Vec3f]]:
    if orientation is None and translation is None:
        return positions, velocities

    trans_vec = (
        Gf.Vec3f(translation[0], translation[1], translation[2])
        if translation is not None
        else Gf.Vec3f(0.0, 0.0, 0.0)
    )
    if orientation is not None:
        # Convert XYZW (Isaac Lab convention) to WXYZ (Gf.Quatf convention)
        rotation = Gf.Rotation(
            Gf.Quatf(orientation[3], orientation[0], orientation[1], orientation[2])
        )
        return [
            Gf.Vec3f(rotation.TransformDir(Gf.Vec3d(pos))) + trans_vec
            for pos in positions
        ], [Gf.Vec3f(rotation.TransformDir(Gf.Vec3d(vel))) for vel in velocities]
    return [pos + trans_vec for pos in positions], velocities
