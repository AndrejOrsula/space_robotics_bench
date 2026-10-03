# Use isaaclab's prim utilities instead of isaacsim.core.utils.prims — the old
# isaacsim.core.utils.prims.create_prim internally uses isaacsim.core.prims.XFormPrim
# which relies on SimulationManager._get_backend_utils(), incompatible with
# isaaclab_physx's PhysxManager replacement.
import types as _types
from typing import TYPE_CHECKING

from isaaclab.sim import bind_physics_material, bind_visual_material, clone
from isaaclab.sim.spawners.materials import spawn_physics_material
from isaaclab.sim.spawners.shapes.shapes import spawn_geom_from_prim_type
from pxr import Usd

from srb.core.sim import schemas
from srb.core.sim.prim_utils import (
    create_prim,
    get_prim_at_path,
    is_prim_path_valid,
)

if TYPE_CHECKING:
    from . import cfg

# Provide a prim_utils-compatible namespace so existing call sites work unchanged.
prim_utils = _types.SimpleNamespace(
    create_prim=create_prim,
    get_prim_at_path=get_prim_at_path,
    is_prim_path_valid=is_prim_path_valid,
)
del _types


@clone
def spawn_plane(
    prim_path: str,
    cfg: "cfg.PlaneCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    scale = (cfg.size[0], cfg.size[1], 1.0)
    attributes = {}  # {"size": size}
    spawn_geom_from_prim_type(
        prim_path, cfg, "Plane", attributes, translation, orientation, scale
    )
    return get_prim_at_path(prim_path)


@clone
def spawn_arrow(
    prim_path: str,
    cfg: "cfg.ArrowCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    if prim_utils.is_prim_path_valid(prim_path):
        raise ValueError(f"A prim already exists at path: '{prim_path}'.")

    prim = prim_utils.create_prim(
        prim_path, "Xform", translation=translation, orientation=orientation
    )

    geom_prim_path = prim_path + "/geometry"
    mesh_prim_path = geom_prim_path + "/mesh"

    container = prim_utils.create_prim(mesh_prim_path, "Xform")

    # Component offsets are local to the root; ``position=`` would be world-space
    # and detach the geometry from translated or rotated roots.
    arrow_body_attributes = {
        "radius": cfg.tail_radius,
        "height": cfg.tail_length,
        "axis": "X",
    }
    arrow_body_translation = (cfg.tail_length / 2.0, 0, 0)
    arrow_body = prim_utils.create_prim(
        str(container.GetPath().AppendChild("arrow_body")),
        "Cylinder",
        translation=arrow_body_translation,
        attributes=arrow_body_attributes,
    )

    arrow_head_attributes = {
        "radius": cfg.head_radius,
        "height": cfg.head_length,
        "axis": "X",
    }
    arrow_head_translation = (cfg.tail_length + cfg.head_length / 2.0, 0, 0)
    arrow_head = prim_utils.create_prim(
        str(container.GetPath().AppendChild("arrow_head")),
        "Cone",
        translation=arrow_head_translation,
        attributes=arrow_head_attributes,
    )

    if cfg.collision_props is not None:
        schemas.define_collision_properties(
            str(arrow_body.GetPath()), cfg.collision_props
        )
        schemas.define_collision_properties(
            str(arrow_head.GetPath()), cfg.collision_props
        )

    if cfg.visual_material is not None:
        if not cfg.visual_material_path.startswith("/"):
            material_path = f"{geom_prim_path}/{cfg.visual_material_path}"
        else:
            material_path = cfg.visual_material_path
        cfg.visual_material.func(material_path, cfg.visual_material)
        bind_visual_material(mesh_prim_path, material_path)

    if cfg.physics_material is not None:
        if not cfg.physics_material_path.startswith("/"):
            material_path = f"{geom_prim_path}/{cfg.physics_material_path}"
        else:
            material_path = cfg.physics_material_path
        spawn_physics_material(material_path, cfg.physics_material)
        bind_physics_material(mesh_prim_path, material_path)

    return prim


@clone
def spawn_pinned_arrow(
    prim_path: str,
    cfg: "cfg.PinnedArrowCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    if prim_utils.is_prim_path_valid(prim_path):
        raise ValueError(f"A prim already exists at path: '{prim_path}'.")

    prim = prim_utils.create_prim(
        prim_path, "Xform", translation=translation, orientation=orientation
    )

    geom_prim_path = prim_path + "/geometry"
    mesh_prim_path = geom_prim_path + "/mesh"

    container = prim_utils.create_prim(mesh_prim_path, "Xform")

    pin_attributes = {"radius": cfg.pin_radius, "height": cfg.pin_length, "axis": "Z"}
    pin_translation = (0, 0, cfg.pin_length / 2.0 + cfg.tail_radius)
    pin = prim_utils.create_prim(
        str(container.GetPath().AppendChild("pin_body")),
        "Cylinder",
        translation=pin_translation,
        attributes=pin_attributes,
    )

    arrow_body_attributes = {
        "radius": cfg.tail_radius,
        "height": cfg.tail_length,
        "axis": "X",
    }
    arrow_body_translation = (cfg.tail_length / 2.0, 0, cfg.pin_length)
    arrow_body = prim_utils.create_prim(
        str(container.GetPath().AppendChild("arrow_body")),
        "Cylinder",
        translation=arrow_body_translation,
        attributes=arrow_body_attributes,
    )

    arrow_head_attributes = {
        "radius": cfg.head_radius,
        "height": cfg.head_length,
        "axis": "X",
    }
    arrow_head_translation = (
        cfg.tail_length + cfg.head_length / 2.0,
        0,
        cfg.pin_length,
    )
    arrow_head = prim_utils.create_prim(
        str(container.GetPath().AppendChild("arrow_head")),
        "Cone",
        translation=arrow_head_translation,
        attributes=arrow_head_attributes,
    )

    if cfg.collision_props is not None:
        schemas.define_collision_properties(
            str(arrow_body.GetPath()), cfg.collision_props
        )
        schemas.define_collision_properties(
            str(arrow_head.GetPath()), cfg.collision_props
        )
        schemas.define_collision_properties(str(pin.GetPath()), cfg.collision_props)

    if cfg.visual_material is not None:
        if not cfg.visual_material_path.startswith("/"):
            material_path = f"{geom_prim_path}/{cfg.visual_material_path}"
        else:
            material_path = cfg.visual_material_path
        cfg.visual_material.func(material_path, cfg.visual_material)
        bind_visual_material(mesh_prim_path, material_path)

    if cfg.physics_material is not None:
        if not cfg.physics_material_path.startswith("/"):
            material_path = f"{geom_prim_path}/{cfg.physics_material_path}"
        else:
            material_path = cfg.physics_material_path
        spawn_physics_material(material_path, cfg.physics_material)
        bind_physics_material(mesh_prim_path, material_path)

    return prim


@clone
def spawn_pinned_sphere(
    prim_path: str,
    cfg: "cfg.PinnedSphereCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
) -> Usd.Prim:
    if prim_utils.is_prim_path_valid(prim_path):
        raise ValueError(f"A prim already exists at path: '{prim_path}'.")

    prim = prim_utils.create_prim(
        prim_path, "Xform", translation=translation, orientation=orientation
    )

    geom_prim_path = prim_path + "/geometry"
    mesh_prim_path = geom_prim_path + "/mesh"

    container = prim_utils.create_prim(mesh_prim_path, "Xform")

    pin_attributes = {"radius": cfg.pin_radius, "height": cfg.pin_length, "axis": "Z"}
    pin_translation = (0, 0, cfg.pin_length / 2.0)
    pin = prim_utils.create_prim(
        str(container.GetPath().AppendChild("pin_body")),
        "Cylinder",
        translation=pin_translation,
        attributes=pin_attributes,
    )
    sphere_attributes = {"radius": cfg.sphere_radius}
    sphere_translation = (0, 0, cfg.pin_length)
    sphere = prim_utils.create_prim(
        str(container.GetPath().AppendChild("pin_head")),
        "Sphere",
        translation=sphere_translation,
        attributes=sphere_attributes,
    )

    if cfg.collision_props is not None:
        schemas.define_collision_properties(str(sphere.GetPath()), cfg.collision_props)
        schemas.define_collision_properties(str(pin.GetPath()), cfg.collision_props)

    if cfg.visual_material is not None:
        if not cfg.visual_material_path.startswith("/"):
            material_path = f"{geom_prim_path}/{cfg.visual_material_path}"
        else:
            material_path = cfg.visual_material_path
        cfg.visual_material.func(material_path, cfg.visual_material)
        bind_visual_material(mesh_prim_path, material_path)

    if cfg.physics_material is not None:
        if not cfg.physics_material_path.startswith("/"):
            material_path = f"{geom_prim_path}/{cfg.physics_material_path}"
        else:
            material_path = cfg.physics_material_path
        spawn_physics_material(material_path, cfg.physics_material)
        bind_physics_material(mesh_prim_path, material_path)

    return prim
