"""Shared containment, visual ground, and framing for landscaping tasks."""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaaclab.envs import ViewerCfg

from srb.core.asset import AssetBaseCfg
from srb.core.sim import CuboidCfg, PreviewSurfaceCfg

if TYPE_CHECKING:
    from srb.assets.scenery.terrain import Plane

#: Dark compacted bed, contrasted with loose regolith.
BED_COLOR: tuple[float, float, float] = (0.26, 0.24, 0.21)
#: Lighter surrounding surface.
APRON_COLOR: tuple[float, float, float] = (0.52, 0.50, 0.47)
#: Piled boundary material, between the two.
BERM_COLOR: tuple[float, float, float] = (0.45, 0.42, 0.38)
#: Loose regolith, lighter and warmer than the bed.
REGOLITH_COLOR: tuple[float, float, float] = (0.46, 0.37, 0.25)

#: Fixed sun orientation (roll, pitch, yaw in degrees) for deterministic variants.
#:
#: Set this only when sunlight-orientation randomization is disabled.
SITE_SUNLIGHT_RPY: tuple[float, float, float] = (45.0, 14.0, 0.0)

#: Thickness of the visual bed slab (m).
BED_SLAB_THICKNESS_M: float = 0.04
#: Offset of the slab top below the collision plane (m).
BED_SLAB_DEPTH_M: float = 0.021
#: Apron half-width beyond the containment, as a multiple of the bed edge.
APRON_EXTENT_FACTOR: float = 6.0


def _dull_surface(color: tuple[float, float, float]) -> PreviewSurfaceCfg:
    """Create a rough, non-metallic surface."""

    return PreviewSurfaceCfg(diffuse_color=color, roughness=0.98, metallic=0.0)


def regolith_material() -> PreviewSurfaceCfg:
    """Create the shared loose-regolith material."""

    return PreviewSurfaceCfg(diffuse_color=REGOLITH_COLOR, roughness=0.95, metallic=0.0)


def plane_half_extent(bed_size_m: tuple[float, float]) -> tuple[float, float]:
    """Convert full bed edge lengths to ``PlaneCfg.size`` half-extents."""

    return (0.5 * float(bed_size_m[0]), 0.5 * float(bed_size_m[1]))


def landscaping_containment(
    bed_size_m: tuple[float, float] = (4.0, 4.0),
    wall_height_m: float = 0.3,
) -> "Plane":
    """Build the graded bed and its containment walls."""

    from srb.assets.scenery.terrain import Plane

    terrain = Plane()
    terrain.asset_cfg.spawn.size = plane_half_extent(bed_size_m)
    terrain.asset_cfg.spawn.visual_material = _dull_surface(BED_COLOR)
    terrain.wall_height = float(wall_height_m)
    terrain.obstacle_count = 0
    return terrain


def bed_visual_slab(
    bed_size_m: tuple[float, float], *, prim_name: str = "bed_visual"
) -> AssetBaseCfg:
    """Build a visual slab matching the full bed edge lengths."""

    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{prim_name}",
        spawn=CuboidCfg(
            size=(*(float(v) for v in bed_size_m), BED_SLAB_THICKNESS_M),
            visual_material=_dull_surface(BED_COLOR),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, -(BED_SLAB_DEPTH_M + 0.5 * BED_SLAB_THICKNESS_M))
        ),
    )


def containment_berm_ring(
    bed_size_m: tuple[float, float],
    wall_height_m: float,
    *,
    prim_prefix: str = "berm",
    width_m: float = 0.26,
) -> dict[str, AssetBaseCfg]:
    """Build four visual berms just outside the containment walls."""

    half_x = 0.5 * float(bed_size_m[0])
    half_y = 0.5 * float(bed_size_m[1])
    height = max(0.3 * float(wall_height_m), 0.05)
    offset = 0.5 * width_m
    # Long enough to close the corners.
    span_x = 2.0 * half_x + 2.0 * width_m
    span_y = 2.0 * half_y + 2.0 * width_m

    material = _dull_surface(BERM_COLOR)
    placements = {
        f"{prim_prefix}_x_pos": ((half_x + offset, 0.0), (width_m, span_y)),
        f"{prim_prefix}_x_neg": ((-(half_x + offset), 0.0), (width_m, span_y)),
        f"{prim_prefix}_y_pos": ((0.0, half_y + offset), (span_x, width_m)),
        f"{prim_prefix}_y_neg": ((0.0, -(half_y + offset)), (span_x, width_m)),
    }
    return {
        name: AssetBaseCfg(
            prim_path=f"{{ENV_REGEX_NS}}/{name}",
            spawn=CuboidCfg(
                size=(float(size_x), float(size_y), height),
                visual_material=material,
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=(float(pos_x), float(pos_y), 0.5 * height - BED_SLAB_DEPTH_M)
            ),
        )
        for name, ((pos_x, pos_y), (size_x, size_y)) in placements.items()
    }


def site_apron(
    bed_size_m: tuple[float, float],
    *,
    prim_name: str = "site_apron",
    max_extent_m: float | None = None,
) -> AssetBaseCfg:
    """Build a visual ground apron; cap its edge for multi-environment scenes."""

    extent = APRON_EXTENT_FACTOR * max(float(bed_size_m[0]), float(bed_size_m[1]))
    if max_extent_m is not None:
        extent = min(extent, max(float(max_extent_m), float(max(bed_size_m))))
    thickness = 0.2
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{prim_name}",
        spawn=CuboidCfg(
            size=(extent, extent, thickness),
            visual_material=_dull_surface(APRON_COLOR),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, -(BED_SLAB_DEPTH_M + BED_SLAB_THICKNESS_M + 0.5 * thickness))
        ),
    )


def site_viewer(bed_size_m: tuple[float, float]) -> ViewerCfg:
    """Frame the bed from a consistent three-quarter view."""

    reach = 0.8 * max(float(bed_size_m[0]), float(bed_size_m[1]))
    return ViewerCfg(
        eye=(-reach, -reach, 0.8 * reach),
        lookat=(0.0, 0.0, 0.10),
        origin_type="env",
    )


def apply_site_visuals(
    scene,
    *,
    bed_size_m: tuple[float, float],
    wall_height_m: float,
    bed_prim_name: str = "bed_visual",
    apron_max_extent_m: float | None = None,
) -> None:
    """Attach the visual slab, berms, and apron to *scene*."""

    setattr(scene, bed_prim_name, bed_visual_slab(bed_size_m, prim_name=bed_prim_name))
    scene.site_apron = site_apron(bed_size_m, max_extent_m=apron_max_extent_m)
    for name, cfg in containment_berm_ring(bed_size_m, wall_height_m).items():
        setattr(scene, name, cfg)
