"""Isaac-bound procedural TerrainGenerator preset construction."""

from __future__ import annotations

from srb.core.sim import (
    HfDiscreteObstaclesTerrainCfg,
    HfInvertedPyramidSlopedTerrainCfg,
    HfInvertedPyramidStairsTerrainCfg,
    HfPyramidSlopedTerrainCfg,
    HfPyramidStairsTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfWaveTerrainCfg,
    MeshBoxTerrainCfg,
    MeshFloatingRingTerrainCfg,
    MeshGapTerrainCfg,
    MeshHillTerrainCfg,
    MeshInvertedPyramidStairsTerrainCfg,
    MeshPitTerrainCfg,
    MeshPyramidStairsTerrainCfg,
    MeshRailsTerrainCfg,
    MeshRandomGridTerrainCfg,
    MeshRepeatedBoxesTerrainCfg,
    MeshRepeatedCylindersTerrainCfg,
    MeshRepeatedPyramidsTerrainCfg,
    MeshTiltedTerrainCfg,
    MeshWaveTerrainCfg,
    PreviewSurfaceCfg,
    TerrainGeneratorCfg,
    TerrainImporterCfg,
)

DEFAULT_MESH_SUB_TERRAIN_NAMES = (
    "mesh_pyramid_stairs",
    "mesh_pyramid_stairs_inv",
    "mesh_random_grid",
    "mesh_rails",
    "mesh_pit",
    "mesh_box_steps",
    "mesh_gap",
    "mesh_floating_ring",
    "mesh_repeated_pyramids",
    "mesh_repeated_boxes",
    "mesh_repeated_cylinders",
    "mesh_wave",
    "mesh_hill",
    "mesh_tilted",
)
DEFAULT_HEIGHT_FIELD_SUB_TERRAIN_NAMES = (
    "hf_random_uniform",
    "hf_pyramid_slope",
    "hf_pyramid_slope_inv",
    "hf_pyramid_stairs",
    "hf_pyramid_stairs_inv",
    "hf_discrete_obstacles",
    "hf_waves",
)


def balance_sub_terrain_proportions(
    asset_cfg: TerrainImporterCfg,
) -> TerrainImporterCfg:
    sub_terrains = asset_cfg.terrain_generator.sub_terrains
    proportion = 1.0 / len(sub_terrains)
    for sub_terrain in sub_terrains.values():
        sub_terrain.proportion = proportion
    return asset_cfg


def build_mesh_terrain_generator_sub_terrains() -> dict[str, object]:
    return {
        "mesh_pyramid_stairs": MeshPyramidStairsTerrainCfg(
            step_height_range=(0.05, 0.2), step_width=0.3
        ),
        "mesh_pyramid_stairs_inv": MeshInvertedPyramidStairsTerrainCfg(
            step_height_range=(0.05, 0.2), step_width=0.3
        ),
        "mesh_random_grid": MeshRandomGridTerrainCfg(
            grid_width=0.45, grid_height_range=(0.05, 0.2)
        ),
        "mesh_rails": MeshRailsTerrainCfg(
            rail_thickness_range=(0.1, 0.2),
            rail_height_range=(0.1, 0.4),
        ),
        "mesh_pit": MeshPitTerrainCfg(pit_depth_range=(0.1, 0.3)),
        "mesh_box_steps": MeshBoxTerrainCfg(box_height_range=(0.1, 0.3)),
        "mesh_gap": MeshGapTerrainCfg(gap_width_range=(0.1, 0.4)),
        "mesh_floating_ring": MeshFloatingRingTerrainCfg(
            ring_width_range=(0.2, 0.5),
            ring_height_range=(0.1, 0.3),
            ring_thickness=0.1,
        ),
        "mesh_repeated_pyramids": MeshRepeatedPyramidsTerrainCfg(
            object_type="cone",
            object_params_start=MeshRepeatedPyramidsTerrainCfg.ObjectCfg(
                num_objects=50, height=0.1, radius=0.2
            ),
            object_params_end=MeshRepeatedPyramidsTerrainCfg.ObjectCfg(
                num_objects=100, height=0.3, radius=0.4
            ),
        ),
        "mesh_repeated_boxes": MeshRepeatedBoxesTerrainCfg(
            object_type="box",
            object_params_start=MeshRepeatedBoxesTerrainCfg.ObjectCfg(
                num_objects=50, height=0.1, size=(0.2, 0.2)
            ),
            object_params_end=MeshRepeatedBoxesTerrainCfg.ObjectCfg(
                num_objects=100, height=0.3, size=(0.4, 0.4)
            ),
        ),
        "mesh_repeated_cylinders": MeshRepeatedCylindersTerrainCfg(
            object_type="cylinder",
            object_params_start=MeshRepeatedCylindersTerrainCfg.ObjectCfg(
                num_objects=50, height=0.1, radius=0.2
            ),
            object_params_end=MeshRepeatedCylindersTerrainCfg.ObjectCfg(
                num_objects=100, height=0.3, radius=0.4
            ),
        ),
        "mesh_wave": MeshWaveTerrainCfg(),
        "mesh_hill": MeshHillTerrainCfg(),
        "mesh_tilted": MeshTiltedTerrainCfg(),
    }


def build_height_field_terrain_generator_sub_terrains() -> dict[str, object]:
    return {
        "hf_random_uniform": HfRandomUniformTerrainCfg(
            noise_range=(0.0, 0.1),
            noise_step=0.02,
            border_width=0.2,
        ),
        "hf_pyramid_slope": HfPyramidSlopedTerrainCfg(slope_range=(0.2, 0.5)),
        "hf_pyramid_slope_inv": HfInvertedPyramidSlopedTerrainCfg(
            slope_range=(0.2, 0.5)
        ),
        "hf_pyramid_stairs": HfPyramidStairsTerrainCfg(
            step_height_range=(0.05, 0.2), step_width=0.3
        ),
        "hf_pyramid_stairs_inv": HfInvertedPyramidStairsTerrainCfg(
            step_height_range=(0.05, 0.2), step_width=0.3
        ),
        "hf_discrete_obstacles": HfDiscreteObstaclesTerrainCfg(
            obstacle_width_range=(0.3, 0.8),
            obstacle_height_range=(0.1, 0.4),
            num_obstacles=75,
        ),
        "hf_waves": HfWaveTerrainCfg(amplitude_range=(0.1, 0.3), num_waves=5),
    }


def build_default_terrain_generator_asset_cfg() -> TerrainImporterCfg:
    return balance_sub_terrain_proportions(
        TerrainImporterCfg(
            prim_path="/World/terrain",
            terrain_type="generator",
            visual_material=PreviewSurfaceCfg(diffuse_color=(0.38, 0.35, 0.31)),
            terrain_generator=TerrainGeneratorCfg(
                size=(7.5, 7.5),
                border_width=10.0,
                color_scheme="none",
                sub_terrains={
                    **build_mesh_terrain_generator_sub_terrains(),
                    **build_height_field_terrain_generator_sub_terrains(),
                },
            ),
        )
    )


__all__ = [
    "DEFAULT_HEIGHT_FIELD_SUB_TERRAIN_NAMES",
    "DEFAULT_MESH_SUB_TERRAIN_NAMES",
    "balance_sub_terrain_proportions",
    "build_default_terrain_generator_asset_cfg",
    "build_height_field_terrain_generator_sub_terrains",
    "build_mesh_terrain_generator_sub_terrains",
]
