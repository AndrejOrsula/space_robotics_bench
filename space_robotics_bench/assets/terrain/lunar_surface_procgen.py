from os import path
from typing import Any, Dict, List, Optional

from omni.isaac.lab.utils import configclass

import space_robotics_bench.core.sim as sim_utils
from space_robotics_bench.paths import SRB_ASSETS_DIR_SRB_TERRAIN
from space_robotics_bench.utils.path import abs_listdir


@configclass
class LunarSurfaceProcgenCfg(sim_utils.BlenderNodesAssetCfg):
    name: str = "lunar_surface"
    autorun_scripts: List[str] = abs_listdir(
        path.join(SRB_ASSETS_DIR_SRB_TERRAIN, "lunar_surface_procgen")
    )

    # Geometry
    geometry_nodes: Dict[str, Dict[str, Any]] = {
        "LunarTerrain": {
            "density": 0.1,  # Density of the terrain in meters per vertex
            "scale": (10.0, 10.0, 1.0),  # Metric scale of the mesh
            "flat_area_size": 0.0,  # Size of a flat round area at the centre of the mesh in meters
            "rock_mesh_boolean": False,  # Flag to enable mesh boolean between the terrain and rocks
        }
    }
    decimate_face_count: Optional[int] = None
    decimate_angle_limit: Optional[float] = None

    # Material
    material: Optional[str] = "LunarSurface"
    texture_resolution: int = 4096
