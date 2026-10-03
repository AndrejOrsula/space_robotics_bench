from typing import Literal

from isaaclab.terrains import TerrainGeneratorCfg
from isaaclab.terrains import TerrainImporterCfg as __TerrainImporterCfg

from srb.core.sim import RigidBodyMaterialCfg, VisualMaterialCfg
from srb.utils.cfg import configclass


@configclass
class TerrainImporterCfg(__TerrainImporterCfg):
    # Note: The types of this class are patched here to support its use within Pydantic, but the behaviour remains unchanged
    terrain_generator: TerrainGeneratorCfg | None = None
    terrain_type: Literal["generator", "plane", "usd"] = "generator"
    visual_material: VisualMaterialCfg | None = None
    physics_material: RigidBodyMaterialCfg = RigidBodyMaterialCfg()
