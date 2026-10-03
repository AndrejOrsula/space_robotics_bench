from dataclasses import MISSING

from srb.core.asset import (
    ArticulationCfg,
    AssetBaseCfg,
    RigidObjectCfg,
    TerrainImporterCfg,
)
from srb.core.env import InteractiveSceneCfg
from srb.utils.cfg import configclass


@configclass
class BaseSceneCfg(InteractiveSceneCfg):
    num_envs: int = 1
    replicate_physics: bool = False

    ## Illumination
    sunlight: AssetBaseCfg | None = None
    skydome: AssetBaseCfg | None = None

    ## Scenery
    scenery: AssetBaseCfg | TerrainImporterCfg | None = None

    ## Robot
    robot: RigidObjectCfg | ArticulationCfg = MISSING  # type: ignore[assignment]
    payload: AssetBaseCfg | None = None
    manipulator: ArticulationCfg | None = None
    end_effector: AssetBaseCfg | None = None
