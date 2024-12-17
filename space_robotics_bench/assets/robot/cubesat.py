from os import path

from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.assets.rigid_object import RigidObjectCfg

import space_robotics_bench.core.assets as asset_utils
import space_robotics_bench.core.sim as sim_utils
from space_robotics_bench.core.actions import (
    SpacecraftActionCfg,
    SpacecraftActionGroupCfg,
)
from space_robotics_bench.core.sim.spawners.multi import MultiAssetCfg
from space_robotics_bench.paths import SRB_ASSETS_DIR


def cubesat_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    action_scale: float = 0.05,
    **kwargs,
) -> asset_utils.SpacecraftCfg:
    frame_base = "TODO"
    return asset_utils.SpacecraftCfg(
        ## Model
        asset_cfg=RigidObjectCfg(
            spawn=MultiAssetCfg(assets_cfg=[]),
            init_state=ArticulationCfg.InitialStateCfg(),
        ).replace(prim_path=prim_path, **kwargs),
        ## Actions
        action_cfg=SpacecraftActionGroupCfg(
            flight=SpacecraftActionCfg(
                asset_name=asset_name,
                scale=action_scale,
            )
        ),
        ## Frames
        frame_base=asset_utils.FrameCfg(
            prim_relpath=frame_base,
        ),
    )
