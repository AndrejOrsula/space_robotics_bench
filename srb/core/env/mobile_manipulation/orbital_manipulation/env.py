from dataclasses import MISSING

from srb.core.asset import Articulation, AssetVariant, OrbitalManipulator
from srb.core.default_assets import get_default_asset_factories
from srb.core.env.manipulation.env import (
    ManipulationEnv,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.env.mobile.orbital.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
)
from srb.core.scene_introspection import get_scene_articulation
from srb.utils.cfg import configclass

from .._helpers import finalize_mobile_manipulation_cfg


@configclass
class OrbitalManipulationSceneCfg(OrbitalSceneCfg, ManipulationSceneCfg):
    pass


@configclass
class OrbitalManipulationEventCfg(OrbitalEventCfg, ManipulationEventCfg):
    pass


@configclass
class OrbitalManipulationEnvCfg(OrbitalEnvCfg):
    ## Assets
    robot: OrbitalManipulator | AssetVariant = (
        get_default_asset_factories().orbital_manipulation_robot()
    )
    _robot: OrbitalManipulator = MISSING  # type: ignore[assignment]
    ## Scene
    scene: OrbitalManipulationSceneCfg = OrbitalManipulationSceneCfg()

    ## Events
    events: OrbitalManipulationEventCfg = OrbitalManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    def __post_init__(self):
        finalize_mobile_manipulation_cfg(self, super().__post_init__)


class OrbitalManipulationEnv(OrbitalEnv, ManipulationEnv):
    cfg: OrbitalManipulationEnvCfg

    def __init__(self, cfg: OrbitalManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._manipulator: Articulation = get_scene_articulation(
            self.scene, "manipulator"
        )
