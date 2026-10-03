from dataclasses import MISSING

from srb.core.asset import AerialManipulator, Articulation, AssetVariant
from srb.core.default_assets import get_default_asset_factories
from srb.core.env.manipulation.env import (
    ManipulationEnv,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.env.mobile.aerial.env import (
    AerialEnv,
    AerialEnvCfg,
    AerialEventCfg,
    AerialSceneCfg,
)
from srb.core.scene_introspection import get_scene_articulation
from srb.utils.cfg import configclass

from .._helpers import finalize_mobile_manipulation_cfg


@configclass
class AerialManipulationSceneCfg(AerialSceneCfg, ManipulationSceneCfg):
    pass


@configclass
class AerialManipulationEventCfg(AerialEventCfg, ManipulationEventCfg):
    pass


@configclass
class AerialManipulationEnvCfg(AerialEnvCfg):
    ## Assets
    robot: AerialManipulator | AssetVariant = (
        get_default_asset_factories().aerial_manipulation_robot()
    )
    _robot: AerialManipulator = MISSING  # type: ignore[assignment]
    ## Scene
    scene: AerialManipulationSceneCfg = AerialManipulationSceneCfg()

    ## Events
    events: AerialManipulationEventCfg = AerialManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    def __post_init__(self):
        finalize_mobile_manipulation_cfg(self, super().__post_init__)


class AerialManipulationEnv(AerialEnv, ManipulationEnv):
    cfg: AerialManipulationEnvCfg

    def __init__(self, cfg: AerialManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._manipulator: Articulation = get_scene_articulation(
            self.scene, "manipulator"
        )
