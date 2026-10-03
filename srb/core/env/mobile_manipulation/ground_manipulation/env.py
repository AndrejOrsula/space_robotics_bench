from dataclasses import MISSING

from srb.core.asset import Articulation, AssetVariant, GroundManipulator
from srb.core.default_assets import get_default_asset_factories
from srb.core.env.manipulation.env import (
    ManipulationEnv,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.env.mobile.ground.env import (
    GroundEnv,
    GroundEnvCfg,
    GroundEventCfg,
    GroundSceneCfg,
)
from srb.core.scene_introspection import get_scene_articulation
from srb.core.sensor import CameraCfg, PinholeCameraCfg, SensorBaseCfg
from srb.utils.cfg import configclass
from srb.utils.math import rpy_to_quat

from .._helpers import finalize_mobile_manipulation_cfg


@configclass
class GroundManipulationSceneCfg(GroundSceneCfg, ManipulationSceneCfg):
    pass


@configclass
class GroundManipulationEventCfg(GroundEventCfg, ManipulationEventCfg):
    pass


@configclass
class GroundManipulationEnvCfg(GroundEnvCfg):
    ## Assets
    robot: GroundManipulator | AssetVariant = (
        get_default_asset_factories().ground_manipulation_robot()
    )
    _robot: GroundManipulator = MISSING  # type: ignore[assignment]
    ## Scene
    scene: GroundManipulationSceneCfg = GroundManipulationSceneCfg()

    ## Sensors
    # `width`/`height` are set explicitly (matching the base `camera_resolution`
    # default) rather than relying on `_add_sensors`'s active-category fill-in:
    # that fill-in is gated on `SensorCategory.VISUAL` being in `active_sensors`
    # (not in the base default set), but `env_sensor_cfgs` entries are
    # structurally validated by `cfg.validate()` regardless of whether their
    # category ends up active -- an inactive VISUAL sensor with MISSING
    # width/height fails `TaskCfg` construction outright for any task that
    # doesn't itself opt into VISUAL sensors (e.g. digital_twin/lunalab).
    env_sensor_cfgs: dict[str, SensorBaseCfg] = {
        "cam_scene": CameraCfg(
            prim_path="{ENV_REGEX_NS}/robot/camera_scene",
            offset=CameraCfg.OffsetCfg(
                convention="world",
                pos=(2.0, 0.0, 1.0),
                rot=rpy_to_quat(0.0, 45.0, 180.0),
            ),
            spawn=PinholeCameraCfg(clipping_range=(0.01, 4.99)),
            width=64,
            height=64,
        ),
    }

    ## Events
    events: GroundManipulationEventCfg = GroundManipulationEventCfg()

    ## Time
    env_rate: float = 1.0 / 150.0
    agent_rate: float = 1.0 / 50.0

    def __post_init__(self):
        finalize_mobile_manipulation_cfg(self, super().__post_init__)


class GroundManipulationEnv(GroundEnv, ManipulationEnv):
    cfg: GroundManipulationEnvCfg

    def __init__(self, cfg: GroundManipulationEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._manipulator: Articulation = get_scene_articulation(
            self.scene, "manipulator"
        )
