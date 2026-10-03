from dataclasses import MISSING

import torch

from srb.core.asset import AerialRobot, AssetVariant, Terrain
from srb.core.default_assets import get_default_asset_factories
from srb.core.domain import Domain
from srb.core.env import ViewerCfg
from srb.core.env.mobile.env import (
    MobileEnv,
    MobileEnvCfg,
    MobileEventCfg,
    MobileSceneCfg,
)
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.utils.cfg import configclass


@configclass
class AerialSceneCfg(MobileSceneCfg):
    env_spacing: float = 64.0


@configclass
class AerialEventCfg(MobileEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-2.5, 2.5),
                "y": (-2.5, 2.5),
                "z": (0.5, 5.0),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {},
        },
    )


@configclass
class AerialEnvCfg(MobileEnvCfg):
    ## Scenario
    domain: Domain = Domain.MARS

    ## Assets
    robot: AerialRobot | AssetVariant = get_default_asset_factories().aerial_robot()
    _robot: AerialRobot = MISSING  # type: ignore[assignment]
    scenery: Terrain | AssetVariant | None = AssetVariant.PROCEDURAL
    ## Scene
    scene: AerialSceneCfg = AerialSceneCfg()

    ## Events
    events: AerialEventCfg = AerialEventCfg()

    ## Time
    env_rate: float = 1.0 / 25.0
    agent_rate: float = 1.0 / 25.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(-10.0, 0.0, 20.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()
        # The default active_sensors set drops TERRAIN-category sensors, which
        # would silently discard the Ingenuity altimeter (HeightmapSensorCfg)
        # that aerial tasks use for terrain clearance (reward + crash
        # termination). It is a single downward ray, so keep it always on.
        from srb.core.sensor.category import SensorCategory

        self.active_sensors.add(SensorCategory.TERRAIN)


class AerialEnv(MobileEnv):
    cfg: AerialEnvCfg
