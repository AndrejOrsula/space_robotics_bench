from dataclasses import MISSING

import torch

from srb import assets
from srb.core.asset import AerialRobot, AssetVariant
from srb.core.domain import Domain
from srb.core.env import ViewerCfg
from srb.core.env.mobile.env import (
    MobileEnv,
    MobileEnvCfg,
    MobileEventCfg,
    MobileSceneCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.utils.cfg import configclass


@configclass
class AerialSceneCfg(MobileSceneCfg):
    env_spacing: float = 64.0


@configclass
class AerialEventCfg(MobileEventCfg):
    randomize_robot_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
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
    robot: AerialRobot | AssetVariant = assets.Ingenuity()
    _robot: AerialRobot = MISSING  # type: ignore

    ## Scene
    scene: AerialSceneCfg = AerialSceneCfg()

    ## Events
    events: AerialEventCfg = AerialEventCfg()

    ## Time
    env_rate: float = 1.0 / 250.0
    agent_rate: float = 1.0 / 25.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(-10.0, 0.0, 20.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()


class AerialEnv(MobileEnv):
    cfg: AerialEnvCfg

    def __init__(self, cfg: AerialEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)
