from dataclasses import MISSING

import torch

from srb import assets
from srb.core.asset import AssetVariant, GroundRobot
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
class GroundSceneCfg(MobileSceneCfg):
    env_spacing = 48.0


@configclass
class GroundEventCfg(MobileEventCfg):
    randomize_robot_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.4, 0.6),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.5),
                "roll": (-0.5 * torch.pi, 0.5 * torch.pi),
                "pitch": (-0.5 * torch.pi, 0.5 * torch.pi),
                "yaw": (-0.5 * torch.pi, 0.5 * torch.pi),
            },
        },
    )


@configclass
class GroundEnvCfg(MobileEnvCfg):
    ## Assets
    robot: GroundRobot | AssetVariant = assets.Perseverance()
    _robot: GroundRobot = MISSING  # type: ignore

    ## Scene
    scene: GroundSceneCfg = GroundSceneCfg()

    ## Events
    events: GroundEventCfg = GroundEventCfg()

    ## Time
    env_rate: float = 1.0 / 100.0
    agent_rate: float = 1.0 / 50.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(7.5, -7.5, 15.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()


class GroundEnv(MobileEnv):
    cfg: GroundEnvCfg

    def __init__(self, cfg: GroundEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)
