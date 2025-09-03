from dataclasses import MISSING

import torch

from srb import assets
from srb.core.action import ThrustAction
from srb.core.asset import AssetVariant, MobileRobot, OrbitalRobot, Scenery
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
from srb.utils.math import deg_to_rad


@configclass
class OrbitalSceneCfg(MobileSceneCfg):
    env_spacing: float = 4.0


@configclass
class OrbitalEventCfg(MobileEventCfg):
    randomize_robot_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.2, 0.2),
                "y": (-0.2, 0.2),
                "z": (-0.2, 0.2),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.2, 0.2),
                "y": (-0.2, 0.2),
                "z": (-0.2, 0.2),
                "roll": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "pitch": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "yaw": (-deg_to_rad(10.0), deg_to_rad(10.0)),
            },
        },
    )


@configclass
class OrbitalEnvCfg(MobileEnvCfg):
    ## Scenario
    domain: Domain = Domain.ORBIT

    ## Assets
    robot: OrbitalRobot | AssetVariant = assets.Cubesat()
    _robot: OrbitalRobot = MISSING  # type: ignore
    scenery: Scenery | MobileRobot | AssetVariant | None = None

    ## Scene
    scene: OrbitalSceneCfg = OrbitalSceneCfg()

    ## Events
    events: OrbitalEventCfg = OrbitalEventCfg()

    ## Time
    env_rate: float = 1.0 / 40.0
    agent_rate: float = 1.0 / 10.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(10.0, -10.0, 10.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()


class OrbitalEnv(MobileEnv):
    cfg: OrbitalEnvCfg

    def __init__(self, cfg: OrbitalEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Check if thrust action is used (fuel tracking)
        self._thrust_action_term_key = next(
            filter(
                lambda term_key: isinstance(
                    self.action_manager._terms[term_key], ThrustAction
                ),
                self.action_manager._terms.keys(),
            ),
            None,
        )
