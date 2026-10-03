from dataclasses import MISSING

import torch

from srb.core.action import ThrustAction
from srb.core.asset import AssetVariant, ExtravehicularScenery, OrbitalRobot
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
from srb.utils.math import radians


@configclass
class OrbitalSceneCfg(MobileSceneCfg):
    env_spacing: float = 4.0


@configclass
class OrbitalEventCfg(MobileEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
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
                "roll": (-radians(10.0), radians(10.0)),
                "pitch": (-radians(10.0), radians(10.0)),
                "yaw": (-radians(10.0), radians(10.0)),
            },
        },
    )


@configclass
class OrbitalEnvCfg(MobileEnvCfg):
    ## Scenario
    domain: Domain = Domain.ORBIT

    ## Assets
    robot: OrbitalRobot | AssetVariant = get_default_asset_factories().orbital_robot()
    _robot: OrbitalRobot = MISSING  # type: ignore[assignment]
    scenery: ExtravehicularScenery | AssetVariant | None = None

    ## Scene
    scene: OrbitalSceneCfg = OrbitalSceneCfg()

    ## Events
    events: OrbitalEventCfg = OrbitalEventCfg()

    ## Time
    env_rate: float = 1.0 / 50.0
    agent_rate: float = 1.0 / 25.0

    ## Sensors
    # Orbital robot cameras author no clipping range, so their far plane is the
    # 1e6 m render default and depth observations fall back to a 20 m bound.
    # 100 m (~39 cm per uint8 depth level) is the orbital default; tasks with
    # their own distance scale override it (`rendezvous` 50 m,
    # `orbital_evasion` 150 m).
    camera_clipping_range: tuple[float, float] | None = (0.01, 100.0)

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(10.0, -10.0, 10.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )


class OrbitalEnv(MobileEnv):
    cfg: OrbitalEnvCfg

    def __init__(self, cfg: OrbitalEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Check if thrust action is used (fuel tracking)
        self._thrust_action_term = self.action_manager.get_term_by_type(ThrustAction)
