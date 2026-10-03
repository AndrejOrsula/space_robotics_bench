from dataclasses import MISSING

import torch

from srb.core.asset import AssetVariant, GroundRobot, Subterrane, Terrain
from srb.core.default_assets import get_default_asset_factories
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
class GroundSceneCfg(MobileSceneCfg):
    env_spacing: float = 32.0


@configclass
class GroundEventCfg(MobileEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (1.0, 2.0),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.5),
                "roll": (-radians(5.0), radians(5.0)),
                "pitch": (-radians(5.0), radians(5.0)),
                "yaw": (-radians(15.0), radians(15.0)),
            },
        },
    )


@configclass
class GroundEnvCfg(MobileEnvCfg):
    ## Assets
    robot: GroundRobot | AssetVariant = get_default_asset_factories().ground_robot()
    _robot: GroundRobot = MISSING  # type: ignore[assignment]
    scenery: Terrain | Subterrane | AssetVariant | None = AssetVariant.PROCEDURAL

    ## Scene
    scene: GroundSceneCfg = GroundSceneCfg()

    ## Events
    events: GroundEventCfg = GroundEventCfg()

    ## Time
    env_rate: float = 1.0 / 50.0
    agent_rate: float = 1.0 / 25.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(7.5, -7.5, 15.0), lookat=(0.0, 0.0, 0.0), origin_type="env"
    )

    ## Spawn
    robot_spawn_height_range: tuple[float, float] | None = None
    """Root height offset range (m) above the robot's default spawn height on reset.

    ``None`` keeps the ``randomize_robot_state`` event term's own range
    (1 to 2 m for the shared ground default, which clears the tallest
    procedural terrain bump but leaves a legged robot falling under lunar
    gravity for the first second of every episode). Setting a range here
    overrides that term, which is otherwise not reachable from the CLI
    (event terms are not serialised).
    """

    def __post_init__(self):
        # Before the base post-init: it deep-copies this term for every
        # ``extra_robots`` entry (the clones must inherit the range) and a
        # robot layout removes it (the range then has nothing to apply to).
        randomize_robot_state = getattr(self.events, "randomize_robot_state", None)
        if self.robot_spawn_height_range is not None and (
            randomize_robot_state is not None
        ):
            pose_range = randomize_robot_state.params["pose_range"]
            pose_range["z"] = tuple(self.robot_spawn_height_range)
        super().__post_init__()


class GroundEnv(MobileEnv):
    cfg: GroundEnvCfg
