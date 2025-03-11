from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing import StepReturn
from srb.core.asset import AssetVariant, Object, RigidObjectCfg
from srb.core.env import (
    OrbitalManipulationEnv,
    OrbitalManipulationEnvCfg,
    OrbitalManipulationEventCfg,
    OrbitalManipulationSceneCfg,
    ViewerCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.tasks.manipulation.debris_capture.asset import select_debris
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d

##############
### Config ###
##############


@configclass
class SceneCfg(OrbitalManipulationSceneCfg):
    debris: RigidObjectCfg = MISSING  # type: ignore


@configclass
class EventCfg(OrbitalManipulationEventCfg):
    randomize_obj_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("debris"),
            "pose_range": {
                "x": (-0.25, 0.25),
                "y": (-0.25, 0.25),
                "z": (-0.25, 0.25),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-0.5 - 0.05, -0.5 + 0.05),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
                "roll": (-torch.pi, torch.pi),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
        },
    )


@configclass
class TaskCfg(OrbitalManipulationEnvCfg):
    ## Assets
    debris: Object | AssetVariant | None = AssetVariant.PROCEDURAL

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 30.0

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(4.0, -4.0, 4.0), lookat=(0.0, -2.0, 0.0), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()

        # Scene: Debris
        self.scene.debris = select_debris(
            self,  # type: ignore
            prim_path="{ENV_REGEX_NS}/debris",
            init_state=RigidObjectCfg.InitialStateCfg(pos=(2.5, 0.0, 0.0)),
            activate_contact_sensors=True,
        )
        self.scene.debris.spawn.seed += self.scene.num_envs  # type: ignore

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############

# TODO[mid]: Implement MDP logic for mobile debris capture


class Task(OrbitalManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        return _compute_step_return(
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            robot_quat=self._robot.data.root_quat_w,
            truncate_episodes=self.cfg.truncate_episodes,
        )


@torch.jit.script
def _compute_step_return(
    *,
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    episode_length: torch.Tensor,
    max_episode_length: int,
    robot_quat: torch.Tensor,
    truncate_episodes: bool,
) -> StepReturn:
    # Robot pose
    rotmat_robot = matrix_from_quat(robot_quat)
    rot6d_robot = rotmat_to_rot6d(rotmat_robot)

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    ##################
    ## Terminations ##
    ##################
    termination = torch.zeros(
        episode_length.size(0), dtype=torch.bool, device=episode_length.device
    )
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros_like(termination)
    )

    return StepReturn(
        {
            # "state": {},
            # "state_dyn": {},
            "proprio": {
                "rot6d_robot": rot6d_robot,
            },
            # "proprio_dyn": {},
        },
        {
            "penalty_action_rate": penalty_action_rate,
        },
        termination,
        truncation,
    )
