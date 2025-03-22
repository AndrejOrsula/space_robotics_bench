from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import AssetVariant, Lander, Scenery
from srb.core.domain import Domain
from srb.core.env import OrbitalEnv, OrbitalEnvCfg, OrbitalEventCfg, OrbitalSceneCfg
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.utils.cfg import configclass
from srb.utils.math import deg_to_rad

##############
### Config ###
##############


@configclass
class SceneCfg(OrbitalSceneCfg):
    env_spacing = 64.0


@configclass
class EventCfg(OrbitalEventCfg):
    randomize_robot_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-5.0, 5.0),
                "y": (-5.0, 5.0),
                "z": (10.0, 25.0),
                "roll": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "pitch": (-deg_to_rad(10.0), deg_to_rad(10.0)),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {
                "x": (-1.0, 1.0),
                "y": (-1.0, 1.0),
                "z": (-5.0, 0.0),
                "roll": (-deg_to_rad(5.0), deg_to_rad(5.0)),
                "pitch": (-deg_to_rad(5.0), deg_to_rad(5.0)),
                "yaw": (-deg_to_rad(5.0), deg_to_rad(5.0)),
            },
        },
    )


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scenario
    domain: Domain = Domain.MOON

    ## Assets
    robot: Lander | AssetVariant = assets.RandomLander()
    _robot: Lander = MISSING  # type: ignore
    scenery: Scenery | AssetVariant | None = AssetVariant.PROCEDURAL

    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    env_rate: float = 1.0 / 25.0
    agent_rate: float = 1.0 / 25.0
    episode_length_s: float = 60.0
    is_finite_horizon: bool = True

    def __post_init__(self):
        super().__post_init__()


############
### Task ###
############

# TODO[mid]: Implement MDP logic for excavation


class Task(OrbitalEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        return _compute_step_return(
            ## Time
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            ## Actions
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            ## States
            # Root
            tf_pos_robot=self._robot.data.root_pos_w,
            vel_lin_robot=self._robot.data.root_lin_vel_b,
            vel_ang_robot=self._robot.data.root_ang_vel_b,
            # IMU
            imu_lin_acc=self._imu_robot.data.lin_acc_b,
            imu_ang_vel=self._imu_robot.data.ang_vel_b,
        )


@torch.jit.script
def _compute_step_return(
    *,
    ## Time
    episode_length: torch.Tensor,
    max_episode_length: int,
    truncate_episodes: bool,
    ## Actions
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    ## States
    # Root
    tf_pos_robot: torch.Tensor,
    vel_lin_robot: torch.Tensor,
    vel_ang_robot: torch.Tensor,
    # IMU
    imu_lin_acc: torch.Tensor,
    imu_ang_vel: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
    # dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Angular velocity
    WEIGHT_ANGULAR_VELOCITY = -0.1
    penalty_angular_velocity = WEIGHT_ANGULAR_VELOCITY * torch.norm(
        vel_ang_robot, dim=-1
    )

    ##################
    ## Terminations ##
    ##################
    # No termination condition
    termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
    # Truncation
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        {
            "state": {
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
            },
            # "state_dyn": {},
            "proprio": {
                "imu_lin_acc": imu_lin_acc,
                "imu_ang_vel": imu_ang_vel,
            },
            # "proprio_dyn": {},
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_angular_velocity": penalty_angular_velocity,
        },
        termination,
        truncation,
    )
