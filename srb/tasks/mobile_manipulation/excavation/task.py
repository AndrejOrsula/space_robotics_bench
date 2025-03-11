from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import AssetBaseCfg, AssetVariant, GroundManipulator
from srb.core.env import (
    GroundManipulationEnv,
    GroundManipulationEnvCfg,
    GroundManipulationEventCfg,
    GroundManipulationSceneCfg,
    ViewerCfg,
)
from srb.core.sim import PyramidParticlesSpawnerCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    rotmat_to_rot6d,
    rpy_to_quat,
    scale_transform,
)

##############
### Config ###
##############


@configclass
class SceneCfg(GroundManipulationSceneCfg):
    env_spacing: float = 8.0

    regolith: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/regolith",
        spawn=PyramidParticlesSpawnerCfg(
            ratio=0.5,
            particle_size=0.01,
            dim_x=MISSING,  # type: ignore
            dim_y=MISSING,  # type: ignore
            dim_z=10,
            velocity=((-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.0)),
            fluid=False,
            friction=1.0,
            cohesion=0.5,
            cast_shadows=False,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.5)),
    )


@configclass
class EventCfg(GroundManipulationEventCfg):
    pass


@configclass
class TaskCfg(GroundManipulationEnvCfg):
    ## Assets
    robot: GroundManipulator | AssetVariant = assets.GenericGroundManipulator(
        mobile_base=assets.Spot(payload=assets.CargoBay()),
        manipulator=assets.Franka(end_effector=assets.Scoop()),
    )
    robot.asset_cfg.init_state.pos = (-1.0, 2.0, 1.0)  # type: ignore
    robot.asset_cfg.init_state.rot = rpy_to_quat(0.0, 0.0, 0.0)  # type: ignore

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 30.0

    viewer: ViewerCfg = ViewerCfg(
        eye=(2.625, 4.05, 1.3), lookat=(0.0, 1.0, -0.15), origin_type="env"
    )

    def __post_init__(self):
        super().__post_init__()

        # Scene: Regolith
        _regolith_dim = int(
            self.env_spacing / self.scene.regolith.spawn.particle_size  # type: ignore
        )
        self.scene.regolith.spawn.dim_x = _regolith_dim  # type: ignore
        self.scene.regolith.spawn.dim_y = _regolith_dim  # type: ignore


############
### Task ###
############

# TODO[mid]: Implement MDP logic for excavation


class Task(GroundManipulationEnv):
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
            joint_pos_robot=self._robot.data.joint_pos,
            robot_quat=self._robot.data.root_quat_w,
            joint_pos_limits_robot=(
                self._robot.data.soft_joint_pos_limits
                if torch.all(torch.isfinite(self._robot.data.soft_joint_pos_limits))
                else None
            ),
            truncate_episodes=self.cfg.truncate_episodes,
        )


@torch.jit.script
def _compute_step_return(
    *,
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    episode_length: torch.Tensor,
    max_episode_length: int,
    joint_pos_robot: torch.Tensor,
    robot_quat: torch.Tensor,
    joint_pos_limits_robot: torch.Tensor | None,
    truncate_episodes: bool,
) -> StepReturn:
    # Robot joints
    if joint_pos_limits_robot is not None:
        joint_pos_normalized = scale_transform(
            joint_pos_robot,
            joint_pos_limits_robot[:, :, 0],
            joint_pos_limits_robot[:, :, 1],
        )
    else:
        joint_pos_normalized = joint_pos_robot

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
            "proprio_dyn": {
                "joint_pos": joint_pos_normalized,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
        },
        termination,
        truncation,
    )
