from functools import cached_property
from typing import Dict, Sequence, Tuple

import gymnasium
import numpy
import torch
from isaaclab.envs import DirectRLEnv as __DirectRLEnv

from srb._typing import StepReturn
from srb.core.asset import Articulation, RigidObject
from srb.core.manager import ActionManager
from srb.core.sim.robot_setup import AssembledBodies, RobotAssembler
from srb.utils.math import combine_frame_transforms, subtract_frame_transforms
from srb.utils.str import resolve_env_prim_path

from .cfg import DirectEnvCfg


class __PostInitCaller(type):
    def __call__(cls, *args, **kwargs):
        obj = type.__call__(cls, *args, **kwargs)
        obj.__post_init__()
        return obj


class DirectEnv(__DirectRLEnv, metaclass=__PostInitCaller):
    cfg: DirectEnvCfg
    _step_return: StepReturn

    def __init__(self, cfg: DirectEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        # Apply visuals
        self.cfg.visuals.func(self.cfg.visuals)

        # Add action manager
        if self.cfg.actions:
            self.action_manager = ActionManager(
                self.cfg.actions,
                env=self,  # type: ignore
            )
            print(self.action_manager)

        ## Get scene assets
        self._robot: Articulation = self.scene["robot"]

    def close(self):
        if not self._is_closed:
            if self.cfg.actions:
                del self.action_manager

        super().close()

    def extract_step_return(self) -> StepReturn:
        raise NotImplementedError

    def _reset_idx(self, env_ids: Sequence[int]):
        if self.cfg.actions:
            self.action_manager.reset(env_ids)

        super()._reset_idx(env_ids)

        # Move assembled bodies to the correct position to avoid physics snapping them in place
        self._update_assembly_fixed_joint_transforms(env_ids)

    def _pre_physics_step(self, actions: torch.Tensor):
        if self.cfg.actions:
            self.action_manager.process_action(actions)
        else:
            super()._pre_physics_step(actions)  # type: ignore

    def _apply_action(self):
        if self.cfg.actions:
            self.action_manager.apply_action()
        else:
            super()._apply_action()  # type: ignore

    def __post_init__(self):
        if self._use_step_return_workflow:
            self._step_return = self.extract_step_return()

            # Verify that all observation components have the correct shape
            for obs_cat, obs_group in self._step_return.observation.items():
                for obs_key, obs_val in obs_group.items():
                    assert obs_val.size(0) == self.num_envs, (
                        f"Observation component '{obs_cat}/{obs_key}' has an incorrect shape. "
                        f"Expected: ({self.num_envs}, ...) | Actual: {obs_val.shape}"
                    )
            # Verify that all reward components have the correct shape
            for rew_key, rew_val in self._step_return.reward.items():
                assert rew_val.shape == (self.num_envs,), (
                    f"Reward component '{rew_key}' has an incorrect shape. "
                    f"Expected: ({self.num_envs},) | Actual: {rew_val.shape}"
                )

        # Automatically determine the action and observation spaces for all sub-classes
        self._update_gym_env_spaces()

    def _update_gym_env_spaces(self):
        # Action space
        self.single_action_space = gymnasium.spaces.Box(
            low=-numpy.inf,
            high=numpy.inf,
            shape=(self.action_manager.total_action_dim,),
        )
        self.action_space = gymnasium.vector.utils.batch_space(
            self.single_action_space, self.num_envs
        )

        # Observation space
        self.single_observation_space = gymnasium.spaces.Dict({})
        for obs_key, obs_buf in self._get_observations().items():
            assert isinstance(obs_buf, (numpy.ndarray, torch.Tensor))
            self.single_observation_space[obs_key] = gymnasium.spaces.Box(
                low=-numpy.inf, high=numpy.inf, shape=obs_buf.shape[1:]
            )
        self.observation_space = gymnasium.vector.utils.batch_space(
            self.single_observation_space, self.num_envs
        )

    @cached_property
    def max_episode_length(self):
        # Wrap lengthy calculation in a cached property
        return super().max_episode_length

    @cached_property
    def _use_step_return_workflow(self) -> bool:
        return self.__class__.extract_step_return is not DirectEnv.extract_step_return

    def _get_dones(self) -> Tuple[torch.Tensor, torch.Tensor]:
        if self._use_step_return_workflow:
            self._step_return = self.extract_step_return()
            if self.cfg.include_extras:
                self.extras["reward_terms"] = self._step_return.reward
            if self._step_return.info:
                self.extras.update(self._step_return.info)
            return self._step_return.termination, self._step_return.truncation
        else:
            return super()._get_dones()  # type: ignore

    def _get_rewards(self) -> torch.Tensor:
        if self._use_step_return_workflow:
            return _sum_rewards(self._step_return.reward)
        else:
            return super()._get_rewards()  # type: ignore

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        if self._use_step_return_workflow:
            return _flatten_observations(self._step_return.observation)
        else:
            return super()._get_observations()  # type: ignore

    def _setup_scene(self):
        super()._setup_scene()

        ## Handle assemblies
        self.joint_assemblies: Dict[str, Sequence[AssembledBodies]] = {}
        for key, assembly_cfg in self.cfg.joint_assemblies.items():
            self.joint_assemblies[key] = tuple(
                RobotAssembler().assemble_rigid_bodies(
                    assembly_cfg.model_copy(
                        update={
                            "base_path": resolve_env_prim_path(
                                assembly_cfg.base_path, i
                            ),
                            "attach_path": resolve_env_prim_path(
                                assembly_cfg.attach_path, i
                            ),
                        }
                    )
                )
                for i in range(self.num_envs)
            )

    def _update_assembly_fixed_joint_transforms(self, env_ids: Sequence[int]):
        for key, assembly_cfg in self.cfg.joint_assemblies.items():
            attach_asset: RigidObject | Articulation = self.scene[key]
            base_asset: RigidObject | Articulation = self.scene[
                assembly_cfg.base_path.rsplit("/", 1)[-1]
            ]
            attach_root_state = attach_asset.data.root_state_w[env_ids]
            attach_body_state = (
                attach_asset.data.body_state_w[
                    env_ids,
                    attach_asset.find_bodies(
                        assembly_cfg.attach_mount_frame.removeprefix("/")
                    )[0][0],
                ]
                if assembly_cfg.attach_mount_frame
                else attach_root_state
            )
            base_body_state = (
                base_asset.data.body_state_w[
                    env_ids,
                    base_asset.find_bodies(
                        assembly_cfg.base_mount_frame.removeprefix("/")
                    )[0][0],
                ]
                if assembly_cfg.base_mount_frame
                else base_asset.data.root_state_w[env_ids]
            )

            pose = torch.cat(
                combine_frame_transforms(
                    base_body_state[:, :3],
                    base_body_state[:, 3:7],
                    *combine_frame_transforms(
                        *subtract_frame_transforms(
                            attach_body_state[:, :3],
                            attach_body_state[:, 3:7],
                            attach_root_state[:, :3],
                            attach_root_state[:, 3:7],
                        ),
                        torch.tensor(
                            assembly_cfg.fixed_joint_offset,
                            dtype=attach_root_state.dtype,
                            device=attach_root_state.device,
                        )
                        .unsqueeze(0)
                        .repeat(attach_root_state.shape[0], 1),
                        torch.tensor(
                            assembly_cfg.fixed_joint_orient,
                            dtype=attach_root_state.dtype,
                            device=attach_root_state.device,
                        )
                        .unsqueeze(0)
                        .repeat(attach_root_state.shape[0], 1),
                    ),
                ),
                dim=1,
            )

            attach_asset.write_root_pose_to_sim(pose, env_ids=env_ids)


@torch.jit.script
def _flatten_observations(
    obs_dict: Dict[str, Dict[str, torch.Tensor]],
) -> Dict[str, torch.Tensor]:
    return {
        obs_cat: torch.cat(
            [torch.flatten(tensor, start_dim=1) for tensor in obs_group.values()], dim=1
        )
        for obs_cat, obs_group in obs_dict.items()
    }


@torch.jit.script
def _sum_rewards(rew_dict: Dict[str, torch.Tensor]) -> torch.Tensor:
    return torch.sum(torch.stack(list(rew_dict.values()), dim=1), dim=1)
