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
from srb.utils import logging
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

    def __post_init__(self):
        if self._use_step_return_workflow:
            self._step_return = self.extract_step_return()

            # Verify that all observation components have the correct shape and are finite
            for obs_cat, obs_group in self._step_return.observation.items():
                for obs_key, obs_val in obs_group.items():
                    assert obs_val.size(0) == self.num_envs, (
                        f"Observation component '{obs_cat}/{obs_key}' has an incorrect shape. "
                        f"Expected: ({self.num_envs}, ...) | Actual: {obs_val.shape}"
                    )
                    assert torch.isfinite(obs_val).all(), (
                        f"Observation component '{obs_cat}/{obs_key}' contains non-finite values."
                    )
            # Verify that all reward components have the correct shape and are finite
            for rew_key, rew_val in self._step_return.reward.items():
                assert rew_val.shape == (self.num_envs,), (
                    f"Reward component '{rew_key}' has an incorrect shape. "
                    f"Expected: ({self.num_envs},) | Actual: {rew_val.shape}"
                )
                assert torch.isfinite(rew_val).all(), (
                    f"Reward component '{rew_key}' contains non-finite values."
                )

        # Automatically determine the action and observation spaces for all sub-classes
        self._update_gym_env_spaces()

        ## Initialize action/observation delay buffers
        action_delay = self.cfg.action_delay_steps
        self._min_action_delay_steps, self._max_action_delay_steps = (
            (action_delay, action_delay)
            if isinstance(action_delay, int)
            else action_delay
        )
        obs_delay = self.cfg.observation_delay_steps
        self._min_obs_delay_steps, self._max_obs_delay_steps = (
            (obs_delay, obs_delay) if isinstance(obs_delay, int) else obs_delay
        )
        # Allocate action delay buffers
        if self._max_action_delay_steps > 0:
            logging.info(
                f"Action delay of maximum {self._max_action_delay_steps} agent steps enabled."
            )
            self._action_history_buffer: torch.Tensor | None = torch.zeros(
                (
                    self._max_action_delay_steps,
                    self.num_envs,
                    self.action_manager.total_action_dim,
                ),
                device=self.device,
            )
            self._action_history_buffer_ptr: int = 0
            self._action_delay_steps = torch.zeros(
                self.num_envs, dtype=torch.long, device=self.device
            )
        else:
            self._action_history_buffer: torch.Tensor | None = None
        # Allocate observation delay buffers
        # TODO[mid]: Handle observation delay for non step return workflows
        if self._use_step_return_workflow and self._max_obs_delay_steps > 0:
            logging.info(
                f"Observation delay of maximum {self._max_obs_delay_steps} agent steps enabled."
            )
            self._obs_history_buffer: Dict[str, Dict[str, torch.Tensor]] | None = {
                cat: {
                    key: torch.zeros(
                        (
                            self._max_obs_delay_steps,
                            self.num_envs,
                            *val.shape[1:],
                        ),
                        dtype=val.dtype,
                        device=self.device,
                    )
                    for key, val in group.items()
                }
                for cat, group in self._step_return.observation.items()
            }
            self._obs_history_buffer_ptr: int = 0
            self._observation_delay_steps = torch.zeros(
                self.num_envs, dtype=torch.long, device=self.device
            )
        else:
            self._obs_history_buffer: Dict[str, Dict[str, torch.Tensor]] | None = None

    def close(self):
        if not self._is_closed:
            if self.cfg.actions:
                del self.action_manager

        super().close()

    def extract_step_return(self) -> StepReturn:
        raise NotImplementedError

    def _extract_step_return_wrapped(self) -> StepReturn:
        step_return = self.extract_step_return()

        # Apply observation delay if enabled
        if self._obs_history_buffer is not None:
            # Store current observations in the history buffer
            for obs_cat, obs_group in step_return.observation.items():
                for obs_key, obs_val in obs_group.items():
                    self._obs_history_buffer[obs_cat][obs_key][
                        self._obs_history_buffer_ptr
                    ] = obs_val

            # Calculate indices to read delayed observations
            read_indices = (
                self._obs_history_buffer_ptr
                - self._observation_delay_steps
                + self._max_obs_delay_steps
            ) % self._max_obs_delay_steps

            # Fetch delayed observations
            delayed_obs = {}
            for obs_cat, obs_group_buffer in self._obs_history_buffer.items():
                delayed_obs[obs_cat] = {}
                for obs_key, history_buffer in obs_group_buffer.items():
                    idx = read_indices.view(
                        1, self.num_envs, *((1,) * (len(history_buffer.shape) - 2))
                    )
                    idx = idx.expand(-1, -1, *history_buffer.shape[2:])
                    delayed_obs[obs_cat][obs_key] = torch.gather(
                        history_buffer, 0, idx
                    ).squeeze(0)

            # Advance the history buffer pointer for the next agent step
            self._obs_history_buffer_ptr = (
                self._obs_history_buffer_ptr + 1
            ) % self._max_obs_delay_steps

            # Randomize observation delay with on-step probability
            if (
                self.cfg.observation_delay_on_step_change_prob > 0.0
                and (self._min_obs_delay_steps < self._max_obs_delay_steps)
                and (
                    self.sim.current_time
                    % self.cfg.observation_delay_on_step_change_freq
                )
                < self.cfg.agent_rate
            ):
                dist = torch.rand((self.num_envs,), device=self.device)
                decrease_delay_indices = (
                    self._observation_delay_steps > self._min_obs_delay_steps
                ) & (dist < self.cfg.observation_delay_on_step_change_prob)
                increase_delay_indices = (
                    self._observation_delay_steps < self._max_obs_delay_steps
                ) & (dist > (1.0 - self.cfg.observation_delay_on_step_change_prob))
                self._observation_delay_steps[decrease_delay_indices] -= 1
                self._observation_delay_steps[increase_delay_indices] += 1

            # Replace observation in step_return with the delayed one
            step_return = StepReturn(
                observation=delayed_obs,
                reward=step_return.reward,
                termination=step_return.termination,
                truncation=step_return.truncation,
                info=step_return.info,
            )

        ## Detect environments that "exploded" due to physics and terminate them
        invalid_envs = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        invalid_keys = []
        for rew_key, rew_val in step_return.reward.items():
            rew_invalid = ~torch.isfinite(rew_val)
            invalid_envs = invalid_envs | rew_invalid
            if rew_invalid.any():
                step_return.reward[rew_key] = torch.where(
                    rew_invalid, torch.zeros_like(rew_val), rew_val
                )
                invalid_keys.append(rew_key)
        for obs_cat, obs_group in step_return.observation.items():
            for obs_key, obs_val in obs_group.items():
                dims = (
                    tuple(range(1, len(obs_val.shape))) if len(obs_val.shape) > 1 else 0
                )
                obs_invalid = (
                    torch.any(~torch.isfinite(obs_val), dim=dims)
                    if dims != 0
                    else ~torch.isfinite(obs_val)
                )
                invalid_envs = invalid_envs | obs_invalid
                if obs_invalid.any():
                    mask = (
                        obs_invalid.unsqueeze(-1).expand_as(obs_val)
                        if len(obs_val.shape) > 1
                        else obs_invalid
                    )
                    step_return.observation[obs_cat][obs_key] = torch.where(
                        mask, torch.zeros_like(obs_val), obs_val
                    )
                    invalid_keys.append(f"{obs_cat}/{obs_key}")
        if invalid_envs.any():
            logging.warning(
                f"Terminating {invalid_envs.sum().item()} environment instance{'s' if invalid_envs.sum().item() > 1 else ''} due to non-finite rewards or observations: {', '.join(invalid_keys)}"
            )
            step_return = StepReturn(
                observation=step_return.observation,
                reward=step_return.reward,
                termination=torch.where(
                    invalid_envs, torch.ones_like(invalid_envs), step_return.termination
                ),
                truncation=step_return.truncation,
                info=step_return.info,
            )

        return step_return

    def _reset_idx(self, env_ids: Sequence[int]):
        if self.cfg.actions:
            self.action_manager.reset(env_ids)

        # Reset action/observation delay
        num_reset_envs: int = len(env_ids)
        if self._action_history_buffer is not None:
            self._action_delay_steps[env_ids] = torch.randint(
                self._min_action_delay_steps,
                self._max_action_delay_steps + 1,
                (num_reset_envs,),
                device=self.device,
            )
            self._action_history_buffer[:, env_ids] = 0.0
        if self._obs_history_buffer is not None:
            self._observation_delay_steps[env_ids] = torch.randint(
                self._min_obs_delay_steps,
                self._max_obs_delay_steps + 1,
                (num_reset_envs,),
                device=self.device,
            )
            for obs_cat in self._obs_history_buffer:
                for obs_key in self._obs_history_buffer[obs_cat]:
                    self._obs_history_buffer[obs_cat][obs_key][:, env_ids] = 0.0

        super()._reset_idx(env_ids)

        # Move assembled bodies to the correct position to avoid physics snapping them in place
        self._update_assembly_fixed_joint_transforms(env_ids)

    def _pre_physics_step(self, actions: torch.Tensor):
        if self.cfg.actions:
            if self._action_history_buffer is not None:
                # Store current action in the history buffer
                self._action_history_buffer[self._action_history_buffer_ptr] = actions

                # Calculate indices to read delayed actions
                read_indices = (
                    self._action_history_buffer_ptr
                    - self._action_delay_steps
                    + self._max_action_delay_steps
                ) % self._max_action_delay_steps

                # Fetch delayed actions
                idx = read_indices.view(1, self.num_envs, 1).expand(
                    -1, -1, self.action_manager.total_action_dim
                )
                delayed_actions = torch.gather(
                    self._action_history_buffer, 0, idx
                ).squeeze(0)

                # Advance the history buffer pointer for the next agent step
                self._action_history_buffer_ptr = (
                    self._action_history_buffer_ptr + 1
                ) % self._max_action_delay_steps

                # Randomize action delay with on-step probability
                if (
                    self.cfg.action_delay_on_step_change_prob > 0.0
                    and (self._min_action_delay_steps < self._max_action_delay_steps)
                    and (
                        self.sim.current_time
                        % self.cfg.action_delay_on_step_change_freq
                    )
                    < self.cfg.agent_rate
                ):
                    dist = torch.rand((self.num_envs,), device=self.device)
                    decrease_delay_indices = (
                        self._action_delay_steps > self._min_action_delay_steps
                    ) & (dist < self.cfg.action_delay_on_step_change_prob)
                    increase_delay_indices = (
                        self._action_delay_steps < self._max_action_delay_steps
                    ) & (dist > (1.0 - self.cfg.action_delay_on_step_change_prob))
                    self._action_delay_steps[decrease_delay_indices] -= 1
                    self._action_delay_steps[increase_delay_indices] += 1

                self.action_manager.process_action(delayed_actions)
            else:
                self.action_manager.process_action(actions)
        else:
            super()._pre_physics_step(actions)  # type: ignore

    def _apply_action(self):
        if self.cfg.actions:
            self.action_manager.apply_action()
        else:
            super()._apply_action()  # type: ignore

    def _update_gym_env_spaces(self):
        # Action space
        self.single_action_space = gymnasium.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.action_manager.total_action_dim,),
        )
        self.action_space = gymnasium.vector.utils.batch_space(
            self.single_action_space, self.num_envs
        )

        # Observation space
        self.single_observation_space = gymnasium.spaces.Dict({})
        for obs_key, obs_buf in self._get_observations().items():
            assert isinstance(obs_buf, (numpy.ndarray, torch.Tensor))
            if isinstance(obs_buf, torch.Tensor):
                match obs_buf.dtype:
                    case torch.float32:
                        dtype = numpy.float32
                    case torch.float64:
                        dtype = numpy.float64
                    case torch.int8:
                        dtype = numpy.int8
                    case torch.uint8:
                        dtype = numpy.uint8
                    case torch.int16:
                        dtype = numpy.int16
                    case torch.uint16:
                        dtype = numpy.uint16
                    case torch.int32:
                        dtype = numpy.int32
                    case torch.uint32:
                        dtype = numpy.uint32
                    case torch.int64:
                        dtype = numpy.int64
                    case torch.uint64:
                        dtype = numpy.uint64
                    case torch.bool:
                        dtype = numpy.bool_
                    case _:
                        raise TypeError(
                            f"Unsupported torch dtype '{obs_buf.dtype}' for observation '{obs_key}'."
                        )
            else:
                dtype = obs_buf.dtype
            match dtype:
                case numpy.float32 | numpy.float64:
                    low = numpy.finfo(dtype).min  # type: ignore
                    high = numpy.finfo(dtype).max  # type: ignore
                case (
                    numpy.int8
                    | numpy.uint8
                    | numpy.int16
                    | numpy.uint16
                    | numpy.int32
                    | numpy.uint32
                    | numpy.int64
                    | numpy.uint64
                ):
                    low = numpy.iinfo(dtype).min
                    high = numpy.iinfo(dtype).max
                case numpy.bool_:
                    low = 0
                    high = 1
                case _:
                    low = -numpy.inf
                    high = numpy.inf
            self.single_observation_space[obs_key] = gymnasium.spaces.Box(
                low=low,
                high=high,
                shape=obs_buf.shape[1:],
                dtype=dtype,  # type: ignore
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
            self._step_return = self._extract_step_return_wrapped()
            if self.cfg.extras:
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
            [
                torch.flatten(obs_group[obs_key], start_dim=1)
                for obs_key in sorted(obs_group.keys())
            ],
            dim=1,
        )
        for obs_cat, obs_group in obs_dict.items()
    }


@torch.jit.script
def _sum_rewards(rew_dict: Dict[str, torch.Tensor]) -> torch.Tensor:
    return torch.sum(torch.stack(list(rew_dict.values()), dim=1), dim=1)
