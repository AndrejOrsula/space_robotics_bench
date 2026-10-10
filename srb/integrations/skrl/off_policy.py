"""Terminal-aware native SKRL 2.x DDPG and TD3 agents."""

from __future__ import annotations

import itertools

import torch
import torch.nn as nn
import torch.nn.functional as F
from skrl import config
from skrl.agents.torch.ddpg import DDPG
from skrl.agents.torch.td3 import TD3

from srb.integrations.skrl.demo_agent import (
    DemoReplayAgentMixin,
    preprocess_training_batch,
    sample_transition_batch,
)


class SrbDDPG(DemoReplayAgentMixin, DDPG):
    """Native SKRL DDPG with SRB timeout bootstrapping."""

    demo_algorithm = "skrl_ddpg"

    def update(self, *, timestep: int, timesteps: int) -> None:
        """Run SKRL DDPG updates with SRB target semantics."""

        del timestep, timesteps
        for _gradient_step in range(self.cfg.gradient_steps):
            (
                sampled_observations,
                sampled_states,
                sampled_actions,
                sampled_rewards,
                sampled_next_observations,
                sampled_next_states,
                sampled_terminated,
                _sampled_truncated,
            ) = sample_transition_batch(self)

            with torch.autocast(
                device_type=self._device_type, enabled=self.cfg.mixed_precision
            ):
                inputs = {
                    "observations": preprocess_training_batch(
                        self._observation_preprocessor, sampled_observations
                    ),
                    "states": preprocess_training_batch(
                        self._state_preprocessor, sampled_states
                    ),
                }
                next_inputs = {
                    "observations": preprocess_training_batch(
                        self._observation_preprocessor, sampled_next_observations
                    ),
                    "states": preprocess_training_batch(
                        self._state_preprocessor, sampled_next_states
                    ),
                }
                with torch.no_grad():
                    next_actions, _ = self.target_policy.act(
                        next_inputs, role="target_policy"
                    )
                    target_q_values, _ = self.target_critic.act(
                        {**next_inputs, "taken_actions": next_actions},
                        role="target_critic",
                    )
                    target_values = self.target_values(
                        sampled_rewards,
                        target_q_values,
                        sampled_terminated,
                        self.cfg.discount_factor,
                    )

                critic_values, _ = self.critic.act(
                    {**inputs, "taken_actions": sampled_actions}, role="critic"
                )
                critic_loss = F.mse_loss(critic_values, target_values)

            self.critic_optimizer.zero_grad()
            self.scaler.scale(critic_loss).backward()
            if config.torch.is_distributed:
                self.critic.reduce_parameters()
            if self.cfg.grad_norm_clip > 0:
                self.scaler.unscale_(self.critic_optimizer)
                nn.utils.clip_grad_norm_(
                    self.critic.parameters(), self.cfg.grad_norm_clip
                )
            self.scaler.step(self.critic_optimizer)

            with torch.autocast(
                device_type=self._device_type, enabled=self.cfg.mixed_precision
            ):
                actions, _ = self.policy.act(inputs, role="policy")
                critic_values, _ = self.critic.act(
                    {**inputs, "taken_actions": actions}, role="critic"
                )
                policy_loss = -critic_values.mean()

            self.policy_optimizer.zero_grad()
            self.scaler.scale(policy_loss).backward()
            if config.torch.is_distributed:
                self.policy.reduce_parameters()
            if self.cfg.grad_norm_clip > 0:
                self.scaler.unscale_(self.policy_optimizer)
                nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.cfg.grad_norm_clip
                )
            self.scaler.step(self.policy_optimizer)
            self.scaler.update()

            self.target_policy.update_parameters(self.policy, polyak=self.cfg.polyak)
            self.target_critic.update_parameters(self.critic, polyak=self.cfg.polyak)

            if self.policy_scheduler:
                self.policy_scheduler.step()
            if self.critic_scheduler:
                self.critic_scheduler.step()

            self.track_data("Loss / Policy loss", policy_loss.item())
            self.track_data("Loss / Critic loss", critic_loss.item())
            self.track_data("Q-network / Q1 (max)", critic_values.max().item())
            self.track_data("Q-network / Q1 (min)", critic_values.min().item())
            self.track_data("Q-network / Q1 (mean)", critic_values.mean().item())
            self.track_data("Target / Target (max)", target_values.max().item())
            self.track_data("Target / Target (min)", target_values.min().item())
            self.track_data("Target / Target (mean)", target_values.mean().item())
            if self.policy_scheduler:
                self.track_data(
                    "Learning / Policy learning rate",
                    self.policy_scheduler.get_last_lr()[0],
                )
            if self.critic_scheduler:
                self.track_data(
                    "Learning / Critic learning rate",
                    self.critic_scheduler.get_last_lr()[0],
                )


class SrbTD3(DemoReplayAgentMixin, TD3):
    """Native SKRL TD3 with SRB timeout bootstrapping."""

    demo_algorithm = "skrl_td3"

    def update(self, *, timestep: int, timesteps: int) -> None:
        """Run SKRL TD3 updates with SRB target semantics."""

        del timestep, timesteps
        for _gradient_step in range(self.cfg.gradient_steps):
            (
                sampled_observations,
                sampled_states,
                sampled_actions,
                sampled_rewards,
                sampled_next_observations,
                sampled_next_states,
                sampled_terminated,
                _sampled_truncated,
            ) = sample_transition_batch(self)

            with torch.autocast(
                device_type=self._device_type, enabled=self.cfg.mixed_precision
            ):
                inputs = {
                    "observations": preprocess_training_batch(
                        self._observation_preprocessor, sampled_observations
                    ),
                    "states": preprocess_training_batch(
                        self._state_preprocessor, sampled_states
                    ),
                }
                next_inputs = {
                    "observations": preprocess_training_batch(
                        self._observation_preprocessor, sampled_next_observations
                    ),
                    "states": preprocess_training_batch(
                        self._state_preprocessor, sampled_next_states
                    ),
                }
                with torch.no_grad():
                    next_actions, _ = self.target_policy.act(
                        next_inputs, role="target_policy"
                    )
                    if self._smooth_regularization_noise is not None:
                        noises = torch.clamp(
                            self._smooth_regularization_noise.sample(
                                next_actions.shape
                            ),
                            min=-self.cfg.smooth_regularization_clip,
                            max=self.cfg.smooth_regularization_clip,
                        )
                        next_actions.add_(noises)
                        next_actions.clamp_(
                            min=self._min_actions, max=self._max_actions
                        )

                    target_q1_values, _ = self.target_critic_1.act(
                        {**next_inputs, "taken_actions": next_actions},
                        role="target_critic_1",
                    )
                    target_q2_values, _ = self.target_critic_2.act(
                        {**next_inputs, "taken_actions": next_actions},
                        role="target_critic_2",
                    )
                    target_q_values = torch.min(target_q1_values, target_q2_values)
                    target_values = self.target_values(
                        sampled_rewards,
                        target_q_values,
                        sampled_terminated,
                        self.cfg.discount_factor,
                    )

                critic_1_values, _ = self.critic_1.act(
                    {**inputs, "taken_actions": sampled_actions}, role="critic_1"
                )
                critic_2_values, _ = self.critic_2.act(
                    {**inputs, "taken_actions": sampled_actions}, role="critic_2"
                )
                critic_loss = F.mse_loss(critic_1_values, target_values) + F.mse_loss(
                    critic_2_values, target_values
                )

            self.critic_optimizer.zero_grad()
            self.scaler.scale(critic_loss).backward()
            if config.torch.is_distributed:
                self.critic_1.reduce_parameters()
                self.critic_2.reduce_parameters()
            if self.cfg.grad_norm_clip > 0:
                self.scaler.unscale_(self.critic_optimizer)
                nn.utils.clip_grad_norm_(
                    itertools.chain(
                        self.critic_1.parameters(), self.critic_2.parameters()
                    ),
                    self.cfg.grad_norm_clip,
                )
            self.scaler.step(self.critic_optimizer)

            self._update_counter += 1
            if not self._update_counter % self.cfg.policy_delay:
                with torch.autocast(
                    device_type=self._device_type, enabled=self.cfg.mixed_precision
                ):
                    actions, _ = self.policy.act(inputs, role="policy")
                    critic_values, _ = self.critic_1.act(
                        {**inputs, "taken_actions": actions}, role="critic_1"
                    )
                    policy_loss = -critic_values.mean()

                self.policy_optimizer.zero_grad()
                self.scaler.scale(policy_loss).backward()
                if config.torch.is_distributed:
                    self.policy.reduce_parameters()
                if self.cfg.grad_norm_clip > 0:
                    self.scaler.unscale_(self.policy_optimizer)
                    nn.utils.clip_grad_norm_(
                        self.policy.parameters(), self.cfg.grad_norm_clip
                    )
                self.scaler.step(self.policy_optimizer)
                self.target_critic_1.update_parameters(
                    self.critic_1, polyak=self.cfg.polyak
                )
                self.target_critic_2.update_parameters(
                    self.critic_2, polyak=self.cfg.polyak
                )
                self.target_policy.update_parameters(
                    self.policy, polyak=self.cfg.polyak
                )

            self.scaler.update()
            if self.policy_scheduler:
                self.policy_scheduler.step()
            if self.critic_scheduler:
                self.critic_scheduler.step()

            if not self._update_counter % self.cfg.policy_delay:
                self.track_data("Loss / Policy loss", policy_loss.item())
            self.track_data("Loss / Critic loss", critic_loss.item())
            self.track_data("Q-network / Q1 (max)", critic_1_values.max().item())
            self.track_data("Q-network / Q1 (min)", critic_1_values.min().item())
            self.track_data("Q-network / Q1 (mean)", critic_1_values.mean().item())
            self.track_data("Q-network / Q2 (max)", critic_2_values.max().item())
            self.track_data("Q-network / Q2 (min)", critic_2_values.min().item())
            self.track_data("Q-network / Q2 (mean)", critic_2_values.mean().item())
            self.track_data("Target / Target (max)", target_values.max().item())
            self.track_data("Target / Target (min)", target_values.min().item())
            self.track_data("Target / Target (mean)", target_values.mean().item())
            if self.policy_scheduler:
                self.track_data(
                    "Learning / Policy learning rate",
                    self.policy_scheduler.get_last_lr()[0],
                )
            if self.critic_scheduler:
                self.track_data(
                    "Learning / Critic learning rate",
                    self.critic_scheduler.get_last_lr()[0],
                )


__all__ = ["SrbDDPG", "SrbTD3"]
