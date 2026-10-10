"""SRB-owned bounded Gaussian policy for the pinned SKRL SAC API."""

from __future__ import annotations

import itertools
import math
import textwrap
from collections.abc import Mapping
from typing import Any, Optional, Sequence, Tuple, Union

import gymnasium
import numpy as np
import torch
import torch.nn as nn  # noqa: F401
import torch.nn.functional as F
from skrl import config
from skrl.agents.torch.sac import SAC
from skrl.models.torch import GaussianMixin, Model
from skrl.utils.model_instantiators.torch.common import generate_containers
from skrl.utils.spaces.torch import unflatten_tensorized_space  # noqa: F401
from torch.distributions import Normal

from srb.integrations.skrl.demo_agent import DemoReplayAgentMixin


class SquashedGaussianMixin(GaussianMixin):
    """Gaussian policy with affine action bounds and a corrected log density.

    SKRL's :class:`GaussianMixin` samples an unbounded normal and clips it.
    Clipping breaks the policy density used by SAC.  This mixin samples in
    latent space, applies ``tanh`` and the action-space affine transform, and
    includes both Jacobians when evaluating the density.
    """

    def __init__(
        self,
        *,
        clip_actions: bool = False,
        clip_mean_actions: bool = False,
        clip_log_std: bool = True,
        min_log_std: float = -20,
        max_log_std: float = 2,
        reduction: str = "sum",
        role: str = "",
        action_epsilon: float = 1.0e-6,
    ) -> None:
        # ``clip_actions`` is intentionally ignored by the sampling path: tanh
        # and the affine map already enforce the declared bounds.
        super().__init__(
            clip_actions=False,
            clip_mean_actions=False,
            clip_log_std=clip_log_std,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            reduction=reduction,
            role=role,
        )
        if not isinstance(self.action_space, gymnasium.spaces.Box):
            raise TypeError(
                "SRB SAC requires a continuous Box action space, got "
                f"{type(self.action_space).__name__}"
            )
        low = np.asarray(self.action_space.low, dtype=np.float64).reshape(-1)
        high = np.asarray(self.action_space.high, dtype=np.float64).reshape(-1)
        if low.shape != high.shape or not low.size:
            raise ValueError("SRB SAC action bounds must be non-empty and aligned")
        if not np.all(np.isfinite(low)) or not np.all(np.isfinite(high)):
            raise ValueError("SRB SAC action bounds must be finite")
        if np.any(high <= low):
            raise ValueError("SRB SAC action bounds must satisfy high > low")
        action_epsilon = float(action_epsilon)
        if (
            not math.isfinite(action_epsilon)
            or action_epsilon <= 0
            or action_epsilon >= 1
        ):
            raise ValueError("action_epsilon must be in the open interval (0, 1)")
        scale = (high - low) / 2.0
        bias = (high + low) / 2.0
        if not np.all(np.isfinite(scale)) or not np.all(np.isfinite(bias)):
            raise ValueError("SRB SAC action bounds produce non-finite affine terms")

        self.register_buffer(
            "_srb_action_low",
            torch.as_tensor(low, dtype=torch.float32, device=self.device),
        )
        self.register_buffer(
            "_srb_action_high",
            torch.as_tensor(high, dtype=torch.float32, device=self.device),
        )
        self.register_buffer(
            "_srb_action_scale",
            torch.as_tensor(scale, dtype=torch.float32, device=self.device),
        )
        self.register_buffer(
            "_srb_action_bias",
            torch.as_tensor(bias, dtype=torch.float32, device=self.device),
        )
        self._srb_action_epsilon = action_epsilon

    def _bounded_action(self, latent: torch.Tensor) -> torch.Tensor:
        return torch.clamp(
            self._srb_action_bias + self._srb_action_scale * torch.tanh(latent),
            min=self._srb_action_low,
            max=self._srb_action_high,
        )

    def _latent_from_action(self, actions: torch.Tensor) -> torch.Tensor:
        normalized = (actions - self._srb_action_bias) / self._srb_action_scale
        limit = torch.full_like(normalized, 1.0 - self._srb_action_epsilon)
        # Keep epsilon below one after casting so atanh stays finite.
        dtype_limit = torch.nextafter(
            torch.ones_like(normalized), torch.zeros_like(normalized)
        )
        limit = torch.minimum(limit, dtype_limit)
        return torch.atanh(normalized.clamp(min=-limit, max=limit))

    def _log_probability(
        self,
        distribution: Normal,
        latent: torch.Tensor,
    ) -> torch.Tensor:
        # Include the affine scale in the density. Evaluate log(1 - tanh(z)^2)
        # stably so the correction stays finite near action bounds.
        log_tanh_jacobian = 2.0 * (math.log(2.0) - latent - F.softplus(-2.0 * latent))
        jacobian = torch.log(self._srb_action_scale) + log_tanh_jacobian
        log_probability = distribution.log_prob(latent) - jacobian
        if self._g_reduction is not None:
            log_probability = self._g_reduction(log_probability, dim=-1)
        if log_probability.dim() != latent.dim():
            log_probability = log_probability.unsqueeze(-1)
        return log_probability

    def act(
        self,
        inputs: Mapping[str, Union[torch.Tensor, Any]],
        *,
        role: str = "",
    ) -> Tuple[torch.Tensor, Mapping[str, Union[torch.Tensor, Any]]]:
        computed = self.compute(inputs, role=role)
        if not isinstance(computed, tuple) or len(computed) != 2:
            raise TypeError("SRB SAC model compute must return (actions, outputs)")
        mean_actions, outputs = computed
        if not isinstance(outputs, Mapping) or "log_std" not in outputs:
            raise KeyError("SRB SAC model compute outputs must contain 'log_std'")
        # Copy output before adding SKRL's required log-probability keys.
        output_values = dict(outputs)
        log_std = output_values["log_std"]
        if not isinstance(mean_actions, torch.Tensor) or not isinstance(
            log_std, torch.Tensor
        ):
            raise TypeError("SRB SAC model compute must return tensor actions/log_std")
        if self._g_clip_log_std:
            log_std = torch.clamp(log_std, self._g_min_log_std, self._g_max_log_std)
            output_values["log_std"] = log_std

        self._g_log_std = log_std
        self._g_num_samples = mean_actions.shape[0]
        distribution = Normal(mean_actions, log_std.exp())
        self._g_distribution = distribution

        sampled_latent = distribution.rsample()
        sampled_actions = self._bounded_action(sampled_latent)
        taken_actions = inputs.get("taken_actions")
        if taken_actions is None:
            log_probability = self._log_probability(distribution, sampled_latent)
        else:
            if not isinstance(taken_actions, torch.Tensor):
                taken_actions = torch.as_tensor(
                    taken_actions, dtype=mean_actions.dtype, device=mean_actions.device
                )
            log_probability = self._log_probability(
                distribution, self._latent_from_action(taken_actions)
            )

        output_values["mean_actions"] = self._bounded_action(mean_actions)
        output_values["log_prob"] = log_probability
        return sampled_actions, output_values


def squashed_gaussian_model(
    *,
    observation_space: Optional[Union[int, Tuple[int], gymnasium.Space]] = None,
    state_space: Optional[Union[int, Tuple[int], gymnasium.Space]] = None,
    action_space: Optional[Union[int, Tuple[int], gymnasium.Space]] = None,
    device: Optional[Union[str, torch.device]] = None,
    clip_actions: bool = False,
    clip_mean_actions: bool = False,
    clip_log_std: bool = True,
    min_log_std: float = -20,
    max_log_std: float = 2,
    reduction: str = "sum",
    initial_log_std: float = 0,
    fixed_log_std: bool = False,
    action_epsilon: float = 1.0e-6,
    network: Sequence[Mapping[str, Any]] = (),
    output: Union[str, Sequence[str]] = "",
    return_source: bool = False,
) -> Union[Model, str]:
    """Instantiate the SRB SAC policy using SKRL's model-config language."""
    if not network:
        raise ValueError("SRB SAC policy requires a non-empty network definition")
    containers, output_spec = generate_containers(
        list(network), output, embed_output=True, indent=1
    )
    networks: list[str] = []
    forward: list[str] = []
    for container in containers:
        networks.append(
            f"self.{container['name']}_container = {container['sequential']}"
        )
        forward.append(
            f"{container['name']} = self.{container['name']}_container({container['input']})"
        )
    if output_spec["modules"]:
        networks.append(f"self.output_layer = {output_spec['modules'][0]}")
        forward.append(f"output = self.output_layer({containers[-1]['name']})")
    if output_spec["output"]:
        forward.append(f"output = {output_spec['output']}")
    else:
        forward[-1] = forward[-1].replace(f"{containers[-1]['name']} =", "output =", 1)

    network_source = textwrap.indent("\n".join(networks), " " * 8)[8:]
    forward_source = textwrap.indent("\n".join(forward), " " * 8)[8:]
    template = f"""class SquashedGaussianModel(SquashedGaussianMixin, Model):
    def __init__(
        self,
        observation_space,
        state_space,
        action_space,
        device=None,
        clip_actions=False,
        clip_mean_actions=False,
        clip_log_std=True,
        min_log_std=-20,
        max_log_std=2,
        reduction="sum",
        role="",
        action_epsilon=1.0e-6,
    ):
        Model.__init__(
            self,
            observation_space=observation_space,
            state_space=state_space,
            action_space=action_space,
            device=device,
        )
        SquashedGaussianMixin.__init__(
            self,
            clip_actions=clip_actions,
            clip_mean_actions=clip_mean_actions,
            clip_log_std=clip_log_std,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            reduction=reduction,
            role=role,
            action_epsilon=action_epsilon,
        )

        {network_source}
        self.log_std_parameter = nn.Parameter(
            torch.full(size=({output_spec["size"]},),
                       fill_value={float(initial_log_std)}),
            requires_grad={not fixed_log_std})

    def compute(self, inputs, role=""):
        observations = unflatten_tensorized_space(
            self.observation_space, inputs.get("observations")
        )
        states = unflatten_tensorized_space(self.state_space, inputs.get("states"))
        taken_actions = unflatten_tensorized_space(self.action_space,
                                                   inputs.get("taken_actions"))
        {forward_source}
        return output, {{"log_std": self.log_std_parameter}}
"""
    if return_source:
        return template

    namespace: dict[str, Any] = {}
    exec(template, globals(), namespace)
    return namespace["SquashedGaussianModel"](
        observation_space=observation_space,
        state_space=state_space,
        action_space=action_space,
        device=device,
        clip_actions=clip_actions,
        clip_mean_actions=clip_mean_actions,
        clip_log_std=clip_log_std,
        min_log_std=min_log_std,
        max_log_std=max_log_std,
        reduction=reduction,
        action_epsilon=action_epsilon,
    )


class SrbSAC(DemoReplayAgentMixin, SAC):
    """SRB SAC transition and infinite-horizon target semantics."""

    demo_algorithm = "skrl_sac"

    def update(self, *, timestep: int, timesteps: int) -> None:
        """Run pinned SKRL SAC updates with SRB timeout bootstrap semantics."""

        from srb.integrations.skrl.demo_agent import (
            preprocess_training_batch,
            sample_transition_batch,
        )

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
                    next_actions, next_outputs = self.policy.act(
                        next_inputs, role="policy"
                    )
                    next_log_prob = next_outputs["log_prob"]
                    target_q1_values, _ = self.target_critic_1.act(
                        {**next_inputs, "taken_actions": next_actions},
                        role="target_critic_1",
                    )
                    target_q2_values, _ = self.target_critic_2.act(
                        {**next_inputs, "taken_actions": next_actions},
                        role="target_critic_2",
                    )
                    target_q_values = (
                        torch.min(target_q1_values, target_q2_values)
                        - self._entropy_coefficient * next_log_prob
                    )
                    target_values = self.target_values(
                        sampled_rewards,
                        target_q_values,
                        sampled_terminated,
                        self.cfg.discount_factor,
                    )

                critic_1_values, _ = self.critic_1.act(
                    {**inputs, "taken_actions": sampled_actions},
                    role="critic_1",
                )
                critic_2_values, _ = self.critic_2.act(
                    {**inputs, "taken_actions": sampled_actions},
                    role="critic_2",
                )
                critic_loss = (
                    F.mse_loss(critic_1_values, target_values)
                    + F.mse_loss(critic_2_values, target_values)
                ) / 2

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

            with torch.autocast(
                device_type=self._device_type, enabled=self.cfg.mixed_precision
            ):
                actions, policy_outputs = self.policy.act(inputs, role="policy")
                log_prob = policy_outputs["log_prob"]
                critic_1_values, _ = self.critic_1.act(
                    {**inputs, "taken_actions": actions},
                    role="critic_1",
                )
                critic_2_values, _ = self.critic_2.act(
                    {**inputs, "taken_actions": actions},
                    role="critic_2",
                )
                policy_loss = (
                    self._entropy_coefficient * log_prob
                    - torch.min(critic_1_values, critic_2_values)
                ).mean()

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

            if self.cfg.learn_entropy:
                with torch.autocast(
                    device_type=self._device_type, enabled=self.cfg.mixed_precision
                ):
                    entropy_loss = -(
                        self.log_entropy_coefficient
                        * (log_prob + self._target_entropy).detach()
                    ).mean()
                self.entropy_optimizer.zero_grad()
                self.scaler.scale(entropy_loss).backward()
                self.scaler.step(self.entropy_optimizer)
                self._entropy_coefficient = torch.exp(
                    self.log_entropy_coefficient.detach()
                )

            self.scaler.update()
            self.target_critic_1.update_parameters(
                self.critic_1, polyak=self.cfg.polyak
            )
            self.target_critic_2.update_parameters(
                self.critic_2, polyak=self.cfg.polyak
            )
            if self.policy_scheduler:
                self.policy_scheduler.step()
            if self.critic_scheduler:
                self.critic_scheduler.step()

            if self.write_interval > 0:
                self.track_data("Loss / Policy loss", policy_loss.item())
                self.track_data("Loss / Critic loss", critic_loss.item())
                for label, values in (
                    ("Q1", critic_1_values),
                    ("Q2", critic_2_values),
                ):
                    self.track_data(f"Q-network / {label} (max)", values.max().item())
                    self.track_data(f"Q-network / {label} (min)", values.min().item())
                    self.track_data(f"Q-network / {label} (mean)", values.mean().item())
                self.track_data("Target / Target (max)", target_values.max().item())
                self.track_data("Target / Target (min)", target_values.min().item())
                self.track_data("Target / Target (mean)", target_values.mean().item())
                if self.cfg.learn_entropy:
                    self.track_data("Loss / Entropy loss", entropy_loss.item())
                    self.track_data(
                        "Coefficient / Entropy coefficient",
                        self._entropy_coefficient.item(),
                    )
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


__all__ = [
    "SrbSAC",
    "SquashedGaussianMixin",
    "squashed_gaussian_model",
]
