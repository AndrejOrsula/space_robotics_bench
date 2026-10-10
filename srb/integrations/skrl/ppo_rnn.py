"""SRB recurrent PPO agent, trainer, and runner for SKRL."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, cast

import torch
from skrl.agents.torch.base import Agent
from skrl.agents.torch.ppo import PPO, PPO_CFG
from skrl.resources.schedulers.torch import KLAdaptiveLR
from skrl.trainers.torch import SequentialTrainer

from srb.integrations.skrl.rnn import (
    RecurrentRolloutBuffer,
    recurrent_gaussian_model,
    recurrent_value_model,
)
from srb.integrations.skrl.runner import SrbSkrlRunner
from srb.integrations.skrl.transition import (
    terminal_final_states,
    terminal_next_states,
)


def _column(tensor: torch.Tensor, *, dtype: torch.dtype | None = None) -> torch.Tensor:
    value = torch.as_tensor(tensor)
    if dtype is not None:
        value = value.to(dtype=dtype)
    if value.ndim == 1:
        value = value.unsqueeze(-1)
    return value


def _preprocess_states(
    preprocessor: Any,
    states: torch.Tensor,
    *,
    train: bool,
    valid_mask: torch.Tensor | None = None,
) -> torch.Tensor:
    """Preprocess sequences without training on padded rows."""
    if states.ndim != 3:
        return cast(torch.Tensor, preprocessor(states, train=train))
    batch, length, features = states.shape
    flat_states = states.reshape(batch * length, features)
    if train and valid_mask is not None:
        mask = torch.as_tensor(valid_mask, device=states.device, dtype=torch.bool)
        if mask.ndim == 3 and mask.shape[-1] == 1:
            mask = mask[..., 0]
        if mask.shape != (batch, length):
            raise ValueError(
                "recurrent sequence mask does not match state sequences: "
                f"{tuple(mask.shape)} != {(batch, length)}"
            )
        flat_mask = mask.reshape(-1)
        if bool(flat_mask.any()):
            # Update RunningStandardScaler on real rows only, then transform
            # padding without training; the loss masks padded outputs.
            preprocessor(flat_states[flat_mask], train=True)
        processed = cast(torch.Tensor, preprocessor(flat_states, train=False))
    else:
        processed = cast(torch.Tensor, preprocessor(flat_states, train=train))
    return processed.reshape(batch, length, -1)


class SrbRecurrentPPO(PPO):  # type: ignore[misc]
    """PPO with per-environment GRU state and padded rollouts."""

    def __init__(
        self,
        *,
        models: dict[str, Any],
        memory: RecurrentRolloutBuffer | None = None,
        observation_space: Any = None,
        state_space: Any = None,
        action_space: Any = None,
        device: str | torch.device | None = None,
        cfg: PPO_CFG | dict[str, Any] = {},
    ) -> None:
        super().__init__(
            models=models,
            memory=memory,  # type: ignore[arg-type]
            observation_space=observation_space,
            state_space=state_space,
            action_space=action_space,
            device=device,
            cfg=cfg,
        )
        if not isinstance(self.memory, RecurrentRolloutBuffer):
            raise TypeError("SrbRecurrentPPO requires RecurrentRolloutBuffer")
        self._rollout_memory = self.memory
        policy = self.policy
        value = self.value
        for model, role in ((policy, "policy"), (value, "value")):
            if model is None or not hasattr(model, "initial_state"):
                raise TypeError(f"recurrent PPO {role} model lacks initial_state()")
        self._policy_hidden: torch.Tensor
        self._value_hidden: torch.Tensor
        self._pending_reset: torch.Tensor
        self._current_policy_hidden: torch.Tensor | None = None
        self._current_value_hidden: torch.Tensor | None = None
        self._current_reset: torch.Tensor | None = None
        self._current_log_prob: torch.Tensor | None = None
        self._rollout = 0
        self._update_count = 0
        self._last_update: dict[str, float] = {}
        self.reset_hidden_state()

    def init(self, trainer_cfg: Mapping[str, Any] | None = None) -> None:
        # PPO.init expects flat SKRL memory. Keep Agent checkpoint/log setup
        # while the SRB buffer owns sequences; initialize recurrent state here.
        if isinstance(trainer_cfg, Mapping):
            trainer_cfg = self._trainer_cfg_type(**trainer_cfg)
        Agent.init(self, trainer_cfg=trainer_cfg)  # type: ignore[arg-type]
        self.enable_training_mode(False)
        self.enable_models_training_mode(False)
        self._rollout = 0
        self.reset_hidden_state()

    @staticmethod
    def _trainer_cfg_type(**values: Any) -> Any:
        """Build the installed SKRL trainer dataclass."""
        from skrl.trainers.torch import SequentialTrainerCfg

        return SequentialTrainerCfg(**values)

    def reset_hidden_state(self, mask: torch.Tensor | None = None) -> None:
        """Reset all or selected recurrent states at episode boundaries."""
        num_envs = self._rollout_memory.num_envs
        if (
            not hasattr(self, "_policy_hidden")
            or self._policy_hidden.shape[1] != num_envs
        ):
            self._policy_hidden = self.policy.initial_state(
                num_envs, device=self.device
            )
            self._value_hidden = self.value.initial_state(num_envs, device=self.device)
            self._pending_reset = torch.zeros(
                num_envs, 1, dtype=torch.bool, device=self.device
            )
            return
        if mask is None:
            self._policy_hidden.zero_()
            self._value_hidden.zero_()
            self._pending_reset.zero_()
            return
        reset = _column(mask).to(device=self.device, dtype=torch.bool)
        if reset.shape != (num_envs, 1):
            raise ValueError(
                f"recurrent reset mask must have shape {(num_envs, 1)}, got {tuple(reset.shape)}"
            )
        reset_hidden = reset.view(1, num_envs, 1)
        self._policy_hidden = torch.where(
            reset_hidden, torch.zeros_like(self._policy_hidden), self._policy_hidden
        )
        self._value_hidden = torch.where(
            reset_hidden, torch.zeros_like(self._value_hidden), self._value_hidden
        )
        self._pending_reset = torch.where(
            reset, torch.ones_like(self._pending_reset), self._pending_reset
        )

    def _apply_pending_reset(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        reset = self._pending_reset.clone()
        if reset.any():
            mask = reset.view(1, -1, 1)
            self._policy_hidden = torch.where(
                mask, torch.zeros_like(self._policy_hidden), self._policy_hidden
            )
            self._value_hidden = torch.where(
                mask, torch.zeros_like(self._value_hidden), self._value_hidden
            )
            self._pending_reset.zero_()
        return self._policy_hidden.clone(), self._value_hidden.clone(), reset

    def act(
        self,
        observations: torch.Tensor,
        states: torch.Tensor | None = None,
        *,
        timestep: int,
        timesteps: int,
    ) -> tuple[torch.Tensor, Mapping[str, Any]]:
        policy_hidden, value_hidden, reset = self._apply_pending_reset()
        self._current_policy_hidden = policy_hidden
        self._current_value_hidden = value_hidden
        self._current_reset = reset
        observations = torch.as_tensor(
            observations, device=self.device, dtype=torch.float32
        )
        if states is None:
            states = observations
        else:
            states = torch.as_tensor(states, device=self.device, dtype=torch.float32)
        processed_observations = self._observation_preprocessor(observations)
        processed_states = self._state_preprocessor(states)
        with torch.autocast(
            device_type=self._device_type, enabled=self.cfg.mixed_precision
        ):
            actions, outputs = self.policy.act(
                {
                    "observations": processed_observations,
                    "states": processed_states,
                    "rnn_state": policy_hidden,
                    "reset_mask": reset,
                },
                role="policy",
            )
        self._current_log_prob = outputs["log_prob"]
        self._policy_hidden = outputs["rnn_state"].detach()
        return actions, outputs

    def record_transition(
        self,
        *,
        observations: torch.Tensor,
        states: torch.Tensor | None,
        actions: torch.Tensor,
        rewards: torch.Tensor,
        next_observations: torch.Tensor,
        next_states: torch.Tensor | None,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        infos: Any,
        timestep: int,
        timesteps: int,
    ) -> None:
        observations = torch.as_tensor(
            observations, device=self.device, dtype=torch.float32
        )
        next_observations = torch.as_tensor(
            next_observations, device=self.device, dtype=torch.float32
        )
        states_supplied = states is not None
        next_states_supplied = next_states is not None
        if states is None:
            states = observations
        else:
            states = torch.as_tensor(states, device=self.device, dtype=torch.float32)
        if next_states is None:
            if states_supplied:
                raise ValueError(
                    "SKRL recurrent transition requires next_states when states "
                    "are supplied"
                )
            next_states = next_observations
        else:
            next_states = torch.as_tensor(
                next_states, device=self.device, dtype=torch.float32
            )
        terminated = _column(terminated).to(device=self.device, dtype=torch.bool)
        truncated = _column(truncated).to(device=self.device, dtype=torch.bool)
        done = terminated | truncated
        next_observations = terminal_next_states(next_observations, infos, done)
        if next_states_supplied:
            next_states = terminal_final_states(next_states, infos, done)
        else:
            # By contract, observations supply the state snapshot only when
            # no critic stream was provided; shapes alone are insufficient.
            next_states = next_observations
        Agent.record_transition(
            self,
            observations=observations,
            states=states,
            actions=actions,
            rewards=rewards,
            next_observations=next_observations,
            next_states=next_states,
            terminated=terminated,
            truncated=truncated,
            infos=infos,
            timestep=timestep,
            timesteps=timesteps,
        )
        if not self.training:
            self._pending_reset = done
            return
        if (
            self._current_policy_hidden is None
            or self._current_value_hidden is None
            or self._current_reset is None
        ):
            raise RuntimeError("record_transition called before act")
        if self._current_log_prob is None:
            raise RuntimeError("recurrent PPO has no current policy log probability")

        rewards = _column(rewards).to(device=self.device, dtype=torch.float32)
        if self.cfg.rewards_shaper is not None:
            rewards = _column(self.cfg.rewards_shaper(rewards, timestep, timesteps)).to(
                device=self.device, dtype=torch.float32
            )
        processed_observations = self._observation_preprocessor(observations)
        processed_states = self._state_preprocessor(states)
        final_next_states = next_states
        with (
            torch.no_grad(),
            torch.autocast(
                device_type=self._device_type,
                enabled=self.cfg.mixed_precision,
            ),
        ):
            values, value_outputs = self.value.act(
                {
                    "observations": processed_observations,
                    "states": processed_states,
                    "rnn_state": self._current_value_hidden,
                    "reset_mask": self._current_reset,
                },
                role="value",
            )
            next_processed = self._state_preprocessor(final_next_states)
            next_processed_observations = self._observation_preprocessor(
                next_observations
            )
            next_values, _ = self.value.act(
                {
                    "observations": next_processed_observations,
                    "states": next_processed,
                    "rnn_state": value_outputs["rnn_state"],
                },
                role="value",
            )
            values = self._value_preprocessor(_column(values), inverse=True)
            next_values = self._value_preprocessor(_column(next_values), inverse=True)
        if self.cfg.time_limit_bootstrap:
            rewards = rewards + self.cfg.discount_factor * next_values * truncated
        self._value_hidden = value_outputs["rnn_state"].detach()
        self._rollout_memory.add_samples(
            observations=observations,
            states=states,
            actions=actions,
            rewards=rewards,
            next_values=_column(next_values).detach(),
            terminated=terminated,
            truncated=truncated,
            log_prob=_column(self._current_log_prob).detach(),
            values=_column(values).detach(),
            # Store per environment to keep asynchronous episodes isolated.
            policy_hidden=self._current_policy_hidden.transpose(0, 1),
            value_hidden=self._current_value_hidden.transpose(0, 1),
            episode_start=self._current_reset,
        )
        self._pending_reset = done
        self._current_policy_hidden = None
        self._current_value_hidden = None
        self._current_reset = None
        self._current_log_prob = None

    def post_interaction(self, *, timestep: int, timesteps: int) -> None:
        if self.training:
            self._rollout += 1
            if (
                self.cfg.rollouts > 0
                and self._rollout % self.cfg.rollouts == 0
                and timestep >= self.cfg.learning_starts
            ):
                self.enable_models_training_mode(True)
                self.update(timestep=timestep, timesteps=timesteps)
                self.enable_models_training_mode(False)
        Agent.post_interaction(self, timestep=timestep, timesteps=timesteps)

    def _preprocess_value_targets(self) -> None:
        """Normalize stored values and returns."""
        values = self._rollout_memory.stack("values")
        returns = self._rollout_memory.stack("returns")
        value_shape = values.shape
        processed_values = self._value_preprocessor(
            values.reshape(-1, values.shape[-1]), train=True
        ).reshape(value_shape)
        processed_returns = self._value_preprocessor(
            returns.reshape(-1, returns.shape[-1]), train=True
        ).reshape(returns.shape)
        for index, record in enumerate(self._rollout_memory.records):
            record["values"] = processed_values[index]
            record["returns"] = processed_returns[index]

    def _update(self, timestep: int, timesteps: int) -> None:
        if len(self._rollout_memory) == 0:
            return
        self._rollout_memory.compute_targets(
            self.cfg.discount_factor,
            self.cfg.gae_lambda,
            time_limit_bootstrap=self.cfg.time_limit_bootstrap,
        )
        self._preprocess_value_targets()

        cumulative_policy = 0.0
        cumulative_value = 0.0
        cumulative_entropy = 0.0
        updates = 0
        for epoch in range(self.cfg.learning_epochs):
            kl_divergences: list[torch.Tensor] = []
            # Reshuffle sequences into fresh minibatches every epoch.
            batches = self._rollout_memory.sequence_batches(
                mini_batches=self.cfg.mini_batches,
                sequence_length=self._rollout_memory.sequence_length,
            )
            for batch in batches:
                valid = batch["sequence_mask"].to(device=self.device, dtype=torch.bool)
                observations = batch["observations"].to(
                    device=self.device, dtype=torch.float32
                )
                states = batch["states"].to(device=self.device, dtype=torch.float32)
                actions = batch["actions"].to(device=self.device, dtype=torch.float32)
                observations = _preprocess_states(
                    self._observation_preprocessor,
                    observations,
                    train=not epoch,
                    valid_mask=valid,
                )
                states = _preprocess_states(
                    self._state_preprocessor,
                    states,
                    train=not epoch,
                    valid_mask=valid,
                )
                with torch.autocast(
                    device_type=self._device_type,
                    enabled=self.cfg.mixed_precision,
                ):
                    _, policy_outputs = self.policy.act(
                        {
                            "observations": observations,
                            "states": states,
                            "taken_actions": actions,
                            "rnn_state": batch["policy_hidden"],
                            "reset_mask": batch["episode_start"],
                            "sequence_mask": valid,
                        },
                        role="policy",
                    )
                    next_log_prob = policy_outputs["log_prob"]
                    predicted_values, _ = self.value.act(
                        {
                            "observations": observations,
                            "states": states,
                            "rnn_state": batch["value_hidden"],
                            "reset_mask": batch["episode_start"],
                            "sequence_mask": valid,
                        },
                        role="value",
                    )
                    old_log_prob = batch["log_prob"].to(self.device)
                    old_values = batch["values"].to(self.device)
                    returns = batch["returns"].to(self.device)
                    advantages = batch["advantages"].to(self.device)
                    mask = valid.expand_as(next_log_prob)
                    if not mask.any():
                        continue
                    with torch.no_grad():
                        log_ratio = next_log_prob - old_log_prob
                        kl_divergence = ((torch.exp(log_ratio) - 1.0) - log_ratio)[
                            mask
                        ].mean()
                    kl_divergences.append(kl_divergence.detach())
                    if self.cfg.kl_threshold and bool(
                        kl_divergence > self.cfg.kl_threshold
                    ):
                        break
                    ratio = torch.exp(next_log_prob - old_log_prob)
                    surrogate = advantages * ratio
                    clipped_ratio = torch.clamp(
                        ratio,
                        1.0 - self.cfg.ratio_clip,
                        1.0 + self.cfg.ratio_clip,
                    )
                    policy_loss = -torch.minimum(surrogate, advantages * clipped_ratio)[
                        mask
                    ].mean()
                    if self.cfg.value_clip > 0:
                        clipped_values = old_values + torch.clamp(
                            predicted_values - old_values,
                            -self.cfg.value_clip,
                            self.cfg.value_clip,
                        )
                        value_loss = torch.maximum(
                            (predicted_values - returns).square(),
                            (clipped_values - returns).square(),
                        )[mask].mean()
                    else:
                        value_loss = (predicted_values - returns).square()[mask].mean()
                    value_loss = self.cfg.value_loss_scale * value_loss
                    entropy = self.policy.get_entropy(role="policy")
                    entropy_mask = valid.expand_as(entropy)
                    entropy_loss = (
                        -self.cfg.entropy_loss_scale * entropy[entropy_mask].mean()
                    )
                    loss = policy_loss + value_loss + entropy_loss

                self.optimizer.zero_grad()
                self.scaler.scale(loss).backward()
                if self.cfg.grad_norm_clip > 0:
                    self.scaler.unscale_(self.optimizer)
                    torch.nn.utils.clip_grad_norm_(
                        list(self.policy.parameters()) + list(self.value.parameters()),
                        self.cfg.grad_norm_clip,
                    )
                self.scaler.step(self.optimizer)
                self.scaler.update()
                cumulative_policy += float(policy_loss.detach())
                cumulative_value += float(value_loss.detach())
                cumulative_entropy += float(entropy_loss.detach())
                updates += 1

            if self.scheduler:
                if isinstance(self.scheduler, KLAdaptiveLR):
                    if kl_divergences:
                        self.scheduler.step(torch.stack(kl_divergences).mean())
                else:
                    self.scheduler.step()

        if updates:
            self._update_count += 1
            self._last_update = {
                "policy_loss": cumulative_policy / updates,
                "value_loss": cumulative_value / updates,
                "entropy_loss": cumulative_entropy / updates,
            }
            self.track_data("Loss / Policy loss", self._last_update["policy_loss"])
            self.track_data("Loss / Value loss", self._last_update["value_loss"])
            if self.scheduler:
                self.track_data(
                    "Learning / Learning rate", self.scheduler.get_last_lr()[0]
                )
        self._rollout_memory.clear()

    def update(self, *, timestep: int, timesteps: int) -> None:
        """Expose the SKRL 2.x update hook with SRB storage."""
        self._update(timestep=timestep, timesteps=timesteps)

    def load(self, path: str) -> None:
        super().load(path)
        # Reloaded weights start with fresh episode state.
        self.reset_hidden_state()


class SrbRecurrentTrainer(SequentialTrainer):  # type: ignore[misc]
    """Sequential SKRL trainer with per-environment resets."""


class SrbPpoRnnRunner(SrbSkrlRunner):  # type: ignore[misc, unused-ignore]
    """Runner supplying missing SKRL recurrent components."""

    def _component(self, name: str) -> Any:
        normalized = name.lower()
        if normalized in {
            "srbrecurrentgaussian",
            "srbrecurrentgaussianmixin",
            "recurrentgaussianmixin",
        }:
            return recurrent_gaussian_model
        if normalized in {"srbrecurrentvalue", "srbrecurrentvaluemixin"}:
            return recurrent_value_model
        if normalized in {"srbrecurrentrolloutbuffer", "recurrentrolloutbuffer"}:
            return RecurrentRolloutBuffer
        if normalized in {"srbrecurrentppo", "ppo_rnn", "srbrppo"}:
            return SrbRecurrentPPO
        if normalized in {"srbrecurrenttrainer", "recurrenttrainer"}:
            return SrbRecurrentTrainer
        return super()._component(name)

    def _generate_models(
        self, env: Any, cfg: Mapping[str, Any]
    ) -> dict[str, dict[str, Any]]:
        agent_class = str(cfg.get("agent", {}).get("class", "")).lower()
        if agent_class not in {"srbrecurrentppo", "srbrppo", "ppo_rnn"}:
            return cast(dict[str, dict[str, Any]], super()._generate_models(env, cfg))
        models_cfg = cfg.get("models")
        if not isinstance(models_cfg, Mapping):
            raise ValueError("No 'models' are defined in recurrent PPO cfg")
        if bool(models_cfg.get("separate", True)) is False:
            raise ValueError("recurrent PPO requires separate policy and value models")
        observation_space = env.observation_space
        state_space = getattr(env, "state_space", None)
        action_space = env.action_space
        device = env.device
        policy_cfg = dict(models_cfg.get("policy", {}))
        value_cfg = dict(models_cfg.get("value", {}))
        policy_cfg.pop("class", None)
        value_cfg.pop("class", None)
        policy = recurrent_gaussian_model(
            observation_space=observation_space,
            state_space=state_space,
            action_space=action_space,
            device=device,
            **self._process_cfg(policy_cfg),
        )
        value = recurrent_value_model(
            observation_space=observation_space,
            state_space=state_space,
            action_space=action_space,
            device=device,
            **self._process_cfg(value_cfg),
        )
        if isinstance(policy, str) or isinstance(value, str):
            raise RuntimeError("recurrent model factories returned source text")
        return {"agent": {"policy": policy, "value": value}}

    def _generate_agent(
        self, env: Any, cfg: Mapping[str, Any], models: Mapping[str, Any]
    ) -> SrbRecurrentPPO:
        agent_cfg_source = dict(cfg.get("agent", {}))
        agent_cfg_source.pop("class", None)
        agent_cfg_values = self._process_cfg(agent_cfg_source)
        if "lambda" in agent_cfg_values:
            if (
                "gae_lambda" in agent_cfg_values
                and agent_cfg_values["gae_lambda"] != agent_cfg_values["lambda"]
            ):
                raise ValueError("conflicting PPO keys: 'lambda' and 'gae_lambda'")
            agent_cfg_values["gae_lambda"] = agent_cfg_values.pop("lambda")
        if "clip_predicted_values" in agent_cfg_values:
            clip_predicted_values = bool(agent_cfg_values.pop("clip_predicted_values"))
            if not clip_predicted_values:
                agent_cfg_values["value_clip"] = 0.0
        if int(agent_cfg_values.get("random_timesteps", 0)):
            raise ValueError(
                "SRB recurrent PPO does not support random_timesteps; "
                "use a policy rollout from timestep zero"
            )
        observation_space = env.observation_space
        state_space = getattr(env, "state_space", None) or observation_space
        for key in ("observation_preprocessor_kwargs", "state_preprocessor_kwargs"):
            if not isinstance(agent_cfg_values.get(key), dict):
                agent_cfg_values[key] = {}
        if agent_cfg_values.get("observation_preprocessor") is not None:
            agent_cfg_values["observation_preprocessor_kwargs"].update(
                {"size": observation_space, "device": env.device}
            )
        if agent_cfg_values.get("state_preprocessor") is not None:
            agent_cfg_values["state_preprocessor_kwargs"].update(
                {"size": state_space, "device": env.device}
            )
        value_kwargs = agent_cfg_values.get("value_preprocessor_kwargs")
        if not isinstance(value_kwargs, dict):
            agent_cfg_values["value_preprocessor_kwargs"] = {}
        agent_cfg_values["value_preprocessor_kwargs"].update(
            {"size": 1, "device": env.device}
        )
        agent_cfg = PPO_CFG(**agent_cfg_values)
        memory_cfg = dict(cfg.get("memory", {}))
        memory_cfg.pop("class", None)
        memory_size = int(memory_cfg.pop("memory_size", agent_cfg.rollouts))
        if memory_size < agent_cfg.rollouts:
            raise ValueError(
                "recurrent PPO memory_size must be at least rollouts: "
                f"memory_size={memory_size}, rollouts={agent_cfg.rollouts}"
            )
        memory = RecurrentRolloutBuffer(
            memory_size=memory_size,
            num_envs=int(env.num_envs),
            device=env.device,
            **self._process_cfg(memory_cfg),
        )
        return SrbRecurrentPPO(
            models=models["agent"],
            memory=memory,
            observation_space=observation_space,
            state_space=getattr(env, "state_space", None),
            action_space=env.action_space,
            device=env.device,
            cfg=agent_cfg,
        )

    def _generate_trainer(
        self, env: Any, cfg: Mapping[str, Any], agent: Any
    ) -> SrbRecurrentTrainer:
        trainer_cfg = dict(cfg.get("trainer", {}))
        trainer_cfg.pop("class", None)
        return SrbRecurrentTrainer(env=env, agents=agent, cfg=trainer_cfg)


__all__ = [
    "SrbPpoRnnRunner",
    "SrbRecurrentPPO",
    "SrbRecurrentTrainer",
]
