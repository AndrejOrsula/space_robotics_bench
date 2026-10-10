"""SRB recurrent policy models and rollout storage for SKRL.

Models consume observation sequences and explicit hidden states; the agent
tracks one hidden state per environment because pinned SKRL lacks recurrent
PPO and sequence memory.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Optional, Union

import gymnasium
import torch
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model


def _as_sequence(
    states: torch.Tensor,
) -> tuple[torch.Tensor, bool]:
    """Return ``(batch, time, features)`` and whether time was supplied."""
    if states.ndim == 1:
        return states.view(1, 1, -1), False
    if states.ndim == 2:
        return states.unsqueeze(1), False
    if states.ndim == 3:
        return states, True
    raise ValueError(
        "recurrent SKRL states must have shape (batch, features) or "
        f"(batch, time, features), got {tuple(states.shape)}"
    )


def _mask_sequence(
    value: object,
    *,
    batch: int,
    length: int,
    device: torch.device,
    default: bool,
) -> torch.Tensor:
    """Normalize a reset/valid mask to ``(batch, time)``."""
    if value is None:
        return torch.full((batch, length), default, dtype=torch.bool, device=device)
    mask = torch.as_tensor(value, dtype=torch.bool, device=device)
    if mask.ndim == 3 and mask.shape[-1] == 1:
        mask = mask[..., 0]
    if mask.ndim == 1:
        if mask.shape[0] == batch and length == 1:
            mask = mask[:, None]
        elif mask.shape[0] == length and batch == 1:
            mask = mask[None, :]
        else:
            raise ValueError(
                f"recurrent mask shape {tuple(mask.shape)} does not match "
                f"(batch={batch}, time={length})"
            )
    if mask.shape != (batch, length):
        raise ValueError(
            f"recurrent mask shape {tuple(mask.shape)} does not match "
            f"(batch={batch}, time={length})"
        )
    return mask


class _RecurrentCore:
    """Shared GRU implementation used by policy and value models."""

    device: torch.device

    def _init_recurrent_core(
        self,
        *,
        input_size: int,
        hidden_size: int,
        num_layers: int,
        input_key: str,
    ) -> None:
        if hidden_size <= 0:
            raise ValueError("recurrent hidden_size must be positive")
        if num_layers <= 0:
            raise ValueError("recurrent num_layers must be positive")
        self.hidden_size = int(hidden_size)
        self.num_layers = int(num_layers)
        self.input_key = input_key
        self.encoder = torch.nn.Sequential(
            torch.nn.Linear(input_size, hidden_size),
            torch.nn.Tanh(),
        )
        self.recurrent = torch.nn.GRU(
            input_size=hidden_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
        )

    def initial_state(
        self,
        batch_size: int,
        *,
        device: torch.device | str | None = None,
    ) -> torch.Tensor:
        """Create the zero state used at an episode boundary."""
        if batch_size < 0:
            raise ValueError("batch_size must be non-negative")
        target_device = torch.device(device) if device is not None else self.device
        return torch.zeros(
            self.num_layers,
            batch_size,
            self.hidden_size,
            device=target_device,
        )

    def _coerce_state(self, state: object, batch: int) -> torch.Tensor:
        if state is None:
            return self.initial_state(batch, device=self.device)
        hidden = torch.as_tensor(state, dtype=torch.float32, device=self.device)
        if hidden.ndim == 2:
            hidden = hidden.unsqueeze(0)
        expected = (self.num_layers, batch, self.hidden_size)
        if tuple(hidden.shape) != expected:
            raise ValueError(
                f"recurrent hidden state shape {tuple(hidden.shape)} does not "
                f"match {expected}"
            )
        return hidden

    def _forward_recurrent(
        self,
        inputs: Mapping[str, Any],
    ) -> tuple[torch.Tensor, torch.Tensor, bool]:
        states = inputs.get(self.input_key)
        if states is None:
            # Preserve direct callers using the old observation-only input.
            states = inputs.get("observations", inputs.get("states"))
        if not isinstance(states, torch.Tensor):
            states = torch.as_tensor(states, dtype=torch.float32, device=self.device)
        states = states.to(device=self.device, dtype=torch.float32)
        sequence, had_time = _as_sequence(states)
        batch, length, _ = sequence.shape
        hidden = self._coerce_state(
            inputs.get("rnn_state", inputs.get("hidden_state")), batch
        )
        reset = _mask_sequence(
            inputs.get("reset_mask"),
            batch=batch,
            length=length,
            device=self.device,
            default=False,
        )
        valid = _mask_sequence(
            inputs.get("sequence_mask"),
            batch=batch,
            length=length,
            device=self.device,
            default=True,
        )

        outputs: list[torch.Tensor] = []
        zeros = torch.zeros_like(hidden)
        for index in range(length):
            reset_now = reset[:, index].view(1, batch, 1)
            hidden = torch.where(reset_now, zeros, hidden)
            encoded = self.encoder(sequence[:, index : index + 1])
            step_output, next_hidden = self.recurrent(encoded, hidden)
            valid_now = valid[:, index].view(1, batch, 1)
            hidden = torch.where(valid_now, next_hidden, hidden)
            step_output = torch.where(
                valid[:, index].view(batch, 1, 1),
                step_output,
                torch.zeros_like(step_output),
            )
            outputs.append(step_output)
        recurrent_output = torch.cat(outputs, dim=1)
        return recurrent_output, hidden, had_time


class RecurrentGaussianMixin(GaussianMixin):  # type: ignore[misc]
    """Gaussian SKRL mixin that carries an explicit recurrent state."""

    def __init__(
        self,
        *,
        clip_actions: bool = False,
        clip_mean_actions: bool = False,
        clip_log_std: bool = True,
        min_log_std: float = -20.0,
        max_log_std: float = 2.0,
        reduction: str = "sum",
        role: str = "",
    ) -> None:
        super().__init__(
            clip_actions=clip_actions,
            clip_mean_actions=clip_mean_actions,
            clip_log_std=clip_log_std,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            reduction=reduction,  # type: ignore[arg-type]
            role=role,
        )

    def act(
        self,
        inputs: Mapping[str, Any],
        *,
        role: str = "",
    ) -> tuple[torch.Tensor, Mapping[str, Any]]:
        mean_actions, outputs = self.compute(inputs, role=role)
        log_std = outputs["log_std"]

        if self._g_clip_mean_actions:
            mean_actions = torch.clamp(
                mean_actions,
                min=self._g_min_actions,
                max=self._g_max_actions,
            )
        if self._g_clip_log_std:
            log_std = torch.clamp(log_std, self._g_min_log_std, self._g_max_log_std)
            outputs["log_std"] = log_std

        self._g_distribution = torch.distributions.Normal(mean_actions, log_std.exp())
        actions = self._g_distribution.rsample()
        if self._g_clip_actions:
            actions = torch.clamp(
                actions,
                min=self._g_min_actions,
                max=self._g_max_actions,
            )

        taken_actions = inputs.get("taken_actions", actions)
        if not isinstance(taken_actions, torch.Tensor):
            taken_actions = torch.as_tensor(
                taken_actions,
                dtype=mean_actions.dtype,
                device=mean_actions.device,
            )
        log_probability = self._g_distribution.log_prob(taken_actions)
        if self._g_reduction is not None:
            log_probability = self._g_reduction(log_probability, dim=-1)
        if log_probability.dim() != actions.dim():
            log_probability = log_probability.unsqueeze(-1)

        outputs["log_prob"] = log_probability
        outputs["mean_actions"] = mean_actions
        return actions, outputs


class RecurrentGaussianModel(RecurrentGaussianMixin, Model, _RecurrentCore):  # type: ignore[misc]
    """GRU Gaussian policy with sequence-aware hidden-state inputs."""

    def __init__(
        self,
        *,
        observation_space: Union[int, tuple[int], gymnasium.Space[Any]],
        state_space: Union[int, tuple[int], gymnasium.Space[Any], None] = None,
        action_space: Union[int, tuple[int], gymnasium.Space[Any]],
        device: Union[str, torch.device, None] = None,
        hidden_size: int = 128,
        num_layers: int = 1,
        clip_actions: bool = False,
        clip_mean_actions: bool = False,
        clip_log_std: bool = True,
        min_log_std: float = -20.0,
        max_log_std: float = 2.0,
        initial_log_std: float = -2.0,
        reduction: str = "sum",
    ) -> None:
        Model.__init__(
            self,
            observation_space=observation_space,
            state_space=state_space,
            action_space=action_space,
            device=device,
        )
        RecurrentGaussianMixin.__init__(  # type: ignore[call-arg]
            self,
            clip_actions=clip_actions,
            clip_mean_actions=clip_mean_actions,
            clip_log_std=clip_log_std,
            min_log_std=min_log_std,
            max_log_std=max_log_std,
            reduction=reduction,
        )
        self._init_recurrent_core(
            input_size=self.num_observations,
            hidden_size=hidden_size,
            num_layers=num_layers,
            input_key="observations",
        )
        self.output = torch.nn.Linear(hidden_size, self.num_actions)
        self.log_std_parameter = torch.nn.Parameter(
            torch.full((self.num_actions,), float(initial_log_std))
        )

    def compute(
        self,
        inputs: Mapping[str, Any],
        role: str = "",
    ) -> tuple[torch.Tensor, Mapping[str, Any]]:
        recurrent_output, hidden, had_time = self._forward_recurrent(inputs)
        mean_actions = self.output(recurrent_output)
        if not had_time:
            mean_actions = mean_actions[:, 0]
        else:
            valid = _mask_sequence(
                inputs.get("sequence_mask"),
                batch=mean_actions.shape[0],
                length=mean_actions.shape[1],
                device=mean_actions.device,
                default=True,
            )
            mean_actions = torch.where(
                valid.unsqueeze(-1), mean_actions, torch.zeros_like(mean_actions)
            )
        return mean_actions, {
            "rnn_state": hidden,
            "log_std": self.log_std_parameter,
        }


class RecurrentValueModel(DeterministicMixin, Model, _RecurrentCore):  # type: ignore[misc]
    """GRU value model with the same explicit state contract as the policy."""

    def __init__(
        self,
        *,
        observation_space: Union[int, tuple[int], gymnasium.Space[Any]],
        state_space: Union[int, tuple[int], gymnasium.Space[Any], None] = None,
        action_space: Union[int, tuple[int], gymnasium.Space[Any]],
        device: Union[str, torch.device, None] = None,
        hidden_size: int = 128,
        num_layers: int = 1,
        clip_actions: bool = False,
    ) -> None:
        Model.__init__(
            self,
            observation_space=observation_space,
            state_space=state_space,
            action_space=action_space,
            device=device,
        )
        # A value is not an action; clipping it to action-space limits corrupts returns.
        DeterministicMixin.__init__(self, clip_actions=False, role="value")
        input_size = self.num_states if self.num_states > 0 else self.num_observations
        self._init_recurrent_core(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            input_key="states" if self.num_states > 0 else "observations",
        )
        self.output = torch.nn.Linear(hidden_size, 1)

    def compute(
        self,
        inputs: Mapping[str, Any],
        role: str = "",
    ) -> tuple[torch.Tensor, Mapping[str, Any]]:
        recurrent_output, hidden, had_time = self._forward_recurrent(inputs)
        values = self.output(recurrent_output)
        if not had_time:
            values = values[:, 0]
        else:
            valid = _mask_sequence(
                inputs.get("sequence_mask"),
                batch=values.shape[0],
                length=values.shape[1],
                device=values.device,
                default=True,
            )
            values = torch.where(valid.unsqueeze(-1), values, torch.zeros_like(values))
        return values, {"rnn_state": hidden}


def recurrent_gaussian_model(
    observation_space: Optional[Union[int, tuple[int], gymnasium.Space[Any]]] = None,
    state_space: Optional[Union[int, tuple[int], gymnasium.Space[Any]]] = None,
    action_space: Optional[Union[int, tuple[int], gymnasium.Space[Any]]] = None,
    device: Optional[Union[str, torch.device]] = None,
    **kwargs: Any,
) -> RecurrentGaussianModel | str:
    """SKRL-style model factory for the SRB recurrent policy."""
    return_source = bool(kwargs.pop("return_source", False))
    if return_source:
        return "SRB recurrent Gaussian policy (explicit rnn_state input/output)"
    if observation_space is None or action_space is None:
        raise ValueError("recurrent Gaussian model requires observation/action spaces")
    model_kwargs = {
        "hidden_size": int(kwargs.pop("hidden_size", 128)),
        "num_layers": int(kwargs.pop("num_layers", 1)),
        "clip_actions": bool(kwargs.pop("clip_actions", False)),
        "clip_log_std": bool(kwargs.pop("clip_log_std", True)),
        "min_log_std": float(kwargs.pop("min_log_std", -20.0)),
        "max_log_std": float(kwargs.pop("max_log_std", 2.0)),
        "initial_log_std": float(kwargs.pop("initial_log_std", -2.0)),
        "reduction": str(kwargs.pop("reduction", "sum")),
        "clip_mean_actions": bool(kwargs.pop("clip_mean_actions", False)),
    }
    if kwargs:
        raise TypeError(
            "unsupported recurrent Gaussian model options: " + ", ".join(sorted(kwargs))
        )
    return RecurrentGaussianModel(
        observation_space=observation_space,
        state_space=state_space,
        action_space=action_space,
        device=device,
        **model_kwargs,
    )


def recurrent_value_model(
    observation_space: Optional[Union[int, tuple[int], gymnasium.Space[Any]]] = None,
    state_space: Optional[Union[int, tuple[int], gymnasium.Space[Any]]] = None,
    action_space: Optional[Union[int, tuple[int], gymnasium.Space[Any]]] = None,
    device: Optional[Union[str, torch.device]] = None,
    **kwargs: Any,
) -> RecurrentValueModel | str:
    """SKRL-style model factory for the SRB recurrent value function."""
    return_source = bool(kwargs.pop("return_source", False))
    if return_source:
        return "SRB recurrent value model (explicit rnn_state input/output)"
    if observation_space is None or action_space is None:
        raise ValueError("recurrent value model requires observation/action spaces")
    model_kwargs = {
        "hidden_size": int(kwargs.pop("hidden_size", 128)),
        "num_layers": int(kwargs.pop("num_layers", 1)),
        "clip_actions": bool(kwargs.pop("clip_actions", False)),
    }
    if kwargs:
        raise TypeError(
            "unsupported recurrent value model options: " + ", ".join(sorted(kwargs))
        )
    return RecurrentValueModel(
        observation_space=observation_space,
        state_space=state_space,
        action_space=action_space,
        device=device,
        **model_kwargs,
    )


class RecurrentRolloutBuffer:
    """Bounded per-environment rollout storage with padded sequence sampling."""

    def __init__(
        self,
        memory_size: int,
        num_envs: int = 1,
        device: Union[str, torch.device, None] = None,
        sequence_length: int = 16,
        **_: Any,
    ) -> None:
        if memory_size <= 0:
            raise ValueError("recurrent memory_size must be positive")
        if num_envs <= 0:
            raise ValueError("recurrent num_envs must be positive")
        if sequence_length <= 0:
            raise ValueError("recurrent sequence_length must be positive")
        self.memory_size = int(memory_size)
        self.num_envs = int(num_envs)
        self.device = (
            torch.device(device) if device is not None else torch.device("cpu")
        )
        self.sequence_length = int(sequence_length)
        self._records: list[dict[str, torch.Tensor]] = []

    def __len__(self) -> int:
        return len(self._records)

    @property
    def records(self) -> tuple[dict[str, torch.Tensor], ...]:
        return tuple(self._records)

    def clear(self) -> None:
        self._records.clear()

    def add_samples(self, **samples: torch.Tensor) -> None:
        required = {
            "states",
            "actions",
            "rewards",
            "next_values",
            "terminated",
            "truncated",
            "log_prob",
            "values",
            "policy_hidden",
            "value_hidden",
            "episode_start",
        }
        missing = required.difference(samples)
        if missing:
            raise KeyError(f"recurrent rollout is missing fields: {sorted(missing)}")
        # Preserve old callers while storing SKRL 2.x policy observations.
        if "observations" not in samples:
            samples = {**samples, "observations": samples["states"]}
        record: dict[str, torch.Tensor] = {}
        for name, value in samples.items():
            tensor = torch.as_tensor(value, device=self.device).detach().clone()
            if tensor.ndim == 0 or tensor.shape[0] != self.num_envs:
                raise ValueError(
                    f"recurrent rollout field {name!r} must start with "
                    f"num_envs={self.num_envs}, got {tuple(tensor.shape)}"
                )
            record[name] = tensor
        self._records.append(record)
        if len(self._records) > self.memory_size:
            self._records.pop(0)

    def stack(self, name: str) -> torch.Tensor:
        if not self._records:
            raise ValueError("cannot stack an empty recurrent rollout")
        return torch.stack([record[name] for record in self._records], dim=0)

    def compute_targets(
        self,
        discount_factor: float,
        lambda_coefficient: float,
        *,
        time_limit_bootstrap: bool = False,
    ) -> None:
        """Compute GAE while preserving episode boundaries and timeout policy."""
        rewards = self.stack("rewards")
        values = self.stack("values")
        next_values = self.stack("next_values")
        terminated = self.stack("terminated").bool()
        truncated = self.stack("truncated").bool()
        done = terminated | truncated
        advantages = torch.zeros_like(rewards)
        running = torch.zeros_like(rewards[0])
        for index in range(len(self._records) - 1, -1, -1):
            # SKRL's option already folds timeout value into reward; avoid
            # bootstrapping twice. Keep the mask bool because float ``&`` fails.
            keep = ~terminated[index]
            if time_limit_bootstrap:
                keep = keep & ~truncated[index]
            bootstrap = keep.to(rewards.dtype)
            delta = (
                rewards[index]
                + discount_factor * bootstrap * next_values[index]
                - values[index]
            )
            # SRB autoresets after both termination and truncation. A
            # truncation row therefore starts a new episode, so GAE must not
            # carry its running estimate into the preceding episode.
            running = (
                delta
                + discount_factor
                * lambda_coefficient
                * (~done[index]).to(rewards.dtype)
                * running
            )
            advantages[index] = running
        returns = advantages + values
        valid = torch.ones_like(advantages, dtype=torch.bool)
        flat = advantages[valid]
        if flat.numel() > 1:
            advantages = (advantages - flat.mean()) / (flat.std(unbiased=False) + 1e-8)
        for index, record in enumerate(self._records):
            record["returns"] = returns[index]
            record["advantages"] = advantages[index]

    def sequence_batches(
        self,
        *,
        mini_batches: int = 1,
        sequence_length: int | None = None,
    ) -> list[dict[str, torch.Tensor]]:
        """Return padded, environment-isolated sequences split into minibatches."""
        if not self._records:
            return []
        length = int(
            self.sequence_length if sequence_length is None else sequence_length
        )
        if length <= 0:
            raise ValueError("sequence_length must be positive")
        names = tuple(self._records[0])
        stacked = {name: self.stack(name) for name in names}
        total_steps = stacked["states"].shape[0]
        sequences: list[dict[str, torch.Tensor]] = []
        for env_index in range(self.num_envs):
            for start in range(0, total_steps, length):
                stop = min(start + length, total_steps)
                valid_steps = stop - start
                sequence: dict[str, torch.Tensor] = {}
                for name, tensor in stacked.items():
                    value = tensor[start:stop, env_index]
                    if name in {"policy_hidden", "value_hidden"}:
                        sequence[name] = value[0]
                        continue
                    padded = torch.zeros(
                        (length, *value.shape[1:]),
                        dtype=value.dtype,
                        device=value.device,
                    )
                    padded[:valid_steps] = value
                    sequence[name] = padded
                mask = torch.zeros(length, 1, dtype=torch.bool, device=self.device)
                mask[:valid_steps] = True
                sequence["sequence_mask"] = mask
                sequences.append(sequence)

        if mini_batches <= 0:
            raise ValueError("mini_batches must be positive")
        order = torch.randperm(len(sequences), device=self.device)
        chunks = torch.tensor_split(order, min(mini_batches, len(sequences)))
        batches: list[dict[str, torch.Tensor]] = []
        for chunk in chunks:
            selected = [sequences[int(index)] for index in chunk.tolist()]
            batch: dict[str, torch.Tensor] = {}
            for name in selected[0]:
                if name in {"policy_hidden", "value_hidden"}:
                    batch[name] = torch.stack([item[name] for item in selected], dim=1)
                else:
                    batch[name] = torch.stack([item[name] for item in selected], dim=0)
            batches.append(batch)
        return batches


__all__ = [
    "RecurrentGaussianMixin",
    "RecurrentGaussianModel",
    "RecurrentRolloutBuffer",
    "RecurrentValueModel",
    "recurrent_gaussian_model",
    "recurrent_value_model",
]
