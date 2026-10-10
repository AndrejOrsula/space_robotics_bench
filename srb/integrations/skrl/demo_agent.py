"""Shared replay checkpoints and terminal targets for SKRL agents."""

from __future__ import annotations

import dataclasses
import datetime
import os
from collections.abc import Mapping
from typing import Any

import torch
from skrl.agents.torch import Agent

from srb.integrations.skrl.transition import (
    terminal_final_states,
    terminal_next_states,
)


def _transition_tensor_names(agent: Any) -> list[str]:
    """Return the native transition order, including SRB timeout flags."""

    memory = getattr(agent, "memory", None)
    if memory is None:
        raise RuntimeError("SKRL off-policy updates require an initialized memory")
    available = getattr(memory, "tensors", None)
    if not isinstance(available, Mapping):
        available = set(memory.get_tensor_names())
    names = [
        "observations",
        "states",
        "actions",
        "rewards",
        "next_observations",
        "next_states",
        "terminated",
    ]
    if "truncated" in available:
        names.append("truncated")
    elif "truncated" in getattr(agent, "_tensors_names", ()):
        names.append("truncated")
    return [name for name in names if name in available]


def sample_transition_batch(agent: Any) -> tuple[torch.Tensor, ...]:
    """Sample observations/states and SRB masks from an off-policy batch.

    SKRL 2.x separates observations and states and omits truncation flags;
    the SRB mixin adds the flag. Older memories without it imply ``False``.
    """

    memory = getattr(agent, "memory", None)
    if memory is None:
        raise RuntimeError("SKRL off-policy updates require an initialized memory")
    names = _transition_tensor_names(agent)
    batch_size = int(getattr(getattr(agent, "cfg", None), "batch_size", 0))
    if batch_size <= 0:
        raise RuntimeError("SKRL off-policy batch size must be positive")
    values = memory.sample(names=names, batch_size=batch_size)[0]
    sampled = dict(zip(names, values, strict=True))
    observations = sampled.get("observations")
    states = sampled.get("states", observations)
    next_observations = sampled.get("next_observations")
    next_states = sampled.get("next_states", next_observations)
    if next_observations is None:
        next_observations = next_states
    if states is None or next_states is None or observations is None:
        raise RuntimeError(
            "SKRL memory is missing observation/state transition tensors"
        )
    truncated = sampled.get("truncated")
    if truncated is None:
        truncated = torch.zeros_like(sampled["terminated"], dtype=torch.bool)
    return (
        observations,
        states,
        sampled["actions"],
        sampled["rewards"],
        next_observations,
        next_states,
        sampled["terminated"],
        truncated,
    )


def preprocess_training_batch(preprocessor: Any, values: torch.Tensor) -> Any:
    """Normalize replay data without fitting unbiased variance to one row.

    SKRL's ``RunningStandardScaler`` yields NaNs for a singleton with
    ``train=True``. Duplicate that row to keep statistics finite; real batches
    retain the usual training update.
    """

    if values.ndim == 0 or values.shape[0] > 1:
        return preprocessor(values, train=True)
    # Give unbiased variance two identical samples, then keep one row.
    duplicated = torch.cat((values, values), dim=0)
    return preprocessor(duplicated, train=True)[:1]


class DemoReplayAgentMixin(Agent):
    """Add SRB timeout handling and optional replay persistence."""

    def init(self, *args: Any, trainer_cfg: Any = None) -> None:
        """Initialize native memory and add the SRB truncation tensor."""

        if args:
            if len(args) != 1 or trainer_cfg is not None:
                raise TypeError("SKRL agent.init accepts one trainer_cfg value")
            trainer_cfg = args[0]
        if isinstance(trainer_cfg, Mapping):
            # SKRL 2.1 calls dataclasses.asdict; convert legacy mappings here.
            keys = tuple(trainer_cfg)
            if any(not isinstance(key, str) or not key.isidentifier() for key in keys):
                raise TypeError("SKRL trainer_cfg mapping has invalid field names")
            cfg_type = dataclasses.make_dataclass(
                "_SrbTrainerCfg", [(key, object) for key in keys]
            )
            trainer_cfg = cfg_type(**dict(trainer_cfg))
        super().init(trainer_cfg=trainer_cfg)
        memory = getattr(self, "memory", None)
        if memory is None:
            return
        if "truncated" not in memory.get_tensor_names():
            memory.create_tensor(name="truncated", size=1, dtype=torch.bool)
        names = list(getattr(self, "_tensors_names", ()))
        if "truncated" not in names:
            names.append("truncated")
        self._tensors_names = names

    def enable_demo_replay_checkpoint(self, binding: Mapping[str, Any]) -> Any:
        """Enable the opt-in native replay checkpoint module."""

        from srb.integrations.skrl.replay import (
            DemoReplayError,
            install_demo_replay_checkpoint,
        )

        if bool(getattr(self.cfg.experiment, "store_separately", False)):
            raise DemoReplayError(
                "SKRL demo replay requires a combined agent checkpoint; "
                "experiment.store_separately is unsupported"
            )
        self._srb_training_cursor = int(getattr(self, "_srb_training_cursor", 0))
        return install_demo_replay_checkpoint(self, binding)

    def prefill_demo_replay(self, payload: Any, *, replace: bool = False) -> None:
        """Insert an already validated demonstration payload transactionally."""

        from srb.integrations.skrl.replay import prefill_demo_replay

        prefill_demo_replay(self, payload, replace=replace)

    @property
    def demo_replay_restored(self) -> bool:
        """Whether the most recent load restored the opt-in replay state."""

        return bool(getattr(self, "_srb_demo_replay_restored", False))

    def post_interaction(self, *, timestep: int, timesteps: int) -> None:
        """Record the next environment timestep before SKRL checkpointing."""

        if getattr(self, "_srb_demo_replay_checkpoint", None) is not None:
            self._srb_training_cursor = int(timestep) + 1
        super().post_interaction(timestep=timestep, timesteps=timesteps)

    def save(self, path: str) -> None:
        """Save native modules, including replay only for demo runs."""

        if getattr(self, "_srb_demo_replay_checkpoint", None) is None:
            super().save(path)
            return
        from srb.integrations.skrl.replay import save_checkpoint_modules

        save_checkpoint_modules(self, path)

    def load(self, path: str, *, allow_replay: bool = False) -> None:
        """Load native modules with explicit demo replay policy."""

        from srb.integrations.skrl.replay import load_checkpoint_modules

        if (
            not allow_replay
            and getattr(self, "_srb_demo_replay_checkpoint", None) is None
        ):
            # Keep ordinary weights-only loads; restore replay only on request.
            super().load(path)
            return
        load_checkpoint_modules(self, path, allow_replay=allow_replay)

    def write_checkpoint(self, *, timestep: int, timesteps: int) -> None:
        """Use the bounded combined writer for automatic demo checkpoints."""

        if getattr(self, "_srb_demo_replay_checkpoint", None) is None:
            super().write_checkpoint(timestep=timestep, timesteps=timesteps)
            return

        from srb.integrations.skrl.replay import (
            save_checkpoint_modules,
            save_checkpoint_values,
        )

        tag = str(
            timestep
            if timestep is not None
            else datetime.datetime.now().strftime("%y-%m-%d_%H-%M-%S-%f")
        )
        checkpoint_dir = os.path.join(self.experiment_dir, "checkpoints")
        os.makedirs(checkpoint_dir, exist_ok=True)
        save_checkpoint_modules(self, os.path.join(checkpoint_dir, f"agent_{tag}.pt"))

        best = self.checkpoint_best_modules
        if best["modules"] and not best["saved"]:
            save_checkpoint_values(
                best["modules"],
                os.path.join(checkpoint_dir, "best_agent.pt"),
            )
            best["saved"] = True

    def _update(self, timestep: int, timesteps: int) -> None:
        """Compatibility delegate for the former private update entrypoint."""

        self.update(timestep=timestep, timesteps=timesteps)

    @staticmethod
    def target_values(
        rewards: torch.Tensor,
        target_q_values: torch.Tensor,
        terminated: torch.Tensor,
        discount_factor: float,
    ) -> torch.Tensor:
        """Bootstrap through time-limit truncation, but not MDP termination."""

        return rewards + discount_factor * terminated.logical_not() * target_q_values

    def record_transition(self, *args: Any, **kwargs: Any) -> None:
        """Record a 2.x transition with terminal snapshots and timeout flags."""

        if args:
            if len(args) != 9:
                raise TypeError(
                    "SKRL record_transition accepts keyword-only 2.x transition fields"
                )
            (
                states,
                actions,
                rewards,
                next_states,
                terminated,
                truncated,
                infos,
                timestep,
                timesteps,
            ) = args
            # Preserve only the historical positional SRB call shape.
            observations = states
            next_observations = next_states
        else:
            if "observations" not in kwargs and "states" in kwargs:
                kwargs["observations"] = kwargs["states"]
            if "next_observations" not in kwargs and "next_states" in kwargs:
                kwargs["next_observations"] = kwargs["next_states"]
            required = {
                "observations",
                "states",
                "actions",
                "rewards",
                "next_observations",
                "next_states",
                "terminated",
                "truncated",
                "infos",
                "timestep",
                "timesteps",
            }
            missing = required.difference(kwargs)
            if missing:
                raise TypeError(
                    "SKRL record_transition missing fields: "
                    + ", ".join(sorted(missing))
                )
            observations = kwargs.pop("observations")
            states = kwargs.pop("states")
            actions = kwargs.pop("actions")
            rewards = kwargs.pop("rewards")
            next_observations = kwargs.pop("next_observations")
            next_states = kwargs.pop("next_states")
            terminated = kwargs.pop("terminated")
            truncated = kwargs.pop("truncated")
            infos = kwargs.pop("infos")
            timestep = kwargs.pop("timestep")
            timesteps = kwargs.pop("timesteps")
            if kwargs:
                raise TypeError(
                    "SKRL record_transition received unexpected fields: "
                    + ", ".join(sorted(kwargs))
                )

        if (states is None) != (next_states is None):
            raise ValueError(
                "SKRL state transitions must supply both states and next_states"
            )
        state_stream_supplied = states is not None
        if states is None:
            states = observations
            next_states = next_observations

        done = torch.as_tensor(
            terminated, device=next_observations.device, dtype=torch.bool
        ).reshape(-1)
        done = done | torch.as_tensor(
            truncated, device=next_observations.device, dtype=torch.bool
        ).reshape(-1)
        next_observations = terminal_next_states(next_observations, infos, done)
        if state_stream_supplied:
            next_states = terminal_final_states(next_states, infos, done)
        else:
            next_states = next_observations

        # Run provider accounting; the atomic write below adds SRB timeout data.
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
            return

        shaper = getattr(self.cfg, "rewards_shaper", None)
        if shaper is not None:
            rewards = shaper(rewards, timestep, timesteps)
        samples = {
            "observations": observations,
            "states": states,
            "actions": actions,
            "rewards": rewards,
            "next_observations": next_observations,
            "next_states": next_states,
            "terminated": terminated,
            "truncated": truncated,
        }
        available = getattr(self.memory, "tensors", None)
        if not isinstance(available, Mapping):
            available = set(self.memory.get_tensor_names())
        self.memory.add_samples(
            **{
                name: value
                for name, value in samples.items()
                if name in available and value is not None
            }
        )


__all__ = ["DemoReplayAgentMixin", "sample_transition_batch"]
