from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch

from srb._typing.step_return import StepReturn
from srb.core.reward.outcome import RewardOutcomeRuntime

from .step_return import scrub_non_finite_step_return, validate_step_return


@dataclass(frozen=True)
class StepOutcomeCore:
    step_return: StepReturn
    reward: torch.Tensor
    terminated: torch.Tensor
    truncated: torch.Tensor
    info: dict[str, Any]
    extras: dict[str, Any]
    reset_mask: torch.Tensor
    reset_ids: torch.Tensor
    invalid_envs: torch.Tensor
    invalid_keys: tuple[str, ...]


def process_step_outcome_core(
    step_return: StepReturn,
    *,
    num_envs: int,
    device: torch.device | str,
    include_reward_terms: bool = False,
    reward_runtime: RewardOutcomeRuntime | None = None,
) -> StepOutcomeCore:
    # Scrub validates the original structure first, so malformed components
    # are reported before reductions inspect them.
    step_return, invalid_envs, invalid_keys = scrub_non_finite_step_return(
        step_return,
        num_envs=num_envs,
        device=device,
    )
    # Finiteness was checked by the scrub; retain device and structural checks.
    # The structural pass reads metadata only and does not synchronize.
    validate_step_return(
        step_return, num_envs=num_envs, device=device, assume_finite=True
    )

    reward_runtime = reward_runtime or RewardOutcomeRuntime()
    computed_reward = reward_runtime.compute(
        step_return.reward,
        include_terms=include_reward_terms,
    )
    terminated = step_return.termination
    truncated = step_return.truncation
    reset_mask = terminated | truncated
    reset_ids = torch.nonzero(reset_mask, as_tuple=False).flatten()

    info = dict(step_return.info or {})
    extras: dict[str, Any] = computed_reward.extras
    if step_return.info:
        extras.update(step_return.info)
    invalid_keys_tuple = tuple(invalid_keys)
    if invalid_keys_tuple:
        info["invalid_keys"] = invalid_keys_tuple
        info["invalid_envs"] = invalid_envs
        # Preserve scrub diagnostics in the public step extras.
        extras["invalid_keys"] = invalid_keys_tuple
        extras["invalid_envs"] = invalid_envs

    return StepOutcomeCore(
        step_return=step_return,
        reward=computed_reward.reward,
        terminated=terminated,
        truncated=truncated,
        info=info,
        extras=extras,
        reset_mask=reset_mask,
        reset_ids=reset_ids,
        invalid_envs=invalid_envs,
        invalid_keys=invalid_keys_tuple,
    )


__all__ = [
    "StepOutcomeCore",
    "process_step_outcome_core",
]
