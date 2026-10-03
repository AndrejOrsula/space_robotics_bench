from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol

import torch

from srb._typing.step_return import StepReturn

from .observation_layout import ObservationLayout
from .step_return import flatten_observations


class ObservationBuilder(Protocol):
    """Build a flat observation, optionally incorporating event observations."""

    def __call__(
        self,
        step_return: StepReturn,
        *,
        event_observations: dict[str, torch.Tensor] | None = None,
    ) -> dict[str, torch.Tensor]: ...


def build_reset_observation(
    step_return: StepReturn,
    *,
    observation_layout: ObservationLayout | None = None,
) -> dict[str, torch.Tensor]:
    return (
        observation_layout.flatten(step_return.observation)
        if observation_layout is not None
        else flatten_observations(step_return.observation)
    )


def build_step_observation(
    step_return: StepReturn,
    *,
    observation_layout: ObservationLayout | None = None,
    observation_builder: ObservationBuilder | None = None,
) -> dict[str, torch.Tensor]:
    if observation_builder is not None and observation_layout is not None:
        raise ValueError(
            "Use either observation_builder or observation_layout, not both"
        )
    if observation_builder is not None:
        return observation_builder(step_return)
    if observation_layout is not None:
        return observation_layout.flatten(step_return.observation)
    return flatten_observations(step_return.observation)


def final_observation(
    observation: dict[str, torch.Tensor],
    reset_ids: torch.Tensor,
) -> dict[str, torch.Tensor] | None:
    if reset_ids.numel() == 0:
        return None
    return {
        key: value[reset_ids.to(device=value.device)].clone()
        for key, value in observation.items()
    }


def splice_observation_rows(
    observation: dict[str, torch.Tensor],
    fresh: dict[str, torch.Tensor],
    env_ids: torch.Tensor,
) -> dict[str, torch.Tensor]:
    """Return a copy of ``observation`` with rows in ``env_ids`` from ``fresh``."""
    if env_ids.numel() == 0:
        return observation
    if set(observation) != set(fresh):
        missing = sorted(set(observation) - set(fresh))
        extra = sorted(set(fresh) - set(observation))
        raise ValueError(
            "post-reset observation keys differ from the step observation "
            f"(missing={missing}, extra={extra})"
        )
    spliced: dict[str, torch.Tensor] = {}
    for key, value in observation.items():
        replacement = fresh[key]
        if tuple(replacement.shape) != tuple(value.shape):
            raise ValueError(
                f"post-reset observation {key!r} has shape "
                f"{tuple(replacement.shape)}, expected {tuple(value.shape)}"
            )
        merged = value.clone()
        rows = env_ids.to(device=merged.device)
        merged[rows] = replacement[rows].to(device=merged.device, dtype=merged.dtype)
        spliced[key] = merged
    return spliced


def splice_nested_rows(base: Any, fresh: Any, env_ids: torch.Tensor) -> Any:
    """Copy a nested observation mapping, replacing selected tensor rows."""
    if isinstance(base, Mapping):
        if not isinstance(fresh, Mapping) or set(base) != set(fresh):
            raise ValueError("post-reset observation structure differs")
        return {key: splice_nested_rows(base[key], fresh[key], env_ids) for key in base}
    if torch.is_tensor(base):
        if not torch.is_tensor(fresh) or tuple(fresh.shape) != tuple(base.shape):
            raise ValueError("post-reset observation tensor shape differs")
        merged = base.clone()
        rows = env_ids.to(device=merged.device)
        merged[rows] = fresh[rows].to(device=merged.device, dtype=merged.dtype)
        return merged
    raise TypeError(f"unsupported observation leaf: {type(base).__name__}")


__all__ = [
    "ObservationBuilder",
    "build_reset_observation",
    "build_step_observation",
    "final_observation",
    "splice_nested_rows",
    "splice_observation_rows",
]
