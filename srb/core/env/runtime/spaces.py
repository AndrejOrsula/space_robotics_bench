from __future__ import annotations

import gymnasium

from srb._typing.step_return import StepReturn

from .action_layout import ActionLayout
from .observation_layout import _TORCH_TO_NUMPY, ObservationLayout
from .validation import validate_positive_int


def build_action_spaces(
    action_manager: object | None, *, num_envs: int
) -> tuple[gymnasium.spaces.Box, gymnasium.Space]:
    return ActionLayout.from_action_manager(action_manager).build_spaces(
        num_envs=num_envs
    )


def build_observation_spaces(
    step_return: StepReturn, *, num_envs: int
) -> tuple[gymnasium.spaces.Dict, gymnasium.Space]:
    return ObservationLayout.from_step_return(
        step_return,
        num_envs=validate_positive_int(num_envs, name="num_envs"),
    ).build_spaces()


__all__ = [
    "_TORCH_TO_NUMPY",
    "build_action_spaces",
    "build_observation_spaces",
]
