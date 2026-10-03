"""Shared callable policy contract; framework loading stays adapter-local."""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray


class PolicyFunction(Protocol):
    """Callable inference surface shared by evaluation and collection."""

    def __call__(self, observation: Any, /) -> Any: ...


class EpisodeStarter(Protocol):
    """Reset policy-local state for all or selected environments."""

    def __call__(self, done_mask: Any | None = None) -> None: ...


type LoadedPolicy = tuple[PolicyFunction, EpisodeStarter, bool]


def policy_action_to_numpy(action: Any) -> NDArray[Any]:
    """Normalize a framework action and remove only a singleton batch axis."""
    if hasattr(action, "detach"):
        action = action.detach().cpu().numpy()
    action_array = np.asarray(action)
    return (
        action_array[0]
        if action_array.ndim >= 2 and action_array.shape[0] == 1
        else action_array
    )


__all__ = [
    "EpisodeStarter",
    "LoadedPolicy",
    "PolicyFunction",
    "policy_action_to_numpy",
]
