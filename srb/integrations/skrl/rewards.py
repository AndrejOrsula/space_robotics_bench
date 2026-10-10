"""Stateless reward scaling shared by native online and demonstration replay."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class NativeRewardScale:
    """Apply the runner's declared constant scale without timestep dependence."""

    scale: float

    def __call__(self, rewards: Any, *args: Any, **kwargs: Any) -> Any:
        return rewards * self.scale
