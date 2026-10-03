"""Bounded reward terms and episode-length-aware aggregation."""

from __future__ import annotations

from functools import cached_property
from typing import Any

import torch
from pydantic import NonNegativeFloat, PositiveFloat, model_validator

from srb.core.reward.terms import RewardTerm

__all__ = ["TukeyRewardTerm", "RewardNormalizer"]

TERMINAL_PREFIX = "terminal_"


class TukeyRewardTerm(RewardTerm):
    """Flat within ``sweet_spot_m``, then C¹ bisquare falloff to zero at tolerance."""

    sweet_spot_m: NonNegativeFloat = 0.05
    tolerance_m: PositiveFloat = 0.2
    limit: float | None = None

    @model_validator(mode="after")
    def _check_tolerance_exceeds_sweet_spot(self) -> "TukeyRewardTerm":
        if self.tolerance_m <= self.sweet_spot_m:
            raise ValueError(
                f"tolerance_m ({self.tolerance_m}) must exceed sweet_spot_m "
                f"({self.sweet_spot_m})"
            )
        return self

    @cached_property
    def reward_limit_magnitude(self) -> float:
        # The bisquare peaks at 1, so outputs never exceed |weight| whatever the limit.
        if self.limit is None:
            return abs(self.weight)
        return min(abs(self.weight), abs(self.limit))

    @cached_property
    def reward_limit(self) -> float:
        mag = self.reward_limit_magnitude
        return mag if self.weight >= 0.0 else -mag

    @cached_property
    def term_args(self) -> tuple[Any, ...]:
        return (self.sweet_spot_m, self.tolerance_m, self.reward_limit_magnitude)

    @staticmethod
    def term(
        value: torch.Tensor,
        weight: float,
        sweet_spot_m: float,
        tolerance_m: float,
        limit_magnitude: float,
    ) -> torch.Tensor:
        abs_x = value.abs()
        span = tolerance_m - sweet_spot_m
        u = ((abs_x - sweet_spot_m) / span).clamp(min=0.0, max=1.0)
        r = (1.0 - u * u) ** 2
        out = weight * r
        return out.clamp(min=-limit_magnitude, max=limit_magnitude)


class RewardNormalizer:
    """Scale dense terms by horizon length; pass through ``terminal_`` terms."""

    def __init__(self, max_episode_steps: int, terms: dict) -> None:
        self.max_episode_steps = int(max_episode_steps)
        if self.max_episode_steps <= 0:
            raise ValueError(
                f"max_episode_steps must be positive, got {max_episode_steps}"
            )
        self.terms = dict(terms)

    @staticmethod
    def _is_terminal(name: str) -> bool:
        return name.startswith(TERMINAL_PREFIX)

    def compute(
        self, values: dict[str, torch.Tensor]
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Scale and sum registered term values; reject empty or unknown terms."""
        if not values:
            raise ValueError(
                "RewardNormalizer.compute requires at least one term value"
            )
        unknown = set(values) - set(self.terms)
        if unknown:
            raise KeyError(
                f"Values contain unknown terms not registered with the normalizer: {sorted(unknown)}"
            )
        per_term: dict[str, torch.Tensor] = {}
        total: torch.Tensor | None = None
        for name, v in values.items():
            scaled = v if self._is_terminal(name) else v / float(self.max_episode_steps)
            per_term[name] = scaled
            total = scaled if total is None else total + scaled
        return total, per_term  # type: ignore[return-value]
