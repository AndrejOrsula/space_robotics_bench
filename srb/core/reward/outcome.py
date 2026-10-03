"""Reward outcome assembly for environment runtimes."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import torch

from .cfg import BaseRewardCfg, RewardNormalization, RewardNormalizationCfg

RewardTerms = Mapping[str, torch.Tensor]


def sum_reward_terms(reward_terms: RewardTerms) -> torch.Tensor:
    """Sum a non-empty reward-term mapping into a batched reward tensor."""
    if not reward_terms:
        raise ValueError("Cannot sum an empty reward mapping")
    return torch.stack(tuple(reward_terms.values()), dim=0).sum(dim=0)


@dataclass(frozen=True)
class RewardOutcome:
    raw_total: torch.Tensor
    reward: torch.Tensor
    extras: dict[str, Any]


@dataclass(frozen=True)
class RewardOutcomeRuntime:
    """Build normalized environment rewards and reward-related extras."""

    normalization: RewardNormalizationCfg | None = None
    original_mode: RewardNormalization | None = None

    @classmethod
    def from_cfg(
        cls,
        reward_cfg: object | None,
        *,
        max_episode_steps: int,
        is_finite_horizon: bool,
    ) -> "RewardOutcomeRuntime":
        if not isinstance(reward_cfg, BaseRewardCfg):
            return cls()
        if reward_cfg.normalize == RewardNormalization.NONE:
            return cls()
        normalization = reward_cfg.into_normalization_cfg(
            max_episode_steps=max_episode_steps,
            is_finite_horizon=is_finite_horizon,
        )
        if normalization.scale == 0.0:
            from srb.utils import logging

            logging.warning(
                "Reward normalization scale is 0 (all reward-term bounds are zero), "
                "so normalized rewards are 0 and the agent gets no learning signal. "
                "Use non-trivial reward terms, or set "
                "normalize=RewardNormalization.NONE if intentional."
            )
        return cls(
            normalization=normalization,
            original_mode=reward_cfg.normalize,
        )

    @property
    def enabled(self) -> bool:
        return self.normalization is not None

    def describe_normalization(self) -> str | None:
        if self.normalization is None or self.original_mode is None:
            return None
        return self.normalization.describe(self.original_mode)

    def compute(
        self,
        reward_terms: RewardTerms,
        *,
        include_terms: bool = False,
    ) -> RewardOutcome:
        raw_total = sum_reward_terms(reward_terms)
        reward = self.normalize(raw_total)
        extras: dict[str, Any] = {}
        if include_terms:
            extras["reward_terms"] = reward_terms
            if self.normalization is not None:
                extras["reward_terms_normalized"] = self.normalize_terms(reward_terms)
        return RewardOutcome(raw_total=raw_total, reward=reward, extras=extras)

    def normalize_terms(self, reward_terms: RewardTerms) -> dict[str, torch.Tensor]:
        return {name: self.normalize(value) for name, value in reward_terms.items()}

    def normalize(self, reward: torch.Tensor) -> torch.Tensor:
        if self.normalization is None:
            return reward
        if self.normalization.scale == 0.0:
            return torch.zeros_like(reward)
        return reward / float(self.normalization.scale)


__all__ = [
    "RewardOutcome",
    "RewardOutcomeRuntime",
    "RewardTerms",
    "sum_reward_terms",
]
