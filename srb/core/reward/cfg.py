"""Base configuration and normalization for task reward terms."""

from enum import Enum
from functools import cached_property
from typing import Iterable

from pydantic import (
    BaseModel,
    ConfigDict,
    NonNegativeFloat,
    NonPositiveFloat,
    PositiveInt,
)

from srb.core.reward.terms import RewardTerm
from srb.utils.config_introspection import (
    get_config_item,
    iter_config_items_by_type,
    iter_config_values_by_type,
)


class RewardNormalization(str, Enum):
    """Mode for normalizing rewards."""

    NONE = "none"
    """No normalization; raw summed rewards are returned as-is."""

    PER_STEP = "per_step"
    """Normalize each step's reward to the per-step achievable range.

    Divides the raw sum by ``max(best_per_step, |worst_per_step|)``,
    so a perfect step yields +1 and the worst step yields -1 (approximately).
    """

    EPISODE = "episode"
    """Normalize rewards relative to the full episode return range.

    Divides by ``max(best_per_step, |worst_per_step|) * max_episode_steps``,
    so that a perfect episode accumulates to +1 and the worst episode to -1.
    """

    AUTO = "auto"
    """Resolve to ``EPISODE`` for finite-horizon tasks, ``PER_STEP`` otherwise.

    Eagerly resolved at :meth:`BaseRewardCfg.into_normalization_cfg` time using
    the env's ``is_finite_horizon`` flag, so downstream consumers only see a
    concrete mode.
    """


class BaseRewardCfg(BaseModel):
    # Freezing declared fields preserves the bounds snapshot;
    # Hydra can still override fields on each nested RewardTerm.
    model_config = ConfigDict(frozen=True)

    normalize: RewardNormalization = RewardNormalization.AUTO
    normalize_offset_per_step: tuple[NonPositiveFloat, NonNegativeFloat] = (0.0, 0.0)

    def get_bounds(self) -> tuple[float, float]:
        worst_per_step = 0.0
        best_per_step = 0.0

        for term in self.values():
            # Signed bounds preserve both sides of sign-changing terms.
            term_worst, term_best = term.reward_bounds
            worst_per_step += term_worst
            best_per_step += term_best

        offset_per_step_worst, offset_per_step_best = self.normalize_offset_per_step
        worst_per_step += offset_per_step_worst
        best_per_step += offset_per_step_best

        return worst_per_step, best_per_step

    def into_normalization_cfg(
        self, max_episode_steps: int, is_finite_horizon: bool
    ) -> "RewardNormalizationCfg":
        mode = self.normalize
        if mode == RewardNormalization.AUTO:
            mode = (
                RewardNormalization.EPISODE
                if is_finite_horizon
                else RewardNormalization.PER_STEP
            )
        return RewardNormalizationCfg.from_cfg(self, max_episode_steps, mode)

    def items(self) -> Iterable[tuple[str, RewardTerm]]:
        yield from iter_config_items_by_type(self, RewardTerm)

    def keys(self) -> Iterable[str]:
        for term_name, _ in self.items():
            yield term_name

    def values(self) -> Iterable[RewardTerm]:
        yield from iter_config_values_by_type(self, RewardTerm)

    def __len__(self) -> int:
        return sum(1 for _ in self.items())

    def __getitem__(self, key: str) -> RewardTerm:
        term = get_config_item(self, key)
        if not isinstance(term, RewardTerm):
            raise KeyError(f"{key} is not a RewardTerm")
        return term

    def __setitem__(self, key: str, value: RewardTerm) -> None:
        if not isinstance(value, RewardTerm):
            raise ValueError(f"Value must be a RewardTerm, got {type(value)}")
        # Runtime-added terms are allowed without permitting reassignment of
        # declared fields, which would invalidate snapshotted bounds.
        # ``vars(self)`` keeps injected terms visible to keys(), values(), items().
        object.__setattr__(self, key, value)


class RewardNormalizationCfg(BaseModel):
    model_config: ConfigDict = ConfigDict(ignored_types=(cached_property,), frozen=True)

    worst_per_step: NonPositiveFloat
    best_per_step: NonNegativeFloat
    max_episode_steps: PositiveInt
    mode: RewardNormalization

    @classmethod
    def from_cfg(
        cls,
        reward_cfg: BaseRewardCfg,
        max_episode_steps: int,
        mode: RewardNormalization,
    ) -> "RewardNormalizationCfg":
        worst_per_step, best_per_step = reward_cfg.get_bounds()
        return cls(
            worst_per_step=worst_per_step,
            best_per_step=best_per_step,
            max_episode_steps=max_episode_steps,
            mode=mode,
        )

    @cached_property
    def scale(self) -> float:
        base = max(self.best_per_step, abs(self.worst_per_step))
        if self.mode == RewardNormalization.EPISODE:
            return base * self.max_episode_steps
        return base  # PER_STEP

    def _normalized(self, value: float) -> float:
        """Divide by ``scale`` with a zero-scale guard (all-zero bounds)."""
        if self.scale == 0.0:
            return 0.0
        return value / self.scale

    @cached_property
    def worst_per_episode(self) -> float:
        return self.worst_per_step * self.max_episode_steps

    @cached_property
    def worst_per_episode_normalized(self) -> float:
        return self._normalized(self.worst_per_episode)

    @cached_property
    def best_per_episode(self) -> float:
        return self.best_per_step * self.max_episode_steps

    @cached_property
    def best_per_episode_normalized(self) -> float:
        return self._normalized(self.best_per_episode)

    @cached_property
    def worst_per_step_normalized(self) -> float:
        return self._normalized(self.worst_per_step)

    @cached_property
    def best_per_step_normalized(self) -> float:
        return self._normalized(self.best_per_step)

    def normalize(self, reward: float) -> float:
        return self._normalized(reward)

    def describe(self, orig_mode: RewardNormalization) -> str:
        mode_label = (
            f"auto -> {self.mode.value}"
            if orig_mode == RewardNormalization.AUTO
            else self.mode.value
        )
        if self.mode == RewardNormalization.PER_STEP:
            return (
                f"Enabled reward normalization ({mode_label}) with scale "
                f"{self.scale:.4f} that maps achievable per-step rewards "
                f"[{self.worst_per_step:.2f}, {self.best_per_step:.2f}] to "
                f"[{self.worst_per_step_normalized:.2f}, "
                f"{self.best_per_step_normalized:.2f}]."
            )
        return (
            f"Enabled reward normalization ({mode_label}) with scale "
            f"{self.scale:.4f} that maps achievable episode returns "
            f"[{self.worst_per_episode:.2f}, {self.best_per_episode:.2f}] to "
            f"[{self.worst_per_episode_normalized:.2f}, "
            f"{self.best_per_episode_normalized:.2f}]."
        )
