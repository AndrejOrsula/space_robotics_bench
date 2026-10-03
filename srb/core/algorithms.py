"""Supported learning-algorithm identifiers shared by core, experiments, and CLI."""

from __future__ import annotations

from enum import Enum, auto

from typing_extensions import Self


class LowercaseNameEnum(str, Enum):
    """Base for enums with lowercase string conversion and exact name parsing."""

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class StrEnum(LowercaseNameEnum):
    """Base for identifier enums with lowercase string conversion."""

    @classmethod
    def from_str(cls, value: str) -> Self | None:
        if not isinstance(value, str):
            return None
        normalized = value.strip().upper()
        return next((variant for variant in cls if normalized == variant.name), None)


class SupportedRLAlgo(StrEnum):
    """Reinforcement learning algorithms (online, on/off-policy)."""

    # DreamerV3
    DREAMER = auto()

    # TD-MPC2
    TDMPC2 = auto()

    # Stable-Baselines3
    SB3_A2C = auto()
    SB3_ARS = auto()
    SB3_CROSSQ = auto()
    SB3_DDPG = auto()
    SB3_PPO = auto()
    SB3_PPO_LSTM = auto()
    SB3_SAC = auto()
    SB3_TD3 = auto()
    SB3_TQC = auto()
    SB3_TRPO = auto()

    # SBX
    SBX_CROSSQ = auto()
    SBX_DDPG = auto()
    SBX_PPO = auto()
    SBX_SAC = auto()
    SBX_TD3 = auto()
    SBX_TQC = auto()

    # rsl_rl
    RSL_RL_PPO = auto()

    # skrl
    SKRL_A2C = auto()
    SKRL_DDPG = auto()
    SKRL_PPO = auto()
    SKRL_PPO_RNN = auto()
    SKRL_RPO = auto()
    SKRL_SAC = auto()
    SKRL_TD3 = auto()
    SKRL_TRPO = auto()


class SupportedILAlgo(StrEnum):
    """Imitation learning / offline RL algorithms."""

    ROBOMIMIC_BC = auto()
    ROBOMIMIC_BC_RNN = auto()
    ROBOMIMIC_BC_TRANSFORMER = auto()
    ROBOMIMIC_BCQ = auto()
    ROBOMIMIC_CQL = auto()
    ROBOMIMIC_IQL = auto()
    ROBOMIMIC_TD3_BC = auto()
    ROBOMIMIC_GL = auto()
    ROBOMIMIC_HBC = auto()
    ROBOMIMIC_IRIS = auto()
    ROBOMIMIC_DIFFUSION_POLICY = auto()


def known_algorithm_labels() -> tuple[str, ...]:
    """Return every registered public algorithm label in declaration order."""
    return tuple(
        str(member) for enum in (SupportedRLAlgo, SupportedILAlgo) for member in enum
    )


def canonical_algorithm_label(value: str) -> str:
    """Normalize and validate a registered public algorithm label."""
    if not isinstance(value, str):
        raise ValueError(f"Unknown SRB algorithm label {value!r}")
    candidate = value.strip().lower()
    if candidate not in known_algorithm_labels():
        raise ValueError(f"Unknown SRB algorithm label {value!r}")
    return candidate


__all__ = (
    "LowercaseNameEnum",
    "StrEnum",
    "SupportedILAlgo",
    "SupportedRLAlgo",
    "canonical_algorithm_label",
    "known_algorithm_labels",
)
