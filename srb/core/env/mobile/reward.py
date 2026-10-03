"""Base reward configurations for mobile tasks.

Ground, aerial, and orbital mobile tasks share a small common set of penalties.
Concrete tasks extend the appropriate base with task-specific reward terms.
"""

from srb.core.reward import BaseRewardCfg, LinearRewardTerm


class MobileRewardCfg(BaseRewardCfg):
    """Reward terms common to every mobile domain."""

    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-0.1, limit=4.0)
    """Discourages rapid changes in commanded actions.
    Kept small relative to task rewards to avoid paralysing the agent."""


class GroundMobileRewardCfg(MobileRewardCfg):
    """Reward terms common to ground mobile tasks."""


class AerialMobileRewardCfg(MobileRewardCfg):
    """Reward terms common to aerial mobile tasks."""


class OrbitalMobileRewardCfg(MobileRewardCfg):
    """Reward terms common to orbital mobile tasks."""

    penalty_fuel_consumption: LinearRewardTerm = LinearRewardTerm(
        weight=-1.0, limit=8.0
    )
    """Penalises fuel usage to encourage propellant-efficient manoeuvres.
    Reduced from -2.0 so the agent is not over-penalised for necessary movement."""
