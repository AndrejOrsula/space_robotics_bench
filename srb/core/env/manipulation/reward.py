"""Base reward configuration for manipulation tasks.

This module defines the common penalty terms shared by all manipulation tasks.
Concrete tasks extend ``ManipulationRewardCfg`` with task-specific reward terms.
"""

from srb.core.reward import (
    BaseRewardCfg,
    LinearRewardTerm,
    ThresholdRewardTerm,
)


class ManipulationRewardCfg(BaseRewardCfg):
    """Reward terms common to all manipulation environments.

    Every manipulation task inherits these penalties.  Set ``weight=0.0`` on
    any term to disable it for a specific task.
    """

    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-0.1, limit=4.0)
    """Discourages rapid changes in commanded actions.
    Kept small relative to task rewards to avoid paralysing the agent."""

    penalty_joint_torque: LinearRewardTerm = LinearRewardTerm(
        weight=-0.000025, limit=4.0
    )
    """Penalises large joint torques to encourage energy-efficient motion."""

    penalty_joint_acceleration: LinearRewardTerm = LinearRewardTerm(
        weight=-0.0005, limit=4.0
    )
    """Penalises large joint accelerations to encourage smooth motion."""

    penalty_undesired_robot_contacts: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=-0.5, threshold=10.0, operator=">"
    )
    """Penalises collisions with environment geometry (non-task contacts)."""
