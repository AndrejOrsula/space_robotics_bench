"""Base reward configurations for mobile manipulation tasks.

Ground, aerial, and orbital mobile manipulation tasks share action-rate and
undesired-contact penalties.  Concrete tasks extend the appropriate base
with task-specific reward and penalty terms.
"""

from srb.core.env.mobile.reward import MobileRewardCfg
from srb.core.reward import ThresholdRewardTerm


class MobileManipulationRewardCfg(MobileRewardCfg):
    """Reward terms common to every mobile manipulation domain."""

    penalty_undesired_robot_contacts: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=-0.5, threshold=10.0, operator=">"
    )
    """Penalises collisions with environment geometry (non-task contacts)."""


class GroundManipulationRewardCfg(MobileManipulationRewardCfg):
    """Reward terms common to ground mobile manipulation tasks."""


class AerialManipulationRewardCfg(MobileManipulationRewardCfg):
    """Reward terms common to aerial mobile manipulation tasks."""


class OrbitalManipulationRewardCfg(MobileManipulationRewardCfg):
    """Reward terms common to orbital mobile manipulation tasks."""
