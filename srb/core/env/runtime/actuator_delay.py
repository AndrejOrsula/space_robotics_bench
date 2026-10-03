"""Keep delayed actuators at the default pose during reset.

Isaac Lab resets joint targets to zero, then writes them before the first
action. Delayed actuators record that write and replay it later, which can
apply a large unintended torque. Restore default joint targets first so the
recorded reset write preserves the pose.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

import torch

__all__ = ["hold_default_joint_targets"]


def hold_default_joint_targets(
    articulations: Iterable[Any], env_ids: torch.Tensor
) -> int:
    """Restore the default joint position targets of ``articulations``.

    Returns the number of articulations updated. Entries without joint data
    or without ``set_joint_position_target`` are skipped.
    """
    updated = 0
    for articulation in articulations:
        data = getattr(articulation, "data", None)
        default_joint_pos = getattr(data, "default_joint_pos", None)
        setter = getattr(articulation, "set_joint_position_target", None)
        if default_joint_pos is None or setter is None:
            continue
        rows = env_ids.long()
        setter(default_joint_pos[rows], env_ids=rows)
        updated += 1
    return updated
