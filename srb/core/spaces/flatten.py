from __future__ import annotations

import numpy as np
from gymnasium import spaces


def flatten_dict_to_box(space: spaces.Dict) -> spaces.Box:
    from srb.core.env.runtime.observation_layout import ObservationLayout

    if not isinstance(space, spaces.Dict):
        raise TypeError(
            f"flatten_dict_to_box requires a Dict space, got {type(space).__name__}"
        )
    return ObservationLayout.from_spaces(space).merged_space()


def autobound_action_space(
    space: spaces.Box,
    *,
    low: float = -1.0,
    high: float = 1.0,
) -> spaces.Box:
    if space.is_bounded("both"):
        return space
    # Replace only the infinite sides; finite bounds stay the policy's limits.
    new_low = np.where(np.isfinite(space.low), space.low, low).astype(space.dtype)
    new_high = np.where(np.isfinite(space.high), space.high, high).astype(space.dtype)
    inverted = np.argwhere(new_low > new_high)
    if inverted.size:
        raise ValueError(
            "autobound_action_space: a finite bound lies outside the default "
            f"range [{low}, {high}] at indices "
            f"{', '.join(str(index.tolist()) for index in inverted)}"
        )
    return spaces.Box(low=new_low, high=new_high, dtype=space.dtype)
