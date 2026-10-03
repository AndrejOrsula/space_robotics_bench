"""Neutral wrapper configuration models owned by the core layer.

These models are consumed by environment configuration (``srb.core``) and
implemented by ``srb.wrappers``. Keeping them here preserves the dependency
direction ``core -> wrappers`` never being required at import time.
"""

from __future__ import annotations

from enum import Enum, auto

from pydantic import BaseModel, PositiveFloat, PositiveInt


class SmoothingMethod(str, Enum):
    BUTTERWORTH = auto()
    MOVING_AVERAGE = auto()
    SAVGOL = auto()


class FillMethod(str, Enum):
    ZERO = auto()
    FIRST_ACTION = auto()


class ActionSmoothingCfg(BaseModel):
    enable: bool = True
    method: SmoothingMethod = SmoothingMethod.MOVING_AVERAGE

    smoothing_duration: PositiveFloat = 0.25
    poly_order: PositiveInt = 3
    sample_frequency: PositiveFloat | None = None
    fill_on_reset: FillMethod = FillMethod.FIRST_ACTION


class ObservationStackingCfg(BaseModel):
    enable: bool = True
    n: PositiveInt = 4


__all__ = (
    "ActionSmoothingCfg",
    "FillMethod",
    "ObservationStackingCfg",
    "SmoothingMethod",
)
