"""Isaac-side settings for the Isaac-free external-heightmap profiles."""

from __future__ import annotations

import math
from typing import Literal

from srb.terrain.landscaping_external_heightmap import (
    PROFILE_FAILURE_STALE,
    PROFILE_IDEAL,
    PROFILE_NAMES,
    PROFILE_RANDOMIZED,
    ExternalHeightmapProfile,
)
from srb.utils.cfg import configclass

ExternalHeightmapProfileName = Literal["ideal", "randomized", "failure_stale"]


@configclass
class ExternalHeightmapCfg:
    """Select an external-mapper profile and configure its parameters."""

    profile: ExternalHeightmapProfileName = "ideal"
    """Measurement profile: ``ideal``, ``randomized``, or ``failure_stale``."""

    seed: int = 0
    """Seed for the per-environment measurement RNG streams."""

    delay_min_s: float = 0.0
    delay_max_s: float = 0.1
    """Randomized profile: whole-frame acquisition-to-publication delay sampled from this range."""

    drop_probability: float = 0.05
    max_consecutive_drops: int = 2
    """Independent frame-drop probability, capped at consecutive drops."""

    elevation_noise_std_m: float = 0.002
    elevation_noise_clip_m: float = 0.006
    """Per-cell elevation noise ``Normal(0, std)`` clipped to ``+/-clip``."""

    registration_translation_std_m: float = 0.005
    registration_translation_clip_m: float = 0.015
    registration_yaw_std_deg: float = 0.25
    registration_yaw_clip_deg: float = 0.75
    """Per-episode mission registration error; per-frame pose jitter reuses
    these same bounds."""

    stale_hold_updates: int = 6
    """Consecutive held updates for ``failure_stale`` (must stay >= 6)."""

    def to_profile(self) -> ExternalHeightmapProfile:
        """Build the immutable Isaac-free profile."""
        if self.profile not in PROFILE_NAMES:
            raise ValueError(
                f"external_heightmap.profile must be one of {PROFILE_NAMES}, "
                f"got {self.profile!r}"
            )
        if self.profile == PROFILE_IDEAL:
            return ExternalHeightmapProfile(name=PROFILE_IDEAL, seed=self.seed)
        if self.profile == PROFILE_FAILURE_STALE:
            if self.stale_hold_updates < 6:
                raise ValueError(
                    "failure_stale requires at least six consecutive held "
                    f"updates, got {self.stale_hold_updates}"
                )
            return ExternalHeightmapProfile(
                name=PROFILE_FAILURE_STALE,
                stale_hold_updates=self.stale_hold_updates,
                seed=self.seed,
            )
        return ExternalHeightmapProfile(
            name=PROFILE_RANDOMIZED,
            delay_range_s=(self.delay_min_s, self.delay_max_s),
            drop_probability=self.drop_probability,
            max_consecutive_drops=self.max_consecutive_drops,
            elevation_noise_std_m=self.elevation_noise_std_m,
            elevation_noise_clip_m=self.elevation_noise_clip_m,
            registration_translation_std_m=self.registration_translation_std_m,
            registration_translation_clip_m=self.registration_translation_clip_m,
            registration_yaw_std_rad=math.radians(self.registration_yaw_std_deg),
            registration_yaw_clip_rad=math.radians(self.registration_yaw_clip_deg),
            seed=self.seed,
        )


__all__ = ["ExternalHeightmapCfg", "ExternalHeightmapProfileName"]
