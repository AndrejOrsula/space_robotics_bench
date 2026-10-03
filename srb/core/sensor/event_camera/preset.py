"""IMX636 event-camera presets for v2e-style simulators.

Values draw on v2e defaults and public datasheet figures. Their bright-to-dim
ordering and validation-scorecard distinguishability are enforced, but
absolute values remain estimates pending calibration against EVK4 captures.
Per-field provenance appears above the preset tuple.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from srb.utils.failures import InvalidInputError


@dataclass(frozen=True)
class IMX636Preset:
    """One IMX636 sensor-model preset.

    Field semantics match v2e ``EventEmulator`` so a preset can be passed to
    ``V2eBackend`` without per-field plumbing. ``hot_pixel_frac`` is the
    fraction of pixels baked as always-firing for the lifetime of a virtual
    sensor unit (not per episode).
    """

    name: str
    pos_thres: float
    neg_thres: float
    sigma_thres: float
    cutoff_hz: float
    leak_rate_hz: float
    shot_noise_rate_hz: float
    refractory_period_s: float
    hot_pixel_frac: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.hot_pixel_frac) or not (
            0.0 <= self.hot_pixel_frac <= 1.0
        ):
            raise ValueError(
                f"hot_pixel_frac must be in [0, 1]; got {self.hot_pixel_frac!r}"
            )
        for field_name in (
            "pos_thres",
            "neg_thres",
            "sigma_thres",
            "cutoff_hz",
            "leak_rate_hz",
            "shot_noise_rate_hz",
            "refractory_period_s",
        ):
            value = getattr(self, field_name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be non-negative; got {value!r}")


# Provenance of the seed values (apply uniformly across the three IMX636
# illumination presets until a real-EVK4 retune lands):
#   pos_thres / neg_thres : v2e EventEmulator default (0.20). IMX636 datasheet
#     reports ~25% contrast threshold; 0.20 is conservatively below to avoid
#     under-eventing tasks with low-contrast lunar regolith.
#   sigma_thres           : v2e default range scaled by illumination — bright
#     scenes have less per-pixel threshold mismatch.
#   cutoff_hz             : photoreceptor low-pass bandwidth; brighter scenes
#     drive faster photocurrent → higher bandwidth.
#   leak_rate_hz          : v2e EventEmulator default tier (low/mid/high)
#     scaled with shot-noise estimate.
#   shot_noise_rate_hz    : illumination-dominated; orders of magnitude track
#     v2e's published low-light figures (~20 Hz/px) → nominal (~5 Hz/px) →
#     bright sun (~2 Hz/px).
#   refractory_period_s   : 0.5 ms across all presets — IMX636 hardware floor,
#     not an illumination-driven parameter.
#   hot_pixel_frac        : 1e-4 across all presets — pixel-array yield is
#     hardware-driven, not illumination-driven.
_BUILTIN_PRESETS: tuple[IMX636Preset, ...] = (
    IMX636Preset(
        name="imx636_nominal",
        pos_thres=0.20,
        neg_thres=0.20,
        sigma_thres=0.03,
        cutoff_hz=50.0,
        leak_rate_hz=0.1,
        shot_noise_rate_hz=5.0,
        refractory_period_s=5e-4,
        hot_pixel_frac=1e-4,
    ),
    IMX636Preset(
        name="imx636_low_light",
        pos_thres=0.20,
        neg_thres=0.20,
        sigma_thres=0.05,
        cutoff_hz=20.0,
        leak_rate_hz=0.2,
        shot_noise_rate_hz=20.0,
        refractory_period_s=5e-4,
        hot_pixel_frac=1e-4,
    ),
    IMX636Preset(
        name="imx636_outdoor_sunlit",
        pos_thres=0.20,
        neg_thres=0.20,
        sigma_thres=0.02,
        cutoff_hz=70.0,
        leak_rate_hz=0.05,
        shot_noise_rate_hz=2.0,
        refractory_period_s=5e-4,
        hot_pixel_frac=1e-4,
    ),
    IMX636Preset(
        name="clean",
        pos_thres=0.20,
        neg_thres=0.20,
        sigma_thres=0.0,
        cutoff_hz=0.0,
        leak_rate_hz=0.0,
        shot_noise_rate_hz=0.0,
        refractory_period_s=0.0,
        hot_pixel_frac=0.0,
    ),
)

_PRESETS: dict[str, IMX636Preset] = {p.name: p for p in _BUILTIN_PRESETS}


def get_preset(name: str) -> IMX636Preset:
    try:
        return _PRESETS[name]
    except KeyError as e:
        raise InvalidInputError(
            f"Unknown event-camera preset {name!r}; available: {sorted(_PRESETS)}"
        ) from e


def register_preset(preset: IMX636Preset) -> None:
    """Register a custom preset. Idempotent overwrite on name collision."""
    _PRESETS[preset.name] = preset


def list_presets() -> tuple[str, ...]:
    return tuple(sorted(_PRESETS))


__all__ = [
    "IMX636Preset",
    "get_preset",
    "list_presets",
    "register_preset",
]
