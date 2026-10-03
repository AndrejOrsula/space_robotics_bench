"""Carry configured exposure overrides to lights inside spawned USD assets.

Facility projector emission is authored in its USD asset, so it cannot be
configured through the spawner. USD Lux ``exposure`` scales emission by
``2 ** exposure``. This sidecar records overrides during configuration and
applies them after scene setup, when the light prims exist.
"""

from __future__ import annotations

from typing import Any, Mapping

__all__ = [
    "LIGHT_EXPOSURE_OVERRIDES_ATTR",
    "apply_light_exposure_overrides",
    "light_exposure_overrides",
    "record_light_exposure_override",
]

LIGHT_EXPOSURE_OVERRIDES_ATTR = "_light_exposure_overrides"
"""Name of the sidecar attribute on the environment config."""


def light_exposure_overrides(env_cfg: Any) -> dict[str, float]:
    """Return the override map, creating it on *env_cfg* if absent."""

    overrides = getattr(env_cfg, LIGHT_EXPOSURE_OVERRIDES_ATTR, None)
    if overrides is None:
        overrides = {}
        setattr(env_cfg, LIGHT_EXPOSURE_OVERRIDES_ATTR, overrides)
    return overrides


def record_light_exposure_override(
    env_cfg: Any, prim_path: str, exposure: float
) -> None:
    """Ask for every light under *prim_path* to be dimmed by *exposure* stops."""

    light_exposure_overrides(env_cfg)[prim_path] = float(exposure)


def apply_light_exposure_overrides(
    overrides: Mapping[str, float], env_regex_ns: str
) -> int:
    """Author the recorded exposures onto the stage. Returns the light count."""

    import isaaclab.sim as sim_utils
    from pxr import Usd, UsdLux

    applied = 0
    for pattern, exposure in overrides.items():
        resolved = pattern.replace("{ENV_REGEX_NS}", env_regex_ns)
        for root in sim_utils.find_matching_prims(resolved):
            for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                if prim.IsInstanceProxy() or not prim.HasAPI(UsdLux.LightAPI):
                    continue
                UsdLux.LightAPI(prim).CreateExposureAttr(float(exposure))
                applied += 1
    return applied
