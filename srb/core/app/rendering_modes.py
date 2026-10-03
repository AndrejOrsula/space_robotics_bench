"""Isaac RTX presets for ``--rendering_mode``.

Isaac Lab 3.0 removed its rendering presets and launcher argument, so SRB
applies equivalent flattened carb settings after Kit starts. Later env
settings override these presets. The upstream XR preset was removed, so
``rendering_mode=xr`` is empty; the XR Kit experience supplies those settings.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import TYPE_CHECKING, Mapping

if TYPE_CHECKING:
    from isaaclab.app.settings_manager import SettingsManager

RENDERING_MODES: tuple[str, ...] = ("performance", "balanced", "quality", "xr")

_PERFORMANCE: dict[str, bool | int | float] = {
    "/rtx/rtpt/cached/enabled": False,
    "/rtx/rtpt/lightcache/cached/enabled": False,
    "/rtx/rtpt/translucency/virtualMotion/enabled": False,
    "/rtx/rtpt/maxBounces": 2,
    "/rtx/rtpt/splitGlass": False,
    "/rtx/rtpt/splitClearcoat": False,
    "/rtx/rtpt/splitRoughReflection": True,
    "/rtx/rtpt/useAmbientOcclusionForAmbientLight": False,
    "/rtx/sceneDb/ambientLightIntensity": 1.0,
    "/rtx/shadows/enabled": True,
    "/rtx/domeLight/upperLowerStrategy": 3,
    "/rtx/ambientOcclusion/enabled": False,
    "/rtx/ambientOcclusion/denoiserMode": 1,
    "/rtx/raytracing/subpixel/mode": 0,
    "/rtx/raytracing/cached/enabled": False,
    "/rtx/post/dlss/execMode": 0,
    "/rtx/pathtracing/maxSamplesPerLaunch": 1000000,
    "/rtx/viewTile/limit": 1000000,
    "/rtx-transient/dlssg/enabled": False,
    "/rtx-transient/dldenoiser/enabled": False,
}

_BALANCED: dict[str, bool | int | float] = {
    "/rtx/rtpt/cached/enabled": False,
    "/rtx/rtpt/lightcache/cached/enabled": False,
    "/rtx/rtpt/translucency/virtualMotion/enabled": False,
    "/rtx/rtpt/maxBounces": 2,
    "/rtx/rtpt/splitGlass": False,
    "/rtx/rtpt/splitClearcoat": False,
    "/rtx/rtpt/splitRoughReflection": True,
    "/rtx/rtpt/useAmbientOcclusionForAmbientLight": False,
    "/rtx/sceneDb/ambientLightIntensity": 1.0,
    "/rtx/shadows/enabled": True,
    "/rtx/ambientOcclusion/enabled": False,
    "/rtx/ambientOcclusion/denoiserMode": 1,
    "/rtx/raytracing/subpixel/mode": 0,
    "/rtx/raytracing/cached/enabled": True,
    "/rtx/post/dlss/execMode": 1,
    "/rtx/pathtracing/maxSamplesPerLaunch": 1000000,
    "/rtx/viewTile/limit": 1000000,
    "/rtx-transient/dlssg/enabled": False,
    "/rtx-transient/dldenoiser/enabled": True,
}

_QUALITY: dict[str, bool | int | float] = {
    "/rtx/rtpt/maxBounces": 3,
    "/rtx/rtpt/cached/enabled": False,
    "/rtx/rtpt/lightcache/cached/enabled": False,
    "/rtx/rtpt/translucency/virtualMotion/enabled": False,
    "/rtx/rtpt/splitRoughReflection": True,
    "/rtx/rtpt/adaptiveSampling/disocclusion/enabled": True,
    "/rtx/rtpt/adaptiveSampling/disocclusion/spp": 4,
    "/rtx/sceneDb/ambientLightIntensity": 1.0,
    "/rtx/shadows/enabled": True,
    "/rtx/ambientOcclusion/enabled": True,
    "/rtx/ambientOcclusion/denoiserMode": 0,
    "/rtx/raytracing/subpixel/mode": 1,
    "/rtx/raytracing/cached/enabled": True,
    "/rtx/post/dlss/execMode": 2,
    "/rtx/pathtracing/maxSamplesPerLaunch": 1000000,
    "/rtx/viewTile/limit": 1000000,
    "/rtx-transient/dlssg/enabled": False,
}

RENDERING_MODE_PRESETS: Mapping[str, Mapping[str, bool | int | float]] = (
    MappingProxyType(
        {
            "performance": MappingProxyType(_PERFORMANCE),
            "balanced": MappingProxyType(_BALANCED),
            "quality": MappingProxyType(_QUALITY),
            "xr": MappingProxyType({}),
        }
    )
)


def rendering_mode_settings(
    rendering_mode: str | None,
) -> Mapping[str, bool | int | float]:
    """Return a mode's carb settings, or an empty mapping when unset."""
    if rendering_mode is None:
        return MappingProxyType({})
    try:
        return RENDERING_MODE_PRESETS[rendering_mode]
    except KeyError:
        raise ValueError(
            f"unknown rendering mode {rendering_mode!r}; "
            f"expected one of {list(RENDERING_MODES)}"
        ) from None


def apply_rendering_mode(
    rendering_mode: str | None, settings: SettingsManager | None = None
) -> None:
    """Apply a rendering preset to the running Kit app."""
    preset = rendering_mode_settings(rendering_mode)
    if not preset:
        return
    if settings is None:
        from isaaclab.app.settings_manager import get_settings_manager

        settings = get_settings_manager()
    for path, value in preset.items():
        settings.set(path, value)
