"""Launch defaults for ``srb agent`` simulation runtime workflows."""

from __future__ import annotations

import shlex
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

_CAMERA_AUTO_ENABLE_ENVS = frozenset({"srb/excavation_demo"})
# Sensor categories that spawn RTX camera render products and require
# ``--enable_cameras``. Activating one through ``env.active_sensors`` enables
# rendering automatically, avoiding camera-spawn errors in visual-sensor examples.
_CAMERA_SENSOR_CATEGORIES = frozenset({"visual", "event_camera", "events"})
# The polished public demo boots with the high-quality RTX preset by default,
# so event operators do not need to remember ``--rendering_mode quality``.
# Explicit user overrides still win.
_RENDERING_MODE_AUTO_DEFAULTS: dict[str, str] = {"srb/excavation_demo": "quality"}
_HYDRA_BOOL_TRUE = frozenset({"1", "true", "yes", "on"})
_HYDRA_BOOL_FALSE = frozenset({"0", "false", "no", "off"})
_NOISY_PHYSX_LOG_CHANNEL_ARG = "--/log/channels/omni.physx.plugin=error"
_PHYSX_PLUGIN_LOG_CHANNEL_KEY = "--/log/channels/omni.physx.plugin"


@dataclass(frozen=True)
class AgentLaunchPlan:
    """Normalized AppLauncher kwargs for an ``srb agent`` runtime."""

    kwargs: dict[str, Any]


def plan_agent_app_launch(
    *,
    env_id: str,
    headless: bool,
    video_enable: bool,
    hydra_args: Sequence[str],
    kwargs: dict[str, Any],
) -> AgentLaunchPlan:
    """Return AppLauncher kwargs with SRB runtime defaults applied."""

    launcher_kwargs = dict(kwargs)
    enable_cameras_cli = bool(launcher_kwargs.get("enable_cameras", False))
    enable_cameras_auto = should_auto_enable_cameras(env_id, hydra_args)
    # XR requires camera rendering (enforced by preflight), so derive
    # ``enable_cameras`` instead of requiring a redundant flag.
    xr_requested = bool(launcher_kwargs.get("xr", False))
    launcher_kwargs["enable_cameras"] = (
        video_enable or enable_cameras_cli or enable_cameras_auto or xr_requested
    )

    # ``AppLauncher`` honours its own rendering default only when the kwarg is
    # absent; explicitly passing None disables every RTX preset. Demo-class
    # envs auto-pick a preset so the public launch command does not need
    # ``--rendering_mode``; explicit user choices still win.
    if launcher_kwargs.get("rendering_mode") is None:
        auto_rendering_mode = _RENDERING_MODE_AUTO_DEFAULTS.get(env_id)
        if auto_rendering_mode is not None:
            launcher_kwargs["rendering_mode"] = auto_rendering_mode
        else:
            launcher_kwargs.pop("rendering_mode", None)

    # IsaacLab develop prefers explicit visualizer selection. Keep the legacy
    # GUI behavior for ``srb agent`` when the caller did not choose one.
    if not headless:
        launcher_kwargs.setdefault("visualizer", ["kit"])

    if "experience" not in launcher_kwargs:
        launcher_kwargs["experience"] = agent_experience_path(
            headless=headless,
            enable_cameras=bool(launcher_kwargs.get("enable_cameras")),
            xr=bool(launcher_kwargs.get("xr")),
        )

    from srb.cli.preflight import validate_kit_path

    validate_kit_path(
        headless=headless,
        enable_cameras=bool(launcher_kwargs.get("enable_cameras")),
        xr=bool(launcher_kwargs.get("xr")),
        rendering_mode=launcher_kwargs.get("rendering_mode"),
        experience=launcher_kwargs["experience"],
    )

    launcher_kwargs["kit_args"] = with_runtime_default_kit_args(
        launcher_kwargs.get("kit_args")
    )
    return AgentLaunchPlan(kwargs=launcher_kwargs)


def agent_experience_path(
    *,
    headless: bool,
    enable_cameras: bool,
    xr: bool,
) -> str:
    """Return SRB's bundled Kit experience path for runtime agents."""

    from srb.utils.path import SRB_APPS_DIR

    return SRB_APPS_DIR.joinpath(
        f"srb.{'headless.' if headless else ''}"
        f"{'rendering.' if enable_cameras else ''}"
        f"{'xr.' if xr else ''}kit"
    ).as_posix()


def with_runtime_default_kit_args(kit_args: str | None) -> str:
    """Append SRB's runtime Kit defaults unless the user already set them."""

    raw = str(kit_args or "").strip()
    if _has_physx_plugin_log_override(raw):
        return raw
    return f"{raw} {_NOISY_PHYSX_LOG_CHANNEL_ARG}".strip()


def _has_physx_plugin_log_override(kit_args: str) -> bool:
    """Return whether Kit args explicitly configure the PhysX plugin channel."""

    try:
        tokens = shlex.split(kit_args)
    except ValueError:
        tokens = kit_args.split()
    return any(
        token == _PHYSX_PLUGIN_LOG_CHANNEL_KEY
        or token.startswith(f"{_PHYSX_PLUGIN_LOG_CHANNEL_KEY}=")
        for token in tokens
    )


def should_auto_enable_cameras(env_id: str, hydra_args: Sequence[str]) -> bool:
    """Return whether SRB should launch the rendering app for camera sensors."""

    # A user who explicitly activates a camera-backed sensor category always
    # needs the rendering app, regardless of which env they run.
    if requests_camera_sensor_override(hydra_args):
        return True

    if env_id not in _CAMERA_AUTO_ENABLE_ENVS:
        return False

    demo_cameras = get_hydra_bool_override(hydra_args, "env.demo_cameras")
    return True if demo_cameras is None else demo_cameras


def requests_camera_sensor_override(args: Sequence[str]) -> bool:
    """Return whether CLI Hydra args activate a camera-backed sensor category.

    Scans ``env.active_sensors=[...]`` overrides for a category that spawns an
    RTX camera (``visual`` / ``events``). Used to auto-enable camera
    rendering so users requesting visual sensors need not also remember
    ``--enable_cameras``.
    """

    # Repeated Hydra overrides apply in order, so only the last one counts.
    for arg in reversed(args):
        key, sep, value = arg.lstrip("+").partition("=")
        if not sep or key != "env.active_sensors":
            continue
        lowered = value.lower()
        return any(category in lowered for category in _CAMERA_SENSOR_CATEGORIES)
    return False


def get_hydra_bool_override(args: Sequence[str], key: str) -> bool | None:
    """Read a simple ``key=value`` Hydra bool override from CLI leftovers."""

    for arg in reversed(args):
        normalized = arg.lstrip("+")
        if not normalized.startswith(f"{key}="):
            continue
        value = normalized.split("=", 1)[1].strip().strip("\"'").lower()
        if value in _HYDRA_BOOL_TRUE:
            return True
        if value in _HYDRA_BOOL_FALSE:
            return False
    return None


__all__ = [
    "AgentLaunchPlan",
    "agent_experience_path",
    "get_hydra_bool_override",
    "plan_agent_app_launch",
    "requests_camera_sensor_override",
    "should_auto_enable_cameras",
    "with_runtime_default_kit_args",
]
