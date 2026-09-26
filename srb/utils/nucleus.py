"""Resolve Isaac and Isaac Lab assets from the active 6.1 asset root."""

from __future__ import annotations

import os

ASSET_ROOT_ENV = "ISAACSIM_ASSET_ROOT"
DEFAULT_ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
)


def _settings_asset_root() -> str | None:
    """Read Kit's configured asset root without making imports Kit-only."""

    try:
        import carb.settings
    except ImportError:
        return None

    settings = carb.settings.get_settings()
    for key in (
        "/persistent/isaac/asset_root/default",
        "/persistent/isaac/asset_root/cloud",
    ):
        value = settings.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def resolve_asset_root() -> str:
    """Resolve the environment override, Kit setting, or 6.1 production root."""

    configured = os.environ.get(ASSET_ROOT_ENV, "").strip()
    root = configured or _settings_asset_root() or DEFAULT_ASSET_ROOT
    return root.rstrip("/")


NUCLEUS_ASSET_ROOT_DIR = resolve_asset_root()
ISAAC_NUCLEUS_DIR = f"{NUCLEUS_ASSET_ROOT_DIR}/Isaac"
ISAACLAB_NUCLEUS_DIR = f"{ISAAC_NUCLEUS_DIR}/IsaacLab"
ISAAC_ROBOTS_DIR = f"{ISAAC_NUCLEUS_DIR}/Robots_Multiphysics"
ISAACLAB_ROBOTS_DIR = f"{ISAACLAB_NUCLEUS_DIR}/Robots_Multiphysics"


__all__ = [
    "ASSET_ROOT_ENV",
    "DEFAULT_ASSET_ROOT",
    "ISAACLAB_NUCLEUS_DIR",
    "ISAACLAB_ROBOTS_DIR",
    "ISAAC_NUCLEUS_DIR",
    "ISAAC_ROBOTS_DIR",
    "NUCLEUS_ASSET_ROOT_DIR",
    "resolve_asset_root",
]
