"""Compatibility fixes for SimForge's Isaac Lab USD spawner."""

from __future__ import annotations

import logging
from typing import Any


class _ExpectedExistingPrimFilter(logging.Filter):
    """Drop only SimForge's expected second-stage existing-prim warning."""

    def __init__(self, prim_path: str):
        super().__init__()
        self._message = f"A prim already exists at prim path: '{prim_path}'."

    def filter(self, record: logging.LogRecord) -> bool:
        return record.getMessage() != self._message


def install_simforge_isaaclab_compat() -> None:
    """Use direct USD queries and filter SimForge's expected handoff warning.

    SimForge first authors a USD prim so it can add missing APIs, then hands the
    same prim to Isaac Lab to apply configured properties. Filter only the
    resulting existing-prim warning; leave other warnings visible.
    """
    from isaaclab.sim.spawners.from_files import from_files as il_from_files
    from isaaclab.sim.utils import make_uninstanceable
    from simforge.integrations.isaaclab.spawner.from_files import impl as sf_impl

    from srb.core.sim.prim_utils import is_prim_path_valid

    if getattr(sf_impl, "_srb_isaaclab_compat_installed", False):
        return

    original_spawn = sf_impl.__spawn_from_usd
    original_apply_missing_apis = sf_impl._apply_missing_apis

    def _apply_missing_apis(prim_path: str, cfg: Any):
        # Collision meshes in packaged robot USDs are commonly instance
        # proxies. Requested collision/mesh overrides cannot be authored onto
        # those proxies. First let SimForge inspect the still-instanced asset:
        # its missing-API pass deliberately skips instance proxies, preventing
        # visual meshes from being promoted to colliders. Only then make assets
        # requesting collision overrides editable for the following mesh and
        # IsaacLab property passes.
        result = original_apply_missing_apis(prim_path, cfg)
        if (
            getattr(cfg, "collision_props", None) is not None
            or getattr(cfg, "mesh_collision_props", None) is not None
        ):
            make_uninstanceable(prim_path)
        return result

    def _spawn_without_expected_warning(
        prim_path: str,
        cfg: Any,
        translation=None,
        orientation=None,
        **kwargs,
    ):
        logger = il_from_files.logger
        expected_filter = _ExpectedExistingPrimFilter(prim_path)
        logger.addFilter(expected_filter)
        try:
            return original_spawn(
                prim_path,
                cfg,
                translation,
                orientation,
                **kwargs,
            )
        finally:
            logger.removeFilter(expected_filter)

    sf_impl.is_prim_path_valid = is_prim_path_valid
    sf_impl._apply_missing_apis = _apply_missing_apis
    sf_impl.__spawn_from_usd = _spawn_without_expected_warning
    sf_impl._srb_isaaclab_compat_installed = True


__all__ = ["install_simforge_isaaclab_compat"]
