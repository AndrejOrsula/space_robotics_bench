"""Terrain asset lifecycle policy.

This module sits above cache/source availability and below presentation
adapters such as ``srb asset list|doctor|download``. It keeps status-to-action
policy in one place so CLI commands do not each reinterpret availability.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from srb.terrain.asset_commands import asset_download_command
from srb.terrain.availability import (
    TerrainAvailability,
    TerrainAvailabilityStatus,
    assess_terrain_site,
)
from srb.terrain.errors import TerrainError
from srb.terrain.spec import Body, Lod


class TerrainAssetAction(str, Enum):
    READY = "ready"
    DOWNLOAD = "download"
    BAKE = "bake"
    INSTALL_DEPENDENCIES = "install_dependencies"
    BACKFILL_MANIFEST = "backfill_manifest"


@dataclass(frozen=True, slots=True)
class TerrainAssetLifecycle:
    availability: TerrainAvailability
    action: TerrainAssetAction
    download_command: str | None
    next_step: str | None
    blocking_error: TerrainError | None

    @property
    def should_attempt_bake(self) -> bool:
        return self.action in (
            TerrainAssetAction.DOWNLOAD,
            TerrainAssetAction.BAKE,
        )

    @property
    def baked_path(self) -> Path | None:
        return self.availability.baked_path

    def cached_path_for_download(self, *, force: bool) -> Path | None:
        if force:
            return None
        if self.action is not TerrainAssetAction.READY:
            return None
        return self.availability.baked_path


def assess_terrain_asset(
    site: str,
    *,
    body: Body | None = None,
    patch: str = "default",
    lod: Lod = "medium",
    seed: int = 0,
    patch_size_m: float | None = None,
    patch_center_xy_m: tuple[float, float] | None = None,
    use_ortho: bool = True,
    preset_name: str | None = None,
    deshade: bool = False,
) -> TerrainAssetLifecycle:
    """Return lifecycle policy for one curated terrain asset.

    ``deshade`` threads the request-level deshade knob into the
    assessed spec -- see `assess_terrain_site`; default False keeps every
    existing call byte-identical.
    """

    availability = assess_terrain_site(
        site,
        body=body,
        patch=patch,
        lod=lod,
        seed=seed,
        patch_size_m=patch_size_m,
        patch_center_xy_m=patch_center_xy_m,
        use_ortho=use_ortho,
        preset_name=preset_name,
        deshade=deshade,
    )
    command = asset_download_command(
        site,
        patch=patch,
        lod=lod,
        seed=seed,
        patch_size_m=patch_size_m,
        patch_center_xy_m=patch_center_xy_m,
        use_ortho=use_ortho,
        deshade=deshade,
    )

    if availability.status is TerrainAvailabilityStatus.READY:
        return TerrainAssetLifecycle(
            availability=availability,
            action=TerrainAssetAction.READY,
            download_command=command,
            next_step=None,
            blocking_error=None,
        )

    if availability.status is TerrainAvailabilityStatus.KNOWN_BAD_MANIFEST:
        return TerrainAssetLifecycle(
            availability=availability,
            action=TerrainAssetAction.BACKFILL_MANIFEST,
            download_command=None,
            next_step="backfill terrain manifest sha256/source",
            blocking_error=_issue_error(availability),
        )

    if availability.status is TerrainAvailabilityStatus.MISSING_DEPENDENCY:
        next_step = f"install missing dependencies, then run {command}"
        return TerrainAssetLifecycle(
            availability=availability,
            action=TerrainAssetAction.INSTALL_DEPENDENCIES,
            download_command=command,
            next_step=next_step,
            blocking_error=_issue_error(availability),
        )

    action = (
        TerrainAssetAction.BAKE
        if availability.status is TerrainAvailabilityStatus.NEEDS_BAKE
        else TerrainAssetAction.DOWNLOAD
    )
    return TerrainAssetLifecycle(
        availability=availability,
        action=action,
        download_command=command,
        next_step=command,
        blocking_error=None,
    )


def _issue_error(availability: TerrainAvailability) -> TerrainError | None:
    if availability.issue is None:
        return None
    return TerrainError(
        availability.issue.summary,
        remediation=availability.issue.remediation,
    )


__all__ = [
    "TerrainAssetAction",
    "TerrainAssetLifecycle",
    "assess_terrain_asset",
]
