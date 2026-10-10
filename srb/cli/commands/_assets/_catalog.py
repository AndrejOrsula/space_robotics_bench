"""Shared asset-kind catalog for ``srb asset`` commands."""

from __future__ import annotations

from dataclasses import dataclass

BODY_CHOICES = ("moon", "mars", "earth")
TERRAIN_LOD_CHOICES = ("low", "medium", "high")


@dataclass(frozen=True)
class AssetKind:
    """CLI-facing metadata for a supported asset kind."""

    name: str
    target_label: str
    supports_list: bool = True
    supports_info: bool = True
    supports_download: bool = True
    supports_doctor: bool = False
    supports_bake: bool = False
    supports_clean: bool = False


ASSET_KINDS = (
    AssetKind(
        name="terrain",
        target_label="site",
        supports_doctor=True,
        supports_bake=True,
        supports_clean=True,
    ),
    AssetKind(name="rocks", target_label="rock set"),
    AssetKind(name="hdri", target_label="skydome"),
    AssetKind(name="lights", target_label="lighting preset"),
)

_ASSET_KINDS_BY_NAME = {asset_kind.name: asset_kind for asset_kind in ASSET_KINDS}


def asset_kind_choices() -> tuple[str, ...]:
    """Return all asset kinds accepted by general assets commands."""
    return tuple(asset_kind.name for asset_kind in ASSET_KINDS)


def terrain_asset_kind_choices() -> tuple[str, ...]:
    """Return the terrain-only choices accepted by terrain-specific commands."""
    return tuple(
        asset_kind.name
        for asset_kind in ASSET_KINDS
        if asset_kind.supports_doctor and asset_kind.supports_bake
    )


def get_asset_kind(kind: str) -> AssetKind:
    """Return CLI metadata for ``kind`` or raise ``ValueError``."""
    try:
        return _ASSET_KINDS_BY_NAME[kind]
    except KeyError as exc:
        choices = ", ".join(asset_kind_choices())
        raise ValueError(
            f"unknown asset kind {kind!r}; choose one of: {choices}"
        ) from exc


def find_asset_kind(kind: str) -> AssetKind | None:
    """Return CLI metadata for ``kind`` when it is catalogued."""
    return _ASSET_KINDS_BY_NAME.get(kind)
