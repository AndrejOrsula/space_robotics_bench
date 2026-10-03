"""Import-light contract for application-owned terrain metadata composition.

The application binds the terrain transaction without making core import its
implementation. A resolved writer retains the terrain owner's path object;
the caller controls when the existing transaction runs.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

MetadataMutator = Callable[[dict[str, Any]], None]
TerrainMetadataWriter = Callable[[MetadataMutator], None]


@dataclass(frozen=True, slots=True)
class TerrainMetadataAdapter:
    resolve_writer: Callable[[Path], TerrainMetadataWriter]


_adapter: TerrainMetadataAdapter | None = None


def _same_resolver(left: Callable[..., object], right: Callable[..., object]) -> bool:
    """Match a resolver function recreated by reloading its defining module."""
    left_code = getattr(left, "__code__", None)
    right_code = getattr(right, "__code__", None)
    if left is right or left_code is None or right_code is None:
        return False
    return (
        getattr(left, "__module__", None) == "srb.composition"
        and getattr(right, "__module__", None) == "srb.composition"
        and getattr(left, "__qualname__", None) == getattr(right, "__qualname__", None)
        and left_code == right_code
        and getattr(left, "__defaults__", None) == getattr(right, "__defaults__", None)
        and getattr(left, "__kwdefaults__", None)
        == getattr(right, "__kwdefaults__", None)
    )


def _install_terrain_metadata_adapter(adapter: TerrainMetadataAdapter) -> None:
    """Install once during package bootstrap; repeated installation is harmless."""
    global _adapter
    if _adapter is adapter or (
        _adapter is not None
        and _same_resolver(_adapter.resolve_writer, adapter.resolve_writer)
    ):
        return
    if _adapter is not None:
        raise RuntimeError("terrain metadata adapter is already installed")
    _adapter = adapter


def resolve_terrain_metadata_writer(baked_dir: Path) -> TerrainMetadataWriter:
    """Resolve a writer before product publication, without updating metadata."""
    if _adapter is None:
        raise RuntimeError("terrain metadata adapter is not installed")
    return _adapter.resolve_writer(baked_dir)
