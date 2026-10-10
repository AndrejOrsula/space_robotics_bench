"""Bindings from core contracts to application implementations.

Factories defer asset and terrain loading until a core caller needs a value.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from srb.core.default_assets import (
    DefaultAssetFactories,
    install_default_asset_factories,
)
from srb.core.terrain_metadata import (
    MetadataMutator,
    TerrainMetadataAdapter,
    TerrainMetadataWriter,
    _install_terrain_metadata_adapter,
)

if TYPE_CHECKING:
    from srb.core.asset import (
        AerialManipulator,
        AerialRobot,
        GroundManipulator,
        GroundRobot,
        Manipulator,
        OrbitalManipulator,
        OrbitalRobot,
        Pedestal,
    )


def _make_manipulation_robot() -> Manipulator:
    from srb import assets

    return assets.Franka()


def _make_pedestal() -> Pedestal:
    from srb import assets

    return assets.IndustrialPedestal25()


def _make_aerial_robot() -> AerialRobot:
    from srb import assets

    return assets.Ingenuity()


def _make_ground_robot() -> GroundRobot:
    from srb import assets

    return assets.Cadre()


def _make_orbital_robot() -> OrbitalRobot:
    from srb import assets

    return assets.Cubesat()


def _make_aerial_manipulation_robot() -> AerialManipulator:
    from srb import assets

    return assets.GenericAerialManipulator(
        mobile_base=assets.Ingenuity(), manipulator=assets.Franka()
    )


def _make_ground_manipulation_robot() -> GroundManipulator:
    from srb import assets

    return assets.GenericGroundManipulator(
        mobile_base=assets.Spot(), manipulator=assets.Franka()
    )


def _make_orbital_manipulation_robot() -> OrbitalManipulator:
    from srb import assets

    return assets.GenericOrbitalManipulator(
        mobile_base=assets.Cubesat(), manipulator=assets.Franka()
    )


def _resolve_terrain_metadata_writer(baked_dir: Path) -> TerrainMetadataWriter:
    from srb.terrain.cache import cache_paths_from_baked_dir

    paths = cache_paths_from_baked_dir(baked_dir)

    def write(mutator: MetadataMutator) -> None:
        from srb.terrain.cache import update_meta

        update_meta(paths, mutator)

    return write


_DEFAULT_ASSET_FACTORIES = DefaultAssetFactories(
    manipulation_robot=_make_manipulation_robot,
    pedestal=_make_pedestal,
    aerial_robot=_make_aerial_robot,
    ground_robot=_make_ground_robot,
    orbital_robot=_make_orbital_robot,
    aerial_manipulation_robot=_make_aerial_manipulation_robot,
    ground_manipulation_robot=_make_ground_manipulation_robot,
    orbital_manipulation_robot=_make_orbital_manipulation_robot,
)
_TERRAIN_METADATA_ADAPTER = TerrainMetadataAdapter(_resolve_terrain_metadata_writer)


def install_application_bindings() -> None:
    """Install bindings, reusing the same objects on root-package reload."""
    install_default_asset_factories(_DEFAULT_ASSET_FACTORIES)
    _install_terrain_metadata_adapter(_TERRAIN_METADATA_ADAPTER)
