"""Import-light binding for application-owned default asset factories."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb.core.asset import (
        AerialManipulator,
        AerialRobot,
        GroundManipulator,
        GroundRobot,
        Manipulator,
        MobileRobot,
        OrbitalManipulator,
        OrbitalRobot,
        Pedestal,
    )


@dataclass(frozen=True, slots=True)
class DefaultAssetFactories:
    """Typed zero-argument factories for the seven core environment families."""

    manipulation_robot: Callable[[], Manipulator]
    pedestal: Callable[[], Pedestal | MobileRobot]
    aerial_robot: Callable[[], AerialRobot]
    ground_robot: Callable[[], GroundRobot]
    orbital_robot: Callable[[], OrbitalRobot]
    aerial_manipulation_robot: Callable[[], AerialManipulator]
    ground_manipulation_robot: Callable[[], GroundManipulator]
    orbital_manipulation_robot: Callable[[], OrbitalManipulator]


_FACTORIES: DefaultAssetFactories | None = None


def _same_factory(left: Callable[..., object], right: Callable[..., object]) -> bool:
    """Match a function recreated by reloading its defining module."""
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


def _same_factories(left: DefaultAssetFactories, right: DefaultAssetFactories) -> bool:
    return all(
        _same_factory(getattr(left, name), getattr(right, name))
        for name in DefaultAssetFactories.__slots__
    )


def _install_default_asset_factories(factories: DefaultAssetFactories) -> None:
    """Install the process-wide binding once; allow identical reloads."""

    global _FACTORIES
    if _FACTORIES is factories or (
        _FACTORIES is not None and _same_factories(_FACTORIES, factories)
    ):
        return
    if _FACTORIES is not None:
        raise RuntimeError("default asset factories are already installed")
    _FACTORIES = factories


def default_asset_factories() -> DefaultAssetFactories:
    """Return the installed factory object without copying or rebuilding it."""

    if _FACTORIES is None:
        raise RuntimeError("default asset factories are not installed")
    return _FACTORIES


def install_default_asset_factories(factories: DefaultAssetFactories) -> None:
    """Compatibility spelling for the application bootstrap installer."""

    _install_default_asset_factories(factories)


def get_default_asset_factories() -> DefaultAssetFactories:
    """Compatibility spelling for the core configuration getter."""

    return default_asset_factories()


__all__ = [
    "DefaultAssetFactories",
    "default_asset_factories",
    "get_default_asset_factories",
    "install_default_asset_factories",
]
