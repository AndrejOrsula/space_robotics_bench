"""Sim-to-real core with lazy re-exports.

Importing a light submodule such as ``.hardware`` executes this package first;
eager re-exports would pull in ``RealEnv`` and torch for hardware-only callers.
"""

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

_LAZY_ATTRS: dict[str, str] = {
    "RealEnv": "env",
    "RealEnvGenerator": "generator",
    "HardwareCapabilities": "hardware",
    "HardwareInterface": "hardware",
    "HardwareInterfaceCfg": "hardware",
    "HardwareInterfaceRegistry": "hardware",
    "PreparedHardwareAction": "hardware",
    "observation_spaces_from_values": "hardware",
    "HardwareActionMapping": "hardware_mapping",
    "HardwareObservationMapping": "hardware_mapping",
    "HardwareRoutes": "hardware_mapping",
    "HardwareSourceGroups": "hardware_mapping",
}

__all__ = sorted(_LAZY_ATTRS)


def __getattr__(name: str) -> Any:
    submodule = _LAZY_ATTRS.get(name)
    if submodule is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    attr = getattr(import_module(f"{__name__}.{submodule}"), name)
    globals()[name] = attr
    return attr


def __dir__() -> list[str]:
    return sorted(set(_LAZY_ATTRS) | set(globals()))


if TYPE_CHECKING:  # pragma: no cover - type-checker only
    from .env import RealEnv
    from .generator import RealEnvGenerator
    from .hardware import (
        HardwareCapabilities,
        HardwareInterface,
        HardwareInterfaceCfg,
        HardwareInterfaceRegistry,
        PreparedHardwareAction,
        observation_spaces_from_values,
    )
    from .hardware_mapping import (
        HardwareActionMapping,
        HardwareObservationMapping,
        HardwareRoutes,
        HardwareSourceGroups,
    )
