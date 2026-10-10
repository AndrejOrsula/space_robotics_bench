"""Sim-to-real interfaces with lazy exports from the runtime-heavy ``.core``.

PEP 562 defers importing torch and Isaac Sim until a runtime symbol is
requested, keeping lightweight submodules and CLI completion fast.
"""

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

_CORE_LAZY_ATTRS: frozenset[str] = frozenset(
    {
        "HardwareCapabilities",
        "HardwareInterface",
        "HardwareInterfaceCfg",
        "HardwareInterfaceRegistry",
        "RealEnv",
        "RealEnvGenerator",
        "observation_spaces_from_values",
    }
)


def __getattr__(name: str) -> Any:
    if name not in _CORE_LAZY_ATTRS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    try:
        attr = getattr(import_module(f"{__name__}.core"), name)
    except ModuleNotFoundError as exc:
        # Only a missing third-party runtime (torch / Isaac Sim, e.g. on CPU-only
        # test runners) means "unavailable"; a broken SRB import must surface
        # as itself rather than as a misleading missing-runtime error.
        if exc.name is None or exc.name.partition(".")[0] == "srb":
            raise
        raise AttributeError(
            f"module {__name__!r} attribute {name!r} requires the real-runtime "
            f"stack (missing {exc.name!r}) which is unavailable"
        ) from exc
    globals()[name] = attr
    return attr


def __dir__() -> list[str]:
    return sorted(_CORE_LAZY_ATTRS | set(globals()))


if TYPE_CHECKING:  # pragma: no cover - type-checker only
    from srb.interfaces.sim_to_real.core import (
        HardwareCapabilities,
        HardwareInterface,
        HardwareInterfaceCfg,
        HardwareInterfaceRegistry,
        RealEnv,
        RealEnvGenerator,
        observation_spaces_from_values,
    )
