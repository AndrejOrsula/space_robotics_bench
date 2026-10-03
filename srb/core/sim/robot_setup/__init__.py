from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

from .cfg import RobotAssemblerCfg

if TYPE_CHECKING:
    from .assembled_bodies import AssembledBodies as AssembledBodies
    from .assembled_robot import AssembledRobot as AssembledRobot
    from .robot_assembler import RobotAssembler as RobotAssembler

_EXPORTS = {
    "AssembledBodies": ".assembled_bodies",
    "AssembledRobot": ".assembled_robot",
    "RobotAssembler": ".robot_assembler",
}

__all__ = ["AssembledBodies", "AssembledRobot", "RobotAssembler", "RobotAssemblerCfg"]


def __getattr__(name: str) -> Any:
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    try:
        module = import_module(_EXPORTS[name], __name__)
    except ImportError as import_error:
        from srb.utils.isaacsim import enable_isaacsim_extension

        try:
            enable_isaacsim_extension("isaacsim.robot_setup.assembler")
        except RuntimeError as enable_error:
            raise RuntimeError(
                "could not enable the 'isaacsim.robot_setup.assembler' "
                f"extension needed to import {name!r}; the original import "
                f"failed with: {import_error}"
            ) from enable_error
        module = import_module(_EXPORTS[name], __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
