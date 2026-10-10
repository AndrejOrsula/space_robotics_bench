"""Excavation deploy profile: the depth heightmap uses the training grid.

The simulator projects its heightmap onto a 16x16 grid of 3.75 cm cells in
``kinova_base_link``, centred on the scoop target, with a 1.5 m invalid-cell
sentinel (``srb/tasks/manipulation/excavation/heightmap_spec.py``). The
generic ``RosDepthHeightmapInterface`` default (5 cm cells in ``base_link``,
2 m sentinel) would hand the policy a differently scaled map, so a bare
``ros_depth_heightmap`` id is replaced by an interface configured from that
spec, and an operator-supplied instance must match it.

The grid is centred on the *configured default* scoop target
(``RosScoopTargetInterfaceCfg.default_xy_m``). The simulator re-centres the
grid on each commanded target at reset; the real adapter keeps a fixed grid,
so a runtime target override from ``ros_scoop_target`` does not move it.

This profile does not make excavation deployable: no shipped adapter produces
``state/ee_pose_world``, ``ros_optitrack`` is not in the generated hardware
list, and no adapter consumes ``robot/delta_twist`` together with the Kortex
stack (dated blocker in ``docs/src/workflows/sim_to_real.md``).
"""

from __future__ import annotations

import importlib.util
import math
from collections.abc import Mapping, Sequence
from functools import lru_cache
from pathlib import Path
from types import ModuleType
from typing import Any

from srb.interfaces.sim_to_real.core.hardware import HardwareInterface

# The excavation package ``__init__`` imports the Isaac task, so the
# standard-library spec module is loaded from its file rather than imported.
_TASK_HEIGHTMAP_SPEC_PATH = (
    Path(__file__).resolve().parents[3]
    / "tasks"
    / "manipulation"
    / "excavation"
    / "heightmap_spec.py"
)
_FLOAT_TOLERANCE = 1e-9


@lru_cache(maxsize=1)
def _task_heightmap_spec_module() -> ModuleType:
    spec = importlib.util.spec_from_file_location(
        "_srb_excavation_task_heightmap_spec", _TASK_HEIGHTMAP_SPEC_PATH
    )
    if spec is None or spec.loader is None:
        raise ImportError(
            f"cannot load excavation heightmap spec from {_TASK_HEIGHTMAP_SPEC_PATH}"
        )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def excavation_heightmap_spec(target_xy_m: tuple[float, float]) -> dict[str, Any]:
    """Return the training heightmap grid centred on ``target_xy_m``."""

    return dict(_task_heightmap_spec_module().heightmap_spec_for_target_xy(target_xy_m))


def configure_excavation_hardware(
    hardware: Sequence[HardwareInterface | str],
    *,
    action_layout: object | None = None,
) -> tuple[HardwareInterface | str, ...]:
    """Configure the excavation depth heightmap from the training grid."""

    del action_layout  # The heightmap grid does not depend on the action schema.
    from srb.core.sensor.heightmap import HeightmapSpec
    from srb.interfaces.sim_to_real.hardware.ros_depth_heightmap import (
        RosDepthHeightmapInterface,
        RosDepthHeightmapInterfaceCfg,
    )

    expected = HeightmapSpec(**excavation_heightmap_spec(_scoop_target_xy(hardware)))
    configured: list[HardwareInterface | str] = []
    for hw in hardware:
        if isinstance(hw, str) and hw == "ros_depth_heightmap":
            configured.append(
                RosDepthHeightmapInterface(
                    RosDepthHeightmapInterfaceCfg(
                        attach_frame=expected.frame,
                        spec=expected.model_copy(),
                    )
                )
            )
            continue
        if isinstance(hw, RosDepthHeightmapInterface):
            _validate_operator_heightmap(hw, expected.model_dump())
        configured.append(hw)
    return tuple(configured)


def _scoop_target_xy(
    hardware: Sequence[HardwareInterface | str],
) -> tuple[float, float]:
    from srb.interfaces.sim_to_real.hardware.ros_scoop_target import (
        RosScoopTargetInterface,
        RosScoopTargetInterfaceCfg,
    )

    for hw in hardware:
        if isinstance(hw, RosScoopTargetInterface):
            expected_frame = _task_heightmap_spec_module().EXCAVATION_HEIGHTMAP_SPEC[
                "frame"
            ]
            if hw.cfg.frame != expected_frame:
                raise ValueError(
                    "excavation RosScoopTargetInterface frame must match the "
                    f"training heightmap frame {expected_frame!r}; "
                    f"got {hw.cfg.frame!r}"
                )
            x, y = hw.cfg.default_xy_m
            return float(x), float(y)
    x, y = RosScoopTargetInterfaceCfg().default_xy_m
    return float(x), float(y)


def _validate_operator_heightmap(
    hw: HardwareInterface, expected: Mapping[str, Any]
) -> None:
    actual = hw.cfg.spec.model_dump()
    mismatched = [
        f"{field}={actual.get(field)!r} (training {value!r})"
        for field, value in expected.items()
        if not _same(actual.get(field), value)
    ]
    if hw.cfg.attach_frame != expected["frame"]:
        mismatched.append(
            f"attach_frame={hw.cfg.attach_frame!r} (training {expected['frame']!r})"
        )
    if mismatched:
        raise ValueError(
            f"excavation RosDepthHeightmapInterface {hw.name!r} must use the "
            f"training heightmap grid; mismatched: {', '.join(mismatched)}"
        )


def _same(actual: object, expected: object) -> bool:
    if isinstance(expected, float) and isinstance(actual, (int, float)):
        return math.isclose(
            float(actual), expected, rel_tol=0.0, abs_tol=_FLOAT_TOLERANCE
        )
    return actual == expected


__all__ = [
    "configure_excavation_hardware",
    "excavation_heightmap_spec",
]
