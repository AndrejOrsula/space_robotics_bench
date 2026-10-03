from dataclasses import MISSING
from typing import Callable

from srb.core.sim import ShapeCfg, SphereCfg
from srb.utils.cfg import configclass

from . import impl


@configclass
class PlaneCfg(ShapeCfg):
    func: Callable = impl.spawn_plane
    size: tuple[float, float] = MISSING  # type: ignore[assignment]


@configclass
class PinCfg(ShapeCfg):
    pin_radius: float = MISSING  # type: ignore[assignment]
    pin_length: float = MISSING  # type: ignore[assignment]


@configclass
class ArrowCfg(ShapeCfg):
    func: Callable = impl.spawn_arrow

    tail_radius: float = MISSING  # type: ignore[assignment]
    tail_length: float = MISSING  # type: ignore[assignment]
    head_radius: float = MISSING  # type: ignore[assignment]
    head_length: float = MISSING  # type: ignore[assignment]


@configclass
class PinnedArrowCfg(PinCfg, ArrowCfg):
    func: Callable = impl.spawn_pinned_arrow


@configclass
class PinnedSphereCfg(PinCfg, SphereCfg):
    func: Callable = impl.spawn_pinned_sphere
