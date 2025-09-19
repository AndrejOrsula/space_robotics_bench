from dataclasses import MISSING
from typing import Callable

from srb.core.sim import ShapeCfg, SphereCfg
from srb.utils.cfg import configclass

from . import impl


@configclass
class PinCfg(ShapeCfg):
    pin_radius: float = MISSING  # type: ignore
    pin_length: float = MISSING  # type: ignore


@configclass
class ArrowCfg(ShapeCfg):
    func: Callable = impl.spawn_arrow

    tail_radius: float = MISSING  # type: ignore
    tail_length: float = MISSING  # type: ignore
    head_radius: float = MISSING  # type: ignore
    head_length: float = MISSING  # type: ignore


@configclass
class PinnedArrowCfg(PinCfg, ArrowCfg):
    func: Callable = impl.spawn_pinned_arrow


@configclass
class PinnedSphereCfg(PinCfg, SphereCfg):
    func: Callable = impl.spawn_pinned_sphere


@configclass
class SphereCfg(ShapeCfg):
    func: Callable = impl.spawn_sphere
    radius: float = MISSING  # type: ignore
