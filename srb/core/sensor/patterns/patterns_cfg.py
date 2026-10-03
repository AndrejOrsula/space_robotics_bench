from collections.abc import Callable, Sequence
from dataclasses import MISSING

from isaaclab.sensors.ray_caster.patterns import PatternBaseCfg

from srb.utils.cfg import configclass

from . import patterns


@configclass
class MultiPatternBaseCfg(PatternBaseCfg):
    func: Callable = patterns.multi_pattern

    patterns: Sequence[PatternBaseCfg] = MISSING  # type: ignore[assignment]
    offsets_pos: Sequence[tuple[float, float, float] | None] = ()


@configclass
class ExplicitPatternBaseCfg(PatternBaseCfg):
    func: Callable = patterns.explicit_pattern

    ray_starts: Sequence[tuple[float, float, float]] = MISSING  # type: ignore[assignment]
    ray_directions: (
        tuple[float, float, float] | Sequence[tuple[float, float, float]]
    ) = (0.0, 0.0, -1.0)
