from collections.abc import Callable
from typing import Literal, Sequence

from srb.core.sim import ShapeCfg
from srb.utils.cfg import configclass

from .impl import spawn_multi_shape


@configclass
class MultiShapeSpawnerCfg(ShapeCfg):
    func: Callable = spawn_multi_shape

    shapes: Sequence[Literal["cuboid", "sphere", "cylinder", "capsule", "cone"]] = ()
    """Shapes to spawn (keep empty to consider all shapes)"""

    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    """Scale of cuboid (affects other shapes if radius and height remain unset)"""

    radius: float | None = None
    """Radius of sphere|cylinder|capsule|cone (default: self.scale[0])"""

    height: float | None = None
    """Height of cylinder|capsule|cone (default: self.scale[1])"""

    axis: Literal["X", "Y", "Z"] = "Z"
    """Axis of cylinder|capsule|cone"""

    random_choice: bool = True
