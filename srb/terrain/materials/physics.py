"""Per-body rigid-body material parameters used at USD assembly.

Numbers are conservative engineering estimates for regolith. Refine with the
robotics team.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

Body = Literal["moon", "mars", "earth"]


@dataclass(frozen=True)
class PhysicsMaterial:
    static_friction: float
    dynamic_friction: float
    restitution: float
    density_kg_m3: float


_REGISTRY: dict[str, PhysicsMaterial] = {
    "moon": PhysicsMaterial(
        static_friction=0.85,
        dynamic_friction=0.70,
        restitution=0.0,
        density_kg_m3=1500.0,
    ),
    "mars": PhysicsMaterial(
        static_friction=0.75,
        dynamic_friction=0.60,
        restitution=0.0,
        density_kg_m3=1600.0,
    ),
    "earth": PhysicsMaterial(
        static_friction=0.80,
        dynamic_friction=0.60,
        restitution=0.1,
        density_kg_m3=1600.0,
    ),
}


def get_physics_material(body: Body) -> PhysicsMaterial:
    if body not in _REGISTRY:
        raise KeyError(
            f"no physics material is registered for body {body!r}; "
            f"choose one of {', '.join(_REGISTRY)}"
        )
    return _REGISTRY[body]
