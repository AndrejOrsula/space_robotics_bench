"""Shared default USD spawner properties for wheeled ground rovers.

Rovers may override a default by passing the same keyword after unpacking
`wheeled_rover_spawn_properties()` in `UsdFileCfg`; the later value wins.
"""

from __future__ import annotations

from typing import Any

from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
)


def wheeled_rover_spawn_properties() -> dict[str, Any]:
    """Return the shared `UsdFileCfg` keyword arguments for a wheeled rover."""
    return {
        "activate_contact_sensors": True,
        "collision_props": CollisionPropertiesCfg(
            contact_offset=0.02, rest_offset=0.005
        ),
        "rigid_props": RigidBodyPropertiesCfg(
            max_linear_velocity=1.5,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        "articulation_props": ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=4,
        ),
    }
