"""Resolve when Isaac Lab can safely read simulation state from USD.

``use_fabric`` selects the read path; PhysX's
``/physics/fabricUpdateTransformations`` setting selects where transforms are
written. If PhysX writes to Fabric while Isaac Lab reads USD, stale authored
poses can overwrite reset state. Keep Fabric enabled in that case. The rule is
a pure function and does not require Isaac Sim.
"""

from __future__ import annotations

__all__ = [
    "FABRIC_TRANSFORM_UPDATE_SETTING",
    "read_physx_fabric_transform_updates",
    "resolve_use_fabric",
]

FABRIC_TRANSFORM_UPDATE_SETTING = "/physics/fabricUpdateTransformations"
"""Carbonite flag controlling whether PhysX writes transforms to Fabric."""


def resolve_use_fabric(
    *,
    use_fabric: bool,
    device: str,
    physx_writes_transforms_to_fabric: bool,
) -> bool:
    """Decide whether Fabric may be turned off for a simulation.

    :param use_fabric: The value the configuration asks for.
    :param device: The physics device (``"cpu"``, ``"cuda"``, ``"cuda:0"``...).
    :param physx_writes_transforms_to_fabric: Whether PhysX is publishing
        transforms into Fabric, i.e. the value of
        :data:`FABRIC_TRANSFORM_UPDATE_SETTING`.
    :returns: The value to use.

    An explicit ``use_fabric=False`` is returned unchanged. When the
    configuration requests Fabric, it may be disabled only on CPU and when
    PhysX is not writing transforms to Fabric; otherwise USD contains stale
    authored poses.
    """

    if not use_fabric:
        return False
    if physx_writes_transforms_to_fabric:
        return True
    return str(device).strip().lower() != "cpu"


def read_physx_fabric_transform_updates(*, default: bool = False) -> bool:
    """Read the Fabric transform setting, or ``default`` before app startup."""

    try:
        from isaaclab.app.settings_manager import get_settings_manager
    except ImportError:
        return default

    try:
        value = get_settings_manager().get(FABRIC_TRANSFORM_UPDATE_SETTING)
    except (AttributeError, KeyError, RuntimeError):
        # Carb settings may be unavailable before Isaac Sim starts.
        return default
    return default if value is None else bool(value)
