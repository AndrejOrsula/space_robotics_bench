from __future__ import annotations

import threading
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb.core.sensor import SensorBaseCfg


class SensorCategory(Enum):
    PROPRIOCEPTION = "proprioception"
    CONTACT = "contact"
    VISUAL = "visual"
    TACTILE = "tactile"
    TERRAIN = "terrain"
    TRANSFORM = "transform"
    EVENT_CAMERA = "event_camera"
    CUSTOM = "custom"

    @classmethod
    def _missing_(cls, value: object) -> SensorCategory | None:
        # `events` reads like the other modality names in `active_sensors`.
        if isinstance(value, str) and value.lower() == "events":
            return cls.EVENT_CAMERA
        return None


_SENSOR_CATEGORY_REGISTRY: dict[type, SensorCategory] = {}
_registry_initialized = False
#: Guards lazy initialization: concurrent observation and dataset workers
#: must not read a partially populated registry. Held only during init.
_registry_lock = threading.Lock()


def _init_registry():
    """Populate the registry with known IsaacLab sensor types.

    Called lazily on first use to avoid import-time circular dependencies.
    Isaac Lab imports are optional so the registry stays usable in unit-test
    stub envs; Isaac-free sensor types (e.g. event-camera) register
    themselves via their own package ``__init__`` side-effects.
    """
    mapping: dict[type, SensorCategory] = {}
    try:
        from srb.core.sensor import (
            CameraCfg,
            ContactSensorCfg,
            FrameTransformerCfg,
            ImuCfg,
            MultiMeshRayCasterCameraCfg,
            MultiMeshRayCasterCfg,
            RayCasterCameraCfg,
            RayCasterCfg,
            TiledCameraCfg,
        )
    except ImportError:
        pass
    else:
        mapping.update(
            {
                ImuCfg: SensorCategory.PROPRIOCEPTION,
                ContactSensorCfg: SensorCategory.CONTACT,
                CameraCfg: SensorCategory.VISUAL,
                TiledCameraCfg: SensorCategory.VISUAL,
                RayCasterCameraCfg: SensorCategory.VISUAL,
                MultiMeshRayCasterCameraCfg: SensorCategory.VISUAL,
                RayCasterCfg: SensorCategory.TERRAIN,
                MultiMeshRayCasterCfg: SensorCategory.TERRAIN,
                FrameTransformerCfg: SensorCategory.TRANSFORM,
            }
        )

    try:
        from srb.core.sensor.heightmap import HeightmapSensorCfg

        mapping[HeightmapSensorCfg] = SensorCategory.TERRAIN
    except (ImportError, AttributeError):
        # HeightmapSensorCfg requires Isaac Lab; skip when unavailable.
        pass

    try:
        from isaaclab_contrib.sensors.tacsl_sensor import VisuoTactileSensorCfg

        mapping[VisuoTactileSensorCfg] = SensorCategory.TACTILE
    except ImportError:
        pass

    _SENSOR_CATEGORY_REGISTRY.update(mapping)


def get_sensor_category(cfg: "SensorBaseCfg") -> SensorCategory:
    """Determine the category of a sensor config.

    Resolution order:
    1. Walk MRO to find a registered type (supports subclasses automatically)
    2. Check for explicit `.category` attribute (for third-party sensors)
    3. Fall back to CUSTOM
    """
    global _registry_initialized
    if not _registry_initialized:
        with _registry_lock:
            # Re-check under the lock: another thread may have completed
            # initialisation between the test above and acquiring it.
            if not _registry_initialized:
                _init_registry()
                _registry_initialized = True

    for cfg_type in type(cfg).__mro__:
        if cfg_type in _SENSOR_CATEGORY_REGISTRY:
            return _SENSOR_CATEGORY_REGISTRY[cfg_type]

    if hasattr(cfg, "category") and isinstance(cfg.category, SensorCategory):
        return cfg.category

    return SensorCategory.CUSTOM


def register_sensor_category(cfg_type: type, category: SensorCategory) -> None:
    """Register a sensor config type to a category.

    Third-party users can call this to register their custom sensor types,
    or alternatively add a `category: SensorCategory` attribute directly
    on their sensor config class.
    """
    _SENSOR_CATEGORY_REGISTRY[cfg_type] = category
