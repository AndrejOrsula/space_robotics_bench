"""Event-camera sensor (synthetic IMX636) for SRB.

The package owns the standalone event-camera sensor. It is layered so the
Isaac-free parts (preset registry, config
dataclass, representation shapes) are importable in unit tests without booting
Isaac Lab, while the renderer + backend wiring stay behind ``_HAS_ISAAC``
guards.
"""

from __future__ import annotations

from srb.core.sensor.category import SensorCategory, register_sensor_category
from srb.core.sensor.observation import register_observation_builder

from . import backends  # noqa: F401  (side-effect: registers v2e + future backends)
from .backend import (
    EventGenerator,
    EventStreamNoProgressError,
    NoopBackend,
    get_backend_factory,
    list_backends,
    make_backend,
    register_backend,
)
from .config import (
    EventCameraCfg,
    build_event_camera_cfg,
    ros_rot_from_world_convention,
)
from .extras import MissingExtraError
from .observation import (
    build_event_observation,
    representation_shape,
)
from .preset import (
    IMX636Preset,
    get_preset,
    list_presets,
    register_preset,
)
from .recorder import (
    EventManifestRow,
    EventRecorder,
    EventSensorRef,
    iter_event_camera_sensors,
)
from .representations import (
    to_histogram,
    to_raw,
    to_stacked,
    to_time_surface,
    to_voxel_grid,
)
from .seeding import (
    bake_hot_pixel_mask,
    derive_episode_event_seed_pack,
    derive_event_camera_env_seed,
)
from .storage import (
    EVENT_HDF5_VERSION,
    EventEpisodeMetadata,
    EventEpisodeProvenance,
    EventHDF5Writer,
    read_event_hdf5,
)
from .storage_validation import (
    EventHDF5ValidationReport,
    read_validated_event_hdf5,
    validate_event_hdf5_schema,
)
from .weights import (
    WeightArtifact,
    WeightHashMismatchError,
    WeightPlaceholderError,
    fetch_weight_artifact,
)

register_sensor_category(EventCameraCfg, SensorCategory.EVENT_CAMERA)
register_observation_builder(EventCameraCfg, build_event_observation)


__all__ = [
    "EVENT_HDF5_VERSION",
    "EventCameraCfg",
    "EventEpisodeMetadata",
    "EventEpisodeProvenance",
    "EventGenerator",
    "EventHDF5Writer",
    "EventHDF5ValidationReport",
    "EventManifestRow",
    "EventRecorder",
    "EventSensorRef",
    "EventStreamNoProgressError",
    "IMX636Preset",
    "bake_hot_pixel_mask",
    "build_event_camera_cfg",
    "ros_rot_from_world_convention",
    "derive_episode_event_seed_pack",
    "derive_event_camera_env_seed",
    "MissingExtraError",
    "NoopBackend",
    "WeightArtifact",
    "WeightHashMismatchError",
    "WeightPlaceholderError",
    "build_event_observation",
    "fetch_weight_artifact",
    "get_backend_factory",
    "get_preset",
    "iter_event_camera_sensors",
    "list_backends",
    "list_presets",
    "make_backend",
    "read_event_hdf5",
    "read_validated_event_hdf5",
    "register_backend",
    "register_preset",
    "representation_shape",
    "to_histogram",
    "to_raw",
    "to_stacked",
    "to_time_surface",
    "to_voxel_grid",
    "validate_event_hdf5_schema",
]
