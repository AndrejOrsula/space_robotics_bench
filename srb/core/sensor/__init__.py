try:
    from isaaclab.sensors import *  # noqa: F403
    from isaaclab.sensors import patterns as _isaac_patterns
    from isaaclab.sensors.camera.camera_cfg import PinholeCameraCfg  # noqa: F401

    del _isaac_patterns
    _HAS_ISAACLAB_SENSORS = True
except ImportError:
    _HAS_ISAACLAB_SENSORS = False

if _HAS_ISAACLAB_SENSORS:
    from . import patterns  # noqa: F401,E402
from .category import (  # noqa: F401,E402
    SensorCategory,
    get_sensor_category,
    register_sensor_category,
)
from .episode_recording import (  # noqa: F401,E402
    EpisodeRecordingRuntime,
    make_episode_recording_runtime,
    scene_has_event_camera_sensors,
    scene_has_recordable_sensors,
)
from .heightmap import (  # noqa: F401,E402
    HeightmapNoiseCfg,
    HeightmapSensorData,
    HeightmapSpec,
    apply_heightmap_noise,
    postprocess_heightmap_observation,
    project_depth_to_heightmap,
)
from .observation import (  # noqa: F401,E402
    ObservationBuilder,
    get_observation_builder,
    register_observation_builder,
)
from .visual_codec import (  # noqa: F401,E402
    RosImagePayload,
    ros_image_payload,
    video_frame_from_visual_image,
)
from .visual_output import (  # noqa: F401,E402
    VisualSensorOutput,
    construct_visual_observation,
    has_visual_depth_stream,
    is_visual_depth_data_type,
    iter_visual_sensor_outputs,
    scene_has_visual_sensors,
    select_visual_depth_data_type,
    select_visual_rgb_output,
    visual_image_basename,
    visual_stream_name,
)


def _is_optional_contact_sensor_import_error(exc: ImportError) -> bool:
    """Return whether the contact-sensor patch missed optional pxr bindings."""
    return getattr(exc, "name", None) == "pxr"


if _HAS_ISAACLAB_SENSORS:
    try:
        from .contact_sensor import ContactSensor, ContactSensorCfg  # noqa: F401,E402
    except ImportError as exc:
        if not _is_optional_contact_sensor_import_error(exc):
            raise
    from . import imu as _imu_patch  # noqa: F401,E402

try:
    from .heightmap import HeightmapSensor, HeightmapSensorCfg  # noqa: F401
except ImportError:
    pass
