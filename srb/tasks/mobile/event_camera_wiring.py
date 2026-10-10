"""Build mobile-task event-camera configs without importing Isaac Sim.

Landing and Moonfall share the downward-looking Peregrine onboard mount;
velocity tracking and waypoint navigation share Cadre's front mount. Both
builders default to the ``"v2e"`` backend; ``"noop"`` supports backend-free
smoke runs. Preset, resolution, and sub-render rate are recorded in provenance.
"""

from __future__ import annotations

from srb.core.sensor.event_camera import (
    EventCameraCfg,
    build_event_camera_cfg,
    ros_rot_from_world_convention,
)

_ONBOARD_PRIM_NAME = "event_camera_onboard"
_ONBOARD_OFFSET_POS = (-0.685, 0.0, 0.55)
# The Peregrine frame camera pitches 90 degrees in the world convention
# (looking down); materialization offsets event cameras in the ROS convention.
_ONBOARD_OFFSET_ROT = ros_rot_from_world_convention((0.0, 0.5**0.5, 0.0, 0.5**0.5))

_FRONT_PRIM_NAME = "event_camera_front"
_FRONT_OFFSET_POS = (0.2, 0.0, 0.1)
# Looks along the chassis +X like the Cadre frame camera (identity in the
# world convention); materialization offsets event cameras in ROS convention.
_FRONT_OFFSET_ROT = ros_rot_from_world_convention((0.0, 0.0, 0.0, 1.0))


def build_onboard_event_camera_cfg(
    base_path: str,
    *,
    preset_name: str = "imx636_nominal",
    backend: str = "v2e",
    sub_render_hz: float = 250.0,
    resolution: tuple[int, int] = (1280, 720),
) -> EventCameraCfg:
    """Build the downward IMX636 config on a Peregrine base (landing, Moonfall)."""
    return build_event_camera_cfg(
        f"{base_path}/{_ONBOARD_PRIM_NAME}",
        _ONBOARD_OFFSET_POS,
        rot_xyzw=_ONBOARD_OFFSET_ROT,
        preset_name=preset_name,
        backend=backend,
        sub_render_hz=sub_render_hz,
        resolution=resolution,
    )


def build_front_event_camera_cfg(
    base_path: str,
    *,
    preset_name: str = "imx636_nominal",
    backend: str = "v2e",
    sub_render_hz: float = 250.0,
    resolution: tuple[int, int] = (1280, 720),
) -> EventCameraCfg:
    """Build the front IMX636 config on Cadre's ``chassis`` (velocity tracking,
    waypoint navigation)."""
    return build_event_camera_cfg(
        f"{base_path}/{_FRONT_PRIM_NAME}",
        _FRONT_OFFSET_POS,
        rot_xyzw=_FRONT_OFFSET_ROT,
        preset_name=preset_name,
        backend=backend,
        sub_render_hz=sub_render_hz,
        resolution=resolution,
    )


__all__ = [
    "build_front_event_camera_cfg",
    "build_onboard_event_camera_cfg",
]
