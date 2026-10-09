"""Isaac-free event-camera cfg wiring for sample collection.

The camera mounts on the given manipulator flange (``panda_link7`` for Franka).
"""

from __future__ import annotations

from srb.core.sensor.event_camera import EventCameraCfg, build_event_camera_cfg

_WRIST_PRIM_NAME = "event_camera_wrist"
_WRIST_OFFSET_POS = (0.04, 0.0, 0.04)


def build_wrist_event_camera_cfg(
    flange_path: str,
    *,
    preset_name: str = "imx636_nominal",
    backend: str = "v2e",
    sub_render_hz: float = 250.0,
    resolution: tuple[int, int] = (1280, 720),
) -> EventCameraCfg:
    """Build a wrist-mounted IMX636 cfg; caller parameters feed provenance."""
    return build_event_camera_cfg(
        f"{flange_path}/{_WRIST_PRIM_NAME}",
        _WRIST_OFFSET_POS,
        preset_name=preset_name,
        backend=backend,
        sub_render_hz=sub_render_hz,
        resolution=resolution,
    )


__all__ = [
    "build_wrist_event_camera_cfg",
]
