"""Isaac-free config wiring for the excavation wrist event camera."""

from __future__ import annotations

from srb.core.sensor.event_camera import EventCameraCfg, build_event_camera_cfg

# Attach at Kinova Gen3's ``cam_wrist`` point; the scoop sweep provides motion.
_WRIST_PRIM_NAME = "event_camera_wrist"
_WRIST_OFFSET_POS = (0.0, 0.061, -0.003)


def build_wrist_event_camera_cfg(
    flange_path: str,
    *,
    preset_name: str = "imx636_nominal",
    backend: str = "v2e",
    sub_render_hz: float = 250.0,
    resolution: tuple[int, int] = (1280, 720),
) -> EventCameraCfg:
    """Build the wrist camera config; defaults to v2e, with provenance fields."""
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
