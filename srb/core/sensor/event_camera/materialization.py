"""Materialize Isaac-free event-camera configs at the Isaac scene seam.

The adapter builds a native RGB source while retaining the original config in
scene-side metadata for downstream event-camera code.
"""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType
from typing import Any, Callable, TypeAlias

from .config import EventCameraCfg

EVENT_CAMERA_BINDINGS_ATTR = "_srb_event_camera_bindings"
CameraTypeLoader: TypeAlias = Callable[[], tuple[type, type]]


class EventCameraMaterializationError(RuntimeError):
    """Raised when an event-camera source cannot be bound to the built scene."""


def _load_isaac_camera_types() -> tuple[type, type]:
    """Load native camera config types only when scene materialization needs them."""
    from isaaclab.sensors import CameraCfg
    from isaaclab.sensors.camera.camera_cfg import PinholeCameraCfg

    return CameraCfg, PinholeCameraCfg


def _load_native_camera_types() -> tuple[type, type]:
    """Private loader seam retained for Isaac-free tests and integrations."""
    return _load_isaac_camera_types()


def materialize_event_camera_cfg(
    cfg: EventCameraCfg,
    *,
    type_loader: CameraTypeLoader | None = None,
) -> Any:
    """Build the native RGB source config represented by ``cfg``.

    Loads Isaac types lazily, leaves ``cfg`` unchanged, and returns the native
    ``CameraCfg``. The event config is retained by
    :func:`bind_event_camera_sensors`.
    """
    if not isinstance(cfg, EventCameraCfg):
        raise TypeError(
            "materialize_event_camera_cfg expects EventCameraCfg, "
            f"got {type(cfg).__name__}"
        )

    camera_cfg_type, pinhole_cfg_type = (
        type_loader() if type_loader is not None else _load_native_camera_types()
    )
    offset_cfg_type = getattr(camera_cfg_type, "OffsetCfg", None)
    if offset_cfg_type is None:
        raise TypeError("native CameraCfg does not expose OffsetCfg")

    pinhole_cfg = pinhole_cfg_type(
        focal_length=cfg.focal_length_mm,
        horizontal_aperture=cfg.horizontal_aperture_mm,
        clipping_range=cfg.clipping_range,
    )
    # Isaac Lab 3 ``CameraCfg.OffsetCfg.rot`` is scalar-last (x, y, z, w),
    # the same order as ``rot_xyzw``: pass it through unchanged.
    offset_cfg = offset_cfg_type(
        convention="ros",
        pos=cfg.pos,
        rot=tuple(cfg.rot_xyzw),
    )
    return camera_cfg_type(
        prim_path=cfg.prim_path,
        update_period=1.0 / cfg.sub_render_hz,
        height=cfg.resolution[1],
        width=cfg.resolution[0],
        data_types=["rgb"],
        offset=offset_cfg,
        spawn=pinhole_cfg,
    )


def bind_event_camera_sensors(
    scene: object,
    bindings: Mapping[str, EventCameraCfg],
) -> None:
    """Bind event configs to existing native scene sensors.

    Native sensors remain untouched. Immutable bindings are keyed by logical
    name; every configured sensor must exist in the built scene.
    """
    normalized = dict(bindings)
    for sensor_name, cfg in normalized.items():
        if not isinstance(cfg, EventCameraCfg):
            raise TypeError(
                f"event-camera binding {sensor_name!r} must hold EventCameraCfg, "
                f"got {type(cfg).__name__}"
            )

    if normalized:
        from srb.core.scene_introspection import iter_scene_sensors

        try:
            sensors = dict(iter_scene_sensors(scene))
        except (AttributeError, TypeError) as exc:
            raise EventCameraMaterializationError(
                "cannot inspect built scene sensors for event-camera bindings"
            ) from exc

        missing = tuple(
            sensor_name
            for sensor_name in normalized
            if sensor_name not in sensors or sensors[sensor_name] is None
        )
        if missing:
            names = ", ".join(repr(name) for name in missing)
            raise EventCameraMaterializationError(
                "configured event-camera native source sensor(s) missing from "
                f"built scene: {names}"
            )

        unmaterialized = tuple(
            sensor_name
            for sensor_name in normalized
            if isinstance(getattr(sensors[sensor_name], "cfg", None), EventCameraCfg)
            or getattr(sensors[sensor_name], "cfg", None) is None
        )
        if unmaterialized:
            names = ", ".join(repr(name) for name in unmaterialized)
            raise EventCameraMaterializationError(
                "configured event-camera source sensor(s) do not have a native "
                f"cfg after materialization: {names}"
            )

    try:
        setattr(scene, EVENT_CAMERA_BINDINGS_ATTR, MappingProxyType(normalized))
    except (AttributeError, TypeError) as exc:
        raise EventCameraMaterializationError(
            "built scene cannot retain event-camera bindings"
        ) from exc


def get_bound_event_camera_cfgs(scene: object) -> Mapping[str, EventCameraCfg]:
    """Return the original event configs bound to ``scene`` sensors."""
    bindings = getattr(scene, EVENT_CAMERA_BINDINGS_ATTR, {})
    if not isinstance(bindings, Mapping):
        raise TypeError(
            f"scene.{EVENT_CAMERA_BINDINGS_ATTR} must be a mapping, "
            f"got {type(bindings).__name__}"
        )
    return bindings


__all__ = [
    "EVENT_CAMERA_BINDINGS_ATTR",
    "CameraTypeLoader",
    "EventCameraMaterializationError",
    "bind_event_camera_sensors",
    "get_bound_event_camera_cfgs",
    "materialize_event_camera_cfg",
]
