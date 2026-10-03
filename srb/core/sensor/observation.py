from __future__ import annotations

import threading
from collections.abc import Callable
from typing import TYPE_CHECKING

import torch

from srb.core.sensor.visual_output import (
    construct_visual_observation,
    visual_clipping_range,
    visual_image_basename,
)

if TYPE_CHECKING:
    from isaaclab.sensors import SensorBase

    from srb.core.sensor import SensorBaseCfg

ObservationBuilder = Callable[[str, "SensorBase"], dict[str, torch.Tensor]]

_OBSERVATION_BUILDERS: dict[type, ObservationBuilder] = {}
_builders_initialized = False
#: Prevents readers from seeing a partially initialized builder registry.
_builders_lock = threading.Lock()


def _init_builders():
    """Lazily register available built-in builders; missing types are skipped."""
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
        # Isaac-free sensor builders register through their package imports.
        pass
    else:
        builtins = {
            ImuCfg: build_imu_observation,
            ContactSensorCfg: build_contact_observation,
            CameraCfg: build_camera_observation,
            TiledCameraCfg: build_camera_observation,
            RayCasterCameraCfg: build_camera_observation,
            MultiMeshRayCasterCameraCfg: build_camera_observation,
            RayCasterCfg: build_raycaster_observation,
            MultiMeshRayCasterCfg: build_raycaster_observation,
            FrameTransformerCfg: build_frame_transformer_observation,
        }
        for cfg_type, builder in builtins.items():
            _OBSERVATION_BUILDERS.setdefault(cfg_type, builder)

    try:
        from isaaclab_contrib.sensors.tacsl_sensor import VisuoTactileSensorCfg

        _OBSERVATION_BUILDERS[VisuoTactileSensorCfg] = build_tactile_observation
    except ImportError:
        pass


def get_observation_builder(cfg: "SensorBaseCfg") -> ObservationBuilder | None:
    """Find the registered builder, walking the config type's MRO."""
    global _builders_initialized
    if not _builders_initialized:
        with _builders_lock:
            if not _builders_initialized:
                _init_builders()
                _builders_initialized = True

    for cfg_type in type(cfg).__mro__:
        if cfg_type in _OBSERVATION_BUILDERS:
            return _OBSERVATION_BUILDERS[cfg_type]
    return None


def register_observation_builder(cfg_type: type, builder: ObservationBuilder) -> None:
    """Register a custom observation builder for a sensor config type."""
    _OBSERVATION_BUILDERS[cfg_type] = builder


def build_imu_observation(name: str, sensor: "SensorBase") -> dict[str, torch.Tensor]:
    return {
        f"{name}_lin_acc": sensor.data.lin_acc_b,
        f"{name}_ang_vel": sensor.data.ang_vel_b,
    }


def build_contact_observation(
    name: str, sensor: "SensorBase"
) -> dict[str, torch.Tensor]:
    return {
        f"{name}_net_forces": sensor.data.net_forces_w,
    }


def build_frame_transformer_observation(
    name: str, sensor: "SensorBase"
) -> dict[str, torch.Tensor]:
    return {
        f"{name}_pos": sensor.data.target_pos_source,
        f"{name}_quat": sensor.data.target_quat_source,
    }


def build_raycaster_observation(
    name: str, sensor: "SensorBase"
) -> dict[str, torch.Tensor]:
    return {
        f"{name}_ray_hits": sensor.data.ray_hits_w,
    }


def build_camera_observation(
    name: str, sensor: "SensorBase"
) -> dict[str, torch.Tensor]:
    return construct_visual_observation(
        image_basename=visual_image_basename(name),
        data_types=sensor.cfg.data_types,
        clipping_range=visual_clipping_range(sensor.cfg),
        merge_channels=True,
        **sensor.data.output,
    )


def build_tactile_observation(
    name: str, sensor: "SensorBase"
) -> dict[str, torch.Tensor]:
    obs: dict[str, torch.Tensor] = {}
    if hasattr(sensor.data, "output"):
        for key, value in sensor.data.output.items():
            obs[f"{name}_{key}"] = value
    if hasattr(sensor.data, "contact_forces"):
        obs[f"{name}_forces"] = sensor.data.contact_forces
    return obs


__all__ = [
    "ObservationBuilder",
    "build_camera_observation",
    "build_contact_observation",
    "build_frame_transformer_observation",
    "build_imu_observation",
    "build_raycaster_observation",
    "build_tactile_observation",
    "get_observation_builder",
    "register_observation_builder",
]
