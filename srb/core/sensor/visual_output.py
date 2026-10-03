"""Metadata + observation construction for visual sensors.

External-sink codecs (video frames, ROS payloads) live in
:mod:`srb.core.sensor.visual_codec`. Keep this split: the DirectEnv hot path
imports from this module and must not pull in numpy/BGR/ROS encoding helpers.
"""

from __future__ import annotations

import math
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, TypeVar

import torch

_DEPTH_DATA_TYPES = frozenset(
    ("depth", "distance_to_camera", "distance_to_image_plane")
)
# Fallback for VISUAL sensors without ``clipping_range`` or ``max_distance``.
_DEFAULT_VISUAL_CLIPPING_RANGE: tuple[float, float] = (0.0, 1000.0)
# Far planes at or above this render-default bound would quantize real depth
# values to uint8 zero.
_IMPLAUSIBLE_DEPTH_FAR_PLANE: float = 1.0e3
# Substitute this observation bound for implausible far planes; keep explicit
# task ranges such as ``ground_manipulation``'s 4.99 m.
_DEPTH_OBSERVATION_MAX_RANGE: float = 20.0
_POINTCLOUD_DEPTH_PRIORITY = (
    "depth",
    "distance_to_image_plane",
    "distance_to_camera",
)

T = TypeVar("T")


@dataclass(frozen=True)
class VisualChannel:
    """One data type's channel slice inside a merged ``image_<camera>`` tensor.

    ``encoding`` names how the stored values map to physical quantities
    ``rgb_u8``, ``depth_linear_u8``, ``normals_unit``,
    ``segmentation_rgba_u8`` or ``segmentation_id``.
    """

    data_type: str
    start: int
    end: int
    encoding: str


@dataclass(frozen=True)
class VisualSensorOutput:
    sensor_name: str
    sensor: Any
    image_basename: str
    data_types: tuple[str, ...]
    clipping_range: tuple[float, float]


def visual_clipping_range(cfg: Any) -> tuple[float, float]:
    """Resolve clipping from the spawner, ray-caster max distance, or fallback."""
    spawn = getattr(cfg, "spawn", None)
    clipping = getattr(spawn, "clipping_range", None)
    if clipping is not None:
        return tuple(clipping)  # type: ignore[return-value]
    max_distance = getattr(cfg, "max_distance", None)
    if (
        isinstance(max_distance, (int, float))
        and not isinstance(max_distance, bool)
        and math.isfinite(max_distance)
        and max_distance > 0.0
    ):
        return (0.0, float(max_distance))
    return _DEFAULT_VISUAL_CLIPPING_RANGE


def visual_depth_observation_range(
    clipping_range: tuple[float, float],
) -> tuple[float, float]:
    """Map implausible render far planes to the uint8 observation bound."""
    near, far = float(clipping_range[0]), float(clipping_range[1])
    if far >= _IMPLAUSIBLE_DEPTH_FAR_PLANE and near < _DEPTH_OBSERVATION_MAX_RANGE:
        return (near, _DEPTH_OBSERVATION_MAX_RANGE)
    return (near, far)


def visual_image_basename(sensor_name: str) -> str:
    from srb.utils.str import sanitize_cam_name

    return f"image_{sanitize_cam_name(sensor_name)}"


def visual_stream_name(data_type: str) -> str:
    if data_type == "distance_to_image_plane":
        return "image_depth"
    return f"image_{data_type}"


def is_visual_data_type(data_type: str) -> bool:
    return data_type in _VISUAL_OBSERVATION_PROCESSORS


def is_visual_depth_data_type(data_type: str) -> bool:
    return data_type in _DEPTH_DATA_TYPES


def has_visual_depth_stream(data_types: Sequence[str]) -> bool:
    return any(is_visual_depth_data_type(data_type) for data_type in data_types)


def select_visual_depth_data_type(outputs: Mapping[str, object]) -> str | None:
    for data_type in _POINTCLOUD_DEPTH_PRIORITY:
        if data_type in outputs:
            return data_type
    return None


def select_visual_rgb_output(outputs: Mapping[str, T]) -> T | None:
    if "rgb" in outputs:
        return outputs["rgb"]
    if "rgba" in outputs:
        return outputs["rgba"][..., :3]  # type: ignore[index]
    return None


def iter_visual_sensor_outputs(scene: object) -> Iterator[VisualSensorOutput]:
    from srb.core.scene_introspection import iter_scene_sensors
    from srb.core.sensor.category import SensorCategory, get_sensor_category
    from srb.core.sensor.event_camera.materialization import (
        get_bound_event_camera_cfgs,
    )

    event_camera_sensor_names = get_bound_event_camera_cfgs(scene)
    # `cam_front` and `camera_front` both map to `image_front`; one would vanish.
    basenames: dict[str, str] = {}
    for sensor_name, sensor in iter_scene_sensors(scene):
        if sensor_name in event_camera_sensor_names:
            continue
        if get_sensor_category(sensor.cfg) != SensorCategory.VISUAL:
            continue
        image_basename = visual_image_basename(sensor_name)
        if (other := basenames.setdefault(image_basename, sensor_name)) != sensor_name:
            raise ValueError(
                f"Cameras {other!r} and {sensor_name!r} both map to observation "
                f"{image_basename!r}; rename one"
            )
        yield VisualSensorOutput(
            sensor_name=sensor_name,
            sensor=sensor,
            image_basename=image_basename,
            data_types=tuple(sensor.cfg.data_types),
            clipping_range=visual_clipping_range(sensor.cfg),
        )


def scene_has_visual_sensors(scene: object) -> bool:
    return next(iter_visual_sensor_outputs(scene), None) is not None


def _as_tensor(value: Any) -> torch.Tensor:
    """Unwrap Isaac Sim ``ProxyArray`` values; TorchScript requires tensors."""
    if isinstance(value, torch.Tensor):
        return value
    torch_view = getattr(value, "torch", None)
    return torch_view if isinstance(torch_view, torch.Tensor) else value


def construct_visual_observation(
    *,
    image_basename: str,
    data_types: Sequence[str],
    clipping_range: tuple[float, float],
    merge_channels: bool = True,
    **kwargs: torch.Tensor,
) -> dict[str, torch.Tensor]:
    depth_range = visual_depth_observation_range(clipping_range)
    images = {
        f"{image_basename}_{data_type}": (
            _VISUAL_OBSERVATION_PROCESSORS[data_type](
                _as_tensor(kwargs[data_type]), depth_range
            )
            if is_visual_depth_data_type(data_type)
            else _VISUAL_OBSERVATION_PROCESSORS[data_type](
                _as_tensor(kwargs[data_type])
            )
        )
        for data_type in data_types
    }
    if merge_channels:
        return {
            image_basename: torch.cat(
                [images[image_key] for image_key in images.keys()],
                dim=-1,
            )
        }
    return images


_SEGMENTATION_DATA_TYPES = frozenset(
    (
        "semantic_segmentation",
        "instance_segmentation_fast",
        "instance_id_segmentation_fast",
    )
)


def visual_channel_encoding(data_type: str, image: torch.Tensor) -> str:
    """Name the stored-value encoding of one processed visual data type."""
    if data_type in ("rgb", "rgba"):
        return "rgb_u8"
    if is_visual_depth_data_type(data_type):
        return "depth_linear_u8"
    if data_type == "normals":
        return "normals_unit"
    if data_type in _SEGMENTATION_DATA_TYPES:
        if image.dtype == torch.uint8 and int(image.shape[-1]) == 4:
            return "segmentation_rgba_u8"
        return "segmentation_id"
    raise ValueError(f"Unsupported visual data type {data_type!r}")


def visual_channel_layout(
    images: Mapping[str, torch.Tensor],
    *,
    image_basename: str,
    data_types: Sequence[str],
) -> tuple[VisualChannel, ...]:
    """Channel slices of ``construct_visual_observation(merge_channels=False)``.

    The slices follow the concatenation order of the merged tensor, so they
    tile its last dimension.
    """
    channels: list[VisualChannel] = []
    start = 0
    for data_type in data_types:
        image = images[f"{image_basename}_{data_type}"]
        width = int(image.shape[-1])
        channels.append(
            VisualChannel(
                data_type=data_type,
                start=start,
                end=start + width,
                encoding=visual_channel_encoding(data_type, image),
            )
        )
        start += width
    return tuple(channels)


@torch.jit.script
def process_img_u8_as_u8(image: torch.Tensor) -> torch.Tensor:
    return image


@torch.jit.script
def process_rgb_u8(image: torch.Tensor) -> torch.Tensor:
    return image[..., :3]


@torch.jit.script
def process_depth_f32(
    image: torch.Tensor,
    clipping_range: tuple[float, float],
) -> torch.Tensor:
    return (
        image.nan_to_num(
            nan=clipping_range[1],
            posinf=clipping_range[1],
            neginf=clipping_range[1],
        ).clamp(clipping_range[0], clipping_range[1])
        - clipping_range[0]
    ) / (clipping_range[1] - clipping_range[0])


@torch.jit.script
def process_depth_u8(
    image: torch.Tensor,
    clipping_range: tuple[float, float],
) -> torch.Tensor:
    return (255.0 * process_depth_f32(image, clipping_range)).round().to(torch.uint8)


_VISUAL_OBSERVATION_PROCESSORS = {
    "rgb": process_rgb_u8,
    "rgba": process_rgb_u8,
    "depth": process_depth_u8,
    "distance_to_camera": process_depth_u8,
    "distance_to_image_plane": process_depth_u8,
    "normals": process_rgb_u8,
    "semantic_segmentation": process_img_u8_as_u8,
    "instance_segmentation_fast": process_img_u8_as_u8,
    "instance_id_segmentation_fast": process_img_u8_as_u8,
}
