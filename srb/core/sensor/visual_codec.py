"""External-sink codecs for visual sensor outputs.

This module owns conversion of visual sensor outputs to formats consumed by
sinks outside the simulation step path: BGR ``uint8`` frames for video writers
(cv2 / ffmpeg) and ROS sensor_msgs/Image-shaped payloads.

The metadata + construction tier of visual handling lives in
:mod:`srb.core.sensor.visual_output`. Keep these two modules separate: nothing
on the DirectEnv / ObservationAssembly hot path should need to import the
numpy/BGR helpers that live here.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch

from srb.core.sensor.visual_output import is_visual_depth_data_type


@dataclass(frozen=True)
class RosImagePayload:
    encoding: str
    step: int
    data: bytes


def video_frame_from_visual_image(image: torch.Tensor | np.ndarray) -> np.ndarray:
    frame = _as_numpy_array(image)
    frame = _as_uint8_image(frame)

    if frame.ndim == 2:
        frame = np.repeat(frame[..., np.newaxis], 3, axis=-1)
    elif frame.ndim == 3 and frame.shape[-1] == 1:
        frame = np.repeat(frame, 3, axis=-1)
    elif frame.ndim == 3 and frame.shape[-1] == 4:
        frame = frame[..., [2, 1, 0]]
    elif frame.ndim == 3 and frame.shape[-1] == 3:
        frame = frame[..., [2, 1, 0]]

    return np.ascontiguousarray(frame)


def ros_image_payload(data_type: str, image: np.ndarray) -> RosImagePayload | None:
    img_data = np.asarray(image)
    width = img_data.shape[1]
    channels = _channel_count(img_data)

    if is_visual_depth_data_type(data_type):
        depth = _squeeze_single_channel(img_data).astype(np.float32)
        return RosImagePayload(
            encoding="32FC1",
            step=4 * width,
            data=depth.tobytes(),
        )

    if np.issubdtype(img_data.dtype, np.floating) and (
        data_type == "normals" or channels == 2
    ):
        # Signed float vectors (normals, motion vectors) would clip as 8-bit color.
        vectors = np.ascontiguousarray(img_data, dtype=np.float32)
        return RosImagePayload(
            encoding=f"32FC{channels}",
            step=4 * channels * width,
            data=vectors.tobytes(),
        )

    if data_type == "rgba" or channels == 4:
        rgba = _as_uint8_image(img_data)
        return RosImagePayload(
            encoding="rgba8",
            step=4 * width,
            data=rgba.tobytes(),
        )

    if data_type == "rgb" or channels == 3:
        rgb = _as_uint8_image(img_data)
        return RosImagePayload(
            encoding="rgb8",
            step=3 * width,
            data=rgb.tobytes(),
        )

    if (
        channels in (0, 1)
        and np.issubdtype(img_data.dtype, np.integer)
        and img_data.itemsize > 1
    ):
        # Preserve wide segmentation IDs as 32SC1; mono8 would clip and
        # truncate them.
        labels = _squeeze_single_channel(img_data).astype(np.int32)
        return RosImagePayload(
            encoding="32SC1",
            step=4 * width,
            data=labels.tobytes(),
        )

    if channels in (0, 1):
        mono = _squeeze_single_channel(_as_uint8_image(img_data))
        return RosImagePayload(
            encoding="mono8",
            step=width,
            data=mono.tobytes(),
        )

    return None


def _as_numpy_array(image: torch.Tensor | np.ndarray) -> np.ndarray:
    if isinstance(image, torch.Tensor):
        return image.detach().cpu().numpy()
    return np.asarray(image)


def _as_uint8_image(image: np.ndarray) -> np.ndarray:
    if image.dtype == np.uint8:
        return image
    return (255.0 * np.clip(image, 0.0, 1.0)).astype(np.uint8)


def _channel_count(image: np.ndarray) -> int:
    if image.ndim < 3:
        return 0
    return int(image.shape[2])


def _squeeze_single_channel(image: np.ndarray) -> np.ndarray:
    if image.ndim == 3 and image.shape[-1] == 1:
        return image[..., 0]
    return image
