from typing import Dict, Sequence, Tuple

import torch

from srb.core.env import InteractiveScene
from srb.utils.str import sanitize_cam_name

from .cfg import VisualExtCfg


class VisualExt:
    ## Subclass requirements
    scene: InteractiveScene

    def __init__(self, cfg: VisualExtCfg, **kwargs):
        self.__cameras = [
            (
                self.scene.sensors[camera_key],
                f"image_{sanitize_cam_name(camera_key)}",
                cfg.cameras_cfg[camera_key].data_types,
                cfg.cameras_cfg[camera_key].spawn.clipping_range,  # type: ignore
            )
            for camera_key in cfg.cameras_cfg.keys()
        ]

    def _get_observations(self) -> Dict[str, torch.Tensor]:
        return {
            image_key: image
            for camera, image_basename, data_types, clipping_range in self.__cameras
            for image_key, image in construct_observation(
                image_basename=image_basename,
                data_types=data_types,
                clipping_range=clipping_range,  # type: ignore
                **camera.data.output,
            ).items()
        }


def construct_observation(
    *,
    image_basename: str,
    data_types: Sequence[str],
    clipping_range: Tuple[float, float],
    merge_channels: bool = True,
    as_u8: bool = True,
    **kwargs,
) -> Dict[str, torch.Tensor]:
    processors = _PROCESSORS_U8 if as_u8 else _PROCESSORS_F32
    images = {
        f"{image_basename}_{data_type}": processors[data_type](
            kwargs[data_type], clipping_range
        )
        if data_type == "depth"
        else processors[data_type](kwargs[data_type])
        for data_type in data_types
    }
    if merge_channels:
        return {
            image_basename: torch.cat(
                [images[image_key] for image_key in images.keys()],
                dim=-1,
            )
        }
    else:
        return images


@torch.jit.script
def process_img_u8_as_u8(image: torch.Tensor) -> torch.Tensor:
    return image


@torch.jit.script
def process_img_u8_as_f32(image: torch.Tensor) -> torch.Tensor:
    return image.to(torch.float32) / 255.0


@torch.jit.script
def process_img_f32_as_u8(image: torch.Tensor) -> torch.Tensor:
    return (255.0 * image).to(torch.uint8)


@torch.jit.script
def process_img_f32_as_f32(image: torch.Tensor) -> torch.Tensor:
    return image


@torch.jit.script
def process_rgb_u8(image: torch.Tensor) -> torch.Tensor:
    return image[..., :3]


@torch.jit.script
def process_rgb_f32(image: torch.Tensor) -> torch.Tensor:
    return process_rgb_u8(image).to(torch.float32) / 255.0


@torch.jit.script
def process_depth_f32(
    image: torch.Tensor,
    clipping_range: Tuple[float, float],
) -> torch.Tensor:
    return (
        image.nan_to_num(
            nan=clipping_range[1], posinf=clipping_range[1], neginf=clipping_range[1]
        ).clamp(clipping_range[0], clipping_range[1])
        - clipping_range[0]
    ) / (clipping_range[1] - clipping_range[0])


@torch.jit.script
def process_depth_u8(
    image: torch.Tensor,
    clipping_range: Tuple[float, float],
) -> torch.Tensor:
    return (255.0 * process_depth_f32(image, clipping_range)).to(torch.uint8)


_PROCESSORS_U8 = {
    "rgb": process_rgb_u8,
    "depth": process_depth_u8,
    "distance_to_camera": process_depth_u8,
    "normals": process_rgb_u8,
    # "motion_vectors": ...,
    "semantic_segmentation": process_img_u8_as_u8,
    "instance_segmentation_fast": process_img_u8_as_u8,
    "instance_id_segmentation_fast": process_img_u8_as_u8,
}

_PROCESSORS_F32 = {
    "rgb": process_rgb_f32,
    "depth": process_depth_f32,
    "distance_to_camera": process_depth_f32,
    "normals": process_rgb_f32,
    "motion_vectors": process_img_f32_as_f32,
    "semantic_segmentation": process_img_u8_as_f32,
    "instance_segmentation_fast": process_img_u8_as_f32,
    "instance_id_segmentation_fast": process_img_u8_as_f32,
}
