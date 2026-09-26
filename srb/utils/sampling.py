from typing import Iterable

import numpy as np
import torch
from isaaclab.envs.mdp.events import _randomize_prop_by_op  # noqa: F401
from isaaclab.utils.math import (  # noqa: F401
    sample_cylinder,
    sample_gaussian,
    sample_log_uniform,
    sample_triangle,
    sample_uniform,
)
from oxidasim.sampling import (  # noqa: F401
    sample_poisson_disk_2d,
    sample_poisson_disk_2d_looped,
    sample_poisson_disk_2d_parallel,
    sample_poisson_disk_3d,
    sample_poisson_disk_3d_looped,
    sample_poisson_disk_3d_parallel,
)
from pxr import Gf


def sample_grid(
    num_instances: int,
    spacing: float,
    global_pos_offset: np.ndarray | torch.Tensor | Iterable | None = None,
    global_rot_offset: np.ndarray | torch.Tensor | Iterable | None = None,
) -> tuple[
    tuple[int, int],
    tuple[list[tuple[float, float, float]], list[tuple[float, float, float, float]]],
]:
    # Zero instances would divide by zero when sizing the grid.
    if num_instances < 1:
        raise ValueError(
            f"sample_grid needs at least one instance, got {num_instances}"
        )
    if global_pos_offset is not None:
        if isinstance(global_pos_offset, torch.Tensor):
            global_pos_offset = global_pos_offset.detach().cpu().numpy()
        elif not isinstance(global_pos_offset, np.ndarray):
            global_pos_offset = np.asarray(global_pos_offset)
    if global_rot_offset is not None:
        if isinstance(global_rot_offset, torch.Tensor):
            global_rot_offset = global_rot_offset.detach().cpu().numpy()
        elif not isinstance(global_rot_offset, np.ndarray):
            global_rot_offset = np.asarray(global_rot_offset)

    num_per_row = np.ceil(np.sqrt(num_instances))
    num_rows = int(np.ceil(num_instances / num_per_row))
    num_cols = int(np.ceil(num_instances / num_rows))

    row_offset = 0.5 * spacing * (num_rows - 1)
    col_offset = 0.5 * spacing * (num_cols - 1)

    positions = []
    orientations = []

    for i in range(num_instances):
        row = i // num_cols
        col = i % num_cols
        x = row_offset - row * spacing
        y = col * spacing - col_offset

        position = [x, y, 0]
        if global_pos_offset is not None:
            translation = tuple((global_pos_offset + position).tolist())
        else:
            translation = position
        positions.append(translation)

        orientation: Gf.Quatd = Gf.Quatd.GetIdentity()  # type: ignore[misc]
        if global_rot_offset is not None:
            # global_rot_offset is XYZW; Gf.Quatd expects (w, Vec3d(x,y,z))
            orientation = (
                Gf.Quatd(
                    global_rot_offset[3].item(),
                    Gf.Vec3d(global_rot_offset[:3].tolist()),
                )
                * orientation
            )
        orientations.append(
            (
                orientation.GetImaginary()[0],
                orientation.GetImaginary()[1],
                orientation.GetImaginary()[2],
                orientation.GetReal(),
            )
        )

    return ((num_rows, num_cols), (positions, orientations))
