"""Legacy, Isaac-free Leo depth-patch geometry retained for compatibility.

Canonical Raph tasks and deployment use ``SourceGridGeometry`` and the
external-heightmap contract in :mod:`srb.terrain.landscaping_contract`.
Do not extend this rendered-depth grid or its max-range sentinel semantics.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch

from srb.core.sensor.heightmap import bin_points_to_heightmap_grid
from srb.terrain.landscaping_pose import (
    quat_xyzw_to_rotmat_np,
    quat_xyzw_to_rotmat_torch,
    quat_xyzw_to_yaw,
    world_points_to_frame_torch,
)


@dataclass(frozen=True, slots=True)
class LandscapingHeightmapGeometry:
    nx: int = 16
    ny: int = 24
    cell_size_m: float = 0.05
    forward_offset_m: float = 0.40
    lateral_offset_m: float = 0.0
    dead_zone_m: float = 0.40
    frame: str = "leo_base_link"
    max_range_m: float = 3.0

    @property
    def forward_range_m(self) -> tuple[float, float]:
        return (
            self.forward_offset_m,
            self.forward_offset_m + self.ny * self.cell_size_m,
        )

    @property
    def lateral_range_m(self) -> tuple[float, float]:
        half_width = (self.nx * self.cell_size_m) / 2.0
        return (
            self.lateral_offset_m - half_width,
            self.lateral_offset_m + half_width,
        )

    def as_heightmap_spec_kwargs(self) -> dict[str, int | float | str]:
        return {
            "nx": self.nx,
            "ny": self.ny,
            "cell_size_m": self.cell_size_m,
            "forward_offset_m": self.forward_offset_m,
            "lateral_offset_m": self.lateral_offset_m,
            "dead_zone_m": self.dead_zone_m,
            "frame": self.frame,
            "max_range_m": self.max_range_m,
        }

    def ray_starts(self, *, ray_height_m: float) -> list[tuple[float, float, float]]:
        xs = [
            self.forward_offset_m + (i + 0.5) * self.cell_size_m for i in range(self.ny)
        ]
        ys = [
            self.lateral_offset_m + (-self.nx / 2.0 + j + 0.5) * self.cell_size_m
            for j in range(self.nx)
        ]
        return [(x, y, ray_height_m) for x in xs for y in ys]

    def body_xy_grid_np(self) -> np.ndarray:
        """Return ego-centric cell centres as ``(ny, nx, 2)`` float32."""
        xs = np.arange(self.ny, dtype=np.float32)
        ys = np.arange(self.nx, dtype=np.float32)
        x_body = self.forward_offset_m + (xs + 0.5) * self.cell_size_m
        y_body = self.lateral_offset_m + (-self.nx / 2.0 + ys + 0.5) * self.cell_size_m
        gx, gy = np.meshgrid(x_body, y_body, indexing="ij")
        return np.stack([gx, gy], axis=-1).astype(np.float32)

    def body_xy_grid_torch(
        self,
        *,
        device: torch.device | str,
        dtype: torch.dtype,
    ) -> torch.Tensor:
        """Return ego-centric cell centres as ``(ny, nx, 2)``."""
        xs = torch.arange(self.ny, device=device, dtype=dtype)
        ys = torch.arange(self.nx, device=device, dtype=dtype)
        x_body = self.forward_offset_m + (xs + 0.5) * self.cell_size_m
        y_body = self.lateral_offset_m + (-self.nx / 2.0 + ys + 0.5) * self.cell_size_m
        gx, gy = torch.meshgrid(x_body, y_body, indexing="ij")
        return torch.stack([gx, gy], dim=-1)


DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY = LandscapingHeightmapGeometry()


def bin_world_points_to_landscaping_heightmap(
    *,
    points_w: torch.Tensor,
    attach_pos_w: torch.Tensor,
    attach_quat_xyzw: torch.Tensor,
    geometry: LandscapingHeightmapGeometry,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Bin world points directly onto the attach-frame landscaping grid."""
    points_attach = world_points_to_frame_torch(
        points_w=points_w,
        frame_pos_w=attach_pos_w,
        frame_quat_xyzw=attach_quat_xyzw,
    )
    valid = torch.isfinite(points_attach).all(dim=-1)
    return bin_points_to_heightmap_grid(
        points_attach[..., 0],
        points_attach[..., 1],
        points_attach[..., 2],
        valid,
        ny=geometry.ny,
        nx=geometry.nx,
        cell_size_m=geometry.cell_size_m,
        forward_offset_m=geometry.forward_offset_m,
        lateral_offset_m=geometry.lateral_offset_m,
        max_range_m=geometry.max_range_m,
    )


def sample_global_map_at_patch_ranges(
    *,
    global_map: torch.Tensor,
    robot_pos_w: torch.Tensor,
    env_origins: torch.Tensor,
    robot_yaw: torch.Tensor,
    fwd_range: tuple[float, float],
    lat_range: tuple[float, float],
    rows: int,
    cols: int,
    extent: float,
    resolution: int,
) -> torch.Tensor:
    """Bilinearly sample a global square map at ego-centric patch cell centres."""
    sampled, _valid_mask = sample_global_map_at_patch_ranges_with_validity(
        global_map=global_map,
        robot_pos_w=robot_pos_w,
        env_origins=env_origins,
        robot_yaw=robot_yaw,
        fwd_range=fwd_range,
        lat_range=lat_range,
        rows=rows,
        cols=cols,
        extent=extent,
        resolution=resolution,
    )
    return sampled


def sample_global_map_at_patch_ranges_with_validity(
    *,
    global_map: torch.Tensor,
    robot_pos_w: torch.Tensor,
    env_origins: torch.Tensor,
    robot_yaw: torch.Tensor,
    fwd_range: tuple[float, float],
    lat_range: tuple[float, float],
    rows: int,
    cols: int,
    extent: float,
    resolution: int,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Sample a patch and mark cells whose centres lie inside the global map."""
    device = global_map.device
    n_envs = global_map.shape[0]

    fmin, fmax = fwd_range
    lmin, lmax = lat_range
    fwd_step = (fmax - fmin) / rows
    lat_step = (lmax - lmin) / cols

    fwd_c = torch.linspace(
        fmin + fwd_step / 2.0, fmax - fwd_step / 2.0, rows, device=device
    )
    lat_c = torch.linspace(
        lmin + lat_step / 2.0, lmax - lat_step / 2.0, cols, device=device
    )
    gf, gl = torch.meshgrid(fwd_c, lat_c, indexing="ij")
    gf_flat = gf.reshape(1, -1)
    gl_flat = gl.reshape(1, -1)

    cos_y = torch.cos(robot_yaw).unsqueeze(1)
    sin_y = torch.sin(robot_yaw).unsqueeze(1)
    wx = robot_pos_w[:, 0:1] + gf_flat * cos_y - gl_flat * sin_y
    wy = robot_pos_w[:, 1:2] + gf_flat * sin_y + gl_flat * cos_y

    half = extent / 2.0
    cell = extent / resolution
    fx = (wx - env_origins[:, 0:1] + half) / cell - 0.5
    fy = (wy - env_origins[:, 1:2] + half) / cell - 0.5

    # ``fx``/``fy`` address map cell centres: -0.5 and N-0.5 are the physical
    # map edges. Sampling still clamps at the edge, but consumers receive the
    # validity mask so clamped values cannot become observations or success.
    valid_mask = (
        (fx >= -0.5) & (fx < resolution - 0.5) & (fy >= -0.5) & (fy < resolution - 0.5)
    )

    x0 = torch.floor(fx).long().clamp(0, resolution - 1)
    y0 = torch.floor(fy).long().clamp(0, resolution - 1)
    x1 = (x0 + 1).clamp(0, resolution - 1)
    y1 = (y0 + 1).clamp(0, resolution - 1)
    tx = (fx - x0.float()).clamp(0.0, 1.0)
    ty = (fy - y0.float()).clamp(0.0, 1.0)

    env_idx = torch.arange(n_envs, device=device).unsqueeze(1).expand_as(x0)
    v00 = global_map[env_idx, x0, y0]
    v10 = global_map[env_idx, x1, y0]
    v01 = global_map[env_idx, x0, y1]
    v11 = global_map[env_idx, x1, y1]

    out = (
        v00 * (1.0 - tx) * (1.0 - ty)
        + v10 * tx * (1.0 - ty)
        + v01 * (1.0 - tx) * ty
        + v11 * tx * ty
    )
    return (
        out.reshape(n_envs, rows, cols),
        valid_mask.reshape(n_envs, rows, cols),
    )


def sample_global_map_at_heightmap(
    *,
    global_map: torch.Tensor,
    robot_pos_w: torch.Tensor,
    env_origins: torch.Tensor,
    robot_yaw: torch.Tensor,
    geometry: LandscapingHeightmapGeometry,
    extent: float,
    resolution: int,
) -> torch.Tensor:
    """Sample a global map on the landscaping heightmap geometry."""
    return sample_global_map_at_patch_ranges(
        global_map=global_map,
        robot_pos_w=robot_pos_w,
        env_origins=env_origins,
        robot_yaw=robot_yaw,
        fwd_range=geometry.forward_range_m,
        lat_range=geometry.lateral_range_m,
        rows=geometry.ny,
        cols=geometry.nx,
        extent=extent,
        resolution=resolution,
    )


def sample_global_map_at_heightmap_with_validity(
    *,
    global_map: torch.Tensor,
    robot_pos_w: torch.Tensor,
    env_origins: torch.Tensor,
    robot_yaw: torch.Tensor,
    geometry: LandscapingHeightmapGeometry,
    extent: float,
    resolution: int,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Sample a patch and mark cell centres outside the global map.

    Samples remain edge-clamped; the mask alone determines validity. Do not
    fill invalid elevation cells with ``max_range_m``, which is a depth-sensor
    sentinel outside the elevation envelope.
    """
    return sample_global_map_at_patch_ranges_with_validity(
        global_map=global_map,
        robot_pos_w=robot_pos_w,
        env_origins=env_origins,
        robot_yaw=robot_yaw,
        fwd_range=geometry.forward_range_m,
        lat_range=geometry.lateral_range_m,
        rows=geometry.ny,
        cols=geometry.nx,
        extent=extent,
        resolution=resolution,
    )


def sample_single_global_map_at_heightmap_np(
    *,
    global_map: np.ndarray,
    robot_xy: tuple[float, float],
    robot_yaw: float,
    map_center_world_xy: tuple[float, float],
    geometry: LandscapingHeightmapGeometry,
    extent: float,
) -> np.ndarray:
    """NumPy adapter for one target patch on the shared geometry."""
    target_map = np.asarray(global_map, dtype=np.float32)
    resolution = int(target_map.shape[0])
    cells = geometry.body_xy_grid_np()

    cy = np.float32(np.cos(robot_yaw))
    sy = np.float32(np.sin(robot_yaw))
    gx = cells[..., 0]
    gy = cells[..., 1]
    wx = np.float32(robot_xy[0]) + gx * cy - gy * sy
    wy = np.float32(robot_xy[1]) + gx * sy + gy * cy

    half = np.float32(extent / 2.0)
    center_x = np.float32(map_center_world_xy[0])
    center_y = np.float32(map_center_world_xy[1])
    cell = np.float32(extent / float(resolution))

    fx = (wx - center_x + half) / cell - np.float32(0.5)
    fy = (wy - center_y + half) / cell - np.float32(0.5)

    x0 = np.clip(np.floor(fx).astype(np.int64), 0, resolution - 1)
    y0 = np.clip(np.floor(fy).astype(np.int64), 0, resolution - 1)
    x1 = np.clip(x0 + 1, 0, resolution - 1)
    y1 = np.clip(y0 + 1, 0, resolution - 1)
    tx = np.clip(fx - x0.astype(np.float32), 0.0, 1.0).astype(np.float32)
    ty = np.clip(fy - y0.astype(np.float32), 0.0, 1.0).astype(np.float32)

    v00 = target_map[x0, y0]
    v10 = target_map[x1, y0]
    v01 = target_map[x0, y1]
    v11 = target_map[x1, y1]
    out = (
        v00 * (1.0 - tx) * (1.0 - ty)
        + v10 * tx * (1.0 - ty)
        + v01 * (1.0 - tx) * ty
        + v11 * tx * ty
    )
    return out.astype(np.float32, copy=False)


def env_local_patch_heights_to_attach_z(
    *,
    patch_heights_env_local: torch.Tensor,
    robot_pos_w: torch.Tensor,
    env_origins: torch.Tensor,
    robot_quat_xyzw: torch.Tensor,
    geometry: LandscapingHeightmapGeometry,
    valid_mask: torch.Tensor | None = None,
) -> torch.Tensor:
    """Convert sampled env-local world elevations to attach-frame ``+z``.

    The global current and target maps store elevation relative to each
    environment origin. Depth heightmaps instead store the same surface in the
    rover attach frame. This conversion gives sampled maps and real depth
    producers one vertical datum while preserving the max-range sentinel when
    an optional validity mask marks cells invalid.

    ``robot_quat_xyzw`` follows current Isaac Lab's ``(x, y, z, w)``
    convention.
    """
    expected_shape = (robot_pos_w.shape[0], geometry.ny, geometry.nx)
    if tuple(patch_heights_env_local.shape) != expected_shape:
        raise ValueError(
            "patch height shape must match robot batch and geometry: "
            f"{tuple(patch_heights_env_local.shape)} != {expected_shape}"
        )
    if tuple(robot_pos_w.shape) != (expected_shape[0], 3):
        raise ValueError("robot_pos_w must have shape (num_envs, 3)")
    if tuple(env_origins.shape) != (expected_shape[0], 3):
        raise ValueError("env_origins must have shape (num_envs, 3)")
    if tuple(robot_quat_xyzw.shape) != (expected_shape[0], 4):
        raise ValueError("robot_quat_xyzw must have shape (num_envs, 4)")
    if valid_mask is not None and tuple(valid_mask.shape) != expected_shape:
        raise ValueError("valid_mask must have the same shape as patch heights")

    device = patch_heights_env_local.device
    dtype = patch_heights_env_local.dtype
    quat = robot_quat_xyzw.to(device=device, dtype=dtype)
    world_from_attach = quat_xyzw_to_rotmat_torch(quat)
    yaw = torch.atan2(world_from_attach[:, 1, 0], world_from_attach[:, 0, 0])
    cells = geometry.body_xy_grid_torch(device=device, dtype=dtype)
    body_x = cells[..., 0].unsqueeze(0)
    body_y = cells[..., 1].unsqueeze(0)
    cos_yaw = torch.cos(yaw).view(-1, 1, 1)
    sin_yaw = torch.sin(yaw).view(-1, 1, 1)
    rel_world_x = body_x * cos_yaw - body_y * sin_yaw
    rel_world_y = body_x * sin_yaw + body_y * cos_yaw

    base_z_env_local = (
        robot_pos_w.to(device=device, dtype=dtype)[:, 2]
        - env_origins.to(device=device, dtype=dtype)[:, 2]
    ).view(-1, 1, 1)
    rel_world_z = patch_heights_env_local - base_z_env_local

    relative_world = torch.stack((rel_world_x, rel_world_y, rel_world_z), dim=-1)
    relative_attach = torch.einsum(
        "n...i,nij->n...j",
        relative_world,
        world_from_attach,
    )
    attach_z = relative_attach[..., 2]

    if valid_mask is not None:
        attach_z = torch.where(
            valid_mask.to(device=device).bool(),
            attach_z,
            torch.full_like(attach_z, geometry.max_range_m),
        )
    return attach_z


def world_patch_heights_to_attach_z_np(
    *,
    patch_heights_world: np.ndarray,
    robot_position_world_xyz: tuple[float, float, float],
    robot_quat_xyzw: tuple[float, float, float, float],
    geometry: LandscapingHeightmapGeometry,
) -> np.ndarray:
    """Convert one sampled world-elevation patch to attach-frame ``+z``.

    This is the deploy-side NumPy equivalent of
    :func:`env_local_patch_heights_to_attach_z`. Target-map elevations and the
    TF translation must use the same world-frame vertical datum.
    """
    heights = np.asarray(patch_heights_world, dtype=np.float32)
    expected_shape = (geometry.ny, geometry.nx)
    if heights.shape != expected_shape:
        raise ValueError(
            f"patch height shape must match geometry: {heights.shape} != {expected_shape}"
        )

    qx, qy, qz, qw = (float(value) for value in robot_quat_xyzw)
    yaw = quat_xyzw_to_yaw(qx, qy, qz, qw)
    cells = geometry.body_xy_grid_np()
    cos_yaw = np.float32(np.cos(yaw))
    sin_yaw = np.float32(np.sin(yaw))
    rel_world_x = cells[..., 0] * cos_yaw - cells[..., 1] * sin_yaw
    rel_world_y = cells[..., 0] * sin_yaw + cells[..., 1] * cos_yaw
    rel_world_z = heights - np.float32(robot_position_world_xyz[2])

    world_from_attach = quat_xyzw_to_rotmat_np(qx, qy, qz, qw)
    attach_z = (
        rel_world_x * world_from_attach[0, 2]
        + rel_world_y * world_from_attach[1, 2]
        + rel_world_z * world_from_attach[2, 2]
    )
    return attach_z.astype(np.float32, copy=False)


__all__ = [
    "DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY",
    "LandscapingHeightmapGeometry",
    "bin_world_points_to_landscaping_heightmap",
    "env_local_patch_heights_to_attach_z",
    "sample_global_map_at_heightmap",
    "sample_global_map_at_heightmap_with_validity",
    "sample_global_map_at_patch_ranges",
    "sample_global_map_at_patch_ranges_with_validity",
    "sample_single_global_map_at_heightmap_np",
    "world_patch_heights_to_attach_z_np",
]
