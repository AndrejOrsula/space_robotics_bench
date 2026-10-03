"""Grid-pattern heightmap sensor with sensor-realistic noise.

The camera (+z forward, +x right, +y down) maps to the attach frame
(+x forward, +y left, +z up) via ``tf_attach_from_camera``:
``p_attach = tf_attach_from_camera @ p_camera_h``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Optional, Sequence

import torch
from pydantic import BaseModel

__all__ = [
    "HeightmapSpec",
    "HeightmapNoiseCfg",
    "HeightmapSensorData",
    "apply_heightmap_noise",
    "bin_points_to_heightmap_grid",
    "fuse_heightmap_views",
    "postprocess_heightmap_observation",
    "project_depth_to_heightmap",
]


class HeightmapSpec(BaseModel):
    """Grid layout and attach-frame parameters; lengths are metres."""

    nx: int = 16
    """Number of grid columns (lateral / y direction)."""
    ny: int = 16
    """Number of grid rows (forward / x direction)."""
    cell_size_m: float = 0.05
    """Physical size of each grid cell in metres."""
    forward_offset_m: float = 0.3
    """Distance from the attach-frame origin to the nearest row of grid cells."""
    lateral_offset_m: float = 0.0
    """Lateral shift of the grid centre from the attach-frame y=0 plane."""
    dead_zone_m: float = 0.3
    """Points closer than this in the attach-frame +x direction are discarded."""
    frame: str = "base_link"
    """Name of the frame the heightmap is expressed in (used for TF lookup)."""
    max_range_m: float = 2.0
    """Sentinel fill value for grid cells with no valid depth reading."""

    model_config = {"frozen": False}


class HeightmapNoiseCfg(BaseModel):
    """Depth noise models: ideal, D435 (σ ∝ d²), and L515 (σ = σ_fixed + βd)."""

    model: Literal["d435", "l515", "ideal"] = "d435"
    stereo_alpha: float = 0.003
    """Coefficient for σ = stereo_alpha · d²."""
    tof_sigma_fixed_m: float = 0.005
    """Fixed noise floor for the ToF model (metres)."""
    tof_beta: float = 0.002
    """Distance-dependent coefficient for the ToF model."""
    depth_quant_m: float = 0.001
    """Depth quantisation step (0 = disabled)."""
    incidence_dropout_deg: float = 65.0
    """Drop cells whose terrain slope exceeds this angle (cells with four valid neighbours)."""
    random_dropout_rate: float = 0.03
    """Fraction of valid cells independently zeroed at each step (0 = disabled)."""
    domain_rand: bool = True
    """Enable the random perturbations (depth noise and random dropout).

    Quantisation and incidence dropout model deterministic sensor behaviour and
    apply regardless."""

    model_config = {"frozen": False}


@dataclass
class HeightmapSensorData:
    """Heightmap output buffers."""

    heights: torch.Tensor = field(default_factory=lambda: torch.empty(0))
    """Heightmap tensor, shape (N_envs, ny, nx) or (ny, nx) for single-env."""
    valid_mask: torch.Tensor = field(default_factory=lambda: torch.empty(0))
    """Boolean-valued float mask (0/1), same shape as ``heights``."""
    backend: Literal["raycaster", "particle_bin"] = "raycaster"
    """Which sim backend produced this data (informational)."""


def apply_heightmap_noise(
    heights: torch.Tensor,
    valid_mask: torch.Tensor,
    cfg: HeightmapNoiseCfg,
    generator: Optional[torch.Generator] = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Apply quantisation, then sensor noise and dropout when ``cfg.domain_rand``.
    Clones inputs; returns tensors with matching shapes and dtypes.
    """
    out_h = heights.clone()
    out_v = valid_mask.clone()

    if cfg.depth_quant_m > 0:
        out_h = (out_h / cfg.depth_quant_m).round() * cfg.depth_quant_m
    if not cfg.domain_rand:
        return out_h, out_v

    if cfg.model == "d435":
        sigma = cfg.stereo_alpha * out_h.abs() ** 2
    elif cfg.model == "l515":
        sigma = cfg.tof_sigma_fixed_m + cfg.tof_beta * out_h.abs()
    else:  # "ideal"
        sigma = torch.zeros_like(out_h)

    if (sigma > 0).any():
        noise = torch.empty_like(out_h).normal_(mean=0.0, std=1.0, generator=generator)
        out_h = out_h + sigma * noise

    if cfg.random_dropout_rate > 0:
        drop = (
            torch.empty_like(out_v).uniform_(0.0, 1.0, generator=generator)
            < cfg.random_dropout_rate
        )
        out_v = torch.where(drop, torch.zeros_like(out_v), out_v)

    return out_h, out_v


def postprocess_heightmap_observation(
    heights: torch.Tensor,
    valid_mask: torch.Tensor,
    *,
    spec: HeightmapSpec,
    noise: HeightmapNoiseCfg,
    generator: Optional[torch.Generator] = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Apply shared simulation noise and validity masks. Inputs end in `(spec.ny,
    spec.nx)`.
    """
    if heights.shape != valid_mask.shape:
        raise ValueError(
            "heightmap heights and validity must have the same shape: "
            f"{tuple(heights.shape)} != {tuple(valid_mask.shape)}"
        )
    if heights.ndim < 2 or tuple(heights.shape[-2:]) != (spec.ny, spec.nx):
        raise ValueError(
            "heightmap inputs must end in configured grid shape "
            f"{(spec.ny, spec.nx)}, got {tuple(heights.shape)}"
        )
    if not heights.is_floating_point():
        raise TypeError("heightmap heights must use a floating-point dtype")

    noisy_z, noisy_valid = apply_heightmap_noise(
        heights,
        valid_mask.to(device=heights.device, dtype=heights.dtype),
        noise,
        generator,
    )
    original_shape = noisy_z.shape
    batch_z = noisy_z.reshape(-1, spec.ny, spec.nx)
    batch_valid = noisy_valid.reshape(-1, spec.ny, spec.nx)

    cell_x = (
        spec.forward_offset_m
        + (torch.arange(spec.ny, device=heights.device, dtype=heights.dtype) + 0.5)
        * spec.cell_size_m
    )
    batch_valid = batch_valid * (cell_x.view(1, spec.ny, 1) >= spec.dead_zone_m)

    if noise.incidence_dropout_deg < 90.0:
        tan_thresh = torch.tan(
            torch.tensor(
                noise.incidence_dropout_deg * torch.pi / 180.0,
                device=heights.device,
                dtype=heights.dtype,
            )
        )
        pad = torch.nn.functional.pad(batch_z, (1, 1, 1, 1), mode="replicate")
        dz_dx = (pad[:, 2:, 1:-1] - pad[:, :-2, 1:-1]) / (2.0 * spec.cell_size_m)
        dz_dy = (pad[:, 1:-1, 2:] - pad[:, 1:-1, :-2]) / (2.0 * spec.cell_size_m)
        grad_mag = torch.sqrt(dz_dx**2 + dz_dy**2)

        vpad = torch.nn.functional.pad(
            batch_valid,
            (1, 1, 1, 1),
            mode="constant",
            value=0.0,
        )
        neighbors_valid = (
            vpad[:, 2:, 1:-1]
            * vpad[:, :-2, 1:-1]
            * vpad[:, 1:-1, 2:]
            * vpad[:, 1:-1, :-2]
        )
        keep = (grad_mag <= tan_thresh) | (neighbors_valid <= 0.0)
        batch_valid = batch_valid * keep

    batch_z = torch.where(
        batch_valid.bool(),
        batch_z,
        torch.full_like(batch_z, spec.max_range_m),
    )
    return batch_z.reshape(original_shape), batch_valid.reshape(original_shape)


def fuse_heightmap_views(
    views: Sequence[tuple[torch.Tensor, torch.Tensor]],
    *,
    fill_value: float,
    shape: tuple[int, ...] | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Fuse views in order; later views fill invalid cells without replacing earlier
    valid cells.
    """
    if not views:
        if shape is None:
            raise ValueError("shape is required when fusing an empty view sequence")
        return torch.full(shape, fill_value, dtype=torch.float32), torch.zeros(
            shape, dtype=torch.float32
        )

    first_heights, first_valid = views[0]
    if first_heights.shape != first_valid.shape:
        raise ValueError(
            "heightmap view 0 has mismatched height and validity shapes: "
            f"{tuple(first_heights.shape)} != {tuple(first_valid.shape)}"
        )

    expected_shape = tuple(first_heights.shape)
    out_h = torch.full_like(first_heights, fill_value)
    out_v = torch.zeros_like(first_valid, dtype=torch.float32)

    for index, (heights, valid_mask) in enumerate(views):
        if (
            tuple(heights.shape) != expected_shape
            or tuple(valid_mask.shape) != expected_shape
        ):
            raise ValueError(
                "all heightmap views must have the same shape; "
                f"view 0 has {expected_shape}, view {index} has "
                f"{tuple(heights.shape)} and {tuple(valid_mask.shape)}"
            )

        heights = heights.to(device=out_h.device, dtype=out_h.dtype)
        valid = valid_mask.to(device=out_h.device).bool()
        fill_cells = (~out_v.bool()) & valid
        out_h = torch.where(fill_cells, heights, out_h)
        out_v = torch.where(fill_cells, torch.ones_like(out_v), out_v)

    out_h = torch.where(out_v.bool(), out_h, torch.full_like(out_h, fill_value))
    return out_h, out_v


def project_depth_to_heightmap(
    depth_image: torch.Tensor,
    camera_intrinsics: torch.Tensor,
    tf_attach_from_camera: torch.Tensor,
    spec: HeightmapSpec,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Project depth into an attach-frame grid. Attach axes are +x forward, +y left, +z
    up; cells use median z and empty cells use `max_range_m`.
    """
    device = depth_image.device
    H, W = depth_image.shape

    heights = torch.full((spec.ny, spec.nx), float(spec.max_range_m), device=device)
    valid = torch.zeros((spec.ny, spec.nx), device=device)

    if torch.isinf(depth_image).all():
        return heights, valid

    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]

    u = torch.arange(W, device=device).view(1, -1).expand(H, W).float()
    v = torch.arange(H, device=device).view(-1, 1).expand(H, W).float()
    z_cam = depth_image
    x_cam = (u - cx) * z_cam / fx
    y_cam = (v - cy) * z_cam / fy

    valid_px = torch.isfinite(z_cam) & (z_cam > 0)

    pts_cam = torch.stack([x_cam, y_cam, z_cam, torch.ones_like(z_cam)], dim=-1)
    tf = tf_attach_from_camera.to(device=device, dtype=pts_cam.dtype)
    pts_attach = (tf @ pts_cam.view(-1, 4).T).T.view(H, W, 4)

    ax = pts_attach[..., 0]  # forward (+x)
    ay = pts_attach[..., 1]  # lateral (+y left)
    az = pts_attach[..., 2]  # up (+z)

    lateral_lo = -spec.nx * spec.cell_size_m / 2.0 + spec.lateral_offset_m
    lateral_hi = spec.nx * spec.cell_size_m / 2.0 + spec.lateral_offset_m
    forward_hi = spec.forward_offset_m + spec.ny * spec.cell_size_m

    inside = (
        valid_px
        & (ax >= spec.dead_zone_m)
        & (ax >= spec.forward_offset_m)
        & (ax < forward_hi)
        & (ay >= lateral_lo)
        & (ay < lateral_hi)
    )

    if inside.sum() == 0:
        return heights, valid

    col = ((ay - lateral_lo) / spec.cell_size_m).long().clamp(0, spec.nx - 1)
    row = ((ax - spec.forward_offset_m) / spec.cell_size_m).long().clamp(0, spec.ny - 1)

    flat_idx = (row * spec.nx + col).view(-1)[inside.view(-1)]
    z_in = az.view(-1)[inside.view(-1)]

    for idx in flat_idx.unique().tolist():
        mask = flat_idx == idx
        z_cell = z_in[mask]
        r = int(idx) // spec.nx
        c = int(idx) % spec.nx
        heights[r, c] = z_cell.median()
        valid[r, c] = 1.0

    return heights, valid


def build_heightmap_pattern_from_spec(spec: HeightmapSpec, ray_height_m: float = 0.5):
    """Build a row-major downward-ray pattern (+x outer, +y inner), or return `None`
    without Isaac Lab.
    """
    try:
        from srb.core.sensor.patterns import ExplicitPatternBaseCfg
    except ImportError:
        return None
    xs = [spec.forward_offset_m + (i + 0.5) * spec.cell_size_m for i in range(spec.ny)]
    ys = [
        spec.lateral_offset_m + (-spec.nx / 2.0 + j + 0.5) * spec.cell_size_m
        for j in range(spec.nx)
    ]
    ray_starts = [(x, y, ray_height_m) for x in xs for y in ys]
    return ExplicitPatternBaseCfg(ray_starts=ray_starts)


def bin_points_to_heightmap_grid(
    ax: torch.Tensor,
    ay: torch.Tensor,
    az: torch.Tensor,
    valid: torch.Tensor,
    *,
    ny: int,
    nx: int,
    cell_size_m: float,
    forward_offset_m: float,
    lateral_offset_m: float,
    max_range_m: float,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Bin attach-frame points by maximum +z. Inputs are `(N, P)`; outputs are `(N, ny,
    nx)` float32.
    """
    n_envs = ax.shape[0]
    device = ax.device
    lateral_lo = -nx * cell_size_m / 2.0 + lateral_offset_m
    lateral_hi = nx * cell_size_m / 2.0 + lateral_offset_m
    forward_hi = forward_offset_m + ny * cell_size_m

    inside = (
        valid
        & (ax >= forward_offset_m)
        & (ax < forward_hi)
        & (ay >= lateral_lo)
        & (ay < lateral_hi)
    )  # (N, P)

    col = ((ay - lateral_lo) / cell_size_m).long().clamp(0, nx - 1)
    row = ((ax - forward_offset_m) / cell_size_m).long().clamp(0, ny - 1)
    flat = row * nx + col  # (N, P) cell index per point

    neg_inf = torch.tensor(float("-inf"), device=device, dtype=az.dtype)
    az_in = torch.where(inside, az, neg_inf)  # excluded points cannot win the max
    heights = torch.full(
        (n_envs, ny * nx), float("-inf"), device=device, dtype=az.dtype
    )
    heights.scatter_reduce_(1, flat, az_in, reduce="amax", include_self=True)

    valid_mask = torch.isfinite(heights).to(az.dtype)
    heights = torch.where(
        torch.isfinite(heights),
        heights,
        torch.tensor(float(max_range_m), device=device, dtype=az.dtype),
    )
    return heights.view(n_envs, ny, nx), valid_mask.view(n_envs, ny, nx)


try:
    from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg
    from isaaclab.utils.math import quat_apply_inverse

    # Isaac Lab 3.0's dispatcher rejects external subclasses. Use the concrete PhysX
    # raycaster when available with the backend-neutral Isaac Lab config.
    try:
        from isaaclab_physx.sensors.ray_caster import MultiMeshRayCaster
    except ImportError:
        from isaaclab.sensors.ray_caster import MultiMeshRayCaster

    # Probe subclassability here; ImportError rejection reaches the outer handler
    # and disables Isaac support.
    class _SubclassProbe(MultiMeshRayCaster):
        pass

    del _SubclassProbe

    from srb.core.spec import TermSpec, spec
    from srb.utils.cfg import configclass

    _HAS_ISAAC = True
except ImportError:
    _HAS_ISAAC = False

if _HAS_ISAAC:
    # Define sensor before config so instances bind its class_type field. Late binding
    # leaves instances on the parent factory. Static metadata reports default 16×16.

    @spec(
        kind="sensor",
        fields=[
            TermSpec(
                name="heights",
                shape=(16, 16),
                dtype="float32",
                units="m",
                frame="base_link",
                semantic_label="terrain.height_field",
            ),
            TermSpec(
                name="valid_mask",
                shape=(16, 16),
                dtype="float32",
                units="",
                frame="base_link",
                semantic_label="terrain.height_field.validity",
            ),
        ],
    )
    class HeightmapSensor(MultiMeshRayCaster):  # type: ignore[misc]
        """Isaac Lab raycaster that exposes processed heightmaps and ground-truth
        heights.
        """

        cfg: HeightmapSensorCfg  # type: ignore[assignment]

        def __init__(self, cfg: HeightmapSensorCfg) -> None:
            super().__init__(cfg)
            self._hm_data = HeightmapSensorData(
                heights=torch.empty(0),
                valid_mask=torch.empty(0),
                backend="raycaster",
            )
            self._heights_gt: torch.Tensor = torch.empty(0)
            self._noise_gen: Optional[torch.Generator] = None

        def _init_heightmap_buffers(
            self, num_envs: int, ny: int, nx: int, device: str
        ) -> None:
            """Allocate heightmap buffers for the configured grid."""
            spec = self.cfg.spec
            self._hm_data.heights = torch.full(
                (num_envs, ny, nx), spec.max_range_m, dtype=torch.float32, device=device
            )
            self._hm_data.valid_mask = torch.zeros(
                (num_envs, ny, nx), dtype=torch.float32, device=device
            )
            self._heights_gt = torch.full(
                (num_envs, ny, nx), spec.max_range_m, dtype=torch.float32, device=device
            )

        def _build_cell_x_coords(
            self, spec: "HeightmapSpec", device: str
        ) -> torch.Tensor:
            """Return cell-center +x coordinates with shape `(ny, nx)`."""
            x = torch.linspace(
                spec.forward_offset_m + spec.cell_size_m * 0.5,
                spec.forward_offset_m + spec.cell_size_m * (spec.ny - 0.5),
                spec.ny,
                device=device,
            )
            return x.unsqueeze(-1).expand(spec.ny, spec.nx)  # (ny, nx)

        def _update_buffers_impl(self, env_mask) -> None:
            super()._update_buffers_impl(env_mask)

            spec = self.cfg.spec
            ny, nx = spec.ny, spec.nx
            num_rays = ny * nx
            device = str(self._device)
            num_envs = self._num_envs

            if self._hm_data.heights.numel() == 0:
                self._init_heightmap_buffers(num_envs, ny, nx, device)
                self._cell_x_coords = self._build_cell_x_coords(spec, device)

            # (MultiMeshRayCaster stores its RayCasterData in self._data)
            ray_caster_data = self._data  # parent's RayCasterData
            ray_hits = ray_caster_data.ray_hits_w  # (N, ny*nx, 3)
            if ray_hits is None or ray_hits.shape[1] != num_rays:
                import warnings

                n_got = ray_hits.shape[1] if ray_hits is not None else 0
                warnings.warn(
                    f"HeightmapSensor: expected {num_rays} rays (ny={ny}×nx={nx}), "
                    f"got {n_got}. Heightmap will be empty.",
                    RuntimeWarning,
                    stacklevel=2,
                )
                return
            hits_grid = ray_hits.view(num_envs, ny, nx, 3)  # (N, ny, nx, 3)

            origin_w = ray_caster_data.pos_w.view(num_envs, 1, 1, 3)  # (N, 1, 1, 3)
            hit_delta_attach = quat_apply_inverse(
                ray_caster_data.quat_w.view(num_envs, 1, 1, 4), hits_grid - origin_w
            )
            z_attach = hit_delta_attach[..., 2]  # height along attach-frame +z

            miss_dist = spec.max_range_m * 0.95
            hit_dist_from_origin = torch.norm(
                hits_grid - origin_w, dim=-1
            )  # (N, ny, nx)
            valid = (hit_dist_from_origin < miss_dist).float()

            valid = valid * torch.isfinite(z_attach).float()

            z_attach = torch.where(
                valid.bool(), z_attach, torch.full_like(z_attach, spec.max_range_m)
            )

            ground_truth_valid = valid * (
                self._cell_x_coords.unsqueeze(0) >= spec.dead_zone_m
            )
            # Only rows in the outdated-env mask were re-cast; keep the rest.
            import warp as wp

            env_mask = (
                torch.ones(num_envs, 1, 1, dtype=torch.bool, device=device)
                if env_mask is None
                else wp.to_torch(env_mask).to(torch.bool).view(num_envs, 1, 1)
            )
            self._heights_gt[:] = torch.where(
                env_mask,
                torch.where(
                    ground_truth_valid.bool(),
                    z_attach,
                    torch.full_like(z_attach, spec.max_range_m),
                ),
                self._heights_gt,
            )

            if self._noise_gen is None:
                import zlib

                # Seed from the experiment and prim path for reproducibility and per-sensor decorrelation.
                self._noise_gen = torch.Generator(device=device)
                self._noise_gen.manual_seed(
                    torch.initial_seed() ^ zlib.crc32(str(self.cfg.prim_path).encode())
                )

            noisy_z, noisy_valid = postprocess_heightmap_observation(
                z_attach,
                valid,
                spec=spec,
                noise=self.cfg.noise,
                generator=self._noise_gen,
            )
            self._hm_data.heights[:] = torch.where(
                env_mask, noisy_z, self._hm_data.heights
            )
            self._hm_data.valid_mask[:] = torch.where(
                env_mask, noisy_valid, self._hm_data.valid_mask
            )

        @property
        def data(self) -> HeightmapSensorData:  # type: ignore[override]
            """Return processed heightmaps; access triggers lazy sensor updates."""
            self._update_outdated_buffers()
            return self._hm_data

        @property
        def ray_data(self):
            """Return raw raycaster data; access triggers lazy sensor updates."""
            self._update_outdated_buffers()
            return self._data

        def data_ground_truth(self) -> torch.Tensor:
            """Return noise-free heights with geometric dead-zone masking; shape
            `(num_envs, ny, nx)`, float32.
            """
            self._update_outdated_buffers()
            if self._heights_gt.numel() == 0:
                spec = self.cfg.spec
                return torch.full(
                    (self._num_envs, spec.ny, spec.nx),
                    spec.max_range_m,
                    dtype=torch.float32,
                    device=self._device,
                )
            return self._heights_gt

    @configclass
    class HeightmapSensorCfg(MultiMeshRayCasterCfg):  # type: ignore[misc]
        """Raycaster configuration with heightmap grid and noise settings."""

        class_type: type = HeightmapSensor

        spec: HeightmapSpec = HeightmapSpec()
        noise: HeightmapNoiseCfg = HeightmapNoiseCfg()
        include_particles: bool = True

    __all__ += ["HeightmapSensorCfg", "HeightmapSensor"]
