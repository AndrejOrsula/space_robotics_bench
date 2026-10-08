"""Bake albedo, roughness, normal, and AO textures. Seeded inputs are byte-stable.

Texture caps downsample only; defaults preserve the full-resolution v1 path.
AO is computed at its cap before resizing to the final texture shape.

Normals encode DEM detail relative to the mesh, not absolute slope. A stride
mesh crops all textures to its vertex span; a decimated mesh uses its
rasterised top surface."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image

from srb.terrain.errors import EmptyHeightmapError
from srb.terrain.ingest.dem import Dem
from srb.terrain.log import get_logger
from srb.terrain.materials.blender import Layer, blend_layers, ramp_mask
from srb.terrain.materials.derivatives import (
    ambient_occlusion,
    curvature,
    slope_deg,
    smooth,
)
from srb.terrain.materials.presets import MaterialPreset

_log = get_logger("materials.baker")


@dataclass(frozen=True)
class BakedTextures:
    albedo: Path
    roughness: Path
    normal: Path
    ao: Path


@dataclass(frozen=True)
class MeshSurface:
    """Top surface of a terrain mesh on the DEM pixel grid.

    `vertices` is float64 (N, 3): fractional DEM-pixel (column, row) coordinates
    and heights in the DEM datum; `faces` indexes top triangles."""

    vertices: np.ndarray
    faces: np.ndarray


def bake_textures(
    dem: Dem,
    *,
    ortho: Optional[np.ndarray],
    preset: MaterialPreset,
    dest: Path,
    seed: int = 0,
    texture_cap_px: Optional[int] = None,
    ao_max_px: Optional[int] = None,
    mesh_stride_px: int = 1,
    mesh_surface: Optional[MeshSurface] = None,
) -> BakedTextures:
    """Bake albedo, roughness, normal, and AO textures for dem.

    mesh_stride_px gives regular-grid vertex spacing and crops textures to its
    span. mesh_surface supplies an irregular surface and requires stride 1.
    Normals encode only relief finer than the mesh."""
    if (
        isinstance(mesh_stride_px, bool)
        or not isinstance(mesh_stride_px, int)
        or mesh_stride_px < 1
    ):
        raise ValueError(
            f"mesh_stride_px must be a positive integer, got {mesh_stride_px!r}"
        )
    if mesh_surface is not None and mesh_stride_px != 1:
        raise ValueError(
            "mesh_surface describes an irregular mesh and requires "
            f"mesh_stride_px=1, got {mesh_stride_px!r}"
        )
    if mesh_stride_px != 1 and (texture_cap_px is not None or ao_max_px is not None):
        raise ValueError(
            "mesh_stride_px > 1 (v1 LOD stride) cannot be combined with "
            "texture_cap_px/ao_max_px (v2 budgets)"
        )
    if mesh_stride_px > 1:
        crop_rows, crop_cols = _stride_crop_shape(dem.heights.shape[:2], mesh_stride_px)
    else:
        crop_rows, crop_cols = dem.heights.shape[:2]
    dest.mkdir(parents=True, exist_ok=True)

    smoothed = smooth(dem.heights, sigma_px=preset.smooth_px)
    slope = slope_deg(smoothed, gsd_m=dem.gsd_m)
    curv = curvature(smoothed, gsd_m=dem.gsd_m)

    albedo = _bake_albedo(dem, ortho, preset, slope, curv, seed=seed)
    roughness = _bake_roughness(preset, slope)
    if mesh_surface is not None:
        detail = mesh_residual(dem.heights, mesh_surface)
    elif mesh_stride_px > 1:
        detail = _sub_mesh_detail(dem.heights[:crop_rows, :crop_cols], mesh_stride_px)
    else:
        detail = None
    if detail is None:
        normal = _neutral_normal((crop_rows, crop_cols))
    else:
        normal = _bake_normal(
            smooth(detail, sigma_px=preset.smooth_px), dem.gsd_m, preset
        )
        del detail

    h, w = dem.heights.shape[:2]

    target_h = h if texture_cap_px is None else min(h, texture_cap_px)
    target_w = w if texture_cap_px is None else min(w, texture_cap_px)
    ao_h = h if ao_max_px is None else min(h, ao_max_px)
    ao_w = w if ao_max_px is None else min(w, ao_max_px)

    if (target_h, target_w) != (h, w):
        _log.warning(
            "texture cap %d px < native %dx%d; box-downsampling "
            "albedo/roughness/normal to %dx%d",
            texture_cap_px,
            w,
            h,
            target_w,
            target_h,
        )
    if (ao_h, ao_w) != (h, w):
        _log.warning(
            "AO cap %d px < native %dx%d; computing ambient occlusion at "
            "%dx%d before upsampling to %dx%d",
            ao_max_px,
            w,
            h,
            ao_w,
            ao_h,
            target_w,
            target_h,
        )

    ao_smoothed = smoothed
    ao_gsd_m = dem.gsd_m
    if (ao_h, ao_w) != (h, w):
        ao_smoothed = _box_downsample(smoothed, ao_h, ao_w)

        # Scalar GSD correction is exact for square bakes; non-square fixtures tolerate
        # small anisotropic AO error.
        ao_gsd_m = dem.gsd_m * (h / ao_h)

    occlusion = ambient_occlusion(ao_smoothed, gsd_m=ao_gsd_m, num_dirs=16)
    # `ambient_occlusion` returns darkness (0 bright, 1 dark); PBR AO maps store
    # unoccluded as 1.
    ao = 1.0 - occlusion
    if (ao_h, ao_w) != (target_h, target_w):
        ao = _resize_channel(ao, target_h, target_w, Image.Resampling.BILINEAR)

    if (target_h, target_w) != (h, w):
        albedo = _box_downsample(albedo, target_h, target_w)
        roughness = _box_downsample(roughness, target_h, target_w)
        normal = _box_downsample(normal, target_h, target_w)

    if (crop_rows, crop_cols) != (h, w):
        albedo = albedo[:crop_rows, :crop_cols]
        roughness = roughness[:crop_rows, :crop_cols]
        ao = ao[:crop_rows, :crop_cols]

    a = dest / "albedo.png"
    r = dest / "roughness.png"
    n = dest / "normal.png"
    o = dest / "ao.png"

    _write_png(albedo, a)
    _write_png(roughness, r)
    _write_png(normal, n)
    _write_png(ao, o)

    return BakedTextures(albedo=a, roughness=r, normal=n, ao=o)


def _bake_albedo(
    dem: Dem,
    ortho: Optional[np.ndarray],
    preset: MaterialPreset,
    slope: np.ndarray,
    curv: np.ndarray,
    *,
    seed: int,
) -> np.ndarray:
    if ortho is not None:
        base = ortho.astype(np.float32) / 255.0
    else:
        base = _procedural_base(dem.heights.shape, preset, seed=seed)

    bedrock = np.broadcast_to(
        np.array(preset.bedrock_rgb, np.float32), (*base.shape[:2], 3)
    ).copy()
    weathered = np.broadcast_to(
        np.array(preset.weathered_rgb, np.float32), (*base.shape[:2], 3)
    ).copy()
    shadow = np.broadcast_to(
        np.array(preset.shadow_rgb, np.float32), (*base.shape[:2], 3)
    ).copy()

    bedrock_mask = (
        ramp_mask(
            slope, lo=preset.bedrock_slope_deg, hi=preset.bedrock_slope_deg + 10.0
        )
        * 0.7
    )
    weathered_mask = np.clip(curv, 0.0, None)
    if weathered_mask.max() > 0:
        weathered_mask = (weathered_mask / weathered_mask.max() * 0.5).astype(
            np.float32
        )
    shadow_mask = np.clip(-curv, 0.0, None)
    if shadow_mask.max() > 0:
        shadow_mask = (shadow_mask / shadow_mask.max() * 0.4).astype(np.float32)

    out = blend_layers(
        base,
        [
            Layer(color=bedrock, mask=bedrock_mask),
            Layer(color=weathered, mask=weathered_mask),
            Layer(color=shadow, mask=shadow_mask),
        ],
    )
    return np.clip(out, 0.0, 1.0)


def _procedural_base(
    shape: tuple[int, int],
    preset: MaterialPreset,
    *,
    seed: int,
) -> np.ndarray:
    h, w = shape
    rng = np.random.default_rng(seed)
    grain = rng.uniform(-0.05, 0.05, size=(h, w)).astype(np.float32)
    base = np.broadcast_to(np.array(preset.base_rgb, np.float32), (h, w, 3)).copy()
    base += grain[..., None]
    return np.clip(base, 0.0, 1.0)


def _bake_roughness(preset: MaterialPreset, slope: np.ndarray) -> np.ndarray:
    base = np.full(slope.shape, preset.roughness, dtype=np.float32)

    drop = ramp_mask(
        slope, lo=preset.bedrock_slope_deg, hi=preset.bedrock_slope_deg + 10.0
    ) * (preset.roughness - 0.4)
    return np.clip(base - drop, 0.0, 1.0)


def _neutral_normal(shape: tuple[int, ...]) -> np.ndarray:
    """Return the neutral tangent-space normal (0, 0, 1) encoded as RGB."""
    h, w = shape
    rgb = np.empty((h, w, 3), dtype=np.float32)
    rgb[..., 0:2] = 0.5
    rgb[..., 2] = 1.0
    return rgb


_BLOCK_PX = 1 << 18
_FACE_CHUNK = 1 << 19


def _stride_crop_shape(shape: tuple[int, ...], stride: int) -> tuple[int, int]:
    """Return the DEM span covered by a stride grid.

    Raise EmptyHeightmapError if fewer than 2x2 vertices remain."""
    rows, cols = int(shape[0]), int(shape[1])
    sub_rows = -(-rows // stride)
    sub_cols = -(-cols // stride)
    if sub_rows < 2 or sub_cols < 2:
        raise EmptyHeightmapError(
            f"mesh_stride_px={stride} leaves a {sub_rows}x{sub_cols} vertex grid "
            f"for a {rows}x{cols} DEM; need at least 2x2",
            remediation=(
                "Enlarge the patch (patch_size_m) or choose a finer lod so the "
                "strided mesh has at least 2x2 vertices."
            ),
        )
    return (sub_rows - 1) * stride + 1, (sub_cols - 1) * stride + 1


def _sub_mesh_detail(heights: np.ndarray, stride: int) -> np.ndarray:
    """Return DEM height minus the stride-grid mesh surface as float32.

    Input must be cropped to the vertex span. Each quad splits along a-d, matching
    `mesh.builder._build_grid`.
    Candidate blocks cap at `max_block_px` except when one DEM row is wider; faces
    process in `_FACE_CHUNK` chunks."""
    rows, cols = heights.shape
    crop_rows, crop_cols = _stride_crop_shape((rows, cols), stride)
    if (rows, cols) != (crop_rows, crop_cols):
        raise ValueError(
            f"heights {rows}x{cols} are not cropped to the stride-{stride} "
            f"vertex span {crop_rows}x{crop_cols}"
        )
    sub = heights[::stride, ::stride]
    sub_rows, sub_cols = sub.shape
    x = np.arange(cols, dtype=np.float64) / stride
    j0 = np.minimum(np.floor(x).astype(np.intp), sub_cols - 2)
    fx = x - j0
    out = np.empty((rows, cols), dtype=np.float32)
    block_rows = max(1, _BLOCK_PX // max(cols, 1))
    for r0 in range(0, rows, block_rows):
        r1 = min(rows, r0 + block_rows)
        y = np.arange(r0, r1, dtype=np.float64) / stride
        i0 = np.minimum(np.floor(y).astype(np.intp), sub_rows - 2)
        fy = (y - i0)[:, None]
        top = sub[i0].astype(np.float64)
        bottom = sub[i0 + 1].astype(np.float64)
        a = top[:, j0]
        b = top[:, j0 + 1]
        c = bottom[:, j0]
        d = bottom[:, j0 + 1]

        surface = a + fx * (b - a) + fy * (d - b)
        lower = np.broadcast_to(fy >= fx, surface.shape)
        fy_l = np.broadcast_to(fy, surface.shape)[lower]
        fx_l = np.broadcast_to(fx, surface.shape)[lower]
        a_l = a[lower]
        surface[lower] = a_l + fy_l * (c[lower] - a_l) + fx_l * (d[lower] - c[lower])
        out[r0:r1] = heights[r0:r1] - surface
    return out


def mesh_residual(
    heights: np.ndarray,
    surface: MeshSurface,
    *,
    max_block_px: int = _BLOCK_PX,
) -> np.ndarray:
    """Return DEM height minus the rasterised mesh surface as float32.

    Pixel centres inside top-surface triangles use barycentric interpolation;
    uncovered pixels get zero residual. Shared edges have the same height from
    either triangle. Rasterisation uses bounded float64 blocks and numpy."""
    if max_block_px < 1:
        raise ValueError(f"max_block_px must be positive, got {max_block_px!r}")
    rows, cols = heights.shape
    residual = np.zeros((rows, cols), dtype=np.float32)
    vertices = np.asarray(surface.vertices, dtype=np.float64)
    faces = np.asarray(surface.faces, dtype=np.intp)
    if faces.size == 0:
        return residual

    for f0 in range(0, len(faces), _FACE_CHUNK):
        _rasterise_residual(
            residual, heights, vertices, faces[f0 : f0 + _FACE_CHUNK], max_block_px
        )
    return residual


def _rasterise_residual(
    residual: np.ndarray,
    heights: np.ndarray,
    vertices: np.ndarray,
    faces: np.ndarray,
    max_block_px: int,
) -> None:
    """Write heights minus surface for faces in input order."""
    rows, cols = heights.shape
    tri = vertices[faces]
    p0 = tri[:, 0]
    e1 = tri[:, 1] - p0
    e2 = tri[:, 2] - p0
    det = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]

    # Coordinate roundoff can put exact edge samples just outside triangles.
    px_eps = 1e-3
    bary_eps = 1e-4
    c0 = np.clip(np.ceil(tri[:, :, 0].min(axis=1) - px_eps), 0, cols)
    c1 = np.clip(np.floor(tri[:, :, 0].max(axis=1) + px_eps), -1, cols - 1)
    r0 = np.clip(np.ceil(tri[:, :, 1].min(axis=1) - px_eps), 0, rows)
    r1 = np.clip(np.floor(tri[:, :, 1].max(axis=1) + px_eps), -1, rows - 1)
    c0 = c0.astype(np.int64)
    r0 = r0.astype(np.int64)
    n_c = np.maximum(c1.astype(np.int64) - c0 + 1, 0)
    n_r = np.maximum(r1.astype(np.int64) - r0 + 1, 0)
    keep = (np.abs(det) > 1e-12) & (n_c > 0) & (n_r > 0)
    if not keep.any():
        return
    idx = np.flatnonzero(keep)

    p0x, p0y, z0 = (np.ascontiguousarray(p0[:, k]) for k in range(3))
    e1x, e1y, e1z = (np.ascontiguousarray(e1[:, k]) for k in range(3))
    e2x, e2y, e2z = (np.ascontiguousarray(e2[:, k]) for k in range(3))
    inv_det = np.zeros_like(det)
    inv_det[idx] = 1.0 / det[idx]

    strip_rows = np.maximum(1, max_block_px // n_c[idx])
    n_strips = -(-n_r[idx] // strip_rows)
    s_tri = np.repeat(idx, n_strips)
    s_first = np.repeat(np.cumsum(n_strips) - n_strips, n_strips)
    s_k = np.arange(s_tri.size, dtype=np.int64) - s_first
    s_rows_per = np.repeat(strip_rows, n_strips)
    s_r0 = r0[s_tri] + s_k * s_rows_per
    s_nr = np.minimum(s_rows_per, r0[s_tri] + n_r[s_tri] - s_r0)
    s_nc = n_c[s_tri]
    s_count = s_nr * s_nc

    ends = np.cumsum(s_count)
    start = 0
    n_strip = s_tri.size
    while start < n_strip:
        base = ends[start - 1] if start else 0
        stop = int(np.searchsorted(ends, base + max_block_px, side="right"))
        stop = max(stop, start + 1)
        sl = slice(start, stop)
        counts = s_count[sl]
        t = np.repeat(s_tri[sl], counts)
        local = np.arange(int(counts.sum()), dtype=np.int64) - np.repeat(
            np.cumsum(counts) - counts, counts
        )
        nc = np.repeat(s_nc[sl], counts)
        rr = np.repeat(s_r0[sl], counts) + local // nc
        cc = c0[t] + local % nc
        del local, nc
        dx = cc - p0x[t]
        dy = rr - p0y[t]
        inv = inv_det[t]
        l1 = (dx * e2y[t] - dy * e2x[t]) * inv
        l2 = (e1x[t] * dy - e1y[t] * dx) * inv
        del dx, dy, inv
        inside = (l1 >= -bary_eps) & (l2 >= -bary_eps) & (l1 + l2 <= 1.0 + bary_eps)
        t_in = t[inside]
        z = z0[t_in] + l1[inside] * e1z[t_in] + l2[inside] * e2z[t_in]
        r_in = rr[inside]
        c_in = cc[inside]
        residual[r_in, c_in] = heights[r_in, c_in] - z
        start = stop


def _bake_normal(
    heights: np.ndarray,
    gsd_m: float,
    preset: MaterialPreset,
) -> np.ndarray:
    """Encode heights as a normal map in the mesh tangent frame.

    UV tangent +u follows columns; bitangent +v follows north (decreasing rows).
    Input is mesh-relative detail, not the absolute DEM."""
    h = heights.astype(np.float64)

    gy, gx = np.gradient(h, gsd_m)

    # World y points north (decreasing rows), so the bitangent component is +gy.

    nx = -gx * preset.normal_strength
    ny = gy * preset.normal_strength
    nz = np.ones_like(nx)
    norm = np.sqrt(nx**2 + ny**2 + nz**2)
    nx /= norm
    ny /= norm
    nz /= norm

    rgb = np.stack(
        [
            (nx * 0.5 + 0.5),
            (ny * 0.5 + 0.5),
            (nz * 0.5 + 0.5),
        ],
        axis=-1,
    )
    return np.clip(rgb, 0.0, 1.0).astype(np.float32)


def _resize_channel(
    chan: np.ndarray, target_h: int, target_w: int, resample
) -> np.ndarray:
    """Resize one 2D float32 channel with PIL, without 8-bit quantisation."""
    img = Image.fromarray(chan.astype(np.float32))
    resized = img.resize((target_w, target_h), resample=resample)
    return np.asarray(resized, dtype=np.float32)


def _box_downsample(arr: np.ndarray, target_h: int, target_w: int) -> np.ndarray:
    """Area-average a 2D or channel-last array to the requested shape.

    Targets must not exceed the input shape. Subsampling aliases terrain detail;
    PIL's BOX filter silently becomes nearest-neighbour when enlarged."""
    h, w = arr.shape[:2]
    if target_h > h or target_w > w:
        raise ValueError(
            f"_box_downsample must not upscale: target ({target_h}, "
            f"{target_w}) exceeds native ({h}, {w})"
        )
    if (target_h, target_w) == (h, w):
        return arr
    if arr.ndim == 2:
        return _resize_channel(arr, target_h, target_w, Image.Resampling.BOX)
    channels = [
        _resize_channel(arr[..., c], target_h, target_w, Image.Resampling.BOX)
        for c in range(arr.shape[-1])
    ]
    return np.stack(channels, axis=-1)


def _write_png(arr01: np.ndarray, path: Path) -> None:
    img = (arr01 * 255.0 + 0.5).clip(0, 255).astype(np.uint8)
    Image.fromarray(img).save(path, format="PNG", optimize=False)
