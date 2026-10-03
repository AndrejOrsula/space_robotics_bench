from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import trimesh

if TYPE_CHECKING:
    from . import mesh_terrains_cfg


def wave_terrain(
    difficulty: float, cfg: mesh_terrains_cfg.MeshWaveTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate a terrain with procedurally generated waves at a random orientation."""
    num_x_points = int(round(cfg.size[0] / cfg.resolution)) + 1
    num_y_points = int(round(cfg.size[1] / cfg.resolution)) + 1
    x = np.linspace(0, cfg.size[0], num_x_points)
    y = np.linspace(0, cfg.size[1], num_y_points)
    x_grid, y_grid = np.meshgrid(x, y)
    z_grid = np.zeros_like(x_grid)

    def lerp(val_range, d):
        return val_range[0] + d * (val_range[1] - val_range[0])

    num_ripples = np.random.randint(
        cfg.num_ripples_range[0], int(lerp(cfg.num_ripples_range, difficulty)) + 1
    )
    max_amplitude = lerp(cfg.amplitude_range, difficulty)
    max_wavelength = lerp(cfg.wavelength_range, difficulty)
    max_smoothing = lerp(cfg.smoothing_range, difficulty)
    noise_scale = lerp(cfg.noise_scale_range, difficulty)
    noise_period = lerp(cfg.noise_period_range, difficulty)
    noise_octaves = round(lerp(cfg.noise_octaves_range, difficulty))
    noise_lacunarity = lerp(cfg.noise_lacunarity_range, difficulty)
    start_offset = np.random.uniform(*cfg.start_offset_range)
    orientation_rad = np.random.uniform(*cfg.orientation_range)
    c, s = np.cos(orientation_rad), np.sin(orientation_rad)
    # Start the projection at zero so the positive start offset lands inside
    # the terrain for every orientation (it is all-negative in the third quadrant).
    x_rot = x_grid * c + y_grid * s
    x_rot -= x_rot.min()

    x_current = start_offset
    for _ in range(num_ripples):
        amp = np.random.uniform(cfg.amplitude_range[0], max_amplitude)
        wav = np.random.uniform(cfg.wavelength_range[0], max_wavelength)
        smooth = np.random.uniform(cfg.smoothing_range[0], max_smoothing)
        gap = np.random.uniform(*cfg.gap_range)
        x_next = x_current + wav

        mask = (x_rot > x_current) & (x_rot <= x_next)
        if np.any(mask):
            x_local = x_rot[mask] - x_current
            if smooth >= 1.0:
                z_grid[mask] += (
                    amp
                    / 2
                    * (np.sin(2 * np.pi * (x_local - cfg.phase) / wav - np.pi / 2) + 1)
                    + cfg.z_offset
                )
            else:
                a_max = 1 - 2 / np.pi * np.arccos(1 - smooth)
                inner = (1 - smooth) * np.sin(
                    2 * np.pi * (x_local - cfg.phase) / wav - np.pi / 2
                )
                # The '+ amp / 2' term shifts the wave up so it starts at z=0.
                shaping_function = (
                    1 - 2 / np.pi * np.arccos(np.clip(inner, -1.0, 1.0))
                ) / a_max
                z_grid[mask] += amp / 2 * shaping_function + amp / 2 + cfg.z_offset
        x_current = x_next + gap

    noise_res = (
        max(1, int(cfg.size[1] / noise_period)),
        max(1, int(cfg.size[0] / noise_period)),
    )
    noise = (
        _generate_fractal_noise_2d(
            z_grid.shape, noise_res, noise_octaves, 0.5, noise_lacunarity, (True, True)
        )
        * noise_scale
    )
    z_grid += noise
    border_px = int(cfg.border_width / cfg.resolution)
    z_grid = _apply_seamless_border(z_grid, border_px)

    mesh = _create_solid_mesh_from_heightmap(x_grid, y_grid, z_grid, cfg.thickness)
    origin = _terrain_origin(cfg, x_grid, y_grid, z_grid)
    return [mesh], origin


def hill_terrain(
    difficulty: float, cfg: mesh_terrains_cfg.MeshHillTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    num_x_points = int(round(cfg.size[0] / cfg.resolution)) + 1
    num_y_points = int(round(cfg.size[1] / cfg.resolution)) + 1
    x = np.linspace(0, cfg.size[0], num_x_points)
    y = np.linspace(0, cfg.size[1], num_y_points)
    x_grid, y_grid = np.meshgrid(x, y)
    z_grid = np.zeros_like(x_grid)

    def lerp(val_range, d):
        return val_range[0] + d * (val_range[1] - val_range[0])

    num_hills = np.random.randint(
        cfg.num_hills_range[0], int(lerp(cfg.num_hills_range, difficulty)) + 1
    )
    max_slope = lerp(cfg.slope_range, difficulty)
    max_beta = lerp(cfg.beta_range, difficulty)
    max_radius = lerp(cfg.hill_radius_range, difficulty)
    noise_scale = lerp(cfg.noise_scale_range, difficulty)
    noise_period = lerp(cfg.noise_period_range, difficulty)
    noise_octaves = round(lerp(cfg.noise_octaves_range, difficulty))
    noise_lacunarity = lerp(cfg.noise_lacunarity_range, difficulty)

    for _ in range(num_hills):
        slope = np.random.uniform(cfg.slope_range[0], max_slope)
        beta = np.random.uniform(cfg.beta_range[0], max_beta)
        max_fitting_radius = min(cfg.size[0], cfg.size[1]) / 2
        radius_hi = min(max_radius, max_fitting_radius)
        radius_lo = min(cfg.hill_radius_range[0], radius_hi)
        radius = np.random.uniform(radius_lo, radius_hi)
        center_x = np.random.uniform(radius, cfg.size[0] - radius)
        center_y = np.random.uniform(radius, cfg.size[1] - radius)

        dist_from_center = np.sqrt((x_grid - center_x) ** 2 + (y_grid - center_y) ** 2)
        dist_local = radius - dist_from_center
        m = np.tan(slope)
        c = m / (2 * beta)
        b = beta * c**2

        hill_mask = dist_from_center < radius
        z_grid[hill_mask] += np.where(
            dist_local[hill_mask] < c,
            beta * dist_local[hill_mask] ** 2,
            m * (dist_local[hill_mask] - c) + b,
        )

    noise_res = (
        max(1, int(cfg.size[1] / noise_period)),
        max(1, int(cfg.size[0] / noise_period)),
    )
    noise = (
        _generate_fractal_noise_2d(
            z_grid.shape, noise_res, noise_octaves, 0.5, noise_lacunarity, (True, True)
        )
        * noise_scale
    )
    z_grid += noise
    border_px = int(cfg.border_width / cfg.resolution)
    z_grid = _apply_seamless_border(z_grid, border_px)

    mesh = _create_solid_mesh_from_heightmap(x_grid, y_grid, z_grid, cfg.thickness)
    origin = _terrain_origin(cfg, x_grid, y_grid, z_grid)
    return [mesh], origin


def tilted_ground_terrain(
    difficulty: float, cfg: mesh_terrains_cfg.MeshTiltedTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    num_x_points = int(round(cfg.size[0] / cfg.resolution)) + 1
    num_y_points = int(round(cfg.size[1] / cfg.resolution)) + 1
    x = np.linspace(0, cfg.size[0], num_x_points)
    y = np.linspace(0, cfg.size[1], num_y_points)
    x_grid, y_grid = np.meshgrid(x, y)

    def lerp(val_range, d):
        return val_range[0] + d * (val_range[1] - val_range[0])

    max_abs_pitch = lerp(cfg.max_abs_pitch_range, difficulty)
    max_abs_roll = lerp(cfg.max_abs_roll_range, difficulty)
    pitch = np.random.uniform(-max_abs_pitch, max_abs_pitch)
    roll = np.random.uniform(-max_abs_roll, max_abs_roll)
    noise_scale = lerp(cfg.noise_scale_range, difficulty)
    noise_period = lerp(cfg.noise_period_range, difficulty)
    noise_octaves = round(lerp(cfg.noise_octaves_range, difficulty))
    noise_lacunarity = lerp(cfg.noise_lacunarity_range, difficulty)

    z_grid = -x_grid * np.tan(pitch) + y_grid * np.tan(roll)
    noise_res = (
        max(1, int(cfg.size[1] / noise_period)),
        max(1, int(cfg.size[0] / noise_period)),
    )
    noise = (
        _generate_fractal_noise_2d(
            z_grid.shape, noise_res, noise_octaves, 0.5, noise_lacunarity, (True, True)
        )
        * noise_scale
    )
    z_grid += noise
    border_px = int(cfg.border_width / cfg.resolution)
    z_grid = _apply_seamless_border(z_grid, border_px)

    mesh = _create_solid_mesh_from_heightmap(x_grid, y_grid, z_grid, cfg.thickness)
    origin = _terrain_origin(cfg, x_grid, y_grid, z_grid)
    return [mesh], origin


def _terrain_origin(cfg, x_grid, y_grid, z_grid) -> np.ndarray:
    """Place the terrain origin at the highest surface point near its center."""
    center_x, center_y = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    in_center_window = (np.abs(x_grid - center_x) <= 1.0) & (
        np.abs(y_grid - center_y) <= 1.0
    )
    origin_z = float(np.max(z_grid[in_center_window]))
    return np.array([center_x, center_y, origin_z])


def _interpolant(t):
    return t * t * t * (t * (t * 6 - 15) + 10)


def _generate_perlin_noise_2d(shape, res, tileable=(False, False)):
    angles = 2 * np.pi * np.random.rand(res[0] + 1, res[1] + 1)
    gradients = np.dstack((np.cos(angles), np.sin(angles)))
    if tileable[0]:
        gradients[-1, :] = gradients[0, :]
    if tileable[1]:
        gradients[:, -1] = gradients[:, 0]

    x = np.linspace(0, res[0], shape[0], endpoint=False)
    y = np.linspace(0, res[1], shape[1], endpoint=False)
    grid_x, grid_y = np.meshgrid(x, y, indexing="ij")

    ix0, iy0 = grid_x.astype(int), grid_y.astype(int)
    ix1, iy1 = (ix0 + 1), (iy0 + 1)
    frac_x, frac_y = grid_x - ix0, grid_y - iy0

    if tileable[0]:
        ix1 %= res[0]
    if tileable[1]:
        iy1 %= res[1]

    g00 = gradients[ix0, iy0]
    g10 = gradients[ix1, iy0]
    g01 = gradients[ix0, iy1]
    g11 = gradients[ix1, iy1]

    v00 = np.dstack((frac_x, frac_y))
    v10 = np.dstack((frac_x - 1, frac_y))
    v01 = np.dstack((frac_x, frac_y - 1))
    v11 = np.dstack((frac_x - 1, frac_y - 1))

    n00 = np.sum(v00 * g00, axis=2)
    n10 = np.sum(v10 * g10, axis=2)
    n01 = np.sum(v01 * g01, axis=2)
    n11 = np.sum(v11 * g11, axis=2)

    t = _interpolant(np.dstack((frac_x, frac_y)))
    n0 = n00 * (1 - t[..., 0]) + n10 * t[..., 0]
    n1 = n01 * (1 - t[..., 0]) + n11 * t[..., 0]

    return np.sqrt(2) * (n0 * (1 - t[..., 1]) + n1 * t[..., 1])


def _generate_fractal_noise_2d(
    shape, res, octaves=1, persistence=0.5, lacunarity=2, tileable=(False, False)
):
    noise = np.zeros(shape)
    frequency = 1.0
    amplitude = 1.0
    for _ in range(int(round(octaves))):
        octave_res = (max(1, int(frequency * res[0])), max(1, int(frequency * res[1])))
        noise += amplitude * _generate_perlin_noise_2d(shape, octave_res, tileable)
        frequency *= lacunarity
        amplitude *= persistence
    return noise


def _create_solid_mesh_from_heightmap(
    x_grid, y_grid, z_grid, thickness: float
) -> trimesh.Trimesh:
    rows, cols = z_grid.shape
    top_vertices = np.stack(
        [x_grid.flatten(), y_grid.flatten(), z_grid.flatten()], axis=1
    )
    bottom_vertices = top_vertices.copy()
    bottom_vertices[:, 2] = np.min(z_grid) - thickness
    vertices = np.vstack([top_vertices, bottom_vertices])
    num_verts = top_vertices.shape[0]

    top_faces = []
    for i in range(rows - 1):
        for j in range(cols - 1):
            v0, v1, v2, v3 = (
                i * cols + j,
                i * cols + j + 1,
                (i + 1) * cols + j,
                (i + 1) * cols + j + 1,
            )
            top_faces.extend([[v0, v1, v2], [v1, v3, v2]])
    bottom_faces = [
        [f[0] + num_verts, f[2] + num_verts, f[1] + num_verts] for f in top_faces
    ]

    side_faces = []
    for j in range(cols - 1):
        v0, v1, v2, v3 = j, j + 1, j + num_verts, j + 1 + num_verts
        side_faces.extend([[v0, v2, v1], [v1, v2, v3]])
        v0, v1, v2, v3 = (
            (rows - 1) * cols + j,
            (rows - 1) * cols + j + 1,
            (rows - 1) * cols + j + num_verts,
            (rows - 1) * cols + j + 1 + num_verts,
        )
        side_faces.extend([[v0, v1, v2], [v1, v3, v2]])
    for i in range(rows - 1):
        v0, v1, v2, v3 = (
            i * cols,
            (i + 1) * cols,
            i * cols + num_verts,
            (i + 1) * cols + num_verts,
        )
        side_faces.extend([[v0, v1, v2], [v1, v3, v2]])
        v0, v1, v2, v3 = (
            i * cols + cols - 1,
            (i + 1) * cols + cols - 1,
            i * cols + cols - 1 + num_verts,
            (i + 1) * cols + cols - 1 + num_verts,
        )
        side_faces.extend([[v0, v2, v1], [v1, v2, v3]])

    return trimesh.Trimesh(
        vertices=vertices, faces=np.vstack([top_faces, bottom_faces, side_faces])
    )


def _apply_seamless_border(z_grid, border_pixels: int) -> np.ndarray:
    if border_pixels <= 0:
        return z_grid
    fade = _interpolant(np.linspace(0, 1, border_pixels))
    z_grid[:border_pixels, :] *= fade.reshape(-1, 1)
    z_grid[-border_pixels:, :] *= fade[::-1].reshape(-1, 1)
    z_grid[:, :border_pixels] *= fade.reshape(1, -1)
    z_grid[:, -border_pixels:] *= fade[::-1].reshape(1, -1)
    return z_grid
