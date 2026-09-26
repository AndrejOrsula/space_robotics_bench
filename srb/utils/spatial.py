"""Pure NumPy spatial helpers; no Isaac Sim or torch dependency."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

__all__ = [
    "poisson_disk_positions",
    "HeightGrid",
    "build_height_grid",
    "pick_camera_up",
]


def pick_camera_up(
    eye,
    lookat,
    preferred_up: tuple[float, float, float] = (0.0, 0.0, 1.0),
    eps: float = 1e-3,
) -> tuple[float, float, float]:
    """Keep preferred up unless collinear (``|cos| > 1-eps``); then use Y, X.

    A zero view returns ``preferred_up`` unchanged.
    """
    eye_a = np.asarray(eye, dtype=np.float64)
    lookat_a = np.asarray(lookat, dtype=np.float64)
    up = np.asarray(preferred_up, dtype=np.float64)

    fwd = lookat_a - eye_a
    n = float(np.linalg.norm(fwd))
    if n == 0.0:
        return tuple(float(v) for v in up)
    fwd /= n

    up_norm = float(np.linalg.norm(up))
    if up_norm == 0.0:
        up = np.array([0.0, 0.0, 1.0])
        up_norm = 1.0
    if abs(float(fwd @ (up / up_norm))) <= 1.0 - eps:
        return tuple(float(v) for v in up)

    # Collinear with preferred up — fall back to the most-orthogonal world axis.
    for cand in ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)):
        c = np.asarray(cand, dtype=np.float64)
        if abs(float(fwd @ c)) <= 1.0 - eps:
            return cand
    return (0.0, 1.0, 0.0)


def poisson_disk_positions(
    count: int,
    *,
    spacing: float,
    region_half: float | None = None,
    seed: int = 0,
) -> list[tuple[float, float]]:
    """Sample seeded XY points at least ``spacing`` apart.

    Non-positive counts return empty. The square region grows as needed; an
    explicit ``region_half`` is floored at a feasible extent to avoid slow
    retries for undersized regions.
    """
    if count <= 0:
        return []

    radius = float(spacing)
    rng = np.random.default_rng(seed)

    # Feasible half-extent for a roughly square packing.
    feasible_half = 0.5 * radius * math.ceil(math.sqrt(max(count, 1))) + radius
    if region_half is None:
        half = feasible_half
    else:
        # Avoid repeated rejected attempts when the explicit region is too tight.
        half = max(float(region_half), feasible_half)

    radius_sq = radius * radius
    points: list[tuple[float, float]] = []
    attempts = 0
    max_attempts = 2000 * max(count, 1)
    while len(points) < count:
        if attempts > max_attempts:
            # Grow and retry when the region is still too tight.
            half *= 1.25
            points = []
            attempts = 0
        cand_x, cand_y = rng.uniform(-half, half, size=2)
        if all(
            (cand_x - px) ** 2 + (cand_y - py) ** 2 >= radius_sq for px, py in points
        ):
            points.append((float(cand_x), float(cand_y)))
        attempts += 1
    return points


@dataclass
class HeightGrid:
    """2D max-height field with filled holes; cells hold the max Z per XY bin."""

    grid: np.ndarray  # shape (nx, ny), float; no NaNs after construction
    x0: float
    y0: float
    res: float

    @property
    def shape(self) -> tuple[int, int]:
        return self.grid.shape  # type: ignore[return-value]

    def sample(self, xy: np.ndarray, surface_offset: float = 0.0) -> np.ndarray:
        """Look up each XY height and add ``surface_offset``."""
        xy = np.asarray(xy, dtype=np.float64)
        nx, ny = self.grid.shape
        pi = np.clip(((xy[:, 0] - self.x0) / self.res).astype(int), 0, nx - 1)
        pj = np.clip(((xy[:, 1] - self.y0) / self.res).astype(int), 0, ny - 1)
        return self.grid[pi, pj] + surface_offset


def build_height_grid(
    verts: np.ndarray,
    res: float = 0.2,
    dilation_passes: int = 8,
) -> HeightGrid:
    """Bin vertices by XY and take each cell's maximum Z.

    Empty cells receive nearest-neighbor dilation, then the global median.
    Raises ``ValueError`` for empty or malformed input and nonpositive ``res``.
    """
    verts = np.asarray(verts, dtype=np.float64)
    if verts.size == 0:
        raise ValueError("build_height_grid requires at least one vertex.")
    if verts.ndim != 2 or verts.shape[1] < 3:
        raise ValueError(
            f"build_height_grid requires (N, 3) vertices, got {verts.shape}."
        )
    if not np.isfinite(verts[:, :3]).all():
        raise ValueError("build_height_grid requires finite vertex coordinates.")
    if not math.isfinite(res) or res <= 0.0:
        raise ValueError(
            f"build_height_grid requires a positive resolution, got {res}."
        )

    vx, vy, vz = verts[:, 0], verts[:, 1], verts[:, 2]
    x0, y0 = float(vx.min()), float(vy.min())
    nx = int(np.ceil((vx.max() - x0) / res)) + 1
    ny = int(np.ceil((vy.max() - y0) / res)) + 1

    grid = np.full((nx, ny), np.nan, dtype=np.float64)
    gi = np.clip(((vx - x0) / res).astype(int), 0, nx - 1)
    gj = np.clip(((vy - y0) / res).astype(int), 0, ny - 1)
    # fmax.at ignores the NaN initialization; maximum.at would propagate it.
    np.fmax.at(grid, (gi, gj), vz)

    # Fill holes by nearest-neighbour dilation.
    for _ in range(dilation_passes):
        holes = np.isnan(grid)
        if not holes.any():
            break
        filled = grid.copy()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            # Keep shifts bounded; do not wrap across opposite edges.
            shifted = np.full_like(grid, np.nan)
            shifted[max(dx, 0) : nx + min(dx, 0), max(dy, 0) : ny + min(dy, 0)] = grid[
                max(-dx, 0) : nx + min(-dx, 0), max(-dy, 0) : ny + min(-dy, 0)
            ]
            take = holes & ~np.isnan(shifted)
            filled[take] = shifted[take]
        grid = filled

    global_med = float(np.nanmedian(grid))
    grid = np.nan_to_num(grid, nan=global_med)

    return HeightGrid(grid=grid, x0=x0, y0=y0, res=float(res))
