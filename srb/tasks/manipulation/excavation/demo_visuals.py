"""Best-effort visual polish for the excavation demo."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Any

import numpy as np
import torch

from srb.utils import logging
from srb.utils.str import resolve_env_prim_path

from .task import _PILE_FORWARD_M

if TYPE_CHECKING:
    from .task_demo import DemoTask


class ExcavationDemoVisuals:
    """Visual regolith deformation and dust effects for demo runs."""

    def __init__(self, env: "DemoTask") -> None:
        self._env = env
        self._cfg = env.cfg
        self._enabled = False
        self._stage = None
        self._Sdf: Any = None
        self._UsdGeom: Any = None
        self._Gf: Any = None
        self._Vt: Any = None

        self._grid_n = max(9, int(self._cfg.demo_surface_resolution))
        self._size_x = float(self._cfg.demo_surface_size_m[0])
        self._size_y = float(self._cfg.demo_surface_size_m[1])
        self._cell_area_m2 = (self._size_x / (self._grid_n - 1)) * (
            self._size_y / (self._grid_n - 1)
        )
        self._mesh_prims: list[Any] = []
        self._dust_prims: list[Any | None] = []
        self._z_grids: list[np.ndarray] = []
        self._volume_l = [0.0 for _ in range(env.num_envs)]
        self._dust_rng = np.random.default_rng(7)

        self._x_grid, self._y_grid, self._base_z = self._build_base_grid()
        self._face_indices = self._quad_indices()
        self._face_counts = [4] * (len(self._face_indices) // 4)

        try:
            from isaaclab.sim.utils import get_current_stage
            from pxr import Gf, Sdf, UsdGeom, Vt

            self._stage = get_current_stage()
            self._Sdf = Sdf
            self._UsdGeom = UsdGeom
            self._Gf = Gf
            self._Vt = Vt
        except Exception as e:
            logging.warning(f"Excavation demo visuals disabled: {e}")
            return

        try:
            for env_id in range(env.num_envs):
                self._create_env_visuals(env_id)
        except Exception as e:
            logging.warning(f"Failed to create excavation demo visuals: {e}")
            return

        self._enabled = bool(self._mesh_prims)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def reset(self, env_ids: Sequence[int]) -> None:
        if not self._enabled:
            return
        for env_id in env_ids:
            if int(env_id) >= len(self._z_grids):
                continue
            i = int(env_id)
            self._z_grids[i][:] = self._base_z
            self._volume_l[i] = 0.0
            self._write_mesh_points(i)
            self._set_dust_visible(i, False)

    def update(
        self,
        *,
        ee_pos_b: torch.Tensor,
        pen_depth: torch.Tensor,
        contact_force: torch.Tensor,
    ) -> list[float]:
        if not self._enabled:
            return self._volume_l

        ee_np = ee_pos_b.detach().cpu().numpy()
        pen_np = pen_depth.reshape(-1).detach().cpu().numpy()
        contact_np = contact_force.norm(dim=-1).detach().cpu().numpy()

        for env_id in range(min(self._env.num_envs, len(self._z_grids))):
            ee = ee_np[env_id]
            pen = float(pen_np[env_id])
            contact = float(contact_np[env_id])
            active = self._should_deform(ee, pen, contact)

            if active:
                self._deform(env_id, ee, pen, contact)
                self._write_mesh_points(env_id)
                self._update_dust(env_id, ee, contact)
            else:
                self._set_dust_visible(env_id, False)

            removed_m3 = float(
                np.maximum(self._base_z - self._z_grids[env_id], 0.0).sum()
            )
            self._volume_l[env_id] = removed_m3 * self._cell_area_m2 * 1000.0

        return self._volume_l

    def _build_base_grid(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        xs = np.linspace(
            _PILE_FORWARD_M - 0.5 * self._size_x,
            _PILE_FORWARD_M + 0.5 * self._size_x,
            self._grid_n,
            dtype=np.float32,
        )
        ys = np.linspace(
            -0.5 * self._size_y,
            0.5 * self._size_y,
            self._grid_n,
            dtype=np.float32,
        )
        x_grid, y_grid = np.meshgrid(xs, ys, indexing="ij")
        rx = (x_grid - _PILE_FORWARD_M) / (0.26 * self._size_x)
        ry = y_grid / (0.30 * self._size_y)
        mound = 0.11 * np.exp(-0.5 * (rx * rx + ry * ry))
        pad = 0.006 + 0.010 * np.exp(-0.5 * ((y_grid / self._size_y) ** 2))
        base_z = (pad + mound).astype(np.float32)
        return x_grid, y_grid, base_z

    def _create_env_visuals(self, env_id: int) -> None:
        assert self._stage is not None
        root = resolve_env_prim_path("{ENV_REGEX_NS}/excavation_demo", env_id)
        mesh_path = f"{root}/regolith_surface"
        dust_path = f"{root}/regolith_dust"

        mesh = self._UsdGeom.Mesh.Define(self._stage, self._Sdf.Path(mesh_path))
        mesh.GetPointsAttr().Set(
            self._Vt.Vec3fArray.FromNumpy(self._points_from_z(self._base_z))
        )
        mesh.GetFaceVertexCountsAttr().Set(self._Vt.IntArray(self._face_counts))
        mesh.GetFaceVertexIndicesAttr().Set(self._Vt.IntArray(self._face_indices))
        mesh.CreateSubdivisionSchemeAttr().Set("none")
        mesh.CreateDoubleSidedAttr().Set(True)
        mesh.CreateDisplayColorAttr().Set([self._Gf.Vec3f(0.50, 0.49, 0.45)])
        mesh.CreateDisplayOpacityAttr().Set([0.72])

        self._mesh_prims.append(mesh)
        self._z_grids.append(self._base_z.copy())

        dust = None
        if int(self._cfg.demo_dust_particles) > 0:
            n_dust = int(self._cfg.demo_dust_particles)
            dust = self._UsdGeom.Points.Define(self._stage, self._Sdf.Path(dust_path))
            dust.GetPointsAttr().Set(
                self._Vt.Vec3fArray.FromNumpy(np.zeros((n_dust, 3), dtype=np.float32))
            )
            dust.GetWidthsAttr().Set(self._Vt.FloatArray([0.010] * n_dust))
            dust.CreateDisplayColorAttr().Set([self._Gf.Vec3f(0.75, 0.72, 0.64)])
            dust.CreateDisplayOpacityAttr().Set([0.68])
            self._UsdGeom.Imageable(dust.GetPrim()).GetVisibilityAttr().Set("invisible")
        self._dust_prims.append(dust)

    def _points_from_z(self, z_grid: np.ndarray) -> np.ndarray:
        points = np.empty((self._grid_n, self._grid_n, 3), dtype=np.float32)
        points[:, :, 0] = self._x_grid
        points[:, :, 1] = self._y_grid
        points[:, :, 2] = z_grid
        return points.reshape(-1, 3)

    def _quad_indices(self) -> list[int]:
        indices: list[int] = []
        for i in range(self._grid_n - 1):
            for j in range(self._grid_n - 1):
                x = 0.25 * (
                    self._x_grid[i, j]
                    + self._x_grid[i + 1, j]
                    + self._x_grid[i, j + 1]
                    + self._x_grid[i + 1, j + 1]
                )
                y = 0.25 * (
                    self._y_grid[i, j]
                    + self._y_grid[i + 1, j]
                    + self._y_grid[i, j + 1]
                    + self._y_grid[i + 1, j + 1]
                )
                rx = (float(x) - _PILE_FORWARD_M) / (0.48 * self._size_x)
                ry = float(y) / (0.46 * self._size_y)
                boundary = (
                    1.0
                    + 0.08 * np.sin(17.0 * float(x))
                    + 0.05 * np.cos(19.0 * float(y))
                )
                if np.sqrt(rx * rx + ry * ry) > boundary:
                    continue
                a = i * self._grid_n + j
                indices.extend([a, a + 1, a + self._grid_n + 1, a + self._grid_n])
        return indices

    def _write_mesh_points(self, env_id: int) -> None:
        self._mesh_prims[env_id].GetPointsAttr().Set(
            self._Vt.Vec3fArray.FromNumpy(self._points_from_z(self._z_grids[env_id]))
        )

    def _should_deform(self, ee: np.ndarray, pen: float, contact: float) -> bool:
        x_min = _PILE_FORWARD_M - 0.55 * self._size_x
        x_max = _PILE_FORWARD_M + 0.55 * self._size_x
        y_max = 0.55 * self._size_y
        inside_pad = x_min <= float(ee[0]) <= x_max and abs(float(ee[1])) <= y_max
        return inside_pad and (pen > 0.004 or contact > 8.0)

    def _deform(self, env_id: int, ee: np.ndarray, pen: float, contact: float) -> None:
        z = self._z_grids[env_id]
        radius = float(self._cfg.demo_surface_brush_radius_m)
        dx = self._x_grid - float(ee[0])
        dy = self._y_grid - float(ee[1])
        dist = np.sqrt(dx * dx + dy * dy)
        brush = np.exp(-0.5 * (dist / radius) ** 2)
        ring = np.exp(-0.5 * ((dist - 1.55 * radius) / (0.34 * radius)) ** 2)

        contact_gain = min(contact / 160.0, 1.0)
        pen_gain = min(max(pen, 0.0) / 0.08, 1.0)
        strength = float(self._cfg.demo_surface_deform_rate_m) * (
            0.45 + contact_gain + pen_gain
        )

        min_z = self._base_z - float(self._cfg.demo_surface_max_depth_m)
        berm_cap = self._base_z + float(self._cfg.demo_surface_berm_height_m)
        z_next = np.maximum(z - strength * brush, min_z)
        z_next = np.minimum(z_next + 0.35 * strength * ring, berm_cap)

        idx = np.arange(self._grid_n, dtype=np.float32)
        border_i = np.minimum(idx, (self._grid_n - 1) - idx)[:, None]
        border_j = np.minimum(idx, (self._grid_n - 1) - idx)[None, :]
        border = np.minimum(border_i, border_j)
        edge_weight = np.clip(border / 3.0, 0.0, 1.0)
        self._z_grids[env_id] = (
            self._base_z * (1.0 - edge_weight) + z_next * edge_weight
        )

    def _update_dust(self, env_id: int, ee: np.ndarray, contact: float) -> None:
        dust = self._dust_prims[env_id]
        if dust is None:
            return

        n_dust = int(self._cfg.demo_dust_particles)
        scale = 0.035 + 0.035 * min(contact / 160.0, 1.0)
        cloud = self._dust_rng.normal(0.0, 1.0, size=(n_dust, 3)).astype(np.float32)
        cloud[:, 2] = np.abs(cloud[:, 2])
        center = np.array(
            [float(ee[0]), float(ee[1]), max(0.035, min(float(ee[2]), 0.30))],
            dtype=np.float32,
        )
        points = center + cloud * np.array(
            [scale, scale, 0.55 * scale], dtype=np.float32
        )
        dust.GetPointsAttr().Set(self._Vt.Vec3fArray.FromNumpy(points))
        self._set_dust_visible(env_id, True)

    def _set_dust_visible(self, env_id: int, visible: bool) -> None:
        if env_id >= len(self._dust_prims) or self._dust_prims[env_id] is None:
            return
        visibility = "inherited" if visible else "invisible"
        self._UsdGeom.Imageable(
            self._dust_prims[env_id].GetPrim()
        ).GetVisibilityAttr().Set(visibility)
