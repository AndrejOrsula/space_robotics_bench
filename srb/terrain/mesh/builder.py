"""Build heightmap meshes with optional skirts, solid shells, and face budgets.

Solid shells are watertight when ``skirts=False``. With both options enabled,
skirts remain separate and the combined mesh is not watertight. Face budgets
apply only to the fused path: decimate the stride-1 grid, then rebuild UVs and
extract its boundary for skirts and shells. Without a budget, LOD striding is
unchanged.
"""

from __future__ import annotations

from typing import Literal

import numpy as np
import trimesh

from srb.terrain.errors import (
    DegenerateGeometryError,
    EmptyHeightmapError,
    MeshDecimationUnavailableError,
)
from srb.terrain.ingest.dem import Dem
from srb.terrain.log import get_logger

Lod = Literal["low", "medium", "high"]

LOD_FACTORS: dict[Lod, int] = {"high": 1, "medium": 2, "low": 4}

SKIRT_DROP_M = 5.0
SOLID_BOTTOM_DROP_M = 10.0

_log = get_logger("mesh.builder")


def heightmap_to_mesh(
    dem: Dem,
    *,
    lod: Lod = "medium",
    skirts: bool = True,
    solid: bool = True,
    center_xy: bool = False,
    z_origin: Literal["none", "center", "mean", "min"] = "none",
    face_budget: int | None = None,
) -> trimesh.Trimesh:
    if dem.heights.size <= 1:
        raise EmptyHeightmapError(
            "heightmap needs at least two elevation values to mesh"
        )
    if dem.gsd_m <= 0:
        raise DegenerateGeometryError(f"gsd_m must be positive, got {dem.gsd_m}")

    # Fused face budgets decimate a stride-1 grid; otherwise LOD strides it.
    factor = LOD_FACTORS[lod] if face_budget is None else 1
    h_arr = dem.heights[::factor, ::factor] if factor > 1 else dem.heights
    h_arr = _normalize_heights(h_arr, z_origin=z_origin)
    h, w = h_arr.shape
    if h < 2 or w < 2:
        raise EmptyHeightmapError(
            f"after LOD={lod}, grid is {h}x{w}; use lod='high' or a larger heightmap"
        )

    gsd = dem.gsd_m * factor
    grid_v, grid_f, grid_uv = _build_grid(h_arr, gsd)
    if center_xy:
        grid_v[:, 0] -= 0.5 * (w - 1) * gsd
        grid_v[:, 1] -= 0.5 * (h - 1) * gsd

    # Decimation removes the lattice, so pass its extracted boundary explicitly.
    border_idx: list[int] | None = None
    if face_budget is not None and grid_f.shape[0] > face_budget:
        grid_v, grid_f, grid_uv, border_idx = _decimate_grid(
            grid_v, grid_f, face_budget=face_budget
        )

    all_v = [grid_v]
    all_f = [grid_f]
    all_uv = [grid_uv]
    v_offset = len(grid_v)

    if skirts:
        sk_v, sk_f, sk_uv = _build_skirts(
            grid_v, grid_uv, h, w, skirt_v_offset=v_offset, border_idx=border_idx
        )
        all_v.append(sk_v)
        all_f.append(sk_f)  # already fully global
        all_uv.append(sk_uv)
        v_offset += len(sk_v)

    if solid:
        # Top-grid indices are global; pass the offset for the shell vertices.
        so_v, so_f, so_uv = _build_solid_shell(
            grid_v, h, w, shell_v_offset=v_offset, border_idx=border_idx
        )
        all_v.append(so_v)
        all_f.append(so_f)  # already fully global
        all_uv.append(so_uv)

    vertices = np.vstack(all_v)
    faces = np.vstack(all_f)
    uvs = np.vstack(all_uv)

    visual = trimesh.visual.TextureVisuals(uv=uvs)
    return trimesh.Trimesh(
        vertices=vertices,
        faces=faces,
        process=False,
        visual=visual,
    )


def center_z_origin_m(h_arr: np.ndarray) -> float:
    """Return the Z offset used by ``z_origin="center"``.

    Uses the center pixel, falling back to ``nanmedian`` if it is non-finite.
    Pass the same already-strided array used for meshing. This aligns the
    origin, though sampling a full-resolution DEM can still differ locally
    from a lower-LOD mesh surface.
    """

    h, w = h_arr.shape
    origin = float(h_arr[h // 2, w // 2])
    if not np.isfinite(origin):
        origin = float(np.nanmedian(h_arr))
    return origin


def _normalize_heights(
    h_arr: np.ndarray,
    *,
    z_origin: Literal["none", "center", "mean", "min"],
) -> np.ndarray:
    if z_origin == "none":
        return h_arr
    if z_origin not in ("center", "mean", "min"):
        raise ValueError("z_origin must be one of: 'none', 'center', 'mean', 'min'")

    h_arr = h_arr.astype(np.float64, copy=True)
    if z_origin == "center":
        origin = center_z_origin_m(h_arr)
    elif z_origin == "mean":
        origin = float(np.nanmean(h_arr))
    else:
        origin = float(np.nanmin(h_arr))

    if not np.isfinite(origin):
        raise DegenerateGeometryError(
            f"could not compute finite z origin for {z_origin!r}"
        )
    h_arr -= origin
    return h_arr


def _build_grid(h_arr: np.ndarray, gsd: float):
    """Build the top surface vertices, faces and UVs from the height array."""
    h, w = h_arr.shape
    ys, xs = np.indices((h, w), dtype=np.float64)
    vertices = np.column_stack(
        [
            (xs * gsd).ravel(),
            ((h - 1 - ys) * gsd).ravel(),  # flip so y grows north
            h_arr.astype(np.float64).ravel(),
        ]
    )
    uvs = np.column_stack(
        [
            (xs / max(w - 1, 1)).ravel(),
            (1.0 - ys / max(h - 1, 1)).ravel(),
        ]
    )
    # quad → 2 triangles, CCW from above
    idx = np.arange(h * w).reshape(h, w)
    a = idx[:-1, :-1]
    b = idx[:-1, 1:]
    c = idx[1:, :-1]
    d = idx[1:, 1:]
    tri1 = np.stack([a, c, d], axis=-1).reshape(-1, 3)
    tri2 = np.stack([a, d, b], axis=-1).reshape(-1, 3)
    faces = np.concatenate([tri1, tri2], axis=0)
    return vertices, faces, uvs


def _build_skirts(
    grid_v: np.ndarray,
    grid_uvs: np.ndarray,
    h: int,
    w: int,
    *,
    skirt_v_offset: int,
    border_idx: list[int] | None = None,
):
    """Extrude the perimeter downward to fill visual gaps at the horizon.

    Returned face indices are fully global; the caller adds no offset. UVs
    mirror each border vertex's edge texel. ``border_idx`` replaces the grid
    perimeter lookup after decimation removes the ``h x w`` lattice.
    """
    z_drop = float(np.nanmin(grid_v[:, 2])) - SKIRT_DROP_M

    if border_idx is None:
        border_idx = _perimeter_indices(h, w)
    n = len(border_idx)

    skirt_v = grid_v[border_idx].copy()
    skirt_v[:, 2] = z_drop

    skirt_uv = grid_uvs[border_idx].copy()

    faces = []
    for i in range(n - 1):
        top_a = border_idx[i]
        top_b = border_idx[i + 1]
        bot_a = skirt_v_offset + i
        bot_b = skirt_v_offset + (i + 1)
        faces.append([top_a, bot_b, bot_a])
        faces.append([top_a, top_b, bot_b])
    top_a = border_idx[n - 1]
    top_b = border_idx[0]
    bot_a = skirt_v_offset + (n - 1)
    bot_b = skirt_v_offset + 0
    faces.append([top_a, bot_b, bot_a])
    faces.append([top_a, top_b, bot_b])

    return skirt_v, np.array(faces, dtype=np.int64), skirt_uv


def _build_solid_shell(
    grid_v: np.ndarray,
    h: int,
    w: int,
    *,
    shell_v_offset: int,
    border_idx: list[int] | None = None,
):
    """Build watertight side walls and a bottom fan cap.

    Returned face indices are fully global; the caller adds no offset.
    ``border_idx`` replaces the grid perimeter after decimation removes the
    lattice. It must use ``_perimeter_indices``' clockwise winding: reversed
    input creates an inward-facing shell, so boundary extraction normalises it.
    """
    z_bottom = float(np.nanmin(grid_v[:, 2])) - SOLID_BOTTOM_DROP_M

    if border_idx is None:
        border_idx = _perimeter_indices(h, w)
    n = len(border_idx)

    bot_ring = grid_v[border_idx, :3].copy()
    bot_ring[:, 2] = z_bottom

    centroid = np.array(
        [bot_ring[:, 0].mean(), bot_ring[:, 1].mean(), z_bottom],
        dtype=np.float64,
    )

    shell_v = np.vstack([bot_ring, centroid[np.newaxis, :]])
    shell_uv = np.zeros((n + 1, 2), dtype=np.float64)

    side_faces = []
    for i in range(n - 1):
        top_a = border_idx[i]
        top_b = border_idx[i + 1]
        bot_a = shell_v_offset + i
        bot_b = shell_v_offset + (i + 1)
        # Match skirt winding so the shell faces outward.
        side_faces.append([top_a, bot_b, bot_a])
        side_faces.append([top_a, top_b, bot_b])

    top_a = border_idx[n - 1]
    top_b = border_idx[0]
    bot_a = shell_v_offset + (n - 1)
    bot_b = shell_v_offset + 0
    side_faces.append([top_a, bot_b, bot_a])
    side_faces.append([top_a, top_b, bot_b])

    cen = shell_v_offset + n
    bottom_faces = []
    # Keep the cap's normal downward to match the outward-facing side walls.
    for i in range(n - 1):
        bottom_faces.append([cen, shell_v_offset + i, shell_v_offset + i + 1])
    bottom_faces.append([cen, shell_v_offset + n - 1, shell_v_offset + 0])

    all_faces = np.array(side_faces + bottom_faces, dtype=np.int64)
    return shell_v, all_faces, shell_uv


def _perimeter_indices(h: int, w: int) -> list[int]:
    """Ordered perimeter vertex indices into a (h, w) grid, walking CW from top-left.

    Returns exactly 2*(h+w)-4 unique indices (no repeated closing vertex).
    The implicit closing edge runs from the last element back to element 0.
    """
    h_w = h * w
    top = list(range(0, w))  # 0 .. w-1           (w verts)
    right = list(range(2 * w - 1, h_w, w))  # top-right already in top, skip it
    bottom = list(range(h_w - 2, h_w - w - 1, -1))  # skip bottom-right (last of right)
    left = list(range(h_w - 2 * w, 0, -w))  # skip both corners (in bottom / top)
    return top + right + bottom + left


def _decimate_grid(
    grid_v: np.ndarray,
    grid_f: np.ndarray,
    *,
    face_budget: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[int]]:
    """Quadric-decimate to ``face_budget`` and restore UVs and boundary.

    Recompute UVs against the original grid extents because decimation can
    move boundary vertices inward. Extract and normalize the boundary because
    decimation removes the lattice used by ``_perimeter_indices``. Wrap
    decimator failures in ``MeshDecimationUnavailableError``; face culling
    would meet the count while corrupting mesh quality. Caller skips this
    function when already within budget.
    """
    natural_faces = grid_f.shape[0]
    x0 = float(grid_v[:, 0].min())
    y0 = float(grid_v[:, 1].min())
    extent_x = float(grid_v[:, 0].max()) - x0
    extent_y = float(grid_v[:, 1].max()) - y0

    top_mesh = trimesh.Trimesh(vertices=grid_v, faces=grid_f, process=False)
    try:
        decimated = top_mesh.simplify_quadric_decimation(face_count=face_budget)
    except Exception as exc:
        raise MeshDecimationUnavailableError(
            face_budget=face_budget, natural_face_count=natural_faces, cause=exc
        ) from exc

    v = decimated.vertices
    f = decimated.faces
    uvs = np.column_stack(
        [
            (v[:, 0] - x0) / extent_x,
            (v[:, 1] - y0) / extent_y,
        ]
    )

    border_idx = _extract_boundary_loop(decimated)

    _log.warning(
        "mesh face budget %d < natural %d faces; quadric-decimated grid to %d faces",
        face_budget,
        natural_faces,
        f.shape[0],
    )

    return v, f, uvs, border_idx


def _extract_boundary_loop(mesh: trimesh.Trimesh) -> list[int]:
    """Return one ordered boundary cycle, wound like ``_perimeter_indices``.

    Directed triangle edges avoid ``Trimesh.outline()``'s SciPy dependency.
    Reject multiple loops and malformed boundaries rather than selecting an
    arbitrary component.
    """
    faces = np.asarray(mesh.faces, dtype=np.int64)
    if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
        raise DegenerateGeometryError("decimated grid mesh has no triangle faces")

    directed_edges = np.vstack(
        (
            faces[:, [0, 1]],
            faces[:, [1, 2]],
            faces[:, [2, 0]],
        )
    )
    canonical_edges = np.sort(directed_edges, axis=1)
    _, inverse, counts = np.unique(
        canonical_edges,
        axis=0,
        return_inverse=True,
        return_counts=True,
    )
    if np.any(counts > 2):
        raise DegenerateGeometryError("decimated grid mesh has non-manifold edges")
    boundary_edges = directed_edges[counts[inverse] == 1]
    if len(boundary_edges) < 3:
        raise DegenerateGeometryError("decimated grid mesh has no boundary loop")

    outgoing: dict[int, int] = {}
    incoming: dict[int, int] = {}
    for raw_start, raw_end in boundary_edges:
        start = int(raw_start)
        end = int(raw_end)
        if start == end or start in outgoing or end in incoming:
            raise DegenerateGeometryError(
                "decimated grid boundary is not one consistently wound manifold cycle"
            )
        outgoing[start] = end
        incoming[end] = start

    if set(outgoing) != set(incoming):
        raise DegenerateGeometryError(
            "decimated grid boundary has unmatched incoming/outgoing edges"
        )

    start = int(boundary_edges[0, 0])
    loop: list[int] = []
    current = start
    while current not in loop:
        loop.append(current)
        current = outgoing[current]
    if current != start or len(loop) != len(boundary_edges):
        raise DegenerateGeometryError(
            "decimated grid mesh has multiple boundary loops (expected exactly 1)"
        )

    if _signed_area_xy(mesh.vertices[loop, :2]) > 0:
        loop.reverse()
    return loop


def _signed_area_xy(points_xy: np.ndarray) -> float:
    """Return the shoelace area; positive is CCW, negative is clockwise.

    The closing edge is implicit. Match ``_perimeter_indices``' clockwise
    winding for all grid aspect ratios.
    """
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    x_next = np.roll(x, -1)
    y_next = np.roll(y, -1)
    return float(0.5 * np.sum(x * y_next - x_next * y))
