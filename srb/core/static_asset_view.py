"""Lazy pose view for static assets stored in Isaac Lab 3 scene extras.

The view uses Isaac Lab USD helpers and scalar-last ``(x, y, z, w)``
quaternions. It accesses Kit only when prims or poses are requested.
"""

from __future__ import annotations

import inspect
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import torch


class XformPrimView:
    """Lazy ``XFormPrim``-style view over the prims matching one path expression."""

    def __init__(self, prim_path: str, *, device: str = "cpu", name: str = ""):
        self.prim_path = prim_path
        self.name = name
        self._device = device
        self._prims: list[Any] | None = None

    def __repr__(self) -> str:
        return f"XformPrimView(name={self.name!r}, prim_path={self.prim_path!r})"

    @property
    def prims(self) -> list[Any]:
        """USD prims matching the path expression, resolved once."""
        if self._prims is None:
            from isaaclab.sim.utils import find_matching_prims

            self._prims = list(find_matching_prims(self.prim_path))
        return self._prims

    @property
    def prim_paths(self) -> list[str]:
        return [prim.GetPath().pathString for prim in self.prims]

    @property
    def count(self) -> int:
        return len(self.prims)

    def get_world_poses(
        self, indices: Any | None = None
    ) -> tuple["torch.Tensor", "torch.Tensor"]:
        """World positions ``(N, 3)`` and orientations ``(N, 4)`` as ``(x, y, z, w)``."""
        import torch
        from isaaclab.sim.utils import resolve_prim_pose

        positions: list[tuple[float, ...]] = []
        orientations: list[tuple[float, ...]] = []
        for index in self._selected(indices):
            position, xyzw = resolve_prim_pose(self.prims[index])
            positions.append(tuple(position))
            orientations.append(tuple(xyzw))
        return (
            torch.tensor(positions, dtype=torch.float32, device=self._device).reshape(
                -1, 3
            ),
            torch.tensor(
                orientations, dtype=torch.float32, device=self._device
            ).reshape(-1, 4),
        )

    def set_world_poses(
        self,
        positions: Any | None = None,
        orientations: Any | None = None,
        indices: Any | None = None,
    ) -> None:
        """Author world poses on the selected prims; ``orientations`` are ``(x, y, z, w)``.

        A pose component left ``None`` keeps its current world value. Rows of
        ``positions`` and ``orientations`` line up with ``indices`` (or with
        every prim when ``indices`` is ``None``); a single row is broadcast.
        """
        from isaaclab.sim.utils import (
            convert_world_pose_to_local,
            resolve_prim_pose,
            standardize_xform_ops,
        )

        selected = self._selected(indices)
        position_rows = _rows(positions, len(selected))
        orientation_rows = _rows(orientations, len(selected))
        for row, index in enumerate(selected):
            prim = self.prims[index]
            current_position, current_xyzw = resolve_prim_pose(prim)
            position = (
                tuple(position_rows[row])
                if position_rows is not None
                else tuple(current_position)
            )
            xyzw = (
                tuple(orientation_rows[row])
                if orientation_rows is not None
                else tuple(current_xyzw)
            )
            if len(xyzw) != 4:
                raise ValueError(f"expected (x, y, z, w) orientation, got {xyzw}")
            local_position, local_xyzw = convert_world_pose_to_local(
                position, xyzw, prim.GetParent()
            )
            standardize_xform_ops(
                prim, translation=local_position, orientation=local_xyzw
            )

    def _selected(self, indices: Any | None) -> list[int]:
        if indices is None:
            return list(range(self.count))
        return [int(index) for index in _as_list(indices)]


def _as_list(values: Any) -> list[Any]:
    tolist = getattr(values, "tolist", None)
    listed = tolist() if callable(tolist) else list(values)
    return listed if isinstance(listed, list) else [listed]


def _rows(values: Any | None, count: int) -> list[list[float]] | None:
    """Normalise a pose array-like to ``count`` rows, broadcasting a single row."""
    if values is None:
        return None
    rows = _as_list(values)
    if rows and not isinstance(rows[0], (list, tuple)):
        rows = [rows]
    rows = [[float(value) for value in row] for row in rows]
    if len(rows) == 1 and count != 1:
        rows = rows * count
    if len(rows) != count:
        raise ValueError(f"expected {count} pose row(s), got {len(rows)}")
    return rows


def _has_attribute(entity: Any, name: str) -> bool:
    """``hasattr`` without invoking descriptors such as properties."""
    try:
        inspect.getattr_static(entity, name)
    except AttributeError:
        return False
    return True


def static_asset_prim_path(entity: Any) -> str | None:
    """Return an extras asset's prim path, or ``None`` if it is already a view.

    Isaac Lab 3 extras may contain either a config or an asset with its config
    on ``cfg``. Detect views statically so checking ``prims`` does not resolve
    the property and access Kit.
    """
    if _has_attribute(entity, "prims") or _has_attribute(entity, "set_world_poses"):
        return None
    prim_path = getattr(entity, "prim_path", None)
    if isinstance(prim_path, str):
        return prim_path
    prim_path = getattr(getattr(entity, "cfg", None), "prim_path", None)
    return prim_path if isinstance(prim_path, str) else None


def is_static_asset_cfg(entity: Any) -> bool:
    """Is ``entity`` an Isaac Lab 3 ``extras`` entry (a config or an ``Asset``, not a view)?"""
    return static_asset_prim_path(entity) is not None


__all__ = ["XformPrimView", "is_static_asset_cfg", "static_asset_prim_path"]
