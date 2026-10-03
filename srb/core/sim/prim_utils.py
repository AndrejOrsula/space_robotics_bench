"""USD prim helpers that avoid Isaac Lab's deprecated query wrappers.

Isaac Sim 6.1 ships no ``isaacsim.core.utils`` extension any more (only
``isaacsim.core.experimental.*``), so the three helpers SRB still needed from
it -- ``find_unique_string_name``, ``get_articulation_root_api_prim_path`` and
``move_articulation_root`` -- are ported here with the 5.x semantics (breadth
first search that includes the queried prim; ``_N`` suffixes from 1).
"""

from __future__ import annotations

from collections.abc import Callable

from isaaclab.sim.utils import create_prim, get_current_stage
from pxr import PhysxSchema, Usd, UsdPhysics


def get_prim_at_path(prim_path: str, fabric: bool = False):
    """Return the prim at ``prim_path`` from the current USD stage."""
    stage = get_current_stage(fabric=fabric)
    return stage.GetPrimAtPath(prim_path) if stage is not None else None


def is_prim_path_valid(prim_path: str, fabric: bool = False) -> bool:
    """Return whether ``prim_path`` identifies a valid prim."""
    prim = get_prim_at_path(prim_path, fabric=fabric)
    return prim is not None and prim.IsValid()


def find_unique_string_name(
    initial_name: str, is_unique_fn: Callable[[str], bool]
) -> str:
    """Return ``initial_name`` or the first ``initial_name_N`` (N from 1) that
    satisfies ``is_unique_fn``."""
    if is_unique_fn(initial_name):
        return initial_name
    iterator = 1
    while True:
        candidate = f"{initial_name}_{iterator}"
        if is_unique_fn(candidate):
            return candidate
        iterator += 1


def find_first_prim_with_api(prim: Usd.Prim, api) -> Usd.Prim | None:
    """Breadth-first search from ``prim`` (inclusive) for a prim with ``api``."""
    queue = [prim]
    while queue:
        candidate = queue.pop(0)
        if candidate.HasAPI(api):
            return candidate
        queue.extend(candidate.GetChildren())
    return None


def get_articulation_root_api_prim_path(prim_path: str) -> str:
    """Path of the first prim at or below ``prim_path`` carrying
    ``UsdPhysics.ArticulationRootAPI``; ``prim_path`` itself when none does."""
    prim = get_prim_at_path(prim_path)
    if prim is None or not prim.IsValid():
        return prim_path
    found = find_first_prim_with_api(prim, UsdPhysics.ArticulationRootAPI)
    return found.GetPath().pathString if found is not None else prim_path


_ARTICULATION_ROOT_SCHEMAS = frozenset(
    ("PhysicsArticulationRootAPI", "PhysxArticulationAPI", "NewtonArticulationRootAPI")
)


def move_articulation_root(src_prim: Usd.Prim, dst_prim: Usd.Prim) -> None:
    """Move the articulation root APIs from ``src_prim`` to ``dst_prim``.

    Nothing happens when ``src_prim`` carries no ``ArticulationRootAPI``.
    """
    if not src_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return
    # Isaac Lab also applies `NewtonArticulationRootAPI` to the root it
    # spawns; left behind, it still counts as an articulation root.
    moved = [
        name
        for name in src_prim.GetAppliedSchemas()
        if name in _ARTICULATION_ROOT_SCHEMAS
    ]
    for name in moved:
        src_prim.RemoveAppliedSchema(name)
    dst_prim.ApplyAPI(UsdPhysics.ArticulationRootAPI)
    dst_prim.ApplyAPI(PhysxSchema.PhysxArticulationAPI)
    for name in moved:
        if name not in dst_prim.GetAppliedSchemas():
            dst_prim.AddAppliedSchema(name)


__all__ = [
    "create_prim",
    "find_first_prim_with_api",
    "find_unique_string_name",
    "get_articulation_root_api_prim_path",
    "get_current_stage",
    "get_prim_at_path",
    "is_prim_path_valid",
    "move_articulation_root",
]
