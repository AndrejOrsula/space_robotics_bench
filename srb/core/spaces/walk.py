from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Iterator
from typing import Any

from gymnasium import spaces

LeafPath = tuple[str | int, ...]


def walk_leaves(space: spaces.Space) -> Iterator[tuple[LeafPath, spaces.Space]]:
    yield from _walk(space, ())


def _walk(
    space: spaces.Space,
    prefix: LeafPath,
) -> Iterator[tuple[LeafPath, spaces.Space]]:
    if isinstance(space, spaces.Dict):
        for key, subspace in space.spaces.items():
            yield from _walk(subspace, prefix + (key,))
        return
    if isinstance(space, spaces.Tuple):
        for idx, subspace in enumerate(space.spaces):
            yield from _walk(subspace, prefix + (idx,))
        return
    yield prefix, space


def map_boxes(
    space: spaces.Space,
    fn: Callable[[spaces.Box], spaces.Box],
) -> spaces.Space:
    if isinstance(space, spaces.Dict):
        return spaces.Dict(
            OrderedDict(
                (key, map_boxes(subspace, fn)) for key, subspace in space.spaces.items()
            )
        )
    if isinstance(space, spaces.Tuple):
        return spaces.Tuple(tuple(map_boxes(subspace, fn) for subspace in space.spaces))
    if isinstance(space, spaces.Box):
        return fn(space)
    return space


def apply_to_obs(
    obs: Any,
    space: spaces.Space,
    *,
    leaf_fn: Callable[[Any, spaces.Space], Any],
) -> Any:
    if isinstance(space, spaces.Dict):
        return {
            key: apply_to_obs(obs[key], subspace, leaf_fn=leaf_fn)
            for key, subspace in space.spaces.items()
        }
    if isinstance(space, spaces.Tuple):
        return tuple(
            apply_to_obs(obs[idx], subspace, leaf_fn=leaf_fn)
            for idx, subspace in enumerate(space.spaces)
        )
    return leaf_fn(obs, space)
