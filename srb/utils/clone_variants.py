"""Deterministic per-environment variant assignment for Isaac Lab clone plans."""

from __future__ import annotations

from typing import Any, Sequence

__all__ = ["PerEnvironmentCloneStrategy", "per_environment_world_indices"]


def per_environment_world_indices(group_sizes: Sequence[int], num_clones: int) -> Any:
    """Map each environment to the prototype encoding its ``i % size`` variants.

    Isaac Lab orders prototypes as a Cartesian product of groups, with the last
    group varying fastest. Singleton groups add no information, so only sizes
    greater than one contribute to the mixed-radix index.

    Args:
        group_sizes: Variant count of every env-scoped group, in the planner's
            group order. Sizes below 2 are ignored.
        num_clones: Number of environments.

    Returns:
        A ``[num_clones]`` ``int64`` array of world-prototype indices.
    """
    import numpy as np

    envs = np.arange(num_clones, dtype=np.int64)
    worlds = np.zeros(num_clones, dtype=np.int64)
    for size in group_sizes:
        if size > 1:
            worlds = worlds * size + envs % size
    return worlds


class PerEnvironmentCloneStrategy:
    """Assign environment ``i`` variant ``i % group_size`` in each group.

    This deterministic mapping gives each group ``min(num_clones, group_size)``
    distinct prototypes and avoids missing or uneven prim assignments. Groups
    smaller than the environment count still share prototypes. Isaac Lab's
    planner receives only prototype weights, so this strategy stores group
    sizes captured during scene conversion by
    :func:`srb.core.sim.spawners.multi_asset_compat.install_per_environment_clone_strategy`.
    """

    def __init__(self, group_sizes: Sequence[int]) -> None:
        self.group_sizes = tuple(int(size) for size in group_sizes)
        # Isaac Lab serializes callable cfg fields by name.
        self.__name__ = type(self).__name__
        self.__qualname__ = type(self).__qualname__

    def __call__(self, weights: Any, num_clones: int) -> Any:
        import math

        import numpy as np

        num_worlds = np.asarray(weights).shape[0]
        expected = math.prod(size for size in self.group_sizes if size > 1)
        if num_worlds != expected:
            raise ValueError(
                f"The clone planner offered {num_worlds} world prototypes, but the "
                f"scene's multi-variant groups {self.group_sizes} imply {expected}. "
                "The scene cfg changed after convert_scene_multi_variant_spawners "
                "installed this clone strategy."
            )
        return per_environment_world_indices(self.group_sizes, num_clones)

    def __eq__(self, other: object) -> bool:
        return (
            isinstance(other, PerEnvironmentCloneStrategy)
            and other.group_sizes == self.group_sizes
        )

    def __hash__(self) -> int:
        return hash(self.group_sizes)

    def __repr__(self) -> str:
        return f"{type(self).__name__}(group_sizes={self.group_sizes})"
