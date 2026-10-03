"""Seed Replicator with the seed Isaac Lab resolves for torch, NumPy, and random."""

from __future__ import annotations

from typing import Callable


def seed_replicator(seed: int) -> None:
    """Seed Replicator when its module is importable, as upstream did."""
    try:
        import omni.replicator.core as rep
    except ModuleNotFoundError:
        return
    rep.set_global_seed(seed)


def with_replicator_seed(
    upstream_seed: Callable[[int], int],
) -> staticmethod[[int], int]:
    """Return ``upstream_seed`` as a static ``seed`` that also seeds Replicator."""

    def seed(seed: int = -1) -> int:
        # Resolve the ``-1`` "draw a seed" sentinel upstream first so Replicator
        # receives the same concrete seed as the other generators.
        resolved = upstream_seed(seed)
        seed_replicator(resolved)
        return resolved

    seed.__doc__ = upstream_seed.__doc__
    return staticmethod(seed)
