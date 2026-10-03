"""One-shot handoff of authored local-space particle states before physics."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from threading import Lock

import numpy as np


@dataclass(frozen=True, slots=True)
class AuthoredParticleState:
    """Immutable parent-local particle arrays captured by a spawner."""

    positions: np.ndarray
    velocities: np.ndarray


_AUTHORED_STATES: dict[str, AuthoredParticleState] = {}
_AUTHORED_STATES_LOCK = Lock()


def register_authored_particle_states(
    prim_paths: Sequence[str],
    positions: Sequence[object] | np.ndarray,
    velocities: Sequence[object] | np.ndarray,
) -> None:
    """Register one exact authored state for each cloned particle-set path."""
    paths = tuple(str(path) for path in prim_paths)
    if not paths:
        raise ValueError("at least one particle-set prim path is required")

    positions_array = np.asarray(positions, dtype=np.float32)
    velocities_array = np.asarray(velocities, dtype=np.float32)
    if positions_array.ndim != 2 or positions_array.shape[1:] != (3,):
        raise ValueError(
            "authored particle positions must have shape (num_particles, 3), "
            f"got {positions_array.shape}"
        )
    if velocities_array.shape != positions_array.shape:
        raise ValueError(
            "authored particle velocities must match positions shape "
            f"{positions_array.shape}, got {velocities_array.shape}"
        )

    # Share one immutable copy across clone paths; consumers transform per prim.
    positions_array = np.array(positions_array, dtype=np.float32, copy=True)
    velocities_array = np.array(velocities_array, dtype=np.float32, copy=True)
    positions_array.setflags(write=False)
    velocities_array.setflags(write=False)
    state = AuthoredParticleState(positions_array, velocities_array)

    with _AUTHORED_STATES_LOCK:
        for path in paths:
            _AUTHORED_STATES[path] = state


def consume_authored_particle_states(
    prim_paths: Sequence[str], *, expected_num_particles: int | None = None
) -> tuple[AuthoredParticleState, ...]:
    """Consume captured states in requested path order, exactly once."""
    paths = tuple(str(path) for path in prim_paths)
    # Validate before any deletion so a bad request consumes nothing.
    if len(set(paths)) != len(paths):
        raise ValueError(f"duplicate particle-set prim paths: {paths}")
    with _AUTHORED_STATES_LOCK:
        missing = tuple(path for path in paths if path not in _AUTHORED_STATES)
        if missing:
            raise RuntimeError(
                "authored particle state was not captured for prim path(s): "
                + ", ".join(missing)
            )
        states = tuple(_AUTHORED_STATES[path] for path in paths)
        if expected_num_particles is not None:
            mismatched = tuple(
                path
                for path, state in zip(paths, states, strict=True)
                if state.positions.shape[0] != expected_num_particles
            )
            if mismatched:
                raise RuntimeError(
                    f"authored particle count does not match {expected_num_particles} "
                    "for prim path(s): " + ", ".join(mismatched)
                )
        for path in paths:
            del _AUTHORED_STATES[path]
    return states
