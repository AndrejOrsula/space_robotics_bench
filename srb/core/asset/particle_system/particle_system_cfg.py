"""Configuration for the ParticleSystem asset."""

from __future__ import annotations

from dataclasses import MISSING

from srb.core.sim.spawners.particles.cfg import (
    ParticlesSpawnerCfg,
)
from srb.utils.cfg import configclass


@configclass
class ParticleSystemCfg:
    """Spawn and runtime configuration for a managed PhysX particle system.

    Wraps :class:`ParticlesSpawnerCfg`; the spawner creates USD prims and
    :class:`ParticleSystem` manages particle and system state.
    """

    # ------------------------------------------------------------------
    # Scene prim path
    # ------------------------------------------------------------------
    prim_path: str = MISSING  # type: ignore[assignment]
    """Prim path (with ``{ENV_REGEX_NS}``) for the particle set points prim.

    Example: ``"{ENV_REGEX_NS}/regolith"``
    """

    # ------------------------------------------------------------------
    # Spawner configuration (defines distribution + physics)
    # ------------------------------------------------------------------
    spawner: ParticlesSpawnerCfg = MISSING  # type: ignore[assignment]
    """Spawner config that creates the USD particle system and particle set.

    Use :class:`GridParticlesSpawnerCfg` for uniform grids,
    :class:`PyramidParticlesSpawnerCfg` for tapered pyramidal deposits, or
    :class:`CraterPileParticlesSpawnerCfg` for a mass-conserving crater-and-pile
    layout.
    """

    # ------------------------------------------------------------------
    # Runtime management
    # ------------------------------------------------------------------
    # Placement comes from the scene ``AssetBaseCfg.init_state`` and the read
    # cadence from the task (e.g. ``particles_init_pos`` and
    # ``particles_update_interval``); this cfg carries neither.
    settle_max_steps: int = 10
    """Maximum number of settlement simulation batches during initial reset."""

    settle_step_time: float = 30.0
    """Simulated time (seconds) per settlement batch."""

    settle_vel_threshold: float = 0.01
    """Median velocity norm below which particles are considered settled."""

    settle_min_vel_change_threshold: float = 0.0005
    """Minimum velocity change between settlement steps to continue settling.

    A plateau only counts as settled while the median velocity is at most
    :attr:`settle_plateau_max_vel`."""

    settle_plateau_max_vel: float = 0.1
    """Largest median velocity norm (m/s) at which a velocity plateau counts as
    settled. PBD free fall also plateaus, at the terminal speed
    ``gravity / damping`` (1.62 / 0.15 = 10.8 m/s for lunar regolith)."""

    settle_max_median_drop: float = 10.0
    """Largest drop (m) of any env's median particle height during settling.

    A larger drop means the grains are passing through the ground; settling
    then raises instead of storing the fallen state as the reset default."""

    # ------------------------------------------------------------------
    # Convenience factory methods
    # ------------------------------------------------------------------
