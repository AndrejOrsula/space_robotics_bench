"""Managed PhysX particle systems for SRB environments.

Usage::

    from srb.core.asset.particle_system import (
        ParticleSystem,
        ParticleSystemCfg,
        ParticleSystemData,
    )
"""

from .particle_system import ParticleSystem
from .particle_system_cfg import ParticleSystemCfg
from .particle_system_data import ParticleSystemData

__all__ = [
    "ParticleSystem",
    "ParticleSystemCfg",
    "ParticleSystemData",
]
