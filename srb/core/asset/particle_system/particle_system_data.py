"""Data container for ParticleSystem asset state."""

from __future__ import annotations

import torch


class ParticleSystemData:
    """Stores buffered particle system state data.

    This class holds cached tensors for particle positions and velocities,
    along with default (initial/settled) state for efficient resets.
    All tensors are stored on the specified device and follow the shape
    convention ``(num_envs, max_particles, 3)``.
    """

    def __init__(self, num_envs: int, max_particles: int, device: str):
        """Initialize particle system data buffers.

        Args:
            num_envs: Number of parallel environments.
            max_particles: Maximum number of particles per environment.
            device: Torch device for tensor allocation (e.g., "cuda:0", "cpu").
        """
        self.num_envs = num_envs
        self.max_particles = max_particles
        self.device = device

        # Current state buffers (updated from simulation)
        self.particle_pos_w: torch.Tensor = torch.zeros(
            num_envs, max_particles, 3, device=device, dtype=torch.float32
        )
        """World-frame particle positions. Shape: ``(num_envs, max_particles, 3)``."""

        self.particle_vel_w: torch.Tensor = torch.zeros(
            num_envs, max_particles, 3, device=device, dtype=torch.float32
        )
        """World-frame particle velocities. Shape: ``(num_envs, max_particles, 3)``."""

        # Default state (populated after initial settling)
        self.default_particle_pos_w: torch.Tensor = torch.zeros(
            num_envs, max_particles, 3, device=device, dtype=torch.float32
        )
        """Default (settled) particle positions for reset. Shape: ``(num_envs, max_particles, 3)``."""

        self.default_particle_vel_w: torch.Tensor = torch.zeros(
            num_envs, max_particles, 3, device=device, dtype=torch.float32
        )
        """Default (settled) particle velocities for reset. Shape: ``(num_envs, max_particles, 3)``."""

        self._has_default_state: bool = False
        """Whether the default state has been populated (post-settle)."""

    @property
    def has_default_state(self) -> bool:
        """Whether the default (settled) state has been cached."""
        return self._has_default_state

    def store_default_state(self) -> None:
        """Snapshot the current state as the default state for resets."""
        self.default_particle_pos_w[:] = self.particle_pos_w
        self.default_particle_vel_w[:] = self.particle_vel_w
        self._has_default_state = True

    def update(
        self,
        pos_w: torch.Tensor,
        vel_w: torch.Tensor,
        env_ids: torch.Tensor | None = None,
    ) -> None:
        """Update cached state buffers from simulation data.

        Args:
            pos_w: Particle positions. Shape: ``(num_envs_or_subset, num_particles, 3)``.
            vel_w: Particle velocities. Shape: ``(num_envs_or_subset, num_particles, 3)``.
            env_ids: Environment indices to update. None means all environments.
        """
        if env_ids is None:
            self.particle_pos_w[:] = pos_w
            self.particle_vel_w[:] = vel_w
        else:
            self.particle_pos_w[env_ids] = pos_w
            self.particle_vel_w[env_ids] = vel_w
