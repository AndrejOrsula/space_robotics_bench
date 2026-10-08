"""Isaac-free episode terrain state for landscaping tasks.

The task supplies particle positions and simulator refresh callbacks. This
module owns current and target maps, local sampling, and refresh cadence.
Targets must come from a manifest: per-episode maps or the crater task's
static target. Resets without either source fail; runtime target sampling and
initial-error baselines are intentionally unsupported.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Sequence

import torch

from srb.terrain.landscaping_heightmap import (
    DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY,
    LandscapingHeightmapGeometry,
    sample_global_map_at_heightmap,
)
from srb.terrain.landscaping_maps import (
    compute_elevation_map,
)

ParticlePositions = Callable[[], torch.Tensor]
ParticleStateUpdate = Callable[..., None]


@dataclass(frozen=True, slots=True)
class LandscapingEpisodeTerrainCfg:
    """Configuration needed to maintain episode terrain state."""

    elev_map_extent: float
    elev_map_size: int
    particle_update_interval_steps: int
    elevation_percentile: float = 0.9
    geometry: LandscapingHeightmapGeometry = DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY
    static_target_map: torch.Tensor | None = None
    """Optional static ``(elev_map_size, elev_map_size)`` env-local target.

    When set, every reset writes this exact map for the selected environments.
    The simple crater task supplies the manifest-materialized analytic flat-bed
    target here; the general task instead passes a per-episode ``target_maps``
    argument to :meth:`LandscapingEpisodeTerrain.reset`. Either way the map is
    manifest-owned — this state never samples a target of its own.
    """


class LandscapingEpisodeTerrain:
    """Stateful target/current map owner for terrain-landscaping episodes."""

    def __init__(
        self,
        *,
        num_envs: int,
        device: torch.device | str,
        cfg: LandscapingEpisodeTerrainCfg,
    ) -> None:
        self.cfg = cfg
        self._device = torch.device(device)
        self._num_envs = int(num_envs)
        map_size = int(cfg.elev_map_size)
        self._target_map = torch.zeros(
            self._num_envs, map_size, map_size, device=self._device
        )
        self._static_target: torch.Tensor | None = None
        if cfg.static_target_map is not None:
            static = torch.as_tensor(
                cfg.static_target_map, device=self._device, dtype=torch.float32
            )
            if static.shape != (map_size, map_size):
                raise ValueError(
                    "static_target_map must be "
                    f"({map_size}, {map_size}), got {tuple(static.shape)}"
                )
            if not torch.isfinite(static).all():
                raise ValueError("static_target_map must be fully finite")
            self._static_target = static
        self._particle_update_counter = self._update_interval_steps

    @property
    def target_map(self) -> torch.Tensor:
        """Current per-env target elevation maps, shape ``(num_envs, N, N)``."""
        return self._target_map

    def reset(
        self,
        env_ids: Sequence[int] | torch.Tensor,
        *,
        env_origins: torch.Tensor | None = None,
        default_particle_pos_w: torch.Tensor | None = None,
        target_maps: torch.Tensor | None = None,
    ) -> None:
        """Reset target maps for selected environments.

        ``target_maps`` is the per-episode manifest-owned coarse target for
        the selected rows, shape ``(len(env_ids), N, N)``; it takes precedence
        over the configured ``static_target_map``. One of them must exist:
        there is no runtime sampling fallback.
        """
        del default_particle_pos_w, env_origins  # no reset-derived target
        ids = self._coerce_env_ids(env_ids)
        if ids.numel() == 0:
            return

        if target_maps is not None:
            maps = torch.as_tensor(
                target_maps, device=self._device, dtype=torch.float32
            )
            size = int(self.cfg.elev_map_size)
            if maps.shape != (int(ids.numel()), size, size):
                raise ValueError(
                    "target_maps must be "
                    f"({int(ids.numel())}, {size}, {size}), got {tuple(maps.shape)}"
                )
            if not torch.isfinite(maps).all():
                raise ValueError("target_maps must be fully finite")
            self._target_map[ids] = maps
            return

        if self._static_target is not None:
            self._target_map[ids] = self._static_target
            return

        raise ValueError(
            "landscaping episode reset needs a manifest-owned target: pass "
            "target_maps or configure static_target_map"
        )

    def target_local(
        self,
        *,
        robot_pos_w: torch.Tensor,
        env_origins: torch.Tensor,
        robot_yaw: torch.Tensor,
    ) -> torch.Tensor:
        """Sample the owned target maps on the landscaping heightmap geometry."""
        return sample_global_map_at_heightmap(
            global_map=self._target_map,
            robot_pos_w=robot_pos_w,
            env_origins=env_origins,
            robot_yaw=robot_yaw,
            geometry=self.cfg.geometry,
            extent=self.cfg.elev_map_extent,
            resolution=self.cfg.elev_map_size,
        )

    def compute_current_map(
        self,
        *,
        particle_positions: ParticlePositions | None,
        env_origins: torch.Tensor,
        update_state_from_sim: ParticleStateUpdate | None = None,
        force_update_env_ids: Sequence[int] | torch.Tensor | None = None,
    ) -> torch.Tensor | None:
        """Refresh particle state (on cadence) and bin particles into the global
        current-surface elevation map, shape ``(num_envs, N, N)``.

        Returns ``None`` when no particle source is available. The task computes
        this once per step for diagnostics; the same refresh also populates the
        cached particle positions used by the oracle reward surfaces.
        """
        if particle_positions is None:
            return None

        self._refresh_particle_state(
            update_state_from_sim=update_state_from_sim,
            force_update_env_ids=force_update_env_ids,
        )
        return compute_elevation_map(
            particle_positions(),
            env_origins,
            resolution=self.cfg.elev_map_size,
            extent=self.cfg.elev_map_extent,
            percentile=self.cfg.elevation_percentile,
        )

    def error_from_current(self, current_map: torch.Tensor | None) -> torch.Tensor:
        """Signed error ``target - current`` for a precomputed current map.

        Returns a zero map when ``current_map`` is ``None`` (no particle source).
        """
        if current_map is None:
            return torch.zeros_like(self._target_map)
        return self._target_map - current_map

    def current_local(
        self,
        *,
        current_map: torch.Tensor,
        robot_pos_w: torch.Tensor,
        env_origins: torch.Tensor,
        robot_yaw: torch.Tensor,
    ) -> torch.Tensor:
        """Sample a precomputed global current-surface map on the landscaping
        heightmap geometry, shape ``(num_envs, ny, nx)``.

        Mirrors :meth:`target_local` for map-space diagnostics and callers that
        need aligned current/target patches.
        """
        return sample_global_map_at_heightmap(
            global_map=current_map,
            robot_pos_w=robot_pos_w,
            env_origins=env_origins,
            robot_yaw=robot_yaw,
            geometry=self.cfg.geometry,
            extent=self.cfg.elev_map_extent,
            resolution=self.cfg.elev_map_size,
        )

    def global_error(
        self,
        *,
        particle_positions: ParticlePositions | None,
        env_origins: torch.Tensor,
        update_state_from_sim: ParticleStateUpdate | None = None,
        force_update_env_ids: Sequence[int] | torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Return per-cell signed error ``target - current`` on the global map."""
        current = self.compute_current_map(
            particle_positions=particle_positions,
            env_origins=env_origins,
            update_state_from_sim=update_state_from_sim,
            force_update_env_ids=force_update_env_ids,
        )
        return self.error_from_current(current)

    @property
    def _update_interval_steps(self) -> int:
        return max(1, int(self.cfg.particle_update_interval_steps))

    def _coerce_env_ids(self, env_ids: Sequence[int] | torch.Tensor) -> torch.Tensor:
        return torch.as_tensor(env_ids, device=self._device, dtype=torch.long)

    def _refresh_particle_state(
        self,
        *,
        update_state_from_sim: ParticleStateUpdate | None,
        force_update_env_ids: Sequence[int] | torch.Tensor | None,
    ) -> None:
        self._particle_update_counter -= 1
        refreshed_all = False
        if self._particle_update_counter <= 0:
            if update_state_from_sim is not None:
                update_state_from_sim()
                refreshed_all = True
            self._particle_update_counter = self._update_interval_steps

        force_ids = self._coerce_optional_env_ids(force_update_env_ids)
        if force_ids is not None and force_ids.numel() > 0 and not refreshed_all:
            if update_state_from_sim is not None:
                update_state_from_sim(env_ids=force_ids)

    def _coerce_optional_env_ids(
        self,
        env_ids: Sequence[int] | torch.Tensor | None,
    ) -> torch.Tensor | None:
        if env_ids is None:
            return None
        return torch.as_tensor(env_ids, device=self._device, dtype=torch.long)


__all__ = [
    "LandscapingEpisodeTerrain",
    "LandscapingEpisodeTerrainCfg",
    "ParticlePositions",
    "ParticleStateUpdate",
]
