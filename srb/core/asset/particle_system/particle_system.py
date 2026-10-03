"""Manage PhysX particles through USD in SRB environments.

Particle state uses ``UsdGeom.Points``; system properties use associated
``PhysxParticleSystem`` prims. PhysX solves PBD particles on the GPU, while
USD readback requires ``/physics/updateParticlesToUsd`` to remain enabled.
Isaac Sim 6.1 has no particle tensor view, so USD also exposes system properties.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence

import isaaclab.sim as sim_utils
import numpy as np
import torch
from isaaclab.sim.utils.stage import get_current_stage
from pxr import Gf, Usd, UsdGeom, Vt

from srb.core.particle_binding import (
    check_settle_drop,
    settle_batch_converged,
    validate_particle_system_binding,
)
from srb.core.particle_state_registry import consume_authored_particle_states

from .particle_system_cfg import ParticleSystemCfg
from .particle_system_data import ParticleSystemData

logger = logging.getLogger(__name__)


def _transform_xyz(
    values: np.ndarray,
    transform: Gf.Matrix4d,
    *,
    translate: bool,
) -> np.ndarray:
    """Apply a USD row-vector transform to points or direction vectors."""
    matrix = np.array(
        [[transform[i][j] for j in range(4)] for i in range(4)],
        dtype=np.float32,
    )
    transformed = values @ matrix[:3, :3]
    if translate:
        transformed += matrix[3, :3]
    return transformed.astype(np.float32, copy=False)


class ParticleSystem:
    """Manage a PhysX particle system and its particle sets."""

    def __init__(
        self,
        cfg: ParticleSystemCfg,
        num_envs: int,
        device: str,
    ):
        """Initialize the wrapper."""
        self.cfg = cfg
        self._num_envs = num_envs
        self._device = device

        self._particle_set_prim_paths: list[str] = []
        self._particle_system_prim_paths: list[str] = []
        self._prims: list = []  # USD particle set prims per env

        self._is_initialized: bool = False
        self._is_frozen: bool = False
        self._num_particles: int = 0
        self._data: ParticleSystemData | None = None
        self._authored_initial_pos_w: torch.Tensor | None = None
        self._authored_initial_vel_w: torch.Tensor | None = None

    @property
    def num_envs(self) -> int:
        """Number of parallel environments."""
        return self._num_envs

    @property
    def num_particles(self) -> int:
        """Number of particles per environment."""
        return self._num_particles

    @property
    def device(self) -> str:
        """Torch device string."""
        return self._device

    @property
    def data(self) -> ParticleSystemData:
        """Cached particle state data."""
        if self._data is None:
            raise RuntimeError(
                "ParticleSystem.data accessed before initialize(). "
                "Call initialize() after physics is ready."
            )
        return self._data

    @property
    def is_initialized(self) -> bool:
        """Whether initialize() has been called."""
        return self._is_initialized

    @property
    def is_frozen(self) -> bool:
        """Whether the particle system has been frozen via :meth:`disable`."""
        return self._is_frozen

    @property
    def particle_set_prims(self) -> Sequence[object]:
        """USD particle-set prims in environment order."""
        return tuple(self._prims)

    @property
    def particle_set_prim_paths(self) -> Sequence[str]:
        """USD paths for particle-set prims in environment order."""
        return tuple(self._particle_set_prim_paths)

    def initialize(self, env_regex_ns: str = "/World/envs/env_.*") -> None:
        """Discover spawned prims after scene and physics setup.

        Allocate state buffers. ``env_regex_ns`` resolves ``{ENV_REGEX_NS}`` in
        configured prim paths.
        """
        if self._is_initialized:
            logger.warning("ParticleSystem already initialized, skipping.")
            return

        stage = get_current_stage()

        resolved_path = self.cfg.prim_path.replace("{ENV_REGEX_NS}", env_regex_ns)
        matching_prims = sim_utils.find_matching_prims(resolved_path)

        if not matching_prims:
            raise RuntimeError(
                f"No particle set prims found at '{resolved_path}'. "
                f"Ensure the spawner has been called before initialize()."
            )

        self._prims = matching_prims
        self._particle_set_prim_paths = [str(p.GetPath()) for p in matching_prims]

        if len(self._prims) != self._num_envs:
            # Per-env state writes index prims by env id; a mismatch fails later.
            raise RuntimeError(
                f"Found {len(self._prims)} particle set prims at '{resolved_path}' "
                f"but expected {self._num_envs} environments."
            )

        # Cross-environment bindings can put a set in the wrong collision group.
        validate_particle_system_binding(stage, self._particle_set_prim_paths)

        first_points = UsdGeom.Points(self._prims[0])
        positions_attr = first_points.GetPointsAttr().Get()
        if positions_attr is not None:
            self._num_particles = len(positions_attr)
        else:
            raise RuntimeError(
                f"Could not read particle positions from "
                f"'{self._particle_set_prim_paths[0]}'"
            )

        logger.info(
            f"ParticleSystem initialized: {self._num_envs} envs, "
            f"{self._num_particles} particles each"
        )

        self._data = ParticleSystemData(
            num_envs=self._num_envs,
            max_particles=self._num_particles,
            device=self._device,
        )
        self._capture_authored_initial_state()

        if not self._resolve_system_prim_paths():
            logger.warning(
                "No particle system prims found via relationships. "
                "System-level property access will not be available."
            )

        self._is_initialized = True
        self.update_state_from_sim()

    def _capture_authored_initial_state(self) -> None:
        """Consume exact parent-local arrays and project them into world."""
        states = consume_authored_particle_states(
            self._particle_set_prim_paths,
            expected_num_particles=self._num_particles,
        )
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        positions_w: list[torch.Tensor] = []
        velocities_w: list[torch.Tensor] = []
        for prim, state in zip(self._prims, states, strict=True):
            local_to_world = xform_cache.GetLocalToWorldTransform(prim)
            positions_w.append(
                torch.from_numpy(
                    _transform_xyz(state.positions, local_to_world, translate=True)
                ).to(device=self._device)
            )
            velocities_w.append(
                torch.from_numpy(
                    _transform_xyz(state.velocities, local_to_world, translate=False)
                ).to(device=self._device)
            )
        self._authored_initial_pos_w = torch.stack(positions_w, dim=0)
        self._authored_initial_vel_w = torch.stack(velocities_w, dim=0)

    def restore_authored_initial_state(self) -> None:
        """Restore authored state before capturing the settled default."""
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")
        if self._authored_initial_pos_w is None or self._authored_initial_vel_w is None:
            raise RuntimeError("Authored particle state is no longer available.")
        self.write_state_to_sim(
            pos_w=self._authored_initial_pos_w,
            vel_w=self._authored_initial_vel_w,
        )

    def discard_authored_initial_state(self) -> None:
        """Release rewind buffers after default-state storage."""
        self._authored_initial_pos_w = None
        self._authored_initial_vel_w = None

    def _resolve_system_prim_paths(self) -> list[str]:
        """Resolve and cache distinct system prims referenced by particle sets."""
        if self._particle_system_prim_paths:
            return self._particle_system_prim_paths

        system_paths: set[str] = set()
        for prim in self._prims:
            rel = prim.GetRelationship("physxParticle:particleSystem")
            if rel and rel.GetTargets():
                for target in rel.GetTargets():
                    system_paths.add(str(target))

        self._particle_system_prim_paths = sorted(system_paths)
        return self._particle_system_prim_paths

    def disable(self) -> None:
        """Zero velocities and disable system prims while keeping geometry visible.

        Idempotent; intended for clean renders after particles settle.
        """
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")

        zero_vel = torch.zeros(
            (self._num_envs, self._num_particles, 3),
            device=self._device,
            dtype=torch.float32,
        )
        self.write_state_to_sim(vel_w=zero_vel)

        # Report frozen only when a solver prim was actually disabled.
        if not self._set_system_enabled(False):
            return
        self._is_frozen = True
        logger.info(
            f"Froze {len(self._particle_system_prim_paths)} particle system "
            f"prim(s) ({self._num_particles} particles x {self._num_envs} envs)."
        )

    def enable(self) -> None:
        """Re-enable a previously :meth:`disable`-d particle system."""
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")
        self._set_system_enabled(True)
        self._is_frozen = False
        logger.info("Re-enabled particle system(s).")

    def _set_system_enabled(self, enabled: bool) -> bool:
        """Set ``particleSystemEnabled`` on associated system prims.

        Returns whether at least one valid system prim was updated.
        """
        from pxr import PhysxSchema

        system_paths = self._resolve_system_prim_paths()
        if not system_paths:
            logger.warning(
                "No PhysxParticleSystem prims found; cannot toggle "
                "particleSystemEnabled."
            )
            return False

        stage = get_current_stage()
        updated = False
        for path in system_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                logger.warning(f"Particle system prim '{path}' is not valid.")
                continue
            # PhysxParticleSystem is a concrete schema, not an applied API.
            ps_schema = PhysxSchema.PhysxParticleSystem(prim)
            attr = ps_schema.GetParticleSystemEnabledAttr()
            if not attr or not attr.IsValid():
                attr = ps_schema.CreateParticleSystemEnabledAttr()
            attr.Set(bool(enabled))
            updated = True
        return updated

    def update_state_from_sim(
        self,
        env_ids: Sequence[int] | torch.Tensor | None = None,
    ) -> None:
        """Read selected particle states from USD into cached buffers.

        ``env_ids=None`` updates all environments.
        """
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")

        if env_ids is None:
            ids = range(self._num_envs)
        elif isinstance(env_ids, torch.Tensor):
            ids = env_ids.detach().cpu().tolist()
        else:
            ids = env_ids
        if len(ids) == 0:
            # Nothing selected; torch.stack would reject the empty lists.
            return

        pos_list = []
        vel_list = []
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for i in ids:
            points = UsdGeom.Points(self._prims[i])
            local_to_world = xform_cache.GetLocalToWorldTransform(self._prims[i])

            raw_pos = points.GetPointsAttr().Get()
            if raw_pos is not None:
                local_pos = np.array(raw_pos, dtype=np.float32, copy=True)
                pos_list.append(
                    torch.from_numpy(
                        _transform_xyz(local_pos, local_to_world, translate=True)
                    ).to(device=self._device)
                )
            else:
                pos_list.append(self._data.particle_pos_w[i])

            raw_vel = points.GetVelocitiesAttr().Get()
            if raw_vel is not None:
                local_vel = np.array(raw_vel, dtype=np.float32, copy=True)
                vel_list.append(
                    torch.from_numpy(
                        _transform_xyz(local_vel, local_to_world, translate=False)
                    ).to(device=self._device)
                )
            else:
                vel_list.append(self._data.particle_vel_w[i])

        pos_tensor = torch.stack(pos_list, dim=0)
        vel_tensor = torch.stack(vel_list, dim=0)

        if env_ids is None:
            self._data.update(pos_tensor, vel_tensor)
        else:
            if isinstance(env_ids, torch.Tensor):
                self._data.update(pos_tensor, vel_tensor, env_ids)
            else:
                self._data.update(
                    pos_tensor,
                    vel_tensor,
                    torch.tensor(list(ids), device=self._device, dtype=torch.long),
                )

    def write_state_to_sim(
        self,
        pos_w: torch.Tensor | None = None,
        vel_w: torch.Tensor | None = None,
        env_ids: Sequence[int] | torch.Tensor | None = None,
    ) -> None:
        """Write world-frame positions or velocities to USD.

        Each supplied tensor has shape ``(selected_envs, particles, 3)``.
        ``env_ids=None`` selects all environments; omitted tensors are unchanged.
        """
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")

        if env_ids is None:
            ids = range(self._num_envs)
        elif isinstance(env_ids, torch.Tensor):
            ids = env_ids.detach().cpu().tolist()
        else:
            ids = list(env_ids)

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for idx, env_id in enumerate(ids):
            points = UsdGeom.Points(self._prims[env_id])
            world_to_local = xform_cache.GetLocalToWorldTransform(
                self._prims[env_id]
            ).GetInverse()

            if pos_w is not None:
                world_pos = pos_w[idx].detach().cpu().numpy()
                local_pos = _transform_xyz(world_pos, world_to_local, translate=True)
                points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(local_pos))

            if vel_w is not None:
                world_vel = vel_w[idx].detach().cpu().numpy()
                local_vel = _transform_xyz(world_vel, world_to_local, translate=False)
                points.GetVelocitiesAttr().Set(Vt.Vec3fArray.FromNumpy(local_vel))

        # Update cached data
        if pos_w is not None or vel_w is not None:
            if env_ids is None:
                if pos_w is not None:
                    self._data.particle_pos_w[:] = pos_w
                if vel_w is not None:
                    self._data.particle_vel_w[:] = vel_w
            else:
                ids_tensor = (
                    env_ids
                    if isinstance(env_ids, torch.Tensor)
                    else torch.tensor(ids, device=self._device, dtype=torch.long)
                )
                if pos_w is not None:
                    self._data.particle_pos_w[ids_tensor] = pos_w
                if vel_w is not None:
                    self._data.particle_vel_w[ids_tensor] = vel_w

    def reset(self, env_ids: Sequence[int] | torch.Tensor | None = None) -> None:
        """Restore the cached default particle state.

        ``env_ids=None`` resets all environments. A default must be stored first.
        """
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")
        if not self._data.has_default_state:
            raise RuntimeError(
                "No default state stored. Call store_default_state() after settling."
            )

        if env_ids is None:
            self.write_state_to_sim(
                pos_w=self._data.default_particle_pos_w,
                vel_w=self._data.default_particle_vel_w,
            )
        else:
            if isinstance(env_ids, torch.Tensor):
                ids_tensor = env_ids
            else:
                ids_tensor = torch.tensor(
                    list(env_ids), device=self._device, dtype=torch.long
                )
            self.write_state_to_sim(
                pos_w=self._data.default_particle_pos_w[ids_tensor],
                vel_w=self._data.default_particle_vel_w[ids_tensor],
                env_ids=env_ids,
            )

    def store_default_state(self) -> None:
        """Snapshot current state for resets after particles settle."""
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")
        self.update_state_from_sim()
        self._data.store_default_state()
        logger.info(
            f"Default particle state stored "
            f"({self._num_particles} particles x {self._num_envs} envs)"
        )

    def _particle_system_schemas(self) -> list:
        """Return the ``PhysxParticleSystem`` schema of each system prim."""
        from pxr import PhysxSchema

        stage = get_current_stage()
        schemas = []
        for path in self._resolve_system_prim_paths():
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                raise RuntimeError(f"Particle system prim '{path}' is not valid.")
            schemas.append(PhysxSchema.PhysxParticleSystem(prim))
        return schemas

    def set_particle_contact_offsets(self, values: torch.Tensor) -> None:
        """Set ``particleContactOffset`` per system, in sorted system-prim-path order.

        ``values`` has one entry per system prim. PhysX picks the change up
        through its USD change listener; the offset must stay larger than
        the system's solid and fluid rest offsets.
        """
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")
        schemas = self._particle_system_schemas()
        flat = torch.as_tensor(values).detach().reshape(-1).cpu()
        if flat.numel() != len(schemas):
            raise ValueError(
                f"Expected {len(schemas)} contact offset(s), got {flat.numel()}."
            )
        for schema, value in zip(schemas, flat.tolist(), strict=True):
            schema.CreateParticleContactOffsetAttr().Set(float(value))

    def get_particle_contact_offsets(self) -> torch.Tensor | None:
        """Return ``particleContactOffset`` per system, or ``None`` without systems.

        An unauthored attribute reads as its schema fallback.
        """
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")
        schemas = self._particle_system_schemas()
        if not schemas:
            return None
        return torch.tensor(
            [float(schema.GetParticleContactOffsetAttr().Get()) for schema in schemas],
            dtype=torch.float32,
            device=self._device,
        )

    def settle(
        self,
        sim_step_fn,
        step_dt: float,
        evacuate_fn=None,
        restore_fn=None,
    ) -> None:
        """Settle particles under gravity and store their default state.

        Args:
            sim_step_fn: Simulation step callback, called with ``render=False``.
            step_dt: Simulation timestep in seconds.
            evacuate_fn: Optional callback before settling.
            restore_fn: Optional callback after storing defaults.
        """
        if evacuate_fn is not None:
            evacuate_fn()

        logger.info("Letting particles settle, please be patient...")

        prev_vel_norm = torch.full((), float("inf"), device=self._device)
        self.update_state_from_sim()
        start_median_z = self._data.particle_pos_w[..., 2].median(dim=1).values.tolist()

        for i in range(self.cfg.settle_max_steps):
            steps = max(1, round(self.cfg.settle_step_time / step_dt))
            for _ in range(steps):
                sim_step_fn(render=False)

            self.update_state_from_sim()
            vel_norms = torch.linalg.norm(self._data.particle_vel_w, dim=-1)
            med_vel = torch.median(vel_norms)
            vel_change = torch.abs(med_vel - prev_vel_norm)
            prev_vel_norm = med_vel
            # Never accept free fall through terrain as the reset default.
            check_settle_drop(
                start_median_z,
                self._data.particle_pos_w[..., 2].median(dim=1).values.tolist(),
                self.cfg.settle_max_median_drop,
            )

            if settle_batch_converged(
                float(med_vel),
                float(vel_change),
                speed_threshold=self.cfg.settle_vel_threshold,
                min_speed_change_threshold=self.cfg.settle_min_vel_change_threshold,
                plateau_max_speed=self.cfg.settle_plateau_max_vel,
            ):
                logger.info(
                    f"Particles settled after {i + 1} batches "
                    f"(median vel: {med_vel:.6f})"
                )
                break
            else:
                logger.info(
                    f"Settling batch {i + 1}/{self.cfg.settle_max_steps}: "
                    f"median vel = {med_vel:.4f}, change = {vel_change:.6f}"
                )
        else:
            logger.warning(
                f"Particles did not fully settle after "
                f"{self.cfg.settle_max_steps} batches "
                f"(median vel: {prev_vel_norm:.4f})"
            )

        # Do not replay residual settling motion on every reset.
        zero_vel = torch.zeros_like(self._data.particle_vel_w)
        self.write_state_to_sim(vel_w=zero_vel)

        self.store_default_state()

        # Actor contacts during restore must not enter the cached reset state.
        if restore_fn is not None:
            restore_fn()

    def conform_to_terrain(
        self,
        surface_offset: float = 0.0,
        xy_jitter: float = 0.0,
        jitter_seed: int = 0,
        evacuate_fn=None,
        restore_fn=None,
    ) -> None:
        """Snap particles to terrain mesh heights without running physics.

        Samples terrain mesh vertices into a height grid and assigns each
        particle's Z from its spawn ``(x, y)``. ``evacuate_fn`` runs for parity
        with physics settling; terrain sampling does not depend on robot poses.

        Args:
            surface_offset: Added to sampled terrain height.
            xy_jitter: Maximum uniform XY offset before lookup.
            jitter_seed: Seed for reproducible jitter.
            evacuate_fn: Optional callback before sampling.
            restore_fn: Optional callback after sampling.
        """
        if not self._is_initialized:
            raise RuntimeError("ParticleSystem not initialized.")

        import numpy as np
        import omni.usd
        from pxr import Usd, UsdGeom

        if evacuate_fn is not None:
            evacuate_fn()

        logger.info("Conforming particles to terrain surface (mesh height grid)...")

        # Reset-time scene queries omit the collider; sample mesh geometry.
        stage = omni.usd.get_context().get_stage()
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        keywords = ("surface", "terrain", "ground", "moon", "mars")
        world_pts = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            ident = (prim.GetName() + " " + str(prim.GetPath())).lower()
            if not any(k in ident for k in keywords):
                continue
            raw = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not raw:
                continue
            local = np.asarray(raw, dtype=np.float64)
            m = xform_cache.GetLocalToWorldTransform(prim)
            mat = np.array([[m[i][j] for j in range(4)] for i in range(4)])
            world_pts.append(local @ mat[:3, :3] + mat[3, :3])

        self.update_state_from_sim()
        pos = self._data.particle_pos_w.clone()
        zero_vel = torch.zeros_like(pos)

        if not world_pts:
            logger.warning(
                "No terrain mesh found to conform particles to; leaving them at "
                "spawn height."
            )
            self.write_state_to_sim(vel_w=zero_vel)
            self.store_default_state()
            if restore_fn is not None:
                restore_fn()
            return

        verts = np.concatenate(world_pts, axis=0)

        from srb.utils.spatial import build_height_grid

        hg = build_height_grid(verts, res=0.2)
        nx, ny = hg.shape

        rng = np.random.default_rng(jitter_seed)
        for env_i in range(self._num_envs):
            p = pos[env_i].cpu().numpy().astype(np.float64)
            if xy_jitter > 0.0:
                p[:, :2] += rng.uniform(-xy_jitter, xy_jitter, size=(p.shape[0], 2))
            p[:, 2] = hg.sample(p[:, :2], surface_offset=surface_offset)
            pos[env_i] = torch.tensor(p, device=self._device, dtype=torch.float32)

        self.write_state_to_sim(pos_w=pos, vel_w=zero_vel)
        self.store_default_state()

        if restore_fn is not None:
            restore_fn()

        logger.info(
            f"Conformed {self._num_envs * self._num_particles} particle(s) to "
            f"terrain (z {float(verts[:, 2].min()):.2f}..{float(verts[:, 2].max()):.2f}, "
            f"grid {nx}x{ny})."
        )

    def __repr__(self) -> str:
        return (
            f"ParticleSystem("
            f"num_envs={self._num_envs}, "
            f"num_particles={self._num_particles}, "
            f"initialized={self._is_initialized}, "
            f"systems={len(self._particle_system_prim_paths)}"
            f")"
        )
