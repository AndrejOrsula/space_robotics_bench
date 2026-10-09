"""Deterministic mound leveling against a frozen target manifest.

The pile sits on an intact flat bed, with the rover on its centreline. Reset
configuration is checked against the manifest hash. Reward uses bounded excess
volume; grading MAE is diagnostic. Shovel extension is hidden, so it resets to
the asset default.
"""

from __future__ import annotations

import torch

from srb.assets.scenery.terrain import Plane
from srb.core.sim import MoundPileParticlesSpawnerCfg
from srb.terrain.landscaping_contract import (
    RESET_LAYOUT_VERSION_MOUND_PILE_V1,
    ResetLayoutParams,
    TargetMapManifest,
    analytic_mound_initial_excess_volume_m3,
    compute_reset_layout_sha256,
    load_mound_target_manifest,
)
from srb.utils.cfg import configclass
from srb.utils.failures import InvalidInputError

from .scene_design import SITE_SUNLIGHT_RPY, landscaping_containment
from .task import TaskCfg as BaseTaskCfg
from .task_crater import (
    _DISABLED_BASELINE_RANDOMIZERS,
    CraterReferenceController,
)
from .task_crater import (
    Task as CraterTask,
)

_MOUND_TASK_ID = "terrain_landscaping_mound"


class MoundReferenceController(CraterReferenceController):
    """Test controller that shaves the apex with straight push/reverse passes.

    It raises the blade before reversing; five passes meet the volume gate.
    """

    #: Phase entries are ``(kind, params, exit_x, step_limit)``. Push phases
    #: lower the blade within the x band; reverse phases raise it.
    PHASES: tuple[tuple[str, tuple[float, ...], float, int], ...] = (
        ("push", (-0.42, -0.08), 0.28, 170),
        ("rev", (), -0.66, 80),
        ("push", (-0.40, -0.05), 0.28, 170),
        ("rev", (), -0.66, 80),
        ("push", (-0.38, -0.02), 0.28, 170),
        ("rev", (), -0.66, 80),
        ("push", (-0.36, 0.01), 0.28, 170),
        ("rev", (), -0.66, 80),
        ("push", (-0.34, 0.04), 0.30, 170),
    )
    SHOVEL_DOWN: float = -0.7
    SHOVEL_UP: float = 0.6

    def action(self, mission_pose: torch.Tensor) -> torch.Tensor:
        """Return ``(v_lin, v_ang, shovel)`` for pose ``(x, y, yaw)``.

        This push/reverse plan uses two-element phases; the crater base uses
        three-element phases for its pose-targeted scoop-and-carry cycle.
        """
        pose = torch.as_tensor(mission_pose, dtype=torch.float32).reshape(-1)
        x = float(pose[0])
        while self._phase < len(self.PHASES):
            kind, _params, exit_x, limit = self.PHASES[self._phase]
            reached = x > exit_x if kind == "push" else x < exit_x
            if reached or self._timer >= limit:
                self._phase += 1
                self._timer = 0
                continue
            break
        if self._phase >= len(self.PHASES):
            return torch.tensor([0.0, 0.0, 0.3], dtype=torch.float32)
        kind, params, _exit_x, _limit = self.PHASES[self._phase]
        self._timer += 1
        if kind == "rev":
            return torch.tensor([-self.DRIVE, 0.0, self.SHOVEL_UP], dtype=torch.float32)
        x_min, x_max = params
        shovel = self.SHOVEL_DOWN if x_min < x < x_max else self.SHOVEL_UP
        return torch.tensor([self.DRIVE, 0.0, shovel], dtype=torch.float32)


@configclass
class TaskCfg(BaseTaskCfg):
    """One regolith mound on an intact flat bed with a frozen flat-bed target.

    The analytic pile spawns without excavation; robot pose, sensors, and
    physics settings are deterministic. The shovel resets to its asset default.
    """

    ## 300 steps at 10 Hz; inherited truncation semantics are unchanged.
    env_rate: float = 1.0 / 100.0
    episode_length_s: float = 30.0

    ## Dark bed under lighter regolith.
    scenery: Plane = landscaping_containment()
    bed_visual_prim_name: str = "mound_bed_visual"

    ## Mound-only spawner.
    particles_systems: int = 1
    particles_spawner: str = "mound_pile"
    particles_size: float = 0.05
    particles_dim_xy: int = 48
    particles_height: int = 3
    particles_ratio: float = 0.75

    ## Fixed mound ahead of the rover. Values MUST reproduce the manifest's
    ## ``reset_layout_sha256``; configuration fails on drift.
    pile_center_xy_m: tuple[float, float] = (-0.25, -0.20)
    pile_radius_m: float = 0.32
    pile_height_m: float = 0.24

    robot_spawn_xy_m: tuple[float, float] = (-0.90, -0.20)
    robot_spawn_yaw_rad: float = 0.0
    robot_spawn_z_m: float = 0.40

    ## Oracle diagnostic-map resolution.
    elev_map_size: int = 32

    ## Excess-volume gate reward. ``__post_init__`` replaces this placeholder
    ## with the reference derived from the manifest.
    reward_metric: str = "excess_volume"
    excess_reference_m3: float = 1.0

    def _verify_general_reset_layout(self) -> None:
        """The mound task owns its own reset layout (checked below)."""

    def __post_init__(self):
        if not self.particles:
            raise ValueError("terrain_landscaping_mound requires particles")
        if not isinstance(self.particles_size, (int, float)):
            raise InvalidInputError(
                "terrain_landscaping_mound requires one scalar particles_size"
            )
        self.particles_init_pos = (0.0, 0.0, 0.5 * float(self.particles_size))
        # Raking light reveals bed height. Set before `super()` builds the
        # sunlight asset; randomization is disabled and light is not observed
        # or rewarded.
        self.sunlight_rpy = SITE_SUNLIGHT_RPY

        super().__post_init__()

        # The general task composes the site and framing using this bed name.

        scene_spawner = self.scene.particles0.spawn  # type: ignore[attr-defined]
        managed_spawner = self._particle_system_cfgs["particles0"].spawner
        for spawner in (scene_spawner, managed_spawner):
            if not isinstance(spawner, MoundPileParticlesSpawnerCfg):
                raise TypeError(
                    "terrain_landscaping_mound requires MoundPileParticlesSpawnerCfg"
                )
            spawner.pile_center_xy_m = self.pile_center_xy_m
            spawner.pile_radius_m = self.pile_radius_m
            spawner.pile_height_m = self.pile_height_m
            spawner.velocity = (0.0, 0.0, 0.0)

        # Fixed, documented robot reset (shovel extension is hidden state).
        pose_range = {
            "x": (self.robot_spawn_xy_m[0], self.robot_spawn_xy_m[0]),
            "y": (self.robot_spawn_xy_m[1], self.robot_spawn_xy_m[1]),
            "z": (self.robot_spawn_z_m, self.robot_spawn_z_m),
            "yaw": (self.robot_spawn_yaw_rad, self.robot_spawn_yaw_rad),
        }
        self.events.randomize_robot_state.params["pose_range"] = pose_range

        for event_name in _DISABLED_BASELINE_RANDOMIZERS:
            setattr(self.events, event_name, None)

        self._verify_manifest_reset_layout()

    def _verify_manifest_reset_layout(self) -> None:
        """Verify the frozen mound layout and success settings; set reward scale."""
        manifest = load_mound_target_manifest()
        if manifest.task_id != _MOUND_TASK_ID:
            raise ValueError(
                f"target manifest {manifest.manifest_id!r} identifies task "
                f"{manifest.task_id!r}, not {_MOUND_TASK_ID!r}"
            )

        spawner = self._particle_system_cfgs["particles0"].spawner
        live_layout = ResetLayoutParams(
            version=RESET_LAYOUT_VERSION_MOUND_PILE_V1,
            spawner=self.particles_spawner,
            dim_x=int(spawner.dim_x),
            dim_y=int(spawner.dim_y),
            dim_z=int(spawner.dim_z),
            particle_size_m=float(spawner.particle_size),
            particle_spacing_m=float(spawner.particle_spacing),
            ratio=float(spawner.ratio),
            crater_center_xy_m=(0.0, 0.0),
            crater_radius_m=0.0,
            crater_depth_m=0.0,
            pile_center_xy_m=tuple(float(v) for v in spawner.pile_center_xy_m),
            pile_radius_m=float(spawner.pile_radius_m),
            pile_height_m=float(spawner.pile_height_m),
            translation_m=tuple(float(v) for v in self.particles_init_pos),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            velocity_m_s=tuple(float(v) for v in spawner.velocity),
        )
        live_hash = compute_reset_layout_sha256(live_layout)
        if live_hash != manifest.reset_layout_sha256:
            raise ValueError(
                "terrain_landscaping_mound configuration does not reproduce "
                "the target manifest's reset layout: live reset-layout hash "
                f"{live_hash} != manifest reset_layout_sha256 "
                f"{manifest.reset_layout_sha256}. The fixed mission is owned "
                "by the manifest; change the manifest or "
                f"restore the config. Live layout: {live_layout.canonical_dict()}"
            )
        saved = manifest.success
        live_success = (
            float(self.success_tolerance_m),
            float(self.success_fraction_threshold),
            int(self.success_window_steps),
        )
        manifest_success = (
            float(saved.success_tolerance_m),
            float(saved.fraction_within_tolerance),
            int(saved.sustained_frames),
        )
        if live_success != manifest_success:
            raise ValueError(
                "terrain_landscaping_mound success parameters "
                f"{live_success} differ from the manifest's saved values "
                f"{manifest_success}"
            )
        self.excess_reference_m3 = analytic_mound_initial_excess_volume_m3(manifest)


class Task(CraterTask):
    """Level one regolith mound on a flat bed."""

    cfg: TaskCfg

    MANIFEST_TASK_ID: str = _MOUND_TASK_ID

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg=cfg, runtime_mode=runtime_mode, **kwargs)

    def _build_reference_target_manifest(self) -> TargetMapManifest:
        """Load the checked-in static mound manifest."""
        return load_mound_target_manifest()


__all__ = ["MoundReferenceController", "Task", "TaskCfg"]
