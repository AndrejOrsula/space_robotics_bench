"""Deterministic crater-filling task with a frozen target manifest."""

from __future__ import annotations

import math

import torch

from srb.assets.scenery.terrain import Plane
from srb.core.sim import CraterPileParticlesSpawnerCfg
from srb.terrain.landscaping_contract import (
    RESET_LAYOUT_VERSION_CRATER_PILE_V1,
    ResetLayoutParams,
    TargetMapManifest,
    compute_reset_layout_sha256,
    load_crater_target_manifest,
)
from srb.terrain.landscaping_success import StateQualityParams
from srb.utils.cfg import configclass
from srb.utils.failures import InvalidInputError

from .scene_design import SITE_SUNLIGHT_RPY, landscaping_containment
from .task import Task as BaseTask
from .task import TaskCfg as BaseTaskCfg

_CRATER_TASK_ID = "terrain_landscaping_crater"

_DISABLED_BASELINE_RANDOMIZERS = (
    "randomize_blade_mass",
    "randomize_wheel_joint_friction",
    "randomize_disturbance_force",
    "randomize_regolith_material",
    "randomize_gravity",
    "randomize_sunlight_orientation",
    "randomize_sunlight_intensity",
    "randomize_sunlight_angular_diameter",
    "randomize_sunlight_color_temperature",
    "randomize_skydome_orientation",
)


class CraterReferenceController:
    """TESTING ONLY controller for one fixed crater-and-pile carrying cycle.

    Uses only the actor-legal ``mission_pose`` for motion feedback; shovel
    phases use timers because joint positions are absent from the observation.
    Phase limits sum to 298 actions; waypoint arrivals can advance phases early.
    """

    PHASES: tuple[tuple[str, tuple[float, float] | None, int], ...] = (
        ("lower", None, 50),
        ("scoop", (-0.25, -0.18), 65),
        ("raise", None, 50),
        ("carry", (0.05, 0.12), 60),
        ("dump", None, 50),
        ("retreat", None, 23),
    )
    DRIVE: float = 0.5
    WAYPOINT_TOLERANCE_M: float = 0.08

    def __init__(self) -> None:
        self._phase = 0
        self._timer = 0

    def reset(self) -> None:
        self._phase = 0
        self._timer = 0

    @property
    def done(self) -> bool:
        return self._phase >= len(self.PHASES)

    def _drive_to(
        self, pose: torch.Tensor, target_xy: tuple[float, float]
    ) -> torch.Tensor:
        x, y, yaw = (float(v) for v in pose[:3])
        dx, dy = target_xy[0] - x, target_xy[1] - y
        desired = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
        turn = max(-1.0, min(1.0, 1.6 * yaw_error))
        alignment = max(0.15, 1.0 - abs(yaw_error) / 1.2)
        return torch.tensor([self.DRIVE * alignment, turn, 0.0], dtype=torch.float32)

    def action(self, mission_pose: torch.Tensor) -> torch.Tensor:
        """Return the next ``(v_lin, v_ang, shovel)`` command."""
        pose = torch.as_tensor(mission_pose, dtype=torch.float32).reshape(-1)
        while self._phase < len(self.PHASES):
            kind, target, limit = self.PHASES[self._phase]
            reached = False
            if target is not None:
                dx = float(pose[0]) - target[0]
                dy = float(pose[1]) - target[1]
                reached = math.hypot(dx, dy) <= self.WAYPOINT_TOLERANCE_M
            if reached or self._timer >= limit:
                self._phase += 1
                self._timer = 0
                continue
            break
        if self._phase >= len(self.PHASES):
            return torch.zeros(3, dtype=torch.float32)
        kind, target, _limit = self.PHASES[self._phase]
        self._timer += 1
        if kind == "lower":
            return torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32)
        if kind == "dump":
            return torch.tensor([-1.0, 0.0, -1.0], dtype=torch.float32)
        if kind == "raise":
            return torch.tensor([0.0, 0.0, 1.0], dtype=torch.float32)
        if kind == "retreat":
            return torch.tensor([-1.0, 0.0, 0.0], dtype=torch.float32)
        assert target is not None
        return self._drive_to(pose, target)


@configclass
class TaskCfg(BaseTaskCfg):
    """One crater-and-pile mission with a frozen flat-bed target."""

    ## 300 control steps at 10 Hz; physics runs at 100 Hz (decimation 10).
    ## This cutoff truncates the continuing MDP.
    env_rate: float = 1.0 / 100.0
    episode_length_s: float = 30.0

    ## Dark bed contrasts with the lighter regolith.
    scenery: Plane = landscaping_containment()
    bed_visual_prim_name: str = "crater_bed_visual"

    ## Single bed, dense enough for the 5 cm actor heightmap.
    particles_systems: int = 1
    particles_spawner: str = "crater_pile"
    particles_size: float = 0.05
    particles_dim_xy: int = 48
    particles_height: int = 3
    particles_ratio: float = 0.75

    ## Must reproduce the manifest's ``reset_layout_sha256``.
    crater_center_xy_m: tuple[float, float] = (0.45, 0.20)
    crater_radius_m: float = 0.36
    crater_depth_m: float = 0.11
    pile_center_xy_m: tuple[float, float] = (-0.25, -0.20)
    pile_radius_m: float = 0.32
    pile_height_m: float = 0.28

    robot_spawn_xy_m: tuple[float, float] = (-0.90, 0.0)
    robot_spawn_yaw_rad: float = 0.0
    robot_spawn_z_m: float = 0.40

    ## Oracle diagnostic-map resolution; reward uses native-resolution grading
    ## cells. The actor external-heightmap profile defaults to ``ideal``.
    elev_map_size: int = 32

    def _verify_general_reset_layout(self) -> None:
        """Skip the general task's reset-layout check."""

    def __post_init__(self):
        if not self.particles:
            raise ValueError("terrain_landscaping_crater requires particles")
        if not isinstance(self.particles_size, (int, float)):
            raise InvalidInputError(
                "terrain_landscaping_crater requires one scalar particles_size"
            )
        self.particles_init_pos = (0.0, 0.0, 0.5 * float(self.particles_size))
        # Set before super(), which builds the sunlight asset.
        self.sunlight_rpy = SITE_SUNLIGHT_RPY

        super().__post_init__()

        scene_spawner = self.scene.particles0.spawn  # type: ignore[attr-defined]
        managed_spawner = self._particle_system_cfgs["particles0"].spawner
        for spawner in (scene_spawner, managed_spawner):
            if not isinstance(spawner, CraterPileParticlesSpawnerCfg):
                raise TypeError(
                    "terrain_landscaping_crater requires CraterPileParticlesSpawnerCfg"
                )
            spawner.crater_center_xy_m = self.crater_center_xy_m
            spawner.crater_radius_m = self.crater_radius_m
            spawner.crater_depth_m = self.crater_depth_m
            spawner.pile_center_xy_m = self.pile_center_xy_m
            spawner.pile_radius_m = self.pile_radius_m
            spawner.pile_height_m = self.pile_height_m
            spawner.velocity = (0.0, 0.0, 0.0)

        # Shovel extension is hidden state, so reset to the retracted asset pose.
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
        """Require live reset fields and success parameters to match the manifest."""
        manifest = load_crater_target_manifest()
        if manifest.task_id != _CRATER_TASK_ID:
            raise ValueError(
                f"target manifest {manifest.manifest_id!r} identifies task "
                f"{manifest.task_id!r}, not {_CRATER_TASK_ID!r}"
            )

        spawner = self._particle_system_cfgs["particles0"].spawner
        live_layout = ResetLayoutParams(
            version=RESET_LAYOUT_VERSION_CRATER_PILE_V1,
            spawner=self.particles_spawner,
            dim_x=int(spawner.dim_x),
            dim_y=int(spawner.dim_y),
            dim_z=int(spawner.dim_z),
            particle_size_m=float(spawner.particle_size),
            particle_spacing_m=float(spawner.particle_spacing),
            ratio=float(spawner.ratio),
            crater_center_xy_m=tuple(float(v) for v in spawner.crater_center_xy_m),
            crater_radius_m=float(spawner.crater_radius_m),
            crater_depth_m=float(spawner.crater_depth_m),
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
                "terrain_landscaping_crater configuration does not reproduce "
                "the target manifest's reset layout: live reset-layout hash "
                f"{live_hash} != manifest reset_layout_sha256 "
                f"{manifest.reset_layout_sha256}. The fixed mission is owned "
                "by the manifest; change the manifest or restore "
                f"the config. Live layout: {live_layout.canonical_dict()}"
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
                "terrain_landscaping_crater success parameters "
                f"{live_success} differ from the manifest's saved values "
                f"{manifest_success}"
            )


class Task(BaseTask):
    """Fill one shallow crater from one nearby regolith pile."""

    cfg: TaskCfg

    #: Manifest identity this task requires; the mound sibling overrides it.
    MANIFEST_TASK_ID: str = _CRATER_TASK_ID

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg=cfg, runtime_mode=runtime_mode, **kwargs)
        if self._target_manifest.task_id != type(self).MANIFEST_TASK_ID:
            raise ValueError(
                "loaded target manifest does not identify "
                f"{type(self).MANIFEST_TASK_ID!r}: "
                f"{self._target_manifest.task_id!r}"
            )

    def _build_reference_target_manifest(self) -> TargetMapManifest:
        """Load the checked-in crater manifest."""
        return load_crater_target_manifest()

    def _generate_target_manifest(
        self, *, env_id: int, episode_index: int
    ) -> TargetMapManifest:
        """Reuse the frozen mission for every episode."""
        del env_id, episode_index
        return self._target_manifest

    def _manifest_quality_params(self) -> StateQualityParams:
        """Return success parameters from the verified manifest."""
        success = self._target_manifest.success
        return StateQualityParams(
            success_tolerance_m=float(success.success_tolerance_m),
            fraction_within_tolerance=float(success.fraction_within_tolerance),
            sustained_frames=int(success.sustained_frames),
            manifest_sha256=self._target_manifest.manifest_sha256,
            io_schema_fingerprint=self._io_contract.fingerprint(),
        )


__all__ = ["CraterReferenceController", "Task", "TaskCfg"]
