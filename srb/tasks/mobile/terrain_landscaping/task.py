"""Rover grading of a particle bed from external-heightmap observations.
The actor receives seven contract leaves; oracle particle maps serve reward and
metrics. Each episode uses a hashed, mass-balanced target. The sole reward is
stationary grading quality; safety and effort are metrics. Time limit truncates
the continuing task; pose/velocity envelope exits terminate it."""

from __future__ import annotations

import hashlib
import logging as _logging
import time as _time
from collections.abc import Sequence
from dataclasses import MISSING
from typing import Any

import numpy as np
import torch

import srb.utils.math as math_utils
from srb._typing import StepReturn
from srb.assets.object.tool.shovel import RaphShovel
from srb.assets.robot.mobile.ground.raph_rover import RaphRover
from srb.assets.scenery.terrain import Plane
from srb.core.domain import Domain
from srb.core.env import (
    GroundEnv,
    GroundEnvCfg,
    GroundEventCfg,
    GroundSceneCfg,
    ViewerCfg,
)
from srb.core.env.runtime import episode_outcome_info
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import (
    randomize_external_force_torque_bernoulli,
    randomize_joint_parameters,
    randomize_particle_material_uniform,
    randomize_rigid_body_mass,
    reset_root_state_uniform,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import RewardNormalization
from srb.core.reward.cfg import BaseRewardCfg
from srb.core.scene_introspection import find_scene_sensor
from srb.core.sensor import ContactSensorCfg
from srb.core.sensor.category import SensorCategory
from srb.core.sim import PreviewSurfaceCfg
from srb.terrain.landscaping_contract import (
    LANDSCAPING_IO_CONTRACT,
    SuccessParams,
    TargetMapMaterialization,
    landscaping_deployment_recipe,
)
from srb.terrain.landscaping_external_heightmap import (
    ExternalHeightmapBatch,
    MeasuredExternalHeightmapSource,
    SourceFrame,
    TargetManifest,
    materialize_surface_raster_fast,
    project_actor_observation,
)
from srb.terrain.landscaping_maps import (
    GENERAL_RESET_LAYOUT,
    GENERAL_TARGET_GENERATOR_VERSION,
    GeneralMissionProfile,
    GeneralResetLayoutParams,
    canonical_json_bytes,
    compute_general_reset_layout_sha256,
    general_mission_seed,
    generate_general_target_manifest,
)
from srb.terrain.landscaping_observations import (
    build_landscaping_actor_observation,
)
from srb.terrain.landscaping_pose import (
    mission_pose_from_world_torch,
)
from srb.terrain.landscaping_sim_step import (
    landscaping_base_pose_from_root,
    landscaping_filtered_contact_force_norm,
    landscaping_poison_invalid_rewards,
    landscaping_robot_state_finite,
)
from srb.terrain.landscaping_step import (
    LandscapingStepEvaluation,
    LandscapingStepEvaluator,
    LandscapingStepEvaluatorCfg,
    LandscapingStepSnapshot,
)
from srb.terrain.landscaping_success import (
    LandscapingStateQuality,
    StateQualityParams,
)
from srb.utils.cfg import configclass

from .external_heightmap_cfg import ExternalHeightmapCfg
from .scene_design import (
    apply_site_visuals,
    landscaping_containment,
    plane_half_extent,
    regolith_material,
    site_viewer,
)

_log = _logging.getLogger(__name__)

_DEFAULT_BED_SIZE_M = (4.0, 4.0)


class RewardCfg(BaseRewardCfg):
    normalize: RewardNormalization = RewardNormalization.NONE


_plane_half_extent = plane_half_extent


@configclass
class SceneCfg(GroundSceneCfg):
    env_spacing: float = 10.0
    contacts_wall: ContactSensorCfg = ContactSensorCfg(
        prim_path=MISSING,  # type: ignore[arg-type]
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/wall{i}" for i in range(4)],
    )


@configclass
class EventCfg(GroundEventCfg):
    randomize_robot_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-1.1, 1.1),
                "y": (-1.1, 1.1),
                "z": (0.3, 0.5),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {},
        },
    )
    randomize_blade_mass: SrbEventTermCfg | None = SrbEventTermCfg(
        func=randomize_rigid_body_mass,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("payload"),
            "mass_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
    )
    randomize_wheel_joint_friction: SrbEventTermCfg = SrbEventTermCfg(
        func=randomize_joint_parameters,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["MISSING"]),
            "friction_distribution_params": (0.0, 0.02),
            "operation": "abs",
        },
    )
    randomize_disturbance_force: SrbEventTermCfg = SrbEventTermCfg(
        func=randomize_external_force_torque_bernoulli,
        mode="interval",
        interval_range_s=(0.1, 0.1),
        category=RandomizationCategory.PHYSICS,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["MISSING"]),
            "force_range": (-5.0, 5.0),
            "torque_range": (-0.5, 0.5),
            "probability": 0.05,
        },
    )
    randomize_regolith_material: SrbEventTermCfg = SrbEventTermCfg(
        func=randomize_particle_material_uniform,
        mode="reset",
        category=RandomizationCategory.PHYSICS,
        params={
            "friction_range": (0.6, 1.0),
            "density_range": (2700.0, 3100.0),
        },
    )


@configclass
class TaskCfg(GroundEnvCfg):
    domain: Domain = Domain.MOON
    robot: RaphRover = RaphRover(
        payload=RaphShovel(),
        action_mode="drive_filtered",
    )
    scenery: Plane = landscaping_containment()
    bed_size_m: tuple[float, float] = _DEFAULT_BED_SIZE_M
    bed_wall_height_m: float = 0.3
    bed_visual_prim_name: str = "bed_visual"
    scene: SceneCfg = SceneCfg()
    events: EventCfg = EventCfg()
    reward: RewardCfg = RewardCfg()
    env_rate: float = 1.0 / 100.0
    agent_rate: float = 1.0 / 10.0
    episode_length_s: float = 60.0
    is_finite_horizon: bool = False
    particles: bool = True
    particles_material: PreviewSurfaceCfg = regolith_material()
    particles_spawner: str = "grid"
    particles_size: float = 0.05
    particles_area: float = 0.4
    particles_height: int = 3
    particles_ratio: float = 0.75
    particles_dim_xy: int = 78
    particles_density: float = 2900.0
    particles_friction: float = 0.84
    particles_damping: float = 0.15
    particles_cohesion: float = 0.0
    particle_warmup_steps: int = 100
    particles_max_depenetration_velocity: float = 0.1
    particles_max_velocity: float = 0.5
    elev_map_size: int = 16
    elev_map_extent: float = 4.0
    general_mission_seed: int = 0
    general_mission_profile: GeneralMissionProfile = GeneralMissionProfile()
    general_reset_layout: GeneralResetLayoutParams = GENERAL_RESET_LAYOUT
    external_heightmap: ExternalHeightmapCfg = ExternalHeightmapCfg()
    stuck_velocity_threshold: float = 0.02
    stuck_command_threshold: float = 0.1
    stuck_window_steps: int = 20
    reward_metric: str = "grading_mae"
    excess_reference_m3: float = 1.0
    success_window_steps: int = 20
    success_fraction_threshold: float = 0.9
    success_tolerance_m: float = 0.02
    viewer: ViewerCfg = site_viewer(_DEFAULT_BED_SIZE_M)

    def __post_init__(self):
        if type(self) is TaskCfg and self.elev_map_size != 16:
            raise ValueError(
                "terrain_landscaping requires elev_map_size=16; the general "
                "mission diagnostic grid is fixed by the task contract"
            )
        payload = getattr(self.robot, "payload", None)
        if payload is None:
            if isinstance(self.robot, RaphRover):
                self.robot.payload = RaphShovel()
                payload = self.robot.payload
            else:
                raise TypeError(
                    "terrain_landscaping requires the canonical "
                    "RaphRover + RaphShovel deployment"
                )
        if not isinstance(self.robot, RaphRover):
            raise TypeError("terrain_landscaping requires a RaphRover")
        if not isinstance(payload, RaphShovel):
            raise TypeError("terrain_landscaping requires a RaphShovel payload")
        self.robot.action_mode = "drive_filtered"
        payload.action_mode = "joint_vel"
        landscaping_deployment_recipe(self.robot)
        if self.particles_init_pos is None and isinstance(
            self.particles_size, (int, float)
        ):
            self.particles_init_pos = (0.0, 0.0, 0.5 * float(self.particles_size))
        if str(self.sim.device).lower() != "cpu":
            _log.warning(
                "terrain_landscaping overrides sim.device=%s to cpu: Isaac Sim 6 "
                "suppresses USD particle state readback on the CUDA direct pipeline.",
                self.sim.device,
            )
            self.sim.device = "cpu"
        self.scenery.asset_cfg.spawn.size = _plane_half_extent(self.bed_size_m)
        self.scenery.wall_height = float(self.bed_wall_height_m)
        self.scenery.obstacle_count = 0
        drive_joint_pattern = "drive_joint_.*"
        self.events.randomize_wheel_joint_friction.params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=[drive_joint_pattern]
        )
        self.events.randomize_disturbance_force.params["asset_cfg"] = SceneEntityCfg(
            "robot",
            body_names=[self.robot.frame_base.prim_relpath],
        )
        self.scene.contacts_wall.prim_path = (
            f"{{ENV_REGEX_NS}}/robot/{self.robot.frame_base.prim_relpath}"
        )
        super().__post_init__()
        apply_site_visuals(
            self.scene,
            bed_size_m=self.bed_size_m,
            wall_height_m=self.bed_wall_height_m,
            bed_prim_name=self.bed_visual_prim_name,
            apron_max_extent_m=(
                None if int(self.scene.num_envs) <= 1 else self._env_spacing_for_bed()
            ),
        )
        # Reframe a resized bed unless the viewer was overridden.
        if self.viewer == site_viewer(_DEFAULT_BED_SIZE_M):
            self.viewer = site_viewer(self.bed_size_m)
        if not self.stack:
            self.spacing = self._env_spacing_for_bed()
            self.scene.env_spacing = self.spacing
        if self.particle_warmup_steps < 1:
            raise ValueError("particle_warmup_steps must be positive")
        if self.particles_max_depenetration_velocity <= 0.0:
            raise ValueError("particles_max_depenetration_velocity must be positive")
        if self.particles_max_velocity <= 0.0:
            raise ValueError("particles_max_velocity must be positive")
        for particle_name, particle_cfg in self._particle_system_cfgs.items():
            particle_cfg.settle_max_steps = 1
            particle_cfg.settle_step_time = self.particle_warmup_steps * self.env_rate
            particle_cfg.spawner.max_depenetration_velocity = (
                self.particles_max_depenetration_velocity
            )
            particle_cfg.spawner.max_velocity = self.particles_max_velocity
            scene_particle_cfg = getattr(self.scene, particle_name, None)
            scene_spawner = getattr(scene_particle_cfg, "spawn", None)
            if scene_spawner is not None:
                scene_spawner.max_depenetration_velocity = (
                    self.particles_max_depenetration_velocity
                )
                scene_spawner.max_velocity = self.particles_max_velocity
            for spawner in (particle_cfg.spawner, scene_spawner):
                if spawner is not None:
                    spawner.velocity = (0.0, 0.0, 0.0)
        self.active_sensors.add(SensorCategory.TERRAIN)
        _log.info(
            "Landscaping heightmap: no raycast sensor (PhysX particle sets are "
            "not raycastable on Isaac 6); cached particle positions are binned "
            "directly onto the attach-frame observation grid."
        )
        self._verify_bed_fits_its_containment(self._live_reset_layout())
        self._verify_general_reset_layout()

    def _env_spacing_for_bed(self) -> float:
        return float(max(self.bed_size_m)) + 2.0

    def _live_reset_layout(self) -> GeneralResetLayoutParams:
        spawner = self._particle_system_cfgs["particles0"].spawner
        return GeneralResetLayoutParams(
            version=self.general_reset_layout.version,
            spawner=self.particles_spawner,
            dim_x=int(spawner.dim_x),
            dim_y=int(spawner.dim_y),
            dim_z=int(spawner.dim_z),
            particle_size_m=float(spawner.particle_size),
            particle_spacing_m=float(spawner.particle_spacing),
            ratio=float(spawner.ratio),
            translation_m=tuple(float(v) for v in self.particles_init_pos),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            velocity_m_s=tuple(float(v) for v in spawner.velocity),
        )

    def _verify_general_reset_layout(self) -> None:
        declared = self.general_reset_layout
        live = self._live_reset_layout()
        live_hash = compute_general_reset_layout_sha256(live)
        declared_hash = compute_general_reset_layout_sha256(declared)
        if live_hash != declared_hash:
            raise ValueError(
                "terrain_landscaping configuration does not reproduce the "
                "declared general reset layout: live reset-layout hash "
                f"{live_hash} != declared {declared_hash}. The mission "
                "generator balances cut against fill on that exact lattice; "
                "update env.general_reset_layout together with the particle "
                f"config. Live layout: {live.canonical_dict()}"
            )

    def _verify_bed_fits_its_containment(
        self, layout: GeneralResetLayoutParams
    ) -> None:
        """Check spawned Plane half-extents: Hydra-tunable bed_size_m is absent
        from the contract fingerprint and must stay within the mission envelope."""
        geometry = LANDSCAPING_IO_CONTRACT.geometry
        support_radius = float(LANDSCAPING_IO_CONTRACT.surface_rule.support_radius_m)
        bed_half = float(
            np.abs(layout.flat_bed().world_particle_centres()[:, :2]).max()
        )
        roi_half = 0.5 * geometry.roi_shape[0] * geometry.cell_size_m
        scenery = getattr(self, "_scenery", None)
        if not isinstance(scenery, Plane):
            scenery = self.scenery
        surface_half_min = min(float(v) for v in scenery.asset_cfg.spawn.size)
        surface_half_max = max(float(v) for v in scenery.asset_cfg.spawn.size)
        for limit, name in (
            (
                surface_half_min,
                f"containment surface half-extent {surface_half_min:g} m",
            ),
            (roi_half, f"manifest ROI half-extent {roi_half:g} m"),
        ):
            if bed_half + support_radius > limit + 1e-9:
                raise ValueError(
                    "terrain_landscaping particle bed does not fit its "
                    f"containment: lattice half-extent {bed_half:.4f} m plus "
                    f"the external-heightmap support radius {support_radius:.4f} m exceeds the "
                    f"{name}. Reduce particles_dim_xy or enlarge bed_size_m "
                    f"up to the {2 * roi_half:g} m manifest ROI."
                )
        if surface_half_max > roi_half + 1e-9:
            raise ValueError(
                "terrain_landscaping containment surface is larger than the "
                f"mission frame: bed_size_m {tuple(self.bed_size_m)} gives a "
                f"half-extent of {surface_half_max:g} m, beyond the manifest ROI "
                f"half-extent {roi_half:g} m and the mission-pose "
                "envelope it defines. The rover would gain drivable surface "
                "outside the observation contract and end episodes in safety "
                "terminations, while bed_size_m is not part of "
                "the fingerprint payload so io_schema_fingerprint would still "
                "read unchanged. Reduce bed_size_m to at most "
                f"{2 * roi_half:g} m."
            )


class _LandscapingTaskMixin:
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)
        device = self.device
        n_envs = self.num_envs
        self._contacts_wall = find_scene_sensor(self.scene, "contacts_wall")
        self._regolith_ps = self.get_particle_system(
            "particles0", fallback_to_first=True
        )
        self._particle_read_generation = np.zeros(n_envs, dtype=np.int64)
        self._truth_surface_generation = np.full(n_envs, -1, dtype=np.int64)
        self._truth_surface_cache: list[np.ndarray | None] = [None] * n_envs
        self._envelope_exit = np.zeros(n_envs, dtype=bool)
        self._io_contract = LANDSCAPING_IO_CONTRACT
        self._external_profile = cfg.external_heightmap.to_profile()
        self._episode_index = np.zeros(n_envs, dtype=np.int64)
        self._target_manifest = self._build_reference_target_manifest()
        self._manifest_materialization: TargetMapMaterialization = (
            self._target_manifest.verify(contract=self._io_contract)
        )
        self._episode_manifests: list[TargetManifest] = [self._target_manifest] * n_envs
        self._observable_target_map = torch.from_numpy(
            np.repeat(
                np.array(self._manifest_materialization.target, dtype=np.float32)[None],
                n_envs,
                axis=0,
            )
        ).to(device=device)
        self._observable_quality = self._make_observable_quality()
        self._oracle_quality = self._make_oracle_quality()
        self._step_evaluator = LandscapingStepEvaluator(
            num_envs=n_envs,
            device=device,
            oracle_quality=self._oracle_quality,
            observable_quality=self._observable_quality,
            cfg=LandscapingStepEvaluatorCfg(
                stuck_velocity_threshold=cfg.stuck_velocity_threshold,
                stuck_command_threshold=cfg.stuck_command_threshold,
                stuck_window_steps=cfg.stuck_window_steps,
                reward_metric=cfg.reward_metric,
                excess_reference_m3=cfg.excess_reference_m3,
            ),
        )
        self._external_source = MeasuredExternalHeightmapSource(
            num_envs=n_envs,
            provider=self._external_frame_provider,
            profile=self._external_profile,
        )
        self._last_external_batch: ExternalHeightmapBatch | None = None
        self._last_external_observation: dict[str, dict[str, torch.Tensor]] | None = (
            None
        )
        self._last_external_policy_time_s = 0.0
        self._last_step_evaluation: LandscapingStepEvaluation | None = None
        self._external_timing_ms: dict[str, float] = {}
        self._publish_contract_observation_descriptors()
        self.advance_step_state()

    def _saved_success_params(self) -> SuccessParams:
        return SuccessParams(
            success_tolerance_m=float(self.cfg.success_tolerance_m),
            fraction_within_tolerance=float(self.cfg.success_fraction_threshold),
            sustained_frames=int(self.cfg.success_window_steps),
        )

    def _build_reference_target_manifest(self) -> TargetManifest:
        return self._generate_target_manifest(env_id=0, episode_index=0)

    def _generate_target_manifest(
        self, *, env_id: int, episode_index: int
    ) -> TargetManifest:
        cfg = self.cfg
        return generate_general_target_manifest(
            seed=general_mission_seed(
                base_seed=int(cfg.general_mission_seed),
                env_id=int(env_id),
                episode_index=int(episode_index),
            ),
            success=self._saved_success_params(),
            reset_layout=cfg.general_reset_layout,
            profile=cfg.general_mission_profile,
            contract=self._io_contract,
        )

    def _target_manifests_for_reset(self, env_ids: np.ndarray) -> list[TargetManifest]:
        return [
            self._generate_target_manifest(
                env_id=int(env_id),
                episode_index=int(self._episode_index[int(env_id)]),
            )
            for env_id in env_ids.tolist()
        ]

    @property
    def episode_target_manifests(self) -> tuple[TargetManifest, ...]:
        return tuple(self._episode_manifests)

    def _manifest_quality_params(self) -> StateQualityParams:
        success = self._saved_success_params()
        return StateQualityParams(
            success_tolerance_m=float(success.success_tolerance_m),
            fraction_within_tolerance=float(success.fraction_within_tolerance),
            sustained_frames=int(success.sustained_frames),
            manifest_sha256=GENERAL_TARGET_GENERATOR_VERSION,
            io_schema_fingerprint=self._io_contract.fingerprint(),
        )

    def _make_observable_quality(self) -> LandscapingStateQuality:
        geometry = self._io_contract.geometry
        grading = np.asarray(self._manifest_materialization.grading_indices)
        centres_x = np.asarray(geometry.cell_centres(0))
        centres_y = np.asarray(geometry.cell_centres(1))
        centres = np.stack(
            [centres_x[grading[:, 0]], centres_y[grading[:, 1]]], axis=-1
        )
        return LandscapingStateQuality(
            params=self._manifest_quality_params(),
            map_shape=geometry.source_shape,
            grading_indices=torch.from_numpy(grading.copy()),
            cell_centres_xy=torch.from_numpy(centres.astype(np.float32)),
            cell_size_m=float(geometry.cell_size_m),
            device=self.device,
        )

    def _make_oracle_quality(self) -> LandscapingStateQuality:
        return self._make_observable_quality()

    def _oracle_quality_maps(self) -> tuple[torch.Tensor, torch.Tensor]:
        origins = self.scene.env_origins
        shape = self._io_contract.geometry.source_shape
        surfaces = np.empty((self.num_envs, *shape), dtype=np.float32)
        fallback_rows: list[int] = []
        for row in range(self.num_envs):
            cached = self._truth_surface_cache[row]
            if (
                cached is not None
                and self._truth_surface_generation[row]
                == self._particle_read_generation[row]
            ):
                surfaces[row] = cached
            else:
                fallback_rows.append(row)
        if fallback_rows:
            particle_pos = getattr(
                self._regolith_ps.data.particle_pos_w,
                "torch",
                self._regolith_ps.data.particle_pos_w,
            )
            rows_t = torch.as_tensor(
                fallback_rows, device=self.device, dtype=torch.long
            )
            local = (
                (particle_pos[rows_t] - origins[rows_t].unsqueeze(1))
                .detach()
                .cpu()
                .numpy()
            )
            for local_index, row in enumerate(fallback_rows):
                if not np.isfinite(local[local_index]).all():
                    surfaces[row] = np.nan
                    self._invalidate_truth_surface_cache([row])
                    continue
                surface, _support = materialize_surface_raster_fast(
                    local[local_index],
                    geometry=self._io_contract.geometry,
                    rule=self._io_contract.surface_rule,
                )
                surfaces[row] = surface
                self._truth_surface_cache[row] = surface
                self._truth_surface_generation[row] = self._particle_read_generation[
                    row
                ]
        oracle_current = torch.from_numpy(surfaces).to(device=self.device)
        return oracle_current, self._observable_target_map

    def _publish_contract_observation_descriptors(self) -> None:
        from srb.core.spec.registry import ComponentDescriptor

        vector_terms: list = []
        for key, term in self._io_contract.to_observation_term_specs():
            group, _, name = key.partition("/")
            if len(term.shape) >= 2:
                setattr(
                    self,
                    f"_observation_descriptor_{group}__{name}",
                    ComponentDescriptor(
                        kind="observation",
                        schema_version=1,
                        fields=[term],
                        extra_metadata={
                            "group": f"{group}__{name}",
                            "provenance": self._io_contract.contract_id,
                        },
                    ),
                )
            else:
                vector_terms.append(term)
        self._observation_descriptor_vector = ComponentDescriptor(
            kind="observation",
            schema_version=1,
            fields=vector_terms,
            extra_metadata={
                "group": "vector",
                "provenance": self._io_contract.contract_id,
            },
        )

    def srb_spec_metadata(self) -> dict[str, Any]:
        return {"landscaping_contract": self.landscaping_contract_metadata()}

    def landscaping_contract_metadata(self) -> dict[str, Any]:
        cfg = self.cfg
        missing = object()
        effective_robot = getattr(cfg, "_robot", missing)
        if effective_robot is missing:
            effective_robot = cfg.robot
        deployment_recipe = landscaping_deployment_recipe(effective_robot)
        manifest = self._episode_manifests[0]
        metadata = {
            "contract_id": self._io_contract.contract_id,
            "contract": self._io_contract.canonical_dict(),
            "io_schema_fingerprint": self._io_contract.fingerprint(),
            "deployment_recipe": deployment_recipe,
            "target_manifest_id": manifest.manifest_id,
            "target_manifest_sha256": manifest.manifest_sha256,
            "target_map_sha256": manifest.target_map_sha256,
            "reset_layout_sha256": manifest.reset_layout_sha256,
            "mapper_profile": self._external_profile.canonical_dict(),
            "mapper_profile_sha256": self._external_profile.profile_hash(),
            "projection_layout_fingerprint": (self._projection_layout_fingerprint()),
        }
        if type(cfg) is TaskCfg:
            profile_bytes = canonical_json_bytes(
                cfg.general_mission_profile.canonical_dict()
            )
            metadata.update(
                {
                    "target_identity_kind": "general_mission_family_v1",
                    "target_generator_version": GENERAL_TARGET_GENERATOR_VERSION,
                    "target_profile_sha256": hashlib.sha256(profile_bytes).hexdigest(),
                    "target_reset_layout_sha256": manifest.reset_layout_sha256,
                }
            )
        return metadata

    def _projection_layout_fingerprint(self) -> str | None:
        assembled_space = getattr(self, "single_observation_space", None)
        if assembled_space is None:
            return None
        try:
            from srb.core.env.runtime.observation_layout import (
                NumericObservationProjection,
                contract_detailed_dict_space,
            )

            projection = NumericObservationProjection.from_contract(
                self._io_contract,
                contract_detailed_dict_space(self._io_contract),
                assembled_space,
                allow_unbounded_assembled_bounds=True,
            )
        except Exception as exc:  # noqa: BLE001 - metadata must not break a run
            _log.warning("Failed to derive landscaping projection fingerprint: %s", exc)
            return None
        return projection.layout_fingerprint

    def _monotonic_sim_time_s(self) -> float:
        return float(self.common_step_counter) * float(self.step_dt)

    def _external_frame_provider(self, env_ids: np.ndarray) -> list[SourceFrame | None]:
        if self._regolith_ps is None:
            raise RuntimeError(
                "the external-heightmap source requires a particle system; "
                "terrain_landscaping cannot run with particles disabled"
            )
        now = self._monotonic_sim_time_s()
        t0 = _time.perf_counter()
        ids_t = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        self._update_particle_state_from_sim(env_ids=ids_t)
        rows = [int(i) for i in np.asarray(env_ids).tolist()]
        self._invalidate_truth_surface_cache(rows)
        self._particle_read_generation[rows] += 1
        particle_pos = getattr(
            self._regolith_ps.data.particle_pos_w,
            "torch",
            self._regolith_ps.data.particle_pos_w,
        )
        root_state = getattr(
            self._robot.data.root_state_w, "torch", self._robot.data.root_state_w
        )
        base_pos_w = root_state[:, 0:3]
        base_quat_w = root_state[:, 3:7]
        pose_full = landscaping_base_pose_from_root(
            base_pos_w=base_pos_w[..., :2],
            base_quat_w=base_quat_w,
        )
        mission_pose = mission_pose_from_world_torch(
            base_pos_w=base_pos_w,
            base_yaw=pose_full.yaw,
            env_origins=self.scene.env_origins,
        )
        lin_b = math_utils.quat_apply_inverse(base_quat_w, root_state[:, 7:10])
        ang_b = math_utils.quat_apply_inverse(base_quat_w, root_state[:, 10:13])
        body_velocity = torch.stack([lin_b[:, 0], lin_b[:, 1], ang_b[:, 2]], dim=-1)
        env_origins = self.scene.env_origins
        particles_np = (
            (particle_pos[rows] - env_origins[rows].unsqueeze(1)).detach().cpu().numpy()
        )
        pose_np = mission_pose[rows].detach().cpu().numpy().astype(np.float32)
        vel_np = body_velocity[rows].detach().cpu().numpy().astype(np.float32)
        t1 = _time.perf_counter()
        frames: list[SourceFrame | None] = []
        for row in range(len(rows)):
            self._envelope_exit[rows[row]] = False
            if not (
                np.isfinite(pose_np[row]).all()
                and np.isfinite(vel_np[row]).all()
                and np.isfinite(particles_np[row]).all()
            ):
                self._invalidate_truth_surface_cache([rows[row]])
                frames.append(None)
                continue
            if not self._robot_state_in_envelope(pose_np[row], vel_np[row]):
                self._envelope_exit[rows[row]] = True
                self._invalidate_truth_surface_cache([rows[row]])
                frames.append(None)
                continue
            surface, _support = materialize_surface_raster_fast(
                particles_np[row],
                geometry=self._io_contract.geometry,
                rule=self._io_contract.surface_rule,
            )
            self._truth_surface_cache[rows[row]] = surface
            self._truth_surface_generation[rows[row]] = self._particle_read_generation[
                rows[row]
            ]
            frames.append(
                SourceFrame(
                    elevation=surface,
                    mission_pose=pose_np[row],
                    body_velocity=vel_np[row],
                    acquired_time_s=now,
                )
            )
        t2 = _time.perf_counter()
        self._external_timing_ms["torch_to_numpy_ms"] = (t1 - t0) * 1e3
        self._external_timing_ms["surface_raster_ms"] = (t2 - t1) * 1e3
        return frames

    def _robot_state_in_envelope(
        self, mission_pose: np.ndarray, body_velocity: np.ndarray
    ) -> bool:
        for key, value in (
            ("proprio/mission_pose", mission_pose),
            ("proprio/base_velocity", body_velocity),
        ):
            leaf = self._io_contract.leaf(key)
            low = np.asarray(leaf.low, dtype=value.dtype)
            high = np.asarray(leaf.high, dtype=value.dtype)
            if np.less(value, low).any() or np.greater(value, high).any():
                return False
        return True

    def _reset_idx(self, env_ids: Sequence[int]):
        env_ids_t = torch.as_tensor(env_ids, device=self.device)
        ids_np = np.asarray(env_ids_t.detach().cpu().numpy(), dtype=np.int64).reshape(
            -1
        )
        # Prepare manifests and targets before mutation; commit manifest identity
        # and episode counters only after all reset steps succeed.
        manifests = self._target_manifests_for_reset(ids_np)
        targets = np.stack(
            [
                np.asarray(
                    manifest.verify(contract=self._io_contract).target,
                    dtype=np.float32,
                )
                for manifest in manifests
            ]
        )
        external_source = getattr(self, "_external_source", None)
        completed = False
        try:
            if external_source is not None:
                external_source.invalidate(ids_np)
            self._invalidate_truth_surface_cache(ids_np)
            if hasattr(self, "_step_return"):
                self._step_return = None
            super()._reset_idx(env_ids)
            if external_source is not None:
                reset_batch = external_source.reset(ids_np, manifests, timeout_s=0)
                self._merge_external_batch(reset_batch)
                self._last_external_policy_time_s = self._monotonic_sim_time_s()
                self._cache_external_actor_observation()
                source_targets = np.asarray(
                    external_source.target_sources(ids_np), dtype=np.float32
                )
                if not np.array_equal(source_targets, targets):
                    raise RuntimeError("source target changed during landscaping reset")
                if int(ids_np.shape[0]) == self.num_envs:
                    self._external_source_bootstrapped = True
            self._observable_target_map[env_ids_t] = torch.from_numpy(
                np.array(targets, copy=True)
            ).to(device=self.device)
            self._step_evaluator.reset(env_ids_t)
            self._reset_step_evaluation_rows(ids_np)
            if getattr(self, "_srb_runtime_mode", None) == "isaac" and hasattr(
                self, "_step_return"
            ):
                refreshed_step_return = self.extract_step_return()
            else:
                refreshed_step_return = None
            for env_id, manifest in zip(ids_np.tolist(), manifests, strict=True):
                self._episode_manifests[int(env_id)] = manifest
            self._episode_index[ids_np] += 1
            if refreshed_step_return is not None:
                self._step_return = refreshed_step_return
            completed = True
        finally:
            if not completed:
                if external_source is not None:
                    external_source.invalidate(ids_np)
                self._invalidate_truth_surface_cache(ids_np)
                self._reset_step_evaluation_rows(ids_np)
                self._last_external_batch = None
                self._last_external_observation = None
                if hasattr(self, "_step_return"):
                    self._step_return = None

    def _reset_step_evaluation_rows(self, env_ids: Sequence[int]) -> None:
        evaluation = getattr(self, "_last_step_evaluation", None)
        if not isinstance(evaluation, LandscapingStepEvaluation):
            return
        ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        def zero_selected_rows(value: torch.Tensor) -> torch.Tensor:
            if value.ndim == 0 or value.shape[0] != self.num_envs:
                return value
            cleared = value.clone()
            cleared[ids] = 0
            return cleared

        reward_terms = {
            name: zero_selected_rows(value)
            for name, value in evaluation.reward_terms.items()
        }
        success = evaluation.success.clone()
        success[ids] = False
        metrics = {
            name: zero_selected_rows(value)
            if isinstance(value, torch.Tensor)
            else value
            for name, value in evaluation.metrics.items()
        }
        self._last_step_evaluation = LandscapingStepEvaluation(
            reward_terms=reward_terms,
            success=success,
            metrics=metrics,
        )

    def _invalidate_truth_surface_cache(self, env_ids: Sequence[int]) -> None:
        cache = getattr(self, "_truth_surface_cache", None)
        generations = getattr(self, "_truth_surface_generation", None)
        if cache is None or generations is None:
            return
        for env_id in env_ids:
            row = int(env_id)
            cache[row] = None
            generations[row] = -1

    def extract_step_return(self) -> StepReturn:
        device = self.device
        base_root_state_w = getattr(
            self._robot.data.root_state_w,
            "torch",
            self._robot.data.root_state_w,
        )
        base_root_pos_w = base_root_state_w[:, 0:3]
        base_quat_w = base_root_state_w[:, 3:7]
        base_vel_b = math_utils.quat_apply_inverse(
            base_quat_w, base_root_state_w[:, 7:10]
        )
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(self.num_envs, dtype=torch.bool, device=device)
        )
        obs = self._external_actor_observation()
        batch = self._last_external_batch
        evaluation = self._last_step_evaluation
        if batch is None or evaluation is None:
            raise RuntimeError(
                "landscaping step cache is unavailable; "
                "advance_step_state() must run before extraction"
            )
        termination = torch.from_numpy(self._envelope_exit.copy()).to(device)
        reward_terms = landscaping_poison_invalid_rewards(
            evaluation.reward_terms,
            landscaping_robot_state_finite(base_root_pos_w, base_quat_w, base_vel_b),
        )
        return StepReturn(
            obs,
            reward_terms,
            termination,
            truncation,
            info=episode_outcome_info(
                success=evaluation.success,
                metrics=evaluation.metrics,
            ),
        )

    # Advance the source and evaluator once; extraction reads snapshots,
    # so lifecycle reassembly cannot consume another frame or success window.
    def advance_step_state(self) -> None:
        self._refresh_external_actor_observation()
        self._last_step_evaluation = self._step_evaluator.evaluate(
            self._build_step_snapshot()
        )

    def _build_step_snapshot(self) -> LandscapingStepSnapshot:
        batch = self._last_external_batch
        if batch is None:
            raise RuntimeError("external source batch is unavailable")
        base_root_state_w = getattr(
            self._robot.data.root_state_w,
            "torch",
            self._robot.data.root_state_w,
        )
        base_root_pos_w = base_root_state_w[:, 0:3]
        base_quat_w = base_root_state_w[:, 3:7]
        base_pose = landscaping_base_pose_from_root(
            base_pos_w=base_root_pos_w[..., :2],
            base_quat_w=base_quat_w,
        )
        base_vel_b = math_utils.quat_apply_inverse(
            base_quat_w, base_root_state_w[:, 7:10]
        )
        mission_pose = mission_pose_from_world_torch(
            base_pos_w=base_root_pos_w,
            base_yaw=base_pose.yaw,
            env_origins=self.scene.env_origins,
        )
        observable_current = torch.from_numpy(
            np.array(batch.elevation, dtype=np.float32)
        ).to(device=self.device)
        observable_sequence = torch.from_numpy(
            np.array(batch.sequence, dtype=np.int64)
        ).to(device=self.device)
        oracle_current, oracle_desired = self._oracle_quality_maps()
        prev_action = self._emitted_action_or_manager()
        return LandscapingStepSnapshot(
            oracle_current_map=oracle_current,
            oracle_target_map=oracle_desired,
            observable_current_map=observable_current,
            observable_target_map=self._observable_target_map,
            observable_sequence=observable_sequence,
            mission_pose=mission_pose,
            base_vel_b=base_vel_b,
            base_rpy=base_pose.base_rpy,
            prev_action=prev_action,
            particle_velocity_magnitude=self._mean_particle_velocity_magnitude(),
            wall_force=self._wall_contact_force_norm(),
            truncation=(
                self.episode_length_buf >= self.max_episode_length
                if self.cfg.truncate_episodes
                else torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            ),
        )

    def _refresh_external_actor_observation(self) -> None:
        policy_time = self._monotonic_sim_time_s()
        env_ids = np.arange(self.num_envs, dtype=np.int64)
        if not getattr(self, "_external_source_bootstrapped", False):
            bootstrap_env_ids = torch.arange(
                self.num_envs,
                device=self.device,
                dtype=torch.long,
            )
            super()._reset_idx(bootstrap_env_ids)
            self._external_source.reset(
                env_ids, list(self._episode_manifests), timeout_s=0
            )
            self._external_source_bootstrapped = True
        t0 = _time.perf_counter()
        batch = self._external_source.observe(env_ids, policy_time)
        self._external_source.check_freshness(batch, policy_time)
        t1 = _time.perf_counter()
        self._last_external_batch = batch
        self._last_external_policy_time_s = policy_time
        self._cache_external_actor_observation()
        self._external_timing_ms["observe_ms"] = (t1 - t0) * 1e3

    def _merge_external_batch(self, batch: ExternalHeightmapBatch) -> None:
        batch_ids = np.asarray(batch.env_ids, dtype=np.int64).reshape(-1)
        canonical_ids = np.arange(self.num_envs, dtype=np.int64)
        if (
            batch_ids.shape[0] > self.num_envs
            or np.any(batch_ids < 0)
            or np.any(batch_ids >= self.num_envs)
            or np.unique(batch_ids).shape[0] != batch_ids.shape[0]
        ):
            raise ValueError("external reset batch contains invalid environment ids")
        if batch_ids.shape[0] == self.num_envs and not np.array_equal(
            batch_ids, canonical_ids
        ):
            if not np.array_equal(np.sort(batch_ids), canonical_ids):
                raise ValueError(
                    "full external reset batch must contain every environment id"
                )
            order = np.argsort(batch_ids)
            batch = ExternalHeightmapBatch(
                env_ids=canonical_ids,
                elevation=np.asarray(batch.elevation)[order],
                timestamp_s=np.asarray(batch.timestamp_s)[order],
                sequence=np.asarray(batch.sequence)[order],
                mission_pose=np.asarray(batch.mission_pose)[order],
                body_velocity=np.asarray(batch.body_velocity)[order],
                manifest_hashes=tuple(batch.manifest_hashes[int(i)] for i in order),
                clock=batch.clock,
                mission_frame_id=batch.mission_frame_id,
                elevation_datum_id=batch.elevation_datum_id,
                geometry=batch.geometry,
            )
            batch_ids = canonical_ids
        current = self._last_external_batch
        if current is None:
            if batch.batch_size != self.num_envs:
                if not getattr(self, "_external_source_bootstrapped", False):
                    raise RuntimeError(
                        "cannot merge a partial source batch before bootstrap"
                    )
                source = getattr(self, "_external_source", None)
                if source is None:
                    raise RuntimeError(
                        "cannot recover a partial source batch without its source"
                    )
                current = source._batch_for(canonical_ids)
            else:
                self._last_external_batch = batch
                return
        if batch.batch_size == self.num_envs:
            self._last_external_batch = batch
            return
        elevation = np.array(current.elevation, copy=True)
        timestamp = np.array(current.timestamp_s, copy=True)
        sequence = np.array(current.sequence, copy=True)
        mission_pose = np.array(current.mission_pose, copy=True)
        body_velocity = np.array(current.body_velocity, copy=True)
        manifest_hashes = list(current.manifest_hashes)
        for row, env_id in enumerate(batch_ids.tolist()):
            elevation[env_id] = batch.elevation[row]
            timestamp[env_id] = batch.timestamp_s[row]
            sequence[env_id] = batch.sequence[row]
            mission_pose[env_id] = batch.mission_pose[row]
            body_velocity[env_id] = batch.body_velocity[row]
            manifest_hashes[env_id] = batch.manifest_hashes[row]
        self._last_external_batch = ExternalHeightmapBatch(
            env_ids=np.arange(self.num_envs, dtype=np.int64),
            elevation=elevation,
            timestamp_s=timestamp,
            sequence=sequence,
            mission_pose=mission_pose,
            body_velocity=body_velocity,
            manifest_hashes=tuple(manifest_hashes),
            clock=current.clock,
            mission_frame_id=current.mission_frame_id,
            elevation_datum_id=current.elevation_datum_id,
            geometry=current.geometry,
        )

    def _cache_external_actor_observation(self) -> None:
        batch = self._last_external_batch
        if batch is None:
            raise RuntimeError("external source batch is unavailable")
        t0 = _time.perf_counter()
        leaves_np = project_actor_observation(
            batch,
            self._external_source.target_sources(batch.env_ids),
            policy_time_s=self._last_external_policy_time_s,
        )
        t1 = _time.perf_counter()
        leaves_torch = {
            key: torch.from_numpy(np.array(value, copy=True)).to(
                device=self.device, dtype=torch.float32
            )
            for key, value in leaves_np.items()
        }
        self._last_external_observation = build_landscaping_actor_observation(
            leaves_torch
        )
        self._external_timing_ms["projection_ms"] = (t1 - t0) * 1e3
        self._external_timing_ms["numpy_to_torch_ms"] = (
            _time.perf_counter() - t1
        ) * 1e3

    def _external_actor_observation(self) -> dict[str, dict[str, torch.Tensor]]:
        observation = self._last_external_observation
        if observation is None:
            raise RuntimeError("external actor observation is unavailable")
        return {
            group: {name: value.clone() for name, value in leaves.items()}
            for group, leaves in observation.items()
        }

    def _emitted_action_or_manager(self) -> torch.Tensor:
        parts: list[torch.Tensor] = []
        for _, term in self.action_manager.iter_terms():
            effective = getattr(term, "emitted_action", None)
            if effective is None:
                effective = getattr(term, "processed_actions", None)
            if not isinstance(effective, torch.Tensor):
                return self.action_manager.prev_action
            parts.append(effective)
        if parts:
            effective_action = torch.cat(parts, dim=-1)
            if effective_action.shape == self.action_manager.prev_action.shape:
                return effective_action
        return self.action_manager.prev_action

    def _update_particle_state_from_sim(
        self,
        *,
        env_ids: Sequence[int] | torch.Tensor | None = None,
    ) -> None:
        if env_ids is None:
            self._regolith_ps.update_state_from_sim()
        else:
            self._regolith_ps.update_state_from_sim(env_ids=env_ids)

    def _wall_contact_force_norm(self) -> torch.Tensor:
        if (
            self._contacts_wall is None
            or self._contacts_wall.data.force_matrix_w is None
        ):
            return torch.zeros(self.num_envs, device=self.device)
        return landscaping_filtered_contact_force_norm(
            self._contacts_wall.data.force_matrix_w,
            normalizing_force_n=50.0,
        )

    def _mean_particle_velocity_magnitude(self) -> torch.Tensor:
        if self._regolith_ps is None:
            return torch.zeros((self.num_envs, 1), device=self.device)
        try:
            vel = self._regolith_ps.data.particle_vel_w
            return vel.norm(dim=-1).mean(dim=-1).unsqueeze(-1)
        except AttributeError:
            return torch.zeros((self.num_envs, 1), device=self.device)


class Task(_LandscapingTaskMixin, SrbEnv, GroundEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)
