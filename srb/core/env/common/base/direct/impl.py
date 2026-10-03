"""Direct-control base environment for SRB tasks."""

import functools
import sys
from collections.abc import Iterator, Mapping, Sequence
from functools import cached_property
from types import MappingProxyType
from typing import TYPE_CHECKING

import gymnasium
import numpy as np
import torch
from isaaclab.envs import DirectRLEnv as __DirectRLEnv

from srb._typing import StepReturn
from srb.core.asset import Articulation, AssetBase, RigidObject, TerrainImporter
from srb.core.asset.particle_system import ParticleSystem
from srb.core.env.common.base.seed import with_replicator_seed
from srb.core.env.direct_env_finalize_init_execution import (
    execute_direct_env_finalize_init_plan,
)
from srb.core.env.direct_env_finalize_init_plan import (
    build_direct_env_finalize_init_plan,
)
from srb.core.env.direct_env_init_execution import execute_direct_env_init_plan
from srb.core.env.direct_env_init_plan import build_direct_env_init_plan
from srb.core.env.runtime.action_layout import ActionLayout, validate_finite_action
from srb.core.env.runtime.actuator_delay import hold_default_joint_targets
from srb.core.env.runtime.delay import (
    ActionDelay,
    DelayRuntime,
    plan_delay_runtime_activation,
)
from srb.core.env.runtime.observation_assembly import ObservationAssembly
from srb.core.env.runtime.observation_layout import box_for_dtype_shape
from srb.core.env.runtime.outcome import DirectStepOutcomeAdapter
from srb.core.env.runtime.outcome_observation import (
    final_observation,
    splice_observation_rows,
)
from srb.core.env.runtime.step_return import (
    coerce_step_return_to_torch,
    scrub_non_finite_step_return,
    validate_step_return,
)
from srb.core.manager import ActionManager
from srb.core.randomization import RandomizationView
from srb.core.reward import RewardOutcomeRuntime
from srb.core.scene_introspection import (
    get_scene_entity,
    get_scene_root_asset,
)
from srb.core.sim.spawners.multi_asset_compat import (
    convert_scene_multi_variant_spawners,
)
from srb.core.spec.lifecycle import (
    get_cached_srb_spec,
    init_srb_spec_state,
    invalidate_cached_srb_spec,
    mark_srb_spec_reset,
)
from srb.utils import logging
from srb.utils.failures import attempt_cleanup
from srb.utils.math import combine_frame_transforms, subtract_frame_transforms
from srb.utils.str import resolve_env_prim_path

from .cfg import DirectEnvCfg

if TYPE_CHECKING:
    from srb.core.sim.robot_setup import AssembledBodies


# A leading ``__`` name is mangled inside the class body.
_UPSTREAM_SEED = __DirectRLEnv.seed


class DirectEnv(__DirectRLEnv):
    """Base environment for direct-control SRB tasks."""

    cfg: DirectEnvCfg
    _step_return: StepReturn

    seed = with_replicator_seed(_UPSTREAM_SEED)

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        # Finalize only after the outermost subclass constructor returns.
        if "__init__" in cls.__dict__:
            _orig_init = cls.__dict__["__init__"]

            @functools.wraps(_orig_init)
            def _wrapped(self, *args, __orig=_orig_init, **kw):  # type: ignore[misc]
                depth = getattr(self, "_srb_init_depth", 0)
                owner = depth == 0
                self._srb_init_depth = depth + 1
                initialized = False
                try:
                    try:
                        __orig(self, *args, **kw)
                    finally:
                        self._srb_init_depth -= 1
                    if owner:
                        self._finalize_init()
                    initialized = True
                finally:
                    if owner and not initialized:
                        original_error = sys.exception()
                        assert original_error is not None
                        self._cleanup_failed_init(original_error)

            cls.__init__ = _wrapped  # type: ignore[method-assign]

    def __init__(self, cfg: DirectEnvCfg, **kwargs):
        # Materialize subclass-derived config and clone counts before scene setup.
        cfg.finalize()
        convert_scene_multi_variant_spawners(cfg.scene)

        init_plan = build_direct_env_init_plan(
            actions_enabled=bool(getattr(cfg, "actions", None)),
            particle_systems_enabled=bool(getattr(cfg, "_particle_system_cfgs", {})),
        )

        def _super_init() -> None:
            super(DirectEnv, self).__init__(cfg, **kwargs)
            from srb.core.sensor.event_camera.materialization import (
                bind_event_camera_sensors,
            )

            bind_event_camera_sensors(
                self.scene,
                getattr(self.cfg, "_event_camera_bindings", {}),
            )

        class _Hooks:
            def init_hud_fields(_self) -> None:
                self._hud = None
                self._hud_last_update_step = -1

            def super_init(_self) -> None:
                _super_init()

            def init_srb_spec_state(_self) -> None:
                init_srb_spec_state(self)

            def apply_visuals(_self) -> None:
                self.cfg.visuals.func(self.cfg.visuals)

            def init_action_manager(_self) -> None:
                self.action_manager = ActionManager(
                    self.cfg.actions,
                    env=self,  # type: ignore[arg-type]
                )

            def init_asset_registries(_self) -> None:
                self._scenery: AssetBase | TerrainImporter | None = (
                    self.scene.terrain or get_scene_entity(self.scene, "scenery")
                    if self.cfg.scenery is not None
                    else None
                )
                self._robot = get_scene_root_asset(self.scene, "robot")
                self._robots = {"robot": self._robot}
                for robot_name in self.cfg._robots:
                    if robot_name != "robot":
                        self._robots[robot_name] = get_scene_root_asset(
                            self.scene, robot_name
                        )

                self._particle_systems: dict[str, ParticleSystem] = {}

            def init_particle_systems(_self) -> None:
                _ps_cfgs = getattr(self.cfg, "_particle_system_cfgs", {})
                if _ps_cfgs:
                    _assert_gpu_dynamics_enabled("particle systems")
                for ps_name, ps_cfg in _ps_cfgs.items():
                    ps = ParticleSystem(
                        cfg=ps_cfg,
                        num_envs=self.num_envs,
                        device=self.device,
                    )
                    ps.initialize()
                    self._particle_systems[ps_name] = ps
                if _ps_cfgs:
                    logging.info(
                        f"Initialized {len(self._particle_systems)} particle system(s): "
                        f"{list(self._particle_systems.keys())}"
                    )

            def reapply_viewer_pose(_self) -> None:
                self._reapply_viewer_pose()

        initialized = False
        try:
            execute_direct_env_init_plan(init_plan, _Hooks())
            initialized = True
        finally:
            if getattr(self, "_srb_init_depth", 0) == 0 and not initialized:
                original_error = sys.exception()
                assert original_error is not None
                self._cleanup_failed_init(original_error)

    def _reapply_viewer_pose(self) -> None:
        viewport_camera_controller = getattr(self, "viewport_camera_controller", None)
        if viewport_camera_controller is None:
            return

        origin_type = getattr(viewport_camera_controller.cfg, "origin_type", None)
        if origin_type not in ("world", "env"):
            return

        viewport_camera_controller.update_view_location(
            eye=self.cfg.viewer.eye,
            lookat=self.cfg.viewer.lookat,
        )

    def _warn_if_legacy_lifecycle(self) -> None:
        """Warn if a task class bypasses the owned ``SrbEnv`` lifecycle."""
        from srb.core.env.runtime.lifecycle import (
            _warn_legacy_lifecycle_once,
            runs_owned_lifecycle,
        )

        if runs_owned_lifecycle(type(self)):
            return
        _warn_legacy_lifecycle_once(self)

    def _finalize_init(self) -> None:
        self._warn_if_legacy_lifecycle()
        from srb.core.sensor.episode_recording import scene_has_event_camera_sensors

        plan = build_direct_env_finalize_init_plan(
            step_return_workflow=self._use_step_return_workflow,
            camera_record=bool(getattr(self.cfg, "camera_record", False)),
            event_camera=scene_has_event_camera_sensors(self.scene),
        )

        class _Hooks:
            def init_step_return(_self) -> None:
                self._step_return = coerce_step_return_to_torch(
                    self.extract_step_return()
                )
                validate_step_return(
                    self._step_return, num_envs=self.num_envs, device=self.device
                )

            def init_reward_outcome(_self) -> None:
                self._reward_outcome = RewardOutcomeRuntime.from_cfg(
                    getattr(self.cfg, "reward", None),
                    max_episode_steps=self.max_episode_length,
                    is_finite_horizon=self.cfg.is_finite_horizon,
                )
                self._reward_norm = self._reward_outcome.normalization
                reward_norm_description = self._reward_outcome.describe_normalization()
                if reward_norm_description is not None:
                    logging.info(reward_norm_description)

            def preinit_recorders(_self) -> None:
                self._episode_recorder = None
                self._video_recorder = None

            def update_gym_spaces(_self) -> None:
                self._update_gym_env_spaces()

            def init_direct_step_outcome_adapter(_self) -> None:
                self._direct_step_outcome = DirectStepOutcomeAdapter(
                    num_envs=self.num_envs,
                    device=self.device,
                    reward_runtime=self._reward_outcome,
                    observation_builder=self._observation_assembly.assemble,
                    warn_non_finite=self._warn_non_finite_step_return,
                )

            def configure_delay_runtime(_self) -> None:
                self._configure_delay_runtime()

            def init_episode_recorder(_self) -> None:
                from srb.core.sensor.episode_recording import (
                    make_episode_recording_runtime,
                )

                self._episode_recorder = make_episode_recording_runtime(
                    scene=self.scene,
                    cfg=self.cfg,
                    num_envs=self.num_envs,
                    event_provenance=getattr(self.cfg, "event_provenance", None),
                )
                self._video_recorder = self._episode_recorder

        execute_direct_env_finalize_init_plan(plan, _Hooks())

        # Consume startup physics after task setup to preserve initial poses.
        self.sim.step(render=False)
        self.scene.update(dt=self.physics_dt)

    def _cleanup_failed_init(self, original_error: BaseException) -> None:
        # Release simulation resources so construction can be retried.
        if not getattr(self, "_is_closed", True):
            _, close_error = attempt_cleanup(self.close)
            if isinstance(close_error, Exception):
                original_error.add_note(
                    "environment close failed during constructor cleanup: "
                    f"{close_error!r}"
                )
            elif close_error is not None:
                close_error.add_note(
                    "raised while cleaning up after constructor failure: "
                    f"{original_error!r}"
                )
                raise close_error
            return
        try:
            self._release_assembly_physics_parser()
        except Exception as cleanup_error:
            original_error.add_note(
                "assembly PhysX parser release failed during constructor cleanup: "
                f"{cleanup_error!r}"
            )
        except BaseException as interrupt:
            # Preserve interrupts while retaining the constructor error context.
            interrupt.add_note(
                f"raised while cleaning up after constructor failure: {original_error!r}"
            )
            raise

    def _release_assembly_physics_parser(self) -> None:
        release = getattr(self, "_assembly_physics_release", None)
        if release is not None:
            release()
            self._assembly_physics_release = None

    def close(self):
        active_error = sys.exception()
        close_error: BaseException | None = None
        interrupt: BaseException | None = None

        def remember(error: BaseException, operation: str) -> None:
            nonlocal close_error, interrupt
            if not isinstance(error, Exception) and interrupt is None:
                # Finish teardown before propagating interrupts.
                interrupt = error
            elif active_error is not None:
                active_error.add_note(
                    f"{operation} failed during environment cleanup: {error!r}"
                )
            elif close_error is None:
                close_error = error
            else:
                close_error.add_note(
                    f"{operation} also failed during environment cleanup: {error!r}"
                )

        _, parser_error = attempt_cleanup(self._release_assembly_physics_parser)
        if parser_error is not None:
            remember(parser_error, "assembly PhysX parser release")

        # Failed base construction may call close() before allocating this flag.
        if not getattr(self, "_is_closed", True):
            if self.cfg.actions and hasattr(self, "action_manager"):
                del self.action_manager

            recorder = getattr(self, "_episode_recorder", None)
            if recorder is not None:
                try:
                    if hasattr(recorder, "close"):
                        _, recorder_error = attempt_cleanup(recorder.close)
                        if recorder_error is not None:
                            remember(recorder_error, "episode recorder close")
                finally:
                    self._episode_recorder = None
                    self._video_recorder = None

            if getattr(self, "_particle_systems", None):
                self._particle_systems.clear()

            hud = getattr(self, "_hud", None)
            if hud is not None:
                self._hud = None
                # HUD errors are cosmetic; interrupts still propagate after teardown.
                _, hud_error = attempt_cleanup(hud.close)
                if isinstance(hud_error, Exception):
                    logging.warning(f"Failed to close HUD: {hud_error}")
                elif hud_error is not None:
                    remember(hud_error, "HUD close")

            # Run base teardown even when owned-resource cleanup failed.
            _, base_error = attempt_cleanup(super().close)
            if base_error is not None:
                remember(base_error, "base environment close")

        if interrupt is not None:
            if close_error is not None:
                interrupt.add_note(f"environment cleanup also failed: {close_error!r}")
            raise interrupt
        if close_error is not None:
            raise close_error

    def __del__(self):
        _, error = attempt_cleanup(self.close)
        if error is not None:
            logging.warning(f"Failed to close environment during destruction: {error}")

    def reset(
        self,
        seed: int | None = None,
        options: dict | None = None,
    ):
        # Reseed both delay streams for Gym's reset(seed=...) contract.
        if seed is not None:
            if getattr(self, "_delay_runtime", None) is not None:
                self._delay_runtime.manual_seed(int(seed))
            if getattr(self, "_direct_action_delay", None) is not None:
                self._direct_action_delay.manual_seed(int(seed))
        self._last_final_observation = None
        self._last_final_observation_ids = torch.empty(
            0, dtype=torch.long, device=self.device
        )
        self._last_final_observation_mask = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        # Keep metadata through _reset_idx; always clear the outcome afterward.
        self._direct_explicit_reset_pending = True
        try:
            return super().reset(seed=seed, options=options)
        finally:
            self._direct_explicit_reset_pending = False
            direct_step_outcome = getattr(self, "_direct_step_outcome", None)
            if direct_step_outcome is not None and direct_step_outcome.has_outcome:
                direct_step_outcome.clear()

    @property
    def last_final_observation(self):
        observation = getattr(self, "_last_final_observation", None)
        if observation is None:
            return None
        return {key: value.clone() for key, value in observation.items()}

    @property
    def last_final_observation_ids(self):
        value = getattr(self, "_last_final_observation_ids", None)
        if value is None:
            return torch.empty(0, dtype=torch.long, device=self.device)
        return value.clone()

    @property
    def last_final_observation_mask(self):
        value = getattr(self, "_last_final_observation_mask", None)
        if value is None:
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return value.clone()

    @property
    def randomization(self) -> RandomizationView:
        """Queryable view of active terms; reflects later mutations to ``cfg.events``."""
        return RandomizationView(self.cfg.events)

    @property
    def srb_spec(self):
        """Lazily cached SRB validation EnvSpec, separate from Gymnasium's ``env.spec``."""
        return get_cached_srb_spec(self)

    def invalidate_srb_spec(self) -> None:
        """Drop the cached SRB EnvSpec without resetting lifecycle state."""
        invalidate_cached_srb_spec(self)

    @property
    def particle_systems(self) -> Mapping[str, ParticleSystem]:
        """Read-only particle-system registry keyed by scene name."""
        return MappingProxyType(getattr(self, "_particle_systems", {}))

    def iter_particle_systems(self) -> Iterator[tuple[str, ParticleSystem]]:
        """Iterate over registered particle systems in insertion order."""
        return iter(getattr(self, "_particle_systems", {}).items())

    def get_particle_system(
        self,
        name: str,
        *,
        fallback_to_first: bool = False,
    ) -> ParticleSystem | None:
        """Return a particle system by name, optionally falling back to the first."""
        particle_systems = getattr(self, "_particle_systems", {})
        particle_system = particle_systems.get(name)
        if particle_system is not None or not fallback_to_first:
            return particle_system
        return next(iter(particle_systems.values()), None)

    def extract_step_return(self) -> StepReturn:
        """Compute observations, rewards, and termination data for one step."""
        raise NotImplementedError

    def advance_step_state(self) -> None:
        """Advance per-step task state; runs exactly once per environment step.

        Called once before ``extract_step_return`` per step, never on reset.
        Keep mutations here so extraction remains pure and can run again after
        selective reset. Default no-op.
        """

    def _extract_post_reset_step_return(self, reset_ids: torch.Tensor) -> StepReturn:
        """Extract and delay-reseed a fresh return for a completed reset."""

        step_return = coerce_step_return_to_torch(self.extract_step_return())
        # Zero invalid reset rows; a still-invalid state terminates next step.
        step_return, invalid_envs, invalid_keys = scrub_non_finite_step_return(
            step_return, num_envs=self.num_envs, device=self.device
        )
        if invalid_keys and bool(invalid_envs[reset_ids].any()):
            logging.warning(
                "Non-finite post-reset observation for "
                f"{int(invalid_envs[reset_ids].sum().item())} reset environment(s): "
                f"{', '.join(invalid_keys)}"
            )
        delay_runtime = getattr(self, "_delay_runtime", None)
        if delay_runtime is not None:
            step_return = delay_runtime.reseed_reset_observation(reset_ids, step_return)
        return step_return

    def _assemble_post_reset_observation(
        self,
        reset_ids: torch.Tensor,
        *,
        commit_step_return: bool = False,
    ) -> dict[str, torch.Tensor]:
        """Assemble the first observation of the new episode for ``reset_ids``.

        Re-extract after ``_reset_idx`` and refill delay history without
        advancing the step transform twice. Reuse the latest event tick and
        bypass the pre-reset outcome cache; only ``reset_ids`` rows are spliced.
        """
        step_return = self._extract_post_reset_step_return(reset_ids)
        if commit_step_return:
            self._step_return = step_return
        event_observations: dict[str, torch.Tensor] | None = None
        recorder = getattr(self, "_episode_recorder", None)
        if recorder is not None:
            event_observations = recorder.latest_event_observations() or None
        observations = self._observation_assembly.assemble(
            step_return,
            event_observations=event_observations,
        )
        observations.update(self._ephemeris_observations())
        return observations

    def _extract_step_return_wrapped(self) -> StepReturn:
        step_return = coerce_step_return_to_torch(self.extract_step_return())

        if self._delay_runtime is not None:
            step_return = self._delay_runtime.process_observation(
                step_return,
                current_time=self._delay_current_time(),
                step_dt=self._delay_step_dt(),
            )

        return step_return

    @staticmethod
    def _warn_non_finite_step_return(
        invalid_envs: torch.Tensor,
        invalid_keys: Sequence[str],
    ) -> None:
        invalid_count = invalid_envs.sum().item()
        logging.warning(
            f"Terminating {invalid_count} environment instance"
            f"{'s' if invalid_count > 1 else ''} due to non-finite rewards or "
            f"observations: {', '.join(invalid_keys)}"
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        # The legacy hook snapshots ephemeris after reset, so retain its pre-reset value.
        direct_step_outcome = getattr(self, "_direct_step_outcome", None)
        self._pre_reset_ephemeris_observations = (
            self._ephemeris_observations()
            if getattr(self, "_use_step_return_workflow", False)
            and direct_step_outcome is not None
            and direct_step_outcome.has_outcome
            else None
        )

        # Bootstrap resets can precede recorder and delay-runtime initialization.
        episode_recorder = getattr(self, "_episode_recorder", None)
        if episode_recorder is not None:
            # Do not record an episode before its first step.
            recorded_env_ids = [
                int(env_id)
                for env_id in env_ids
                if int(self.episode_length_buf[env_id]) > 0
            ]
            if recorded_env_ids:
                episode_recorder.save_episodes(
                    recorded_env_ids,
                    outcomes=self._episode_recording_outcomes(recorded_env_ids),
                )

        adapter = getattr(self, "_direct_step_outcome", None)
        if adapter is not None:
            adapter.reset_episode_return(env_ids)

        if self.cfg.actions:
            self.action_manager.reset(env_ids)

        with torch.no_grad():
            delay_runtime = getattr(self, "_delay_runtime", None)
            direct_action_delay = getattr(self, "_direct_action_delay", None)
            if delay_runtime is not None:
                delay_runtime.reset(env_ids)
            elif direct_action_delay is not None:
                direct_action_delay.reset(env_ids)
        super()._reset_idx(env_ids)

        # Reset-time writes replay zero targets through delayed actuators; hold defaults.
        with torch.no_grad():
            hold_default_joint_targets(
                self.scene.articulations.values(),
                torch.as_tensor(env_ids, dtype=torch.long, device=self.device),
            )

        self._update_assembly_fixed_joint_transforms(env_ids)
        # Sensors such as frame transformers skip lazy FK; refresh link poses
        # so post-reset observations see the reset state.
        self.sim.forward()

        try:
            n_envs = int(getattr(self, "num_envs", 0))
        except (TypeError, ValueError):
            # Invalid counts cannot identify a global reset; other errors propagate.
            n_envs = 0
        if n_envs > 0 and len(env_ids) >= n_envs:
            mark_srb_spec_reset(self)
            self._hud_last_update_step = -1

    def _episode_recording_outcomes(
        self,
        env_ids: Sequence[int],
    ) -> dict[int, dict[str, object]] | None:
        adapter = getattr(self, "_direct_step_outcome", None)
        if adapter is None or not adapter.has_outcome:
            return None
        return adapter.pop_episode_recording_outcomes(env_ids)

    def _pre_physics_step(self, actions: torch.Tensor):
        validate_finite_action(actions)
        if self.cfg.actions:
            if self._delay_runtime is not None:
                actions = self._delay_runtime.process_action(
                    actions,
                    current_time=self._delay_current_time(),
                    step_dt=self._delay_step_dt(),
                )
                validate_finite_action(actions)
            elif self._direct_action_delay is not None:
                actions = self._direct_action_delay.process(
                    actions,
                    current_time=self._delay_current_time(),
                    step_dt=self._delay_step_dt(),
                    change_freq=self._delay_cfg.action.change_freq,
                    change_prob=self._delay_cfg.action.change_prob,
                )
                validate_finite_action(actions)
            self.action_manager.process_action(actions)
        else:
            super()._pre_physics_step(actions)  # type: ignore[misc]

    def _apply_action(self):
        try:
            if self.cfg.actions:
                self.action_manager.apply_action()
            else:
                super()._apply_action()  # type: ignore[misc]
        except RuntimeError:
            # Continuing would step with stale actions and corrupt the rollout.
            logging.exception("Physics/runtime error in _apply_action")
            raise

    def _update_gym_env_spaces(self):
        self._action_layout = ActionLayout.from_action_manager(
            self.action_manager
            if self.cfg.actions and hasattr(self, "action_manager")
            else None
        )
        self.single_action_space, self.action_space = self._action_layout.build_spaces(
            num_envs=self.num_envs
        )

        if self._use_step_return_workflow:
            self._observation_assembly = ObservationAssembly.from_first_step(
                scene=self.scene,
                step_return=self._step_return,
                num_envs=self.num_envs,
            )
            self._observation_layout = self._observation_assembly.layout
            extra_terms = tuple(
                (term.output_key, term.dtype, term.shape)
                for term in (
                    *self._observation_assembly.visual_terms,
                    *self._observation_assembly.event_terms,
                )
            )
            if self.cfg.ephemeris.enabled and self.cfg.ephemeris.expose_observations:
                extra_terms += (("ephemeris", torch.float32, (7,)),)
            self.single_observation_space, self.observation_space = (
                self._observation_layout.build_spaces(extra_terms=extra_terms)
            )
        else:
            self.single_observation_space = gymnasium.spaces.Dict({})
            for obs_key, obs_buf in self._get_observations().items():
                assert isinstance(obs_buf, (np.ndarray, torch.Tensor))
                self.single_observation_space[obs_key] = box_for_dtype_shape(
                    key=obs_key,
                    dtype=obs_buf.dtype,
                    shape=tuple(obs_buf.shape[1:]),
                )
            from srb.core.env.runtime.observation_assembly import discover_event_terms

            for term in discover_event_terms(self.scene):
                if term.output_key in self.single_observation_space:
                    raise ValueError(
                        f"Event term '{term.output_key}' collides with an "
                        "existing legacy observation space"
                    )
                self.single_observation_space[term.output_key] = box_for_dtype_shape(
                    key=term.output_key,
                    dtype=term.dtype,
                    shape=term.shape,
                )
            self.observation_space = gymnasium.vector.utils.batch_space(
                self.single_observation_space, self.num_envs
            )

    def _configure_delay_runtime(self) -> None:
        actions_enabled = bool(self.cfg.actions and hasattr(self, "action_manager"))
        action_dim = int(self.action_manager.total_action_dim) if actions_enabled else 0

        delay_plan = plan_delay_runtime_activation(
            self.cfg,
            step_return_workflow=self._use_step_return_workflow,
            actions_enabled=actions_enabled,
            action_dim=action_dim,
        )
        self._delay_cfg = delay_plan.delay_config
        self._delay_runtime: DelayRuntime | None = delay_plan.build_step_return_runtime(
            self.cfg,
            getattr(self, "_step_return", None),
            action_dim=action_dim,
            num_envs=self.num_envs,
            device=self.device,
            actions_enabled=actions_enabled,
        )
        self._direct_action_delay: ActionDelay | None = (
            delay_plan.build_legacy_action_delay(
                action_dim=action_dim,
                num_envs=self.num_envs,
                device=self.device,
            )
        )
        delay_plan.emit_diagnostics(logging)

    def _delay_current_time(self) -> float:
        current_time = getattr(getattr(self, "sim", None), "current_time", None)
        if current_time is not None:
            return float(current_time)
        step_counter = int(getattr(self, "common_step_counter", 0))
        step_dt = float(getattr(self, "step_dt", self._delay_step_dt()))
        return step_counter * step_dt

    def _delay_step_dt(self) -> float:
        return float(getattr(self.cfg, "agent_rate", 1.0))

    @cached_property
    def max_episode_length(self):
        return super().max_episode_length

    @cached_property
    def _use_step_return_workflow(self) -> bool:
        return self.__class__.extract_step_return is not DirectEnv.extract_step_return

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        if self._use_step_return_workflow:
            self.advance_step_state()
            outcome = self._direct_step_outcome.begin_step(
                self._extract_step_return_wrapped(),
                include_reward_terms=bool(self.cfg.extras),
            )
            self._step_return = outcome.step_return
            # Extras persist between hooks; clear prior-step diagnostics.
            self.extras.pop("invalid_keys", None)
            self.extras.pop("invalid_envs", None)
            if outcome.extras:
                self.extras.update(outcome.extras)
            # Snapshot terminal sensors before selective reset; reuse this event tick after reset.
            event_observations: dict[str, torch.Tensor] | None = None
            if self._episode_recorder is not None:
                sim_time_ns = int(max(0.0, self._delay_current_time()) * 1_000_000_000)
                self._episode_recorder.record_step(sim_time_ns=sim_time_ns)
                event_observations = (
                    self._episode_recorder.latest_event_observations() or None
                )
            self._direct_pre_reset_observation = self._direct_step_outcome.observation(
                event_observations=event_observations
            )
            self._direct_pre_reset_observation.update(self._ephemeris_observations())
            return self._direct_step_outcome.dones()
        else:
            return super()._get_dones()  # type: ignore[return-value]

    def _get_rewards(self) -> torch.Tensor:
        if self._use_step_return_workflow:
            reward = self._direct_step_outcome.reward()
        else:
            reward = super()._get_rewards()  # type: ignore[assignment]
        self._update_hud(reward)
        return reward  # type: ignore[return-value]

    def _hud_custom_lines(self, reward: torch.Tensor) -> Sequence[tuple[str, str]]:
        """Return task-specific HUD lines, shown before generic metrics."""
        return ()

    def _hud_lines(self, reward: torch.Tensor) -> Sequence[tuple[str, str]]:
        lines: list[tuple[str, str]] = list(self._hud_custom_lines(reward))

        if self.cfg.hud_show_reward:
            lines.append(("Reward", f"{_tensor_first_float(reward):+.3f}"))

        if self.cfg.hud_show_time:
            elapsed_s = _tensor_first_float(self.episode_length_buf) * float(
                self.cfg.agent_rate
            )
            lines.append(("Time", _format_elapsed_s(elapsed_s)))

        if self.cfg.hud_show_reset_hint:
            lines.append(("Reset", self.cfg.hud_reset_hint))

        return lines[: max(1, int(self.cfg.hud_max_lines))]

    def _update_hud(self, reward: torch.Tensor) -> None:
        if not getattr(self.cfg, "hud", False):
            return

        step = int(_tensor_first_float(self.episode_length_buf))
        interval_steps = max(
            1,
            round(float(self.cfg.hud_update_interval_s) / float(self.cfg.agent_rate)),
        )
        if self._hud_last_update_step == step or step % interval_steps != 0:
            return
        self._hud_last_update_step = step

        if self._hud is None:
            from srb.core.env.common.hud import DemoHud

            self._hud = DemoHud(
                title=self.cfg.hud_title,
                max_lines=self.cfg.hud_max_lines,
            )
        if self._hud is not None and self._hud.enabled:
            self._hud.update(self._hud_lines(reward))

    def _get_sensor_observations(self) -> dict[str, torch.Tensor]:
        """Capture visual sensors for the legacy non-step-return workflow."""
        from srb.core.sensor.visual_output import (
            construct_visual_observation,
            iter_visual_sensor_outputs,
        )

        obs: dict[str, torch.Tensor] = {}
        for visual in iter_visual_sensor_outputs(self.scene):
            obs.update(
                construct_visual_observation(
                    image_basename=visual.image_basename,
                    data_types=visual.data_types,
                    clipping_range=visual.clipping_range,
                    merge_channels=True,
                    **visual.sensor.data.output,
                )
            )
        return obs

    def _get_observations(self) -> dict[str, torch.Tensor]:
        event_observations: dict[str, torch.Tensor] | None = None
        # Consume once so explicit resets and unmatched hooks cannot reuse stale state.
        pre_reset_observation = getattr(self, "_direct_pre_reset_observation", None)
        self._direct_pre_reset_observation = None
        if getattr(self, "_direct_explicit_reset_pending", False):
            pre_reset_observation = None
        if self._episode_recorder is not None and pre_reset_observation is None:
            sim_time_ns = int(max(0.0, self._delay_current_time()) * 1_000_000_000)
            if (
                getattr(self, "_direct_explicit_reset_pending", False)
                and self.common_step_counter > 0
            ):
                # Resets do not advance sim time; record_step would skip it.
                self._episode_recorder.prime_envs(
                    range(self.num_envs), sim_time_ns=sim_time_ns
                )
            else:
                self._episode_recorder.record_step(sim_time_ns=sim_time_ns)
            event_observations = (
                self._episode_recorder.latest_event_observations() or None
            )

        if self._use_step_return_workflow:
            if getattr(self, "_direct_explicit_reset_pending", False):
                # Consume before extraction so repeated reset hooks cannot re-extract.
                self._direct_explicit_reset_pending = False
                direct_step_outcome = getattr(self, "_direct_step_outcome", None)
                if direct_step_outcome is not None and direct_step_outcome.has_outcome:
                    # Reset metadata is consumed; clear cached terminal rows before assembly.
                    direct_step_outcome.clear()
                reset_ids = torch.arange(
                    self.num_envs, dtype=torch.long, device=self.device
                )
                obs = self._assemble_post_reset_observation(
                    reset_ids, commit_step_return=True
                )
            elif (
                getattr(self, "_direct_step_outcome", None) is not None
                and self._direct_step_outcome.has_outcome
            ):
                obs = pre_reset_observation
                if obs is None:
                    obs = self._direct_step_outcome.observation(
                        event_observations=event_observations,
                    )
                self._direct_pre_reset_observation = None
                # Splicing requires matching keys; merge ephemeris before snapshot/splice.
                if pre_reset_observation is None:
                    obs.update(self._ephemeris_observations())
                reset_buf = getattr(self, "reset_buf", None)
                reset_terminated = getattr(self, "reset_terminated", None)
                if isinstance(reset_buf, torch.Tensor) and isinstance(
                    reset_terminated, torch.Tensor
                ):
                    # Visualizer-requested resets mark only ``reset_terminated``.
                    reset_buf = reset_buf | reset_terminated
                if isinstance(reset_buf, torch.Tensor) and bool(reset_buf.any()):
                    reset_ids = reset_buf.nonzero(as_tuple=False).flatten()
                    # Terminal rows use ephemeris captured by ``_reset_idx``.
                    terminal_observation = {
                        **obs,
                        **(
                            getattr(self, "_pre_reset_ephemeris_observations", None)
                            or {}
                        ),
                    }
                    # Match Isaac Lab's post-hook policy noise in terminal snapshots.
                    noise_model = getattr(self, "_observation_noise_model", None)
                    if (
                        getattr(self.cfg, "observation_noise_model", None)
                        and noise_model is not None
                        and "policy" in terminal_observation
                    ):
                        terminal_observation["policy"] = noise_model(
                            terminal_observation["policy"]
                        )
                    self._last_final_observation = final_observation(
                        terminal_observation, reset_ids
                    )
                    self._last_final_observation_ids = reset_ids.clone()
                    self._last_final_observation_mask = reset_buf.clone()
                else:
                    self._last_final_observation = None
                    self._last_final_observation_ids = torch.empty(
                        0, dtype=torch.long, device=self.device
                    )
                    self._last_final_observation_mask = torch.zeros(
                        self.num_envs, dtype=torch.bool, device=self.device
                    )
                if isinstance(reset_buf, torch.Tensor) and bool(reset_buf.any()):
                    if self._episode_recorder is not None:
                        # The terminal tick already consumed this sim time.
                        self._episode_recorder.prime_envs(
                            reset_ids.tolist(),
                            sim_time_ns=int(
                                max(0.0, self._delay_current_time()) * 1_000_000_000
                            ),
                        )
                    obs = splice_observation_rows(
                        obs,
                        self._assemble_post_reset_observation(reset_ids),
                        reset_ids,
                    )
            else:
                obs = self._observation_assembly.assemble(
                    self._step_return,
                    event_observations=event_observations,
                )
        else:
            obs = super()._get_observations()  # type: ignore[return-value]
            obs.update(self._get_sensor_observations())
            if event_observations:
                obs.update(event_observations)
        # Merge after assembly; the matching Gym space is added during setup.
        obs.update(self._ephemeris_observations())
        return obs

    def _ephemeris_observations(self) -> dict[str, torch.Tensor]:
        """Return configured ephemeris observations, zeroing missing or non-finite
        values.
        """
        ephemeris_cfg = getattr(self.cfg, "ephemeris", None)
        if ephemeris_cfg is None or not (
            ephemeris_cfg.enabled and ephemeris_cfg.expose_observations
        ):
            return {}
        from srb.core.ephemeris.observations import ephemeris_observation

        return {
            "ephemeris": torch.nan_to_num(
                ephemeris_observation(
                    getattr(self, "_ephemeris_state", None),
                    getattr(self, "_ephemeris_occluder", None),
                    float(getattr(self, "_ephemeris_eclipse_factor", 1.0)),
                    self._robot.data.root_quat_w,
                ),
                nan=0.0,
                posinf=0.0,
                neginf=0.0,
            )
        }

    def _setup_scene(self):
        super()._setup_scene()

        # Cloning copies env_0 particle targets to other envs; rebind before parsing.
        env_prim_paths = getattr(self.scene, "env_prim_paths", None)
        if env_prim_paths:
            from srb.core.particle_binding import (
                repoint_particle_sets_to_env_systems,
            )

            repoint_particle_sets_to_env_systems(self.sim.stage, env_prim_paths)

        # Rendering writes absolute poses as USD locals, so reset stacks to prevent compounding.
        # Isaac Sim 6.1 Fabric crashes on these stacks with multiple articulated robots.
        if not getattr(self.cfg.sim, "use_fabric", True) and (
            self.sim.is_rendering or self.sim.can_render_rgb_array()
        ):
            from srb.core.sim.absolute_body_poses import (
                make_rigid_body_poses_absolute,
            )

            make_rigid_body_poses_absolute(self.sim.stage)

        # Static decor retains USD joints that PhysX cannot connect; disable before parsing.
        from srb.core.sim.static_decor import disable_joints_on_static_decor

        env_regex_ns = f"{getattr(self.scene, 'env_ns', '/World/envs')}/env_.*"
        disable_joints_on_static_decor(self.cfg.scene, env_regex_ns)

        from srb.core.sim.light_exposure import (
            apply_light_exposure_overrides,
            light_exposure_overrides,
        )

        apply_light_exposure_overrides(light_exposure_overrides(self.cfg), env_regex_ns)

        self.joint_assemblies: dict[str, Sequence["AssembledBodies"]] = {}
        if not self.cfg.joint_assemblies:
            return

        from srb.core.sim.robot_setup import RobotAssembler

        for key, assembly_cfg in self.cfg.joint_assemblies.items():
            self.joint_assemblies[key] = tuple(
                RobotAssembler().assemble_rigid_bodies(
                    assembly_cfg.model_copy(
                        update={
                            "base_path": resolve_env_prim_path(
                                assembly_cfg.base_path, i
                            ),
                            "attach_path": resolve_env_prim_path(
                                assembly_cfg.attach_path, i
                            ),
                        }
                    )
                )
                for i in range(self.num_envs)
            )

        if self.num_envs > 1:
            from srb.core.sim.robot_setup.robot_assembler import (
                prepare_assembled_scene_physics,
            )

            self._assembly_physics_release = prepare_assembled_scene_physics(
                self.sim.stage
            )

    def _update_assembly_fixed_joint_transforms(self, env_ids: Sequence[int]):
        for key, assembly_cfg in self.cfg.joint_assemblies.items():
            attach_asset: RigidObject | Articulation = get_scene_root_asset(
                self.scene, key
            )
            base_asset: RigidObject | Articulation = get_scene_root_asset(
                self.scene,
                assembly_cfg.base_path.rsplit("/", 1)[-1],
            )
            attach_root_state = attach_asset.data.root_state_w[env_ids]
            attach_body_state = (
                attach_asset.data.body_state_w[
                    env_ids,
                    attach_asset.find_bodies(
                        assembly_cfg.attach_mount_frame.removeprefix("/")
                    )[0][0],
                ]
                if assembly_cfg.attach_mount_frame
                else attach_root_state
            )
            base_body_state = (
                base_asset.data.body_state_w[
                    env_ids,
                    base_asset.find_bodies(
                        assembly_cfg.base_mount_frame.removeprefix("/")
                    )[0][0],
                ]
                if assembly_cfg.base_mount_frame
                else base_asset.data.root_state_w[env_ids]
            )

            # attach_root = base_mount * offset * (attach_mount^-1 * attach_root);
            # multiplication order matters when the attach mount is rotated.
            pose = torch.cat(
                combine_frame_transforms(
                    *combine_frame_transforms(
                        base_body_state[:, :3],
                        base_body_state[:, 3:7],
                        torch.tensor(
                            assembly_cfg.fixed_joint_offset,
                            dtype=attach_root_state.dtype,
                            device=attach_root_state.device,
                        )
                        .unsqueeze(0)
                        .repeat(attach_root_state.shape[0], 1),
                        torch.tensor(
                            assembly_cfg.fixed_joint_orient,
                            dtype=attach_root_state.dtype,
                            device=attach_root_state.device,
                        )
                        .unsqueeze(0)
                        .repeat(attach_root_state.shape[0], 1),
                    ),
                    *subtract_frame_transforms(
                        attach_body_state[:, :3],
                        attach_body_state[:, 3:7],
                        attach_root_state[:, :3],
                        attach_root_state[:, 3:7],
                    ),
                ),
                dim=1,
            )

            attach_asset.write_root_pose_to_sim(pose, env_ids=env_ids)


def _assert_gpu_dynamics_enabled(feature: str) -> None:
    """Require GPU dynamics or raise ``RuntimeError``; scenes without its API are
    unchecked. Particle systems stay inert without the scene flag.
    """
    import omni.usd
    from pxr import PhysxSchema, UsdPhysics

    context = omni.usd.get_context()
    stage = context.get_stage() if context is not None else None
    if stage is None:
        return

    disabled = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Scene):
            continue
        attr = PhysxSchema.PhysxSceneAPI(prim).GetEnableGPUDynamicsAttr()
        # Get() includes the false schema default; authored-value checks miss
        # scenes that silently run without GPU dynamics.
        if attr and not attr.Get():
            disabled.append(str(prim.GetPath()))

    if disabled:
        raise RuntimeError(
            f"{feature} require PhysX GPU dynamics, but it is disabled on "
            f"{', '.join(disabled)}. The scene would run with the "
            f"{feature} present and never simulated. Enable "
            "physxScene:enableGPUDynamics on the simulation owner, or "
            "configure the environment without them."
        )


def _tensor_first_float(value: torch.Tensor) -> float:
    return float(value.reshape(-1)[0].detach().cpu().item())


def _format_elapsed_s(elapsed_s: float) -> str:
    if elapsed_s < 60.0:
        return f"{elapsed_s:0.1f}s"
    minutes = int(elapsed_s // 60.0)
    seconds = int(elapsed_s % 60.0)
    return f"{minutes:d}:{seconds:02d}"
