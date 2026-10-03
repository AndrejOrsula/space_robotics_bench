from __future__ import annotations

import functools
import logging
import math
from typing import Any

import gymnasium
import numpy as np
import torch
from gymnasium.utils import seeding
from gymnasium.vector import AutoresetMode

from srb._typing.step_return import StepReturn
from srb.core.env.runtime.action_layout import ActionLayout, validate_finite_action
from srb.core.env.runtime.delay import (
    DelayRuntime,
    plan_delay_runtime_activation,
    resolve_delay_step_dt,
)
from srb.core.env.runtime.lifecycle import RuntimeLifecycle, SrbEnvAdapter
from srb.core.env.runtime.observation_layout import ObservationLayout
from srb.core.env.runtime.step_return import (
    coerce_step_return_to_torch,
    validate_step_return,
)
from srb.core.reward.outcome import RewardOutcomeRuntime
from srb.core.spec.lifecycle import (
    get_cached_srb_spec,
    init_srb_spec_state,
    invalidate_cached_srb_spec,
    mark_srb_spec_reset,
)


class SrbEnv(gymnasium.Env):
    """Minimal owned SRB vector-style Gymnasium environment.

    This class intentionally has no IsaacLab dependency. Concrete subclasses
    provide runtime state directly and implement ``extract_step_return()``.
    """

    is_srb_env = True
    is_vector_env = True
    metadata = {
        "render_modes": [None, "human", "rgb_array"],
        "autoreset_mode": AutoresetMode.SAME_STEP,
    }

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        # DirectEnv wraps production constructors. Wrap other subclasses so
        # finalization runs after leaf initialization, without importing Isaac Lab.
        if getattr(cls, "_srb_direct_runtime", False):
            return
        if "__init__" not in cls.__dict__:
            return
        original_init = cls.__dict__["__init__"]
        if getattr(original_init, "_srb_env_wrapped", False):
            return

        @functools.wraps(original_init)
        def _wrapped(self, *args: Any, __orig=original_init, **kwargs: Any) -> None:
            depth = getattr(self, "_srb_init_depth", 0)
            self._srb_init_depth = depth + 1
            try:
                __orig(self, *args, **kwargs)
            finally:
                self._srb_init_depth -= 1
            if self._srb_init_depth == 0:
                self._finalize_init()

        _wrapped._srb_env_wrapped = True  # type: ignore[attr-defined]
        cls.__init__ = _wrapped  # type: ignore[method-assign]

    def __init__(self, cfg: Any | None = None, **kwargs: Any) -> None:
        runtime_mode = kwargs.pop("runtime_mode", None) or getattr(
            cfg, "runtime_mode", None
        )
        # DirectRLEnv.__init__ calls seed(cfg.seed); set the mode first so it
        # dispatches through Isaac's production seeding implementation.
        self._srb_runtime_mode = runtime_mode
        # Keep Isaac's render_fps metadata write local and preserve SRB's
        # same-step marker for Gymnasium vector wrappers.
        metadata: dict[str, Any] = {}
        for base in reversed(type(self).__mro__):
            base_metadata = base.__dict__.get("metadata")
            if isinstance(base_metadata, dict):
                metadata.update(base_metadata)
        self.metadata = metadata
        if runtime_mode in {"isaac", "legacy"}:
            if cfg is None:
                raise ValueError("an Isaac runtime requires a configuration")
            # DirectEnv owns production initialization of extras, counters,
            # buffers, simulator, and scene.
            super().__init__(cfg, **kwargs)
            return

        super().__init__()
        self.cfg = cfg
        self.render_mode = kwargs.get("render_mode")
        self.scene = kwargs.get("scene", getattr(cfg, "runtime_scene", None))
        if self.scene is None:
            self.scene = getattr(cfg, "scene", None)
        self.sim = kwargs.get("sim", getattr(cfg, "sim", None))

        self.device = self._resolve_device(kwargs.get("device"))
        self.num_envs = self._resolve_num_envs(kwargs.get("num_envs"))
        self.step_dt = self._resolve_step_dt()
        self._max_episode_length = self._resolve_max_episode_length()

        self.extras: dict[str, Any] = {}
        self.common_step_counter = 0
        self._sim_step_counter = int(getattr(self, "_sim_step_counter", 0))
        self.episode_length_buf = torch.zeros(
            self.num_envs, dtype=torch.long, device=self.device
        )
        self.reset_buf = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        init_srb_spec_state(self)
        self._last_final_observation = None
        self._last_final_observation_ids = torch.empty(
            0, dtype=torch.long, device=self.device
        )
        self._last_final_observation_mask = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        self._owned_runtime_initialized = False
        self._current_time = 0.0
        self._is_closed = False
        self._delay_runtime: DelayRuntime | None = None
        self._action_layout: ActionLayout | None = None
        self._observation_layout: ObservationLayout | None = None
        self._owned_lifecycle: RuntimeLifecycle | None = None
        self._reward_outcome: RewardOutcomeRuntime | None = None

        action_manager = kwargs.get("action_manager")
        if action_manager is not None:
            self.action_manager = action_manager
        elif getattr(cfg, "actions", False):
            self.action_manager = self._make_action_manager(getattr(cfg, "actions"))

        self._configure_scene_asset_aliases()

    def _setup_owned_runtime(self) -> None:
        if getattr(self, "_owned_runtime_initialized", False):
            return

        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            # Reuse DirectEnv's initialized assembly, outcome adapter,
            # recorder, reward runtime, spaces, and delay runtime. Rebuilding
            # them would split the production path and consume the outcome twice.
            step_return = getattr(self, "_step_return", None)
            assembly = getattr(self, "_observation_assembly", None)
            direct_step_outcome = getattr(self, "_direct_step_outcome", None)
            if step_return is None or assembly is None:
                raise RuntimeError(
                    "owned production lifecycle requires DirectEnv finalization"
                )
            if direct_step_outcome is None:
                raise RuntimeError(
                    "owned production lifecycle requires DirectStepOutcomeAdapter"
                )
            validate_step_return(
                step_return, num_envs=self.num_envs, device=self.device
            )
            action_layout = getattr(self, "_action_layout", None)
            if not isinstance(action_layout, ActionLayout):
                action_layout = ActionLayout.from_action_manager(
                    getattr(self, "action_manager", None)
                )
                self._action_layout = action_layout
            self._observation_layout = assembly.layout
            self._last_final_observation = getattr(
                self, "_last_final_observation", None
            )
            self._owned_lifecycle = RuntimeLifecycle(
                SrbEnvAdapter(self),
                reward_runtime=getattr(self, "_reward_outcome", None),
            )
            self._owned_runtime_initialized = True
            return

        if not hasattr(self, "extras"):
            self.extras = {}
        if not hasattr(self, "common_step_counter"):
            self.common_step_counter = 0
        if not hasattr(self, "step_dt"):
            self.step_dt = 1.0
        if not hasattr(self, "_max_episode_length"):
            self._max_episode_length = 1
        if not hasattr(self, "episode_length_buf"):
            self.episode_length_buf = torch.zeros(
                self.num_envs, dtype=torch.long, device=self.device
            )
        if not hasattr(self, "reset_buf"):
            self.reset_buf = torch.zeros(
                self.num_envs, dtype=torch.bool, device=self.device
            )
        init_srb_spec_state(self)
        if not hasattr(self, "_last_final_observation"):
            self._last_final_observation = None
        if not hasattr(self, "_last_final_observation_ids"):
            self._last_final_observation_ids = torch.empty(
                0, dtype=torch.long, device=self.device
            )
        if not hasattr(self, "_last_final_observation_mask"):
            self._last_final_observation_mask = torch.zeros(
                self.num_envs, dtype=torch.bool, device=self.device
            )
        if not hasattr(self, "_current_time"):
            self._current_time = 0.0
        if not hasattr(self, "_delay_runtime"):
            self._delay_runtime = None
        if not hasattr(self, "_action_layout"):
            self._action_layout = None
        if not hasattr(self, "_observation_layout"):
            self._observation_layout = None
        if not hasattr(self, "_reward_outcome"):
            self._reward_outcome = None

        step_return = self._extract_step_return_owned()
        validate_step_return(step_return, num_envs=self.num_envs, device=self.device)
        self._step_return = step_return
        self._observation_layout = ObservationLayout.from_step_return(
            step_return,
            num_envs=self.num_envs,
        )

        reward_cfg = getattr(getattr(self, "cfg", None), "reward", None)
        if reward_cfg is not None and hasattr(reward_cfg, "into_normalization_cfg"):
            self._reward_outcome = RewardOutcomeRuntime.from_cfg(
                reward_cfg,
                max_episode_steps=self.max_episode_length,
                is_finite_horizon=bool(
                    getattr(getattr(self, "cfg", None), "is_finite_horizon", False)
                ),
            )

        self._action_layout = ActionLayout.from_action_manager(
            getattr(self, "action_manager", None)
        )
        self.single_action_space, self.action_space = self._action_layout.build_spaces(
            num_envs=self.num_envs
        )
        self.single_observation_space, self.observation_space = (
            self._observation_layout.build_spaces()
        )
        self._configure_delay_runtime(step_return)
        self._owned_lifecycle = RuntimeLifecycle(
            SrbEnvAdapter(self),
            reward_runtime=self._reward_outcome,
        )
        self._owned_runtime_initialized = True

    def _finalize_init(self) -> None:
        if getattr(self, "_srb_env_finalized", False):
            return
        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            # Task reaches this hook through DirectEnv's constructor wrapper.
            # Run its finalizer first, then initialize the owned transaction.
            super()._finalize_init()
        self._setup_owned_runtime()
        self._srb_env_finalized = True

    def extract_step_return(self) -> StepReturn:
        raise NotImplementedError

    @property
    def last_final_observation(self) -> dict[str, torch.Tensor] | None:
        observation = getattr(self, "_last_final_observation", None)
        if observation is None:
            return None
        return {key: value.clone() for key, value in observation.items()}

    @property
    def last_final_observation_ids(self) -> torch.Tensor:
        value = getattr(self, "_last_final_observation_ids", None)
        if value is None:
            return torch.empty(0, dtype=torch.long, device=self.device)
        return value.clone()

    @property
    def last_final_observation_mask(self) -> torch.Tensor:
        value = getattr(self, "_last_final_observation_mask", None)
        if value is None:
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return value.clone()

    def advance_step_state(self) -> None:
        """Update task state once before each step's ``extract_step_return``.

        Keep step mutations here so extraction stays pure and can produce the
        first observation after a selective reset. The default is a no-op.
        """

    def _mark_owned_reset(self) -> None:
        mark_srb_spec_reset(self)

    def _prepare_owned_reset(self, env_ids: torch.Tensor) -> None:
        """Clear DirectEnv's cached outcome after a complete owned reset."""

        if (
            getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}
            and env_ids.numel() == self.num_envs
        ):
            direct_step_outcome = getattr(self, "_direct_step_outcome", None)
            if direct_step_outcome is not None:
                direct_step_outcome.clear()

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            # Preserve DirectEnv's production semantics, including its
            # ``cfg.actions=False`` fallback to Isaac's hook.
            super()._pre_physics_step(actions)
            return

        if getattr(getattr(self, "cfg", None), "actions", False):
            validate_finite_action(actions)
            action_manager = getattr(self, "action_manager", None)
            if action_manager is None:
                raise RuntimeError(
                    "configured synthetic actions require an initialized action_manager"
                )
            if self._delay_runtime is not None:
                actions = self._delay_runtime.process_action(
                    actions,
                    current_time=float(getattr(self, "_current_time", 0.0)),
                    # Delay quantum is cfg.agent_rate, not env.step_dt.
                    step_dt=resolve_delay_step_dt(self),
                )
                validate_finite_action(actions)
            action_manager.process_action(actions)

    def _apply_action(self) -> None:
        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            super()._apply_action()
            return

        action_manager = getattr(self, "action_manager", None)
        if getattr(getattr(self, "cfg", None), "actions", False):
            if action_manager is None:
                raise RuntimeError(
                    "configured synthetic actions require an initialized action_manager"
                )
            action_manager.apply_action()

    def _reset_idx(self, env_ids) -> None:
        env_ids = self._normalize_env_ids(env_ids)
        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            # DirectEnv performs recorder/outcome, action/delay, scene/event,
            # assembly/buffer, HUD, and spec work before delegating to Isaac.
            super()._reset_idx(env_ids)
            return

        action_manager = getattr(self, "action_manager", None)
        if getattr(getattr(self, "cfg", None), "actions", False):
            if action_manager is None:
                raise RuntimeError(
                    "configured synthetic actions require an initialized action_manager"
                )
            action_manager.reset(env_ids)
        if self._delay_runtime is not None:
            self._delay_runtime.reset(env_ids)

        if hasattr(self, "episode_length_buf"):
            self.episode_length_buf[env_ids] = 0

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ):
        self._setup_owned_runtime()
        if self._owned_lifecycle is None:
            raise RuntimeError("owned lifecycle was not initialized")
        outcome = self._owned_lifecycle.reset(seed=seed, options=options)
        # Preserve DirectRLEnv's persistent extras mapping; reset adds no info.
        return outcome.observation, self.extras

    def step(self, actions):
        self._setup_owned_runtime()
        if self._owned_lifecycle is None:
            raise RuntimeError("owned lifecycle was not initialized")
        outcome = self._owned_lifecycle.step(actions)
        # The step transaction already merged reward, episode, and StepReturn
        # info into the persistent extras mapping returned by DirectRLEnv.
        return (
            outcome.observation,
            outcome.reward,
            outcome.terminated,
            outcome.truncated,
            self.extras,
        )

    def render(self, recompute: bool = False):
        self._setup_owned_runtime()
        if self._owned_lifecycle is None:
            raise RuntimeError("owned lifecycle was not initialized")
        return self._owned_lifecycle.render(recompute=recompute)

    @property
    def max_episode_length(self) -> int:
        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            return int(super().max_episode_length)
        return self._max_episode_length

    @property
    def randomization(self):
        cfg = getattr(self, "cfg", None)
        if cfg is None or not hasattr(cfg, "events"):
            raise RuntimeError("SrbEnv.randomization requires cfg.events")
        from srb.core.randomization import RandomizationView

        return RandomizationView(cfg.events)

    @property
    def srb_spec(self):
        return get_cached_srb_spec(self)

    def invalidate_srb_spec(self) -> None:
        invalidate_cached_srb_spec(self)

    def seed(self, seed: int = -1) -> int:
        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            # DirectRLEnv also seeds replicator and calls configure_seed.
            seed = int(super().seed(seed))
        else:
            if seed < 0:
                # Isaac treats negative seeds as requests for a drawn seed.
                seed = int(torch.seed() % (2**31 - 1))
            self.np_random, seed = seeding.np_random(seed)
            torch.manual_seed(seed)
        for name in ("_ephemeris_rng", "_illumination_rng"):
            setattr(self, name, np.random.default_rng(seed))
        # Seed delay generators for direct callers and drawn seeds alike.
        for name in ("_delay_runtime", "_direct_action_delay"):
            delay = getattr(self, name, None)
            if delay is not None:
                delay.manual_seed(seed)
        seen_spaces: set[int] = set()
        for name in (
            "action_space",
            "observation_space",
            "single_action_space",
            "single_observation_space",
        ):
            space = getattr(self, name, None)
            if space is not None and id(space) not in seen_spaces:
                space.seed(seed)
                seen_spaces.add(id(space))
        return seed

    def _extract_step_return_owned(self) -> StepReturn:
        step_return = self.extract_step_return()
        if not isinstance(step_return, StepReturn):
            step_return = StepReturn.from_tuple(step_return)
        # Do not move tensors here: downstream validation must report any
        # host-resident component by name, matching DirectEnv initialization.
        return coerce_step_return_to_torch(step_return)

    def _resolve_device(self, explicit_device: Any | None) -> torch.device:
        if explicit_device is not None:
            return torch.device(explicit_device)
        cfg = getattr(self, "cfg", None)
        cfg_sim_device = getattr(getattr(self, "sim", None), "device", None)
        if cfg_sim_device is not None:
            return torch.device(cfg_sim_device)
        cfg_device = getattr(cfg, "device", None)
        if cfg_device is not None:
            return torch.device(cfg_device)
        return torch.device("cpu")

    def _resolve_num_envs(self, explicit_num_envs: Any | None) -> int:
        candidates = (
            explicit_num_envs,
            getattr(self.scene, "num_envs", None),
            getattr(getattr(self, "cfg", None), "num_envs", None),
        )
        for candidate in candidates:
            if candidate is None:
                continue
            num_envs = int(candidate)
            if num_envs <= 0:
                raise ValueError(f"num_envs must be positive, got {candidate!r}")
            return num_envs
        return 1

    def _resolve_step_dt(self) -> float:
        cfg = getattr(self, "cfg", None)
        if hasattr(cfg, "agent_rate"):
            return float(cfg.agent_rate)
        if hasattr(cfg, "step_dt"):
            return float(cfg.step_dt)
        sim_dt = getattr(getattr(cfg, "sim", None), "dt", None)
        decimation = getattr(cfg, "decimation", None)
        if sim_dt is not None and decimation is not None:
            return float(sim_dt) * float(decimation)
        return 1.0

    def _resolve_max_episode_length(self) -> int:
        cfg = getattr(self, "cfg", None)
        if hasattr(cfg, "max_episode_length"):
            return max(1, int(cfg.max_episode_length))
        episode_length_s = getattr(cfg, "episode_length_s", None)
        if episode_length_s is None:
            return 1
        return max(1, int(math.ceil(float(episode_length_s) / self.step_dt)))

    def _make_action_manager(self, actions_cfg: Any) -> Any:
        try:
            from srb.core.manager import ActionManager
        except (ImportError, ModuleNotFoundError) as exc:
            exc_name = getattr(exc, "name", None)
            if (
                exc_name is None
                or exc_name.startswith("isaaclab")
                or "ActionManager" in str(exc)
            ):
                raise RuntimeError(
                    "SrbEnv requires an injected action_manager when cfg.actions "
                    "is enabled outside an IsaacLab runtime."
                ) from exc
            raise

        return ActionManager(actions_cfg, env=self)

    def _configure_delay_runtime(self, step_return: StepReturn | None = None) -> None:
        if getattr(self, "_srb_runtime_mode", None) in {"isaac", "legacy"}:
            # DirectEnv._finalize_init invokes this hook with no arguments.
            # Keep its proven delay/action configuration intact in production.
            super()._configure_delay_runtime()
            return

        cfg = getattr(self, "cfg", None)
        if step_return is None:
            step_return = getattr(self, "_step_return", None)
        if step_return is None:
            raise RuntimeError("synthetic delay setup requires an initial StepReturn")

        action_manager = getattr(self, "action_manager", None)
        action_dim = int(getattr(action_manager, "total_action_dim", 0))
        delay_plan = plan_delay_runtime_activation(
            cfg,
            step_return_workflow=True,
            actions_enabled=bool(
                getattr(cfg, "actions", False) and action_manager is not None
            ),
            action_dim=action_dim,
        )
        self._delay_runtime = delay_plan.build_step_return_runtime(
            cfg,
            step_return,
            action_dim=action_dim,
            num_envs=self.num_envs,
            device=self.device,
            actions_enabled=bool(
                getattr(cfg, "actions", False) and action_manager is not None
            ),
        )
        delay_plan.emit_diagnostics(logging.getLogger("srb"))

    def _configure_scene_asset_aliases(self) -> None:
        scene = getattr(self, "scene", None)
        cfg = getattr(self, "cfg", None)
        self._scenery = None
        if scene is not None and getattr(cfg, "scenery", None) is not None:
            self._scenery = getattr(scene, "terrain", None) or getattr(
                scene, "scenery", None
            )

        robot = getattr(scene, "robot", None) if scene is not None else None
        self._robot = robot
        self._robots: dict[str, Any] = {}
        if robot is not None:
            self._robots["robot"] = robot
        for robot_name in getattr(cfg, "_robots", {}) or {}:
            robot_asset = (
                getattr(scene, robot_name, None) if scene is not None else None
            )
            if robot_asset is not None:
                self._robots[robot_name] = robot_asset

    def _normalize_env_ids(self, env_ids: Any) -> torch.Tensor:
        if env_ids is None:
            return torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        if isinstance(env_ids, torch.Tensor):
            if env_ids.dtype == torch.bool:
                if env_ids.shape != (self.num_envs,):
                    raise ValueError(
                        f"boolean env_ids mask must have shape {(self.num_envs,)}, "
                        f"got {tuple(env_ids.shape)}"
                    )
                return torch.nonzero(
                    env_ids.to(device=self.device), as_tuple=False
                ).flatten()
            normalized = env_ids.to(device=self.device, dtype=torch.long)
        else:
            normalized = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if normalized.ndim == 0:
            normalized = normalized.reshape(1)
        return normalized

    def _advance_sim_time(self) -> None:
        sim = getattr(self, "sim", None)
        current_time = float(getattr(sim, "current_time", self._current_time))
        next_time = current_time + float(getattr(self, "step_dt", 1.0))
        self._current_time = next_time
        try:
            setattr(sim, "current_time", next_time)
        except (AttributeError, TypeError) as error:
            # Missing or read-only time is expected; other failures may mean
            # simulation time stopped advancing, so keep them visible.
            logging.debug(
                "could not advance sim.current_time on %r: %s",
                type(sim).__name__,
                error,
            )


__all__ = ["SrbEnv"]
