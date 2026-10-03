"""Own step and reset ordering across SRB and legacy DirectEnv runtimes."""

from __future__ import annotations

import math
import time
import warnings
from collections.abc import Callable, Mapping
from dataclasses import dataclass, fields, is_dataclass
from numbers import Integral, Real
from typing import Any, Protocol, cast

import numpy as np
import torch

from srb._typing.step_return import StepReturn
from srb.core.reward.outcome import RewardOutcomeRuntime

from .action_layout import validate_finite_action
from .delay import resolve_delay_step_dt
from .outcome import ResetOutcome, StepOutcome
from .outcome_core import StepOutcomeCore, process_step_outcome_core
from .outcome_observation import final_observation, splice_observation_rows
from .step_return import (
    coerce_step_return_to_torch,
    scrub_non_finite_step_return,
    validate_step_return,
)

ASSET_LOADING_TIMEOUT_S: float = 300.0
"""Upper bound on the render loop that waits for simulator asset loading."""


class AssetLoadingTimeoutError(RuntimeError):
    """Simulator assets did not finish loading within the bounded wait."""


def wait_for_assets_loaded(
    assets_loading: Callable[[], bool],
    render: Callable[[], object],
    *,
    timeout_s: float = ASSET_LOADING_TIMEOUT_S,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Render until loading ends, return the render count, and raise at deadline."""
    deadline = clock() + timeout_s
    renders = 0
    while assets_loading():
        if clock() >= deadline:
            raise AssetLoadingTimeoutError(
                f"simulator assets still loading after {timeout_s:g} s "
                f"({renders} render calls)"
            )
        render()
        renders += 1
    return renders


@dataclass(frozen=True, slots=True)
class LifecycleSettings:
    """Immutable settings for :class:`RuntimeLifecycle`.

    Synthetic mode keeps action, outcome, event, and observation phases active.
    """

    simulation_enabled: bool = False
    decimation: int = 1
    render_interval: int = 1
    physics_dt: float = 1.0
    step_dt: float = 1.0
    action_noise_enabled: bool = False
    observation_noise_enabled: bool = False
    include_reward_terms: bool = False
    interval_events_configured: bool = False
    has_rtx_sensors: bool = False
    num_rerenders_on_reset: int = 0
    wait_for_textures: bool = False
    render_enabled: bool = True

    def __post_init__(self) -> None:
        for name in (
            "simulation_enabled",
            "action_noise_enabled",
            "observation_noise_enabled",
            "include_reward_terms",
            "interval_events_configured",
            "has_rtx_sensors",
            "wait_for_textures",
            "render_enabled",
        ):
            _strict_bool(name, getattr(self, name))
        _strict_int("decimation", self.decimation, minimum=1)
        _strict_int("render_interval", self.render_interval, minimum=1)
        _strict_int("num_rerenders_on_reset", self.num_rerenders_on_reset, minimum=0)
        _strict_real("physics_dt", self.physics_dt, minimum=0.0, strictly_positive=True)
        _strict_real("step_dt", self.step_dt, minimum=0.0, strictly_positive=True)


def _strict_bool(name: str, value: Any) -> None:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a bool, got {type(value).__name__}")


def _strict_int(name: str, value: Any, *, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer, got {type(value).__name__}")
    integer = int(value)
    if integer < minimum:
        raise ValueError(f"{name} must be >= {minimum}, got {integer}")
    return integer


def _strict_real(
    name: str,
    value: Any,
    *,
    minimum: float,
    strictly_positive: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(
            f"{name} must be a finite real number, got {type(value).__name__}"
        )
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite, got {number!r}")
    if strictly_positive and number <= minimum:
        raise ValueError(f"{name} must be > {minimum}, got {number}")
    if not strictly_positive and number < minimum:
        raise ValueError(f"{name} must be >= {minimum}, got {number}")
    return number


class LifecycleAdapter(Protocol):
    """Runtime operations required by the lifecycle owner."""

    @property
    def settings(self) -> LifecycleSettings: ...

    @property
    def num_envs(self) -> int: ...

    @property
    def device(self) -> torch.device: ...

    @property
    def reward_runtime(self) -> RewardOutcomeRuntime | None: ...

    def validate(self) -> None: ...

    def coerce_action(self, action: object) -> Any: ...

    def add_action_noise(self, action: Any) -> Any: ...

    def pre_physics_step(self, action: Any) -> None: ...

    def increment_sim_step_counter(self) -> int: ...

    def apply_action(self) -> None: ...

    def write_scene(self) -> None: ...

    def sim_step(self) -> None: ...

    def sim_is_rendering(self) -> bool: ...

    def render_enabled(self) -> bool: ...

    def render(self, *, skip_app_pumping: bool | None = None) -> None: ...

    @property
    def render_mode(self) -> str | None: ...

    @property
    def render_modes(self) -> object: ...

    def render_rgb_array(self) -> Any: ...

    def update_scene(self, dt: float) -> None: ...

    def increment_episode_counters(self) -> None: ...

    def advance_step_state(self) -> None: ...

    def extract_step_return(self) -> StepReturn: ...

    def extract_reset_step_return(self) -> StepReturn: ...

    def postprocess_step_return(
        self,
        step_return: StepReturn,
        *,
        reset: bool,
        env_ids: torch.Tensor,
    ) -> StepReturn: ...

    def process_step_return(
        self,
        step_return: StepReturn,
        *,
        include_reward_terms: bool,
        reward_runtime: RewardOutcomeRuntime | None,
    ) -> StepOutcomeCore: ...

    def commit_step_pre_reset(self, core: StepOutcomeCore) -> None: ...

    def reset_envs(self, env_ids: torch.Tensor) -> None: ...

    def prime_recorders(self, env_ids: torch.Tensor) -> None: ...

    def forward_sim(self) -> None: ...

    def wait_for_textures(self) -> None: ...

    def interval_events(self, dt: float) -> None: ...

    def build_observation(self, step_return: StepReturn) -> dict[str, torch.Tensor]: ...

    def add_observation_noise(
        self, observation: dict[str, torch.Tensor]
    ) -> dict[str, torch.Tensor]: ...

    def build_post_reset_observation(
        self, reset_ids: torch.Tensor
    ) -> dict[str, torch.Tensor] | None: ...

    def commit_step(self, outcome: StepOutcome) -> None: ...

    def commit_reset(self, outcome: ResetOutcome) -> None: ...

    def seed(self, seed: int) -> None: ...


class RuntimeLifecycle:
    """Execute the complete SRB/DirectEnv step and reset transactions."""

    def __init__(
        self,
        adapter: LifecycleAdapter,
        *,
        reward_runtime: RewardOutcomeRuntime | None = None,
    ) -> None:
        self._adapter = adapter
        self._reward_runtime = reward_runtime
        # Reuse an empty selector to avoid per-step device allocation.
        self._empty_env_ids_buffer: torch.Tensor | None = None
        adapter.validate()

    def _empty_env_ids(self) -> torch.Tensor:
        """Return the shared, read-only empty selector for non-reset steps."""
        buffer = self._empty_env_ids_buffer
        requested = torch.device(self._adapter.device)
        # An index-free device request (``cuda``) matches its resolved index.
        if (
            buffer is not None
            and buffer.device.type == requested.type
            and (requested.index is None or buffer.device.index == requested.index)
        ):
            return buffer
        buffer = torch.empty(0, dtype=torch.long, device=requested)
        self._empty_env_ids_buffer = buffer
        return buffer

    def step(self, action: object) -> StepOutcome:
        """Run one environment step in DirectRLEnv order."""

        settings = self._adapter.settings
        action_tensor = self._adapter.coerce_action(action)
        validate_finite_action(action_tensor)
        if settings.action_noise_enabled:
            action_tensor = self._adapter.add_action_noise(action_tensor)
            validate_finite_action(action_tensor)
        self._adapter.pre_physics_step(action_tensor)

        is_rendering = (
            self._adapter.sim_is_rendering() if settings.simulation_enabled else False
        )
        for _ in range(settings.decimation):
            sim_step_counter = self._adapter.increment_sim_step_counter()
            self._adapter.apply_action()
            if settings.simulation_enabled:
                self._adapter.write_scene()
                self._adapter.sim_step()
                if sim_step_counter % settings.render_interval == 0 and is_rendering:
                    self._adapter.render(
                        skip_app_pumping=not self._adapter.render_enabled()
                    )
                self._adapter.update_scene(settings.physics_dt)

        self._adapter.increment_episode_counters()

        # Extraction is read-only; advance task state exactly once.
        self._adapter.advance_step_state()
        step_return = self._adapter.extract_step_return()
        step_return = self._adapter.postprocess_step_return(
            step_return,
            reset=False,
            env_ids=self._empty_env_ids(),
        )
        core = self._adapter.process_step_return(
            step_return,
            include_reward_terms=settings.include_reward_terms,
            reward_runtime=self._reward_runtime
            or getattr(self._adapter, "reward_runtime", None),
        )
        # Recorder and reset hooks consume the outcome before selective reset.
        self._adapter.commit_step_pre_reset(core)

        # Preserve terminal rows before reset; post-reset extraction does not re-tick recorders.
        assembled: dict[str, torch.Tensor] | None = None
        terminal_observation: dict[str, torch.Tensor] | None = None
        if core.reset_ids.numel() > 0:
            assembled = self._adapter.build_observation(core.step_return)
            pre_reset_observation = assembled
            if settings.observation_noise_enabled:
                pre_reset_observation = self._adapter.add_observation_noise(
                    pre_reset_observation
                )
            terminal_observation = final_observation(
                pre_reset_observation, core.reset_ids
            )
            self._adapter.reset_envs(core.reset_ids)
            if (
                self._adapter.render_enabled()
                and is_rendering
                and settings.has_rtx_sensors
                and settings.num_rerenders_on_reset > 0
            ):
                for _ in range(settings.num_rerenders_on_reset):
                    self._adapter.render()
            # The terminal tick already consumed this sim time; open the new
            # episodes' recordings with the post-reset frame explicitly.
            self._adapter.prime_recorders(core.reset_ids)

        if settings.interval_events_configured:
            self._adapter.interval_events(settings.step_dt)

        if assembled is None:
            assembled = self._adapter.build_observation(core.step_return)
        observation = assembled
        if settings.observation_noise_enabled:
            observation = self._adapter.add_observation_noise(observation)
        if core.reset_ids.numel() > 0:
            post_reset = self._adapter.build_post_reset_observation(core.reset_ids)
            if post_reset is not None:
                if settings.observation_noise_enabled:
                    post_reset = self._adapter.add_observation_noise(post_reset)
                observation = splice_observation_rows(
                    observation, post_reset, core.reset_ids
                )

        outcome = StepOutcome(
            step_return=core.step_return,
            observation=observation,
            reward=core.reward,
            terminated=core.terminated,
            truncated=core.truncated,
            info=core.info,
            extras=core.extras,
            reset_mask=core.reset_mask,
            reset_ids=core.reset_ids,
            final_observation=terminal_observation,
            invalid_envs=core.invalid_envs,
            invalid_keys=core.invalid_keys,
        )
        self._adapter.commit_step(outcome)
        return outcome

    def render(self, recompute: bool = False) -> Any:
        """Render using the ``DirectRLEnv.render`` contract."""

        settings = self._adapter.settings
        if (
            settings.simulation_enabled
            and not settings.has_rtx_sensors
            and not recompute
        ):
            self._adapter.render()

        render_mode = self._adapter.render_mode
        if render_mode == "human" or render_mode is None:
            return None
        if render_mode == "rgb_array":
            return self._adapter.render_rgb_array()
        raise NotImplementedError(
            f"Render mode '{render_mode}' is unsupported. "
            f"Use one of: {self._adapter.render_modes}."
        )

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> ResetOutcome:
        """Reset all environments and return post-reset observations."""

        del options  # Gymnasium compatibility; no options are owned here.
        settings = self._adapter.settings
        if seed is not None:
            self._adapter.seed(seed)

        env_ids = torch.arange(
            self._adapter.num_envs,
            dtype=torch.long,
            device=self._adapter.device,
        )
        # Recorders consume the active outcome during reset.
        self._adapter.reset_envs(env_ids)
        if settings.simulation_enabled:
            self._adapter.write_scene()
            self._adapter.forward_sim()
            if settings.has_rtx_sensors and settings.num_rerenders_on_reset > 0:
                for _ in range(settings.num_rerenders_on_reset):
                    self._adapter.render()
            if settings.wait_for_textures and settings.has_rtx_sensors:
                self._adapter.wait_for_textures()
        # Resets do not advance sim time, so open the new episodes'
        # recordings with the post-reset frame explicitly.
        self._adapter.prime_recorders(env_ids)

        step_return = self._adapter.extract_reset_step_return()
        step_return = self._adapter.postprocess_step_return(
            step_return,
            reset=True,
            env_ids=env_ids,
        )
        validate_step_return(
            step_return,
            num_envs=self._adapter.num_envs,
            device=self._adapter.device,
        )
        observation = self._adapter.build_observation(step_return)
        if settings.observation_noise_enabled:
            observation = self._adapter.add_observation_noise(observation)
        outcome = ResetOutcome(
            step_return=step_return,
            observation=observation,
            info=dict(step_return.info or {}),
        )
        self._adapter.commit_reset(outcome)
        return outcome


class SrbEnvAdapter:
    """Adapter for the owned :class:`SrbEnv` production surface."""

    def __init__(self, env: Any, *, mode: str | None = None) -> None:
        self._env = env
        cfg = getattr(env, "cfg", None)
        self._mode = (
            mode
            or getattr(env, "_srb_runtime_mode", None)
            or getattr(cfg, "runtime_mode", None)
        )
        self._settings: LifecycleSettings | None = None

    @property
    def settings(self) -> LifecycleSettings:
        if self._settings is None:
            self.validate()
        if self._settings is None:
            raise RuntimeError("lifecycle validation did not initialize settings")
        return self._settings

    @property
    def num_envs(self) -> int:
        return int(self._env.num_envs)

    @property
    def device(self) -> torch.device:
        return torch.device(self._env.device)

    @property
    def reward_runtime(self) -> RewardOutcomeRuntime | None:
        return getattr(self._env, "_reward_outcome", None)

    def validate(self) -> None:
        if self._settings is not None:
            return
        if self._mode not in {"synthetic", "isaac", "legacy"}:
            raise RuntimeError(
                "SrbEnv runtime mode must be explicit: use 'synthetic' for a "
                "fake runtime or 'isaac'/'legacy' for a simulator adapter"
            )

        cfg = getattr(self._env, "cfg", None)
        if self._mode == "synthetic":
            self._settings = LifecycleSettings(
                simulation_enabled=False,
                decimation=_strict_int(
                    "cfg.decimation", getattr(cfg, "decimation", 1), minimum=1
                ),
                render_interval=_strict_int(
                    "cfg.render_interval", getattr(cfg, "render_interval", 1), minimum=1
                ),
                physics_dt=_strict_real(
                    "cfg.physics_dt",
                    getattr(cfg, "physics_dt", 1.0),
                    minimum=0.0,
                    strictly_positive=True,
                ),
                step_dt=_strict_real(
                    "env.step_dt",
                    getattr(self._env, "step_dt", 1.0),
                    minimum=0.0,
                    strictly_positive=True,
                ),
                action_noise_enabled=getattr(cfg, "action_noise_model", None)
                is not None,
                observation_noise_enabled=getattr(cfg, "observation_noise_model", None)
                is not None,
                include_reward_terms=bool(getattr(cfg, "extras", False)),
                interval_events_configured=False,
                has_rtx_sensors=False,
                num_rerenders_on_reset=0,
                render_enabled=True,
            )
            return

        sim = getattr(self._env, "sim", None)
        scene = getattr(self._env, "scene", None)
        missing: list[str] = []
        if sim is None:
            missing.append("sim")
        if scene is None:
            missing.append("scene")
        for name in ("step", "forward", "render"):
            if sim is not None and not callable(getattr(sim, name, None)):
                missing.append(f"sim.{name}")
        for name in ("write_data_to_sim", "update", "reset"):
            if scene is not None and not callable(getattr(scene, name, None)):
                missing.append(f"scene.{name}")
        if missing:
            raise RuntimeError(
                "configured simulator runtime is incomplete; missing "
                + ", ".join(missing)
            )

        events = getattr(cfg, "events", None)
        event_manager = getattr(self._env, "event_manager", None)
        if events is not None and event_manager is None:
            raise RuntimeError(
                "configured events require an event_manager; refusing to skip events"
            )
        available_modes = {
            str(mode) for mode in getattr(event_manager, "available_modes", ())
        }
        configured_modes = _configured_event_modes(events)
        missing_event_modes = configured_modes - available_modes
        if missing_event_modes:
            missing_modes_text = ", ".join(sorted(missing_event_modes))
            if "interval" in missing_event_modes:
                raise RuntimeError(
                    "configured interval events are unavailable; missing modes: "
                    + missing_modes_text
                )
            raise RuntimeError(
                f"configured event modes are unavailable: {missing_modes_text}"
            )
        action_noise = getattr(cfg, "action_noise_model", None)
        if action_noise is not None and not hasattr(self._env, "_action_noise_model"):
            raise RuntimeError(
                "configured action noise has no initialized action noise model"
            )
        observation_noise = getattr(cfg, "observation_noise_model", None)
        if observation_noise is not None and not hasattr(
            self._env, "_observation_noise_model"
        ):
            raise RuntimeError(
                "configured observation noise has no initialized observation noise model"
            )
        if getattr(cfg, "actions", None) and not hasattr(self._env, "action_manager"):
            raise RuntimeError(
                "configured actions require an initialized action_manager"
            )

        sim_cfg = getattr(cfg, "sim", None)
        decimation = _strict_int(
            "cfg.decimation", getattr(cfg, "decimation", 0), minimum=1
        )
        render_interval = _strict_int(
            "cfg.sim.render_interval", getattr(sim_cfg, "render_interval", 0), minimum=1
        )
        physics_dt = _strict_real(
            "cfg.sim.dt",
            getattr(sim_cfg, "dt", 0.0),
            minimum=0.0,
            strictly_positive=True,
        )
        self._settings = LifecycleSettings(
            simulation_enabled=True,
            decimation=decimation,
            render_interval=render_interval,
            physics_dt=physics_dt,
            step_dt=_strict_real(
                "env.step_dt",
                getattr(self._env, "step_dt"),
                minimum=0.0,
                strictly_positive=True,
            ),
            action_noise_enabled=action_noise is not None,
            observation_noise_enabled=observation_noise is not None,
            include_reward_terms=bool(getattr(cfg, "extras", False)),
            interval_events_configured="interval" in configured_modes,
            has_rtx_sensors=bool(getattr(self._env, "has_rtx_sensors", False)),
            num_rerenders_on_reset=_strict_int(
                "cfg.num_rerenders_on_reset",
                getattr(cfg, "num_rerenders_on_reset", 0),
                minimum=0,
            ),
            wait_for_textures=bool(getattr(cfg, "wait_for_textures", False)),
            render_enabled=bool(getattr(self._env, "render_enabled", True)),
        )

    def coerce_action(self, action: object) -> Any:
        layout = getattr(self._env, "_action_layout", None)
        coerce = getattr(layout, "coerce", None)
        if not callable(coerce):
            raise RuntimeError(
                "owned lifecycle requires an ActionLayout for action ingress"
            )
        return coerce(action, num_envs=self.num_envs, device=self.device)

    def add_action_noise(self, action: Any) -> Any:
        model = getattr(self._env, "_action_noise_model", None)
        if model is None:
            raise RuntimeError("action noise was configured but is unavailable")
        if not isinstance(action, torch.Tensor):
            raise TypeError(
                "tensor-only action noise does not support nested action structures"
            )
        noisy = model(action)
        if not isinstance(noisy, torch.Tensor):
            raise TypeError("action noise must return a tensor action")
        if tuple(noisy.shape) != tuple(action.shape):
            raise ValueError(
                "action noise output shape must match coerced action shape: "
                f"expected {tuple(action.shape)}, got {tuple(noisy.shape)}"
            )
        if noisy.device != action.device:
            raise ValueError(
                "action noise output device must match coerced action device: "
                f"expected {action.device}, got {noisy.device}"
            )
        if noisy.dtype != action.dtype:
            raise TypeError(
                "action noise output dtype must match coerced action dtype: "
                f"expected {action.dtype}, got {noisy.dtype}"
            )
        return noisy

    def pre_physics_step(self, action: Any) -> None:
        if not self.settings.simulation_enabled:
            advance = getattr(self._env, "_advance_sim_time", None)
            if callable(advance):
                advance()
        self._env._pre_physics_step(action)

    def increment_sim_step_counter(self) -> int:
        counter = int(getattr(self._env, "_sim_step_counter", 0)) + 1
        self._env._sim_step_counter = counter
        return counter

    def apply_action(self) -> None:
        self._env._apply_action()

    def write_scene(self) -> None:
        self._env.scene.write_data_to_sim()

    def sim_step(self) -> None:
        self._env.sim.step(render=False)

    def sim_is_rendering(self) -> bool:
        return bool(getattr(self._env.sim, "is_rendering", False))

    def render_enabled(self) -> bool:
        return bool(getattr(self._env, "render_enabled", True))

    def render(self, *, skip_app_pumping: bool | None = None) -> None:
        if skip_app_pumping is None:
            self._env.sim.render()
        else:
            self._env.sim.render(skip_app_pumping=skip_app_pumping)

    @property
    def render_mode(self) -> str | None:
        return cast(str | None, getattr(self._env, "render_mode", None))

    @property
    def render_modes(self) -> object:
        metadata = getattr(self._env, "metadata", {})
        if isinstance(metadata, Mapping):
            return metadata.get("render_modes", ())
        return ()

    def render_rgb_array(self) -> Any:
        recorder = getattr(self._env, "video_recorder", None)
        if recorder is None:
            return None
        return recorder.render_rgb_array()

    def update_scene(self, dt: float) -> None:
        self._env.scene.update(dt=dt)

    def increment_episode_counters(self) -> None:
        self._env.episode_length_buf += 1
        self._env.common_step_counter += 1

    def advance_step_state(self) -> None:
        advance = getattr(self._env, "advance_step_state", None)
        if callable(advance):
            advance()

    def extract_step_return(self) -> StepReturn:
        extract = getattr(self._env, "_extract_step_return_owned", None)
        if callable(extract):
            return cast(StepReturn, extract())
        return coerce_step_return_to_torch(
            self._env.extract_step_return(),
            device=getattr(self._env, "device", None),
        )

    def extract_reset_step_return(self) -> StepReturn:
        return self.extract_step_return()

    def postprocess_step_return(
        self,
        step_return: StepReturn,
        *,
        reset: bool,
        env_ids: torch.Tensor,
    ) -> StepReturn:
        delay = getattr(self._env, "_delay_runtime", None)
        if delay is None:
            return step_return
        if reset and self._mode in {"isaac", "legacy"}:
            # Refill rows from the fresh reset extraction without advancing delay.
            reseed = getattr(delay, "reseed_reset_observation", None)
            if not callable(reseed):
                raise RuntimeError("production reset requires delay reseed support")
            return cast(StepReturn, reseed(env_ids, step_return))
        current_time = float(
            getattr(
                getattr(self._env, "sim", None),
                "current_time",
                getattr(self._env, "_current_time", 0.0),
            )
        )
        # Preserve legacy ``cfg.agent_rate`` timing instead of ``sim.dt * decimation``.
        step_dt = resolve_delay_step_dt(self._env)
        if reset:
            return cast(
                StepReturn,
                delay.process_reset_observation(
                    env_ids,
                    step_return,
                    current_time=current_time,
                    step_dt=step_dt,
                ),
            )
        return cast(
            StepReturn,
            delay.process_observation(
                step_return,
                current_time=current_time,
                step_dt=step_dt,
            ),
        )

    def process_step_return(
        self,
        step_return: StepReturn,
        *,
        include_reward_terms: bool,
        reward_runtime: RewardOutcomeRuntime | None,
    ) -> StepOutcomeCore:
        direct_step_outcome = getattr(self._env, "_direct_step_outcome", None)
        if direct_step_outcome is not None:
            # DirectEnv hooks consume this production transition before commit.
            return cast(
                StepOutcomeCore,
                direct_step_outcome.begin_step(
                    step_return,
                    include_reward_terms=include_reward_terms,
                ),
            )
        return process_step_outcome_core(
            step_return,
            num_envs=self.num_envs,
            device=self.device,
            include_reward_terms=include_reward_terms,
            reward_runtime=reward_runtime,
        )

    def commit_step_pre_reset(self, core: StepOutcomeCore) -> None:
        self._env._step_return = core.step_return
        if not core.invalid_keys:
            # Remove stale diagnostics from the persistent extras dict.
            self._env.extras.pop("invalid_keys", None)
            self._env.extras.pop("invalid_envs", None)
        if core.extras:
            self._env.extras.update(core.extras)
        reset_buf = getattr(self._env, "reset_buf", None)
        if isinstance(reset_buf, torch.Tensor):
            reset_buf[...] = core.reset_mask
        else:
            self._env.reset_buf = core.reset_mask.clone()
        reset_terminated = getattr(self._env, "reset_terminated", None)
        if isinstance(reset_terminated, torch.Tensor):
            reset_terminated[...] = core.terminated
        reset_time_outs = getattr(self._env, "reset_time_outs", None)
        if isinstance(reset_time_outs, torch.Tensor):
            reset_time_outs[...] = core.truncated
        reward_buf = getattr(self._env, "reward_buf", None)
        if isinstance(reward_buf, torch.Tensor):
            reward_buf[...] = core.reward
        update_hud = getattr(self._env, "_update_hud", None)
        if callable(update_hud):
            update_hud(core.reward)

    def reset_envs(self, env_ids: torch.Tensor) -> None:
        all_envs = env_ids.numel() == self.num_envs
        try:
            self._env._reset_idx(env_ids)
        finally:
            if all_envs:
                prepare = getattr(self._env, "_prepare_owned_reset", None)
                if callable(prepare):
                    prepare(env_ids)
                else:
                    direct_step_outcome = getattr(
                        self._env, "_direct_step_outcome", None
                    )
                    if direct_step_outcome is not None:
                        direct_step_outcome.clear()

    def prime_recorders(self, env_ids: torch.Tensor) -> None:
        # Before the first step, the next record_step captures the frame.
        if int(getattr(self._env, "common_step_counter", 0)) == 0:
            return
        recorder = getattr(self._env, "_episode_recorder", None)
        prime = getattr(recorder, "prime_envs", None)
        if callable(prime):
            prime(env_ids.tolist(), sim_time_ns=self._recorder_sim_time_ns())

    def forward_sim(self) -> None:
        self._env.sim.forward()

    def wait_for_textures(self) -> None:
        physics_manager = getattr(self._env.sim, "physics_manager", None)
        assets_loading = getattr(physics_manager, "assets_loading", None)
        if not callable(assets_loading):
            return
        wait_for_assets_loaded(assets_loading, self._env.sim.render)

    def interval_events(self, dt: float) -> None:
        event_manager = getattr(self._env, "event_manager", None)
        if event_manager is None:
            raise RuntimeError("configured interval events have no event_manager")
        event_manager.apply(mode="interval", dt=dt)

    def build_observation(self, step_return: StepReturn) -> dict[str, torch.Tensor]:
        # Keep configured production visual/event observations on DirectEnv's assembly.
        assembly = getattr(self._env, "_observation_assembly", None)
        if assembly is None:
            assembly = getattr(self._env, "_owned_observation_assembly", None)
        if assembly is not None:
            event_observations: dict[str, torch.Tensor] | None = None
            recorder = getattr(self._env, "_episode_recorder", None)
            if getattr(assembly, "event_terms", ()):
                if recorder is None:
                    raise RuntimeError(
                        "event observations are configured but no episode recorder is available"
                    )
                self._record_event_step(recorder)
                event_observations = recorder.latest_event_observations() or None
            elif recorder is not None:
                self._record_event_step(recorder)
                event_observations = recorder.latest_event_observations() or None
            direct_step_outcome = getattr(self._env, "_direct_step_outcome", None)
            if direct_step_outcome is not None and direct_step_outcome.has_outcome:
                # Preserve DirectEnv's memoized observation and terminal-row contract.
                observations = cast(
                    dict[str, torch.Tensor],
                    direct_step_outcome.observation(
                        event_observations=event_observations,
                    ),
                )
            else:
                # Global reset and synthetic paths assemble directly.
                observations = cast(
                    dict[str, torch.Tensor],
                    assembly.assemble(
                        step_return,
                        event_observations=event_observations,
                    ),
                )
        else:
            layout = getattr(self._env, "_observation_layout", None)
            if layout is None:
                raise RuntimeError(
                    "owned lifecycle requires an ObservationLayout or "
                    "ObservationAssembly for observation delivery"
                )
            observations = cast(
                dict[str, torch.Tensor],
                layout.flatten(step_return.observation),
            )

        ephemeris_observations = getattr(self._env, "_ephemeris_observations", None)
        if callable(ephemeris_observations):
            observations.update(ephemeris_observations())
        return observations

    def _record_event_step(self, recorder: Any) -> None:
        recorder.record_step(sim_time_ns=self._recorder_sim_time_ns())

    def _recorder_sim_time_ns(self) -> int:
        clock = getattr(self._env, "_delay_current_time", None)
        if callable(clock):
            current_time = clock()
        else:
            sim = getattr(self._env, "sim", None)
            current_time = getattr(sim, "current_time", None)
            if current_time is None:
                step_counter = int(getattr(self._env, "common_step_counter", 0))
                step_dt = float(getattr(self._env, "step_dt", 0.0))
                current_time = step_counter * step_dt
        return int(max(0.0, float(current_time)) * 1_000_000_000)

    def build_post_reset_observation(
        self, reset_ids: torch.Tensor
    ) -> dict[str, torch.Tensor] | None:
        """Build the first observation for reset rows without reusing stale cache."""
        assemble = getattr(self._env, "_assemble_post_reset_observation", None)
        if callable(assemble):
            return cast(dict[str, torch.Tensor], assemble(reset_ids))
        step_return = self.extract_step_return()
        # Same non-finite policy as the step path: a non-finite post-reset row
        # is zeroed rather than handed to the policy.
        step_return, _invalid_envs, _invalid_keys = scrub_non_finite_step_return(
            step_return, num_envs=self.num_envs, device=self.device
        )
        delay = getattr(self._env, "_delay_runtime", None)
        if delay is not None:
            step_return = cast(
                StepReturn, delay.reseed_reset_observation(reset_ids, step_return)
            )
        assembly = getattr(self._env, "_observation_assembly", None)
        if assembly is None:
            assembly = getattr(self._env, "_owned_observation_assembly", None)
        if assembly is not None:
            recorder = getattr(self._env, "_episode_recorder", None)
            event_observations = (
                (recorder.latest_event_observations() or None)
                if recorder is not None
                else None
            )
            observations = cast(
                dict[str, torch.Tensor],
                assembly.assemble(step_return, event_observations=event_observations),
            )
        else:
            layout = getattr(self._env, "_observation_layout", None)
            if layout is None:
                raise RuntimeError(
                    "owned lifecycle requires an ObservationLayout or "
                    "ObservationAssembly for observation delivery"
                )
            observations = cast(
                dict[str, torch.Tensor],
                layout.flatten(step_return.observation),
            )
        ephemeris_observations = getattr(self._env, "_ephemeris_observations", None)
        if callable(ephemeris_observations):
            observations.update(ephemeris_observations())
        return observations

    def add_observation_noise(
        self, observation: dict[str, torch.Tensor]
    ) -> dict[str, torch.Tensor]:
        model = getattr(self._env, "_observation_noise_model", None)
        if model is None:
            raise RuntimeError("observation noise was configured but is unavailable")
        if "vector" not in observation:
            raise RuntimeError(
                "configured observation noise requires a packed 'vector' observation"
            )
        noisy = dict(observation)
        noisy["vector"] = model(observation["vector"])
        return noisy

    def commit_step(self, outcome: StepOutcome) -> None:
        self._env._step_return = outcome.step_return
        if hasattr(self._env, "obs_buf"):
            self._env.obs_buf = outcome.observation
        self._env._last_final_observation = outcome.final_observation
        self._env._last_final_observation_ids = outcome.reset_ids.clone()
        self._env._last_final_observation_mask = outcome.reset_mask.clone()

    def commit_reset(self, outcome: ResetOutcome) -> None:
        self._env._step_return = outcome.step_return
        if hasattr(self._env, "obs_buf"):
            self._env.obs_buf = outcome.observation
        self._env._last_final_observation = None
        self._env._last_final_observation_ids = torch.empty(
            0, dtype=torch.long, device=self.device
        )
        self._env._last_final_observation_mask = torch.zeros(
            self.num_envs, dtype=torch.bool, device=self.device
        )
        if self._mode == "synthetic":
            # Synthetic has no DirectRLEnv reset path to clear these buffers.
            if hasattr(self._env, "reset_terminated"):
                self._env.reset_terminated.zero_()
            if hasattr(self._env, "reset_time_outs"):
                self._env.reset_time_outs.zero_()
            self._env.reset_buf.zero_()
        mark_reset = getattr(self._env, "_mark_owned_reset", None)
        if callable(mark_reset):
            mark_reset()

    def seed(self, seed: int) -> None:
        # Seed delay generators with a negative seed's drawn value; avoid sentinel restarts.
        effective = self._env.seed(seed)
        if isinstance(effective, int):
            seed = effective
        for name in ("_ephemeris_rng", "_illumination_rng"):
            setattr(self._env, name, np.random.default_rng(seed))
        delay = getattr(self._env, "_delay_runtime", None)
        if delay is not None:
            delay.manual_seed(seed)
        direct_delay = getattr(self._env, "_direct_action_delay", None)
        if direct_delay is not None:
            direct_delay.manual_seed(seed)


#: Keep one warning per task class, rather than one per process.
_LEGACY_LIFECYCLE_WARNED: set[str] = set()


def _legacy_lifecycle_key(env: Any) -> str:
    """Identify the task a legacy adapter was built for."""

    task_type = type(env)
    return f"{task_type.__module__}.{task_type.__qualname__}"


def runs_owned_lifecycle(task_type: type) -> bool:
    """Whether this task uses ``SrbEnv``, detected without importing it."""
    return any(base.__name__ == "SrbEnv" for base in task_type.__mro__)


def _warn_legacy_lifecycle_once(env: Any) -> None:
    """Emit one deprecation notice per task using the legacy lifecycle."""

    key = _legacy_lifecycle_key(env)
    if key in _LEGACY_LIFECYCLE_WARNED:
        return
    _LEGACY_LIFECYCLE_WARNED.add(key)
    warnings.warn(
        f"{key} uses the legacy DirectEnv lifecycle, which is deprecated; "
        "migrate the task to the owned SrbEnv runtime.",
        DeprecationWarning,
        stacklevel=3,
    )


def _configured_event_modes(events: Any) -> set[str]:
    modes: set[str] = set()
    seen: set[int] = set()

    def visit(value: Any) -> None:
        if value is None or isinstance(value, (str, bytes, int, float, bool)):
            return
        identity = id(value)
        if identity in seen:
            return
        seen.add(identity)
        mode = getattr(value, "mode", None)
        if isinstance(mode, str):
            modes.add(mode)
        if isinstance(value, Mapping):
            for item in value.values():
                visit(item)
            return
        if isinstance(value, (list, tuple, set, frozenset)):
            for item in value:
                visit(item)
            return
        if is_dataclass(value):
            for field in fields(value):
                visit(getattr(value, field.name))
        for item in getattr(value, "__dict__", {}).values():
            visit(item)

    visit(events)
    return modes


__all__ = [
    "LifecycleAdapter",
    "LifecycleSettings",
    "RuntimeLifecycle",
    "SrbEnvAdapter",
]
