"""Dataset collection runners shared by agent workflows."""

from __future__ import annotations

import math
from collections.abc import Mapping
from contextlib import nullcontext
from dataclasses import dataclass
from numbers import Integral, Real
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Literal, cast

from srb.datasets.metadata_validation import normalize_int_value
from srb.utils.failures import (
    FALLBACK_ALLOWED_CATEGORIES,
    FailureError,
    attempt_cleanup,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def __getattr__(name: str) -> Callable[[Any], bool]:
    """Preserve the shared helper alias without loading the runtime on import."""
    if name != "_truthy_any":
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from srb.core.env.runtime.step_return import truthy_any

    globals()[name] = truthy_any
    return truthy_any


@dataclass(frozen=True)
class CollectionRunResult:
    """Result of collecting one dataset artifact."""

    output_path: Path
    attempted_episodes: int
    saved_episodes: int
    total_samples: int


@dataclass
class PolicyActionSource:
    """Action source backed by a loaded policy callable."""

    algo: str
    model_path: Path
    policy_fn: Callable[[Any], Any]
    start_episode_fn: Callable[[], None] | None = None
    deterministic: bool = True

    @property
    def name(self) -> str:
        return self.algo

    def start_episode(self) -> None:
        if self.start_episode_fn is not None:
            self.start_episode_fn()

    def action(self, obs: Any) -> Any:
        return self.policy_fn(obs)


@dataclass
class BuiltinActionSource:
    """Action source for built-in random and zero collectors."""

    collector: Literal["random", "zero"]
    _action_dim: int = 0
    _action_space: Any = None

    @property
    def name(self) -> str:
        return self.collector

    @property
    def deterministic(self) -> bool:
        return self.collector == "zero"

    def bind(self, *, env: "AnyEnv", wrapped_env: Any, action_dim: int) -> None:
        self._action_dim = action_dim
        self._action_space = getattr(
            getattr(env, "unwrapped", env),
            "single_action_space",
            None,
        )

    def seed(self, seed: int) -> None:
        """Seed the action space the random collector samples from."""
        if self._action_space is not None:
            self._action_space.seed(seed)

    def start_episode(self) -> None:
        return None

    def action(self, _obs: Any) -> Any:
        import numpy as np

        if self.collector == "zero":
            return np.zeros(self._action_dim, dtype=np.float32)

        from srb.core.env.runtime.action_space import flatten_action

        if self._action_space is None:
            raise ValueError("random collection requires an environment action space")
        return flatten_action(
            self._action_space.sample(),
            self._action_space,
        ).astype(np.float32, copy=False)


@dataclass
class TeleopActionSource:
    """Action source backed by a teleoperation interface."""

    env: "AnyEnv"
    teleop_interface: Any
    invert_controls: bool = False
    last_event: Any = False

    @property
    def name(self) -> str:
        return "human"

    def start_episode(self) -> None:
        self.teleop_interface.reset()

    def action(self, _obs: Any) -> Any:
        import sys

        unwrapped = getattr(self.env, "unwrapped", self.env)
        real_env_module = sys.modules.get("srb.interfaces.sim_to_real.core.env")
        real_env_type = getattr(real_env_module, "RealEnv", None)
        if isinstance(real_env_type, type) and isinstance(unwrapped, real_env_type):
            import torch

            from srb.core.command_mapping import apply_command_mapping

            mapping = unwrapped.command_mapping
            if mapping is None:
                raise ValueError(
                    "RealEnv does not declare direct command mapping metadata"
                )
            twist, event = self.teleop_interface.advance()
            self.last_event = event
            if self.invert_controls and mapping.task_family == "manipulation":
                twist[:2] *= -1.0
            action = apply_command_mapping(
                mapping,
                torch.as_tensor(twist, dtype=torch.float32),
                bool(event),
            )
            return (
                action.detach()
                .cpu()
                .numpy()
                .astype(unwrapped.single_action_space.dtype, copy=False)
            )

        from srb.core.action.command import map_direct_action_command

        twist, event = self.teleop_interface.advance()
        self.last_event = event
        if self.invert_controls:
            twist[:2] *= -1.0
        return map_direct_action_command(self.env, twist, event).repeat(
            self.env.unwrapped.num_envs,
            1,
        )


@dataclass
class TeleopCollectionControls:
    """Save/discard/reset callbacks for one teleop collection run."""

    should_reset: bool = False
    should_save: bool = False
    should_discard: bool = False

    def request_reset(self) -> None:
        self.should_reset = True

    def request_save(self) -> None:
        self.should_save = True

    def request_discard(self) -> None:
        self.should_discard = True

    def consume_save(self) -> bool:
        requested = self.should_save
        self.should_save = False
        return requested

    def consume_discard(self) -> bool:
        requested = self.should_discard
        self.should_discard = False
        return requested

    def consume_reset(self) -> bool:
        requested = self.should_reset
        self.should_reset = False
        return requested

    def bind(self, teleop_interface: Any) -> None:
        add_callback = getattr(teleop_interface, "add_callback", None)
        if not callable(add_callback):
            return

        callbacks = {
            "L": self.request_reset,
            "SPACE": self.request_save,
            "BACKSPACE": self.request_discard,
        }
        for key, callback in callbacks.items():
            try:
                add_callback(key, callback)
            except (NotImplementedError, ValueError):
                continue


def success_term_from_env_cfg(env_cfg: Any) -> Any | None:
    """Return the configured success termination term, when present."""
    if env_cfg is None or not hasattr(env_cfg, "terminations"):
        return None
    return getattr(env_cfg.terminations, "success", None)


def run_robomimic_collection(
    *,
    env: "AnyEnv",
    sim_app: Any,
    source: Any,
    output_path: str | Path,
    num_episodes: int,
    horizon: int,
    success_only: bool,
    env_kwargs: dict[str, object] | None = None,
    env_cfg: Any = None,
    success_term: Any | None = None,
    seed: int | None = None,
    max_demos: int | None = None,
    max_attempt_multiplier: int = 3,
) -> CollectionRunResult:
    """Collect episodes into robomimic HDF5 using one autonomous action source."""
    _validate_collection_parameters(
        num_episodes=num_episodes,
        horizon=horizon,
        max_demos=max_demos,
        max_attempt_multiplier=max_attempt_multiplier,
    )
    _require_single_env(env)
    import numpy as np

    from srb.datasets.robomimic import TrajectoryRecorder
    from srb.wrappers.robomimic import RobomimicEnvWrapper

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    wrapped_env = RobomimicEnvWrapper(env)
    obs_keys = wrapped_env.get_obs_keys()
    action_dim = int(wrapped_env.get_action_dim())
    action_space = _robomimic_action_space(wrapped_env, env)

    bind = getattr(source, "bind", None)
    if callable(bind):
        bind(env=env, wrapped_env=wrapped_env, action_dim=action_dim)

    recorder = TrajectoryRecorder(
        output_path=output_path,
        env_name=_dataset_env_name(env, env_kwargs),
        obs_keys=obs_keys,
        action_dim=action_dim,
        max_demos=max_demos if max_demos is not None else num_episodes * 2,
        action_space=action_space,
        env_kwargs=env_kwargs,
        demo_io_contract=_demo_io_contract(wrapped_env),
        fps=_configured_collection_fps(env),
    )

    if seed is not None:
        np.random.seed(seed)
        try:
            import torch

            torch.manual_seed(seed)
        except ImportError:
            pass
        wrapped_env.seed(seed)
        # ``action_space.sample()`` draws from the space's own generator, which
        # the global seeds above do not reach.
        seed_source = getattr(source, "seed", None)
        if callable(seed_source):
            seed_source(seed)

    with _FinalizedOnExit(recorder), _inference_context():
        attempted, saved = _run_autonomous_collection_loop(
            env=env,
            wrapped_env=wrapped_env,
            recorder=recorder,
            source=source,
            sim_app=sim_app,
            num_episodes=num_episodes,
            horizon=horizon,
            success_only=success_only,
            success_term=_resolve_success_term(env_cfg, success_term),
            max_attempt_multiplier=max_attempt_multiplier,
            recorded_action_shape=_single_action_shape(action_space, action_dim),
        )
    _report_collection(
        output_path=output_path,
        attempted=attempted,
        saved=saved,
        requested=num_episodes,
        success_only=success_only,
    )
    return CollectionRunResult(
        output_path=output_path,
        attempted_episodes=attempted,
        saved_episodes=saved,
        total_samples=int(getattr(recorder, "total_samples", 0)),
    )


def run_robomimic_teleop_collection(
    *,
    env: "AnyEnv",
    sim_app: Any,
    teleop_interface: Any,
    output_path: str | Path,
    num_episodes: int,
    horizon: int,
    success_only: bool,
    invert_controls: bool = False,
    env_kwargs: dict[str, object] | None = None,
    env_cfg: Any = None,
    success_term: Any | None = None,
    controls: TeleopCollectionControls | None = None,
) -> CollectionRunResult:
    """Collect episodes into robomimic HDF5 using human teleoperation."""
    _validate_collection_parameters(
        num_episodes=num_episodes,
        horizon=horizon,
        max_demos=num_episodes,
        max_attempt_multiplier=1,
    )
    _require_single_env(env)
    from srb.datasets.robomimic import TrajectoryRecorder
    from srb.wrappers.robomimic import RobomimicEnvWrapper

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    wrapped_env = RobomimicEnvWrapper(env)
    recorder = TrajectoryRecorder(
        output_path=output_path,
        env_name=_dataset_env_name(env, env_kwargs),
        obs_keys=wrapped_env.get_obs_keys(),
        action_dim=wrapped_env.get_action_dim(),
        max_demos=num_episodes,
        action_space=_robomimic_action_space(wrapped_env, env),
        env_kwargs=env_kwargs,
        demo_io_contract=_demo_io_contract(wrapped_env),
        fps=_configured_collection_fps(env),
    )

    controls = controls or TeleopCollectionControls()
    controls.bind(teleop_interface)
    source = TeleopActionSource(
        env=env,
        teleop_interface=teleop_interface,
        invert_controls=invert_controls,
    )

    with _FinalizedOnExit(recorder), _inference_context():
        attempted, saved = _run_teleop_collection_loop(
            env=env,
            wrapped_env=wrapped_env,
            recorder=recorder,
            source=source,
            controls=controls,
            sim_app=sim_app,
            num_episodes=num_episodes,
            horizon=horizon,
            success_only=success_only,
            success_term=_resolve_success_term(env_cfg, success_term),
        )
    _report_collection(
        output_path=output_path,
        attempted=attempted,
        saved=saved,
        requested=num_episodes,
        success_only=success_only,
    )
    return CollectionRunResult(
        output_path=output_path,
        attempted_episodes=attempted,
        saved_episodes=saved,
        total_samples=int(getattr(recorder, "total_samples", 0)),
    )


def _require_single_env(env: "AnyEnv") -> None:
    """Reject vectorized collection before creating any output side effects."""
    unwrapped = getattr(env, "unwrapped", env)
    raw_num_envs = getattr(unwrapped, "num_envs", getattr(env, "num_envs", None))
    num_envs = normalize_int_value(raw_num_envs)
    if num_envs != 1:
        raise ValueError(
            f"robomimic collection requires num_envs == 1, got {raw_num_envs!r}"
        )


def _configured_collection_fps(env: Any) -> float | None:
    """Return FPS only when the environment declares a usable step period."""
    unwrapped = getattr(env, "unwrapped", env)
    step_dt = getattr(unwrapped, "step_dt", None)
    if isinstance(step_dt, bool) or not isinstance(step_dt, Real):
        return None
    try:
        period = float(step_dt)
    except (OverflowError, TypeError, ValueError):
        return None
    if not math.isfinite(period) or period <= 0:
        return None
    fps = 1.0 / period
    if not math.isfinite(fps) or fps <= 0:
        return None
    return fps


def _validate_collection_parameters(
    *,
    num_episodes: int,
    horizon: int,
    max_demos: int | None,
    max_attempt_multiplier: int,
) -> None:
    """Reject invalid collection bounds before creating dataset output."""

    for name, value in (
        ("num_episodes", num_episodes),
        ("horizon", horizon),
        ("max_attempt_multiplier", max_attempt_multiplier),
    ):
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise TypeError(f"{name} must be a positive integer")
        if value <= 0:
            raise ValueError(f"{name} must be positive")
    if max_demos is not None:
        if isinstance(max_demos, bool) or not isinstance(max_demos, Integral):
            raise TypeError("max_demos must be a positive integer")
        if max_demos <= 0:
            raise ValueError("max_demos must be positive")


def _dataset_env_name(env: Any, env_kwargs: "Mapping[str, Any] | None") -> str:
    """Return the registered environment id, falling back to its class name."""

    for candidate in (
        (env_kwargs or {}).get("task"),
        getattr(getattr(env, "spec", None), "id", None),
        getattr(getattr(env, "unwrapped", env), "task_name", None),
    ):
        if isinstance(candidate, str) and candidate:
            return candidate
    return str(env.unwrapped.__class__.__name__)


def _recorded_termination(
    terminated: Any,
    truncated: Any,
    *,
    horizon_exhausted: bool,
    operator_stop: bool = False,
) -> "tuple[bool, bool, bool]":
    """Return termination flags, marking collector or operator cuts truncated."""

    from srb.core.env.runtime.step_return import truthy_any as _truthy_any

    step_terminated = _truthy_any(terminated)
    step_truncated = _truthy_any(truncated)
    if (horizon_exhausted or operator_stop) and not step_terminated:
        step_truncated = True
    return step_terminated, step_truncated, step_terminated or step_truncated


def _recorded_next_observation(
    wrapped_env: Any,
    next_obs: Any,
    *,
    terminated: Any,
    truncated: Any,
) -> Any:
    """Use the actual terminal row only when the environment ended an episode."""
    from srb.core.env.runtime.step_return import truthy_any as _truthy_any

    if not (_truthy_any(terminated) or _truthy_any(truncated)):
        return next_obs
    terminal = getattr(wrapped_env, "last_final_observation", None)
    if terminal is not None:
        return terminal
    if _is_srb_env(wrapped_env):
        raise RuntimeError(
            "SRB autoreset collection has no terminal observation snapshot"
        )
    return next_obs


def _is_srb_env(wrapped_env: Any) -> bool:
    getter = getattr(wrapped_env, "get_wrapper_attr", None)
    if callable(getter):
        try:
            return bool(getter("is_srb_env"))
        except AttributeError:
            return False
    return bool(getattr(wrapped_env, "is_srb_env", False))


def _success_term_readable(wrapped_env: Any, *, env_done: bool) -> bool:
    """Whether a success term can still read the state from the ended step."""
    if not env_done:
        return True
    if _is_srb_env(wrapped_env):
        return False
    return getattr(wrapped_env, "last_final_observation", None) is None


def _run_autonomous_collection_loop(
    *,
    env: "AnyEnv",
    wrapped_env: Any,
    recorder: Any,
    source: Any,
    sim_app: Any,
    num_episodes: int,
    horizon: int,
    success_only: bool,
    success_term: Any | None,
    max_attempt_multiplier: int,
    recorded_action_shape: tuple[int, ...],
) -> tuple[int, int]:
    from srb.core.env.runtime.step_return import truthy_any
    from srb.utils import logging

    saved_episodes = 0
    attempted_episodes = 0
    max_attempts = num_episodes * max_attempt_multiplier
    source_name = str(getattr(source, "name", source.__class__.__name__))
    success_signal_seen = False

    while saved_episodes < num_episodes and attempted_episodes < max_attempts:
        if not _sim_is_running(sim_app):
            break

        attempted_episodes += 1
        logging.info(
            f"Starting episode {attempted_episodes} "
            f"(collected: {saved_episodes}/{num_episodes})"
        )

        obs, _ = wrapped_env.reset()
        _start_recorder_episode(recorder, env)
        _start_source_episode(source)
        episode_success = False
        episode_reward = 0.0

        for _step in range(horizon):
            action = _source_action(source, obs)
            next_obs, reward, terminated, truncated, info = wrapped_env.step(action)
            # Checked after the step and before recording it, so a simulator
            # shutdown cuts the episode at a row that says so.
            sim_stopped = not _sim_is_running(sim_app)

            reward_val = _scalar_value(reward)
            env_done = truthy_any(terminated) or truthy_any(truncated)
            term_readable = success_term is not None and _success_term_readable(
                wrapped_env, env_done=env_done
            )
            if term_readable:
                # Resolve the collector's configured stop before recording so
                # an otherwise ongoing final row can carry its truncation.
                episode_success = _evaluate_success_term(
                    success_term=success_term,
                    env=env,
                    logger=logging,
                )
            step_terminated, step_truncated, done_val = _recorded_termination(
                terminated,
                truncated,
                horizon_exhausted=_step == horizon - 1,
                operator_stop=sim_stopped
                or (success_term is not None and episode_success),
            )
            recorder.record_step(
                _single_env_observation(obs),
                _single_env_policy_action(action, recorded_action_shape),
                reward_val,
                done_val,
                _single_env_observation(
                    _recorded_next_observation(
                        wrapped_env,
                        next_obs,
                        terminated=terminated,
                        truncated=truncated,
                    )
                ),
                terminated=step_terminated,
                truncated=step_truncated,
            )
            episode_reward += reward_val

            if done_val and not term_readable:
                # The row ends the episode, whether the environment or the
                # collector ended it: its info describes the final
                # pre-reset state.
                info_success = _episode_info_success(wrapped_env, info)
                if info_success is not None:
                    episode_success = info_success
                    success_signal_seen = True
                elif success_term is not None:
                    # The term cannot read the reset state and the task
                    # published no terminal outcome: success is unknown, so
                    # say so rather than silently recording a failure.
                    logging.warning(
                        f"Episode {attempted_episodes} ended without "
                        "info['success'] and the success term cannot read the "
                        "reset state; success is unavailable and recorded as "
                        "False"
                        + (" (discarded by --success_only)" if success_only else "")
                    )
                elif success_only and not success_signal_seen:
                    # Nothing can have been saved yet: fail before spending
                    # the whole attempt budget on episodes that are discarded.
                    raise ValueError(
                        "--success_only needs a success signal, but an "
                        "episode ended without info['success'] and no success "
                        "term is configured; no episode can ever be saved"
                    )

            if done_val or episode_success:
                break

            obs = next_obs

        if success_only and not episode_success:
            recorder.discard_episode()
            logging.info(
                f"Episode {attempted_episodes} discarded "
                f"(reward={episode_reward:.2f}, not successful)"
            )
            continue

        if recorder.save_episode(success=episode_success):
            saved_episodes += 1
            logging.info(
                f"Episode {attempted_episodes} saved "
                f"(source={source_name}, reward={episode_reward:.2f}, "
                f"success={episode_success})"
            )

    return attempted_episodes, saved_episodes


def _run_teleop_collection_loop(
    *,
    env: "AnyEnv",
    wrapped_env: Any,
    recorder: Any,
    source: TeleopActionSource,
    controls: TeleopCollectionControls,
    sim_app: Any,
    num_episodes: int,
    horizon: int,
    success_only: bool,
    success_term: Any | None,
) -> tuple[int, int]:
    from srb.core.env.runtime.step_return import truthy_any
    from srb.utils import logging

    saved_episodes = 0
    attempted_episodes = 0

    while _sim_is_running(sim_app) and saved_episodes < num_episodes:
        attempted_episodes += 1
        logging.info(
            f"Starting episode {attempted_episodes} "
            f"(collected: {saved_episodes}/{num_episodes})"
        )

        obs, _ = wrapped_env.reset()
        _start_recorder_episode(recorder, env)
        source.start_episode()
        episode_success = False
        episode_discarded = False

        for step in range(horizon):
            action = source.action(obs)
            event = source.last_event
            next_obs, reward, terminated, truncated, info = wrapped_env.step(
                _to_numpy_action(action)
            )
            # A closed simulator window ends the episode at this row; record
            # it as a truncation rather than an in-flight tail.
            sim_stopped = not _sim_is_running(sim_app)

            reward_val = _scalar_value(reward)
            env_done = truthy_any(terminated) or truthy_any(truncated)
            # Resolved before the row is recorded so that an operator save can
            # mark it terminal; acting on the request stays below.
            request = _teleop_collection_request(event=event, controls=controls)
            step_terminated, step_truncated, done_val = _recorded_termination(
                terminated,
                truncated,
                horizon_exhausted=step == horizon - 1,
                operator_stop=sim_stopped or request in ("event:save", "key:save"),
            )
            recorder.record_step(
                _single_env_observation(obs),
                _single_env_action(action),
                reward_val,
                done_val,
                _single_env_observation(
                    _recorded_next_observation(
                        wrapped_env,
                        next_obs,
                        terminated=terminated,
                        truncated=truncated,
                    )
                ),
                terminated=step_terminated,
                truncated=step_truncated,
            )

            if success_term is not None and _success_term_readable(
                wrapped_env, env_done=env_done
            ):
                episode_success = _evaluate_success_term(
                    success_term=success_term,
                    env=env,
                    logger=logging,
                )
            elif done_val:
                # Environment end or collector cut alike.
                info_success = _episode_info_success(wrapped_env, info)
                if info_success is not None:
                    episode_success = info_success
                elif success_term is not None:
                    logging.warning(
                        f"Episode {attempted_episodes} ended without "
                        "info['success'] and the success term cannot read the "
                        "reset state; success is unavailable and recorded as False"
                    )

            obs = next_obs

            if request is not None:
                requested = request.split(":", 1)[1]
                logging.info(f"User requested {requested}")
                if requested == "save":
                    if request == "event:save":
                        episode_success = _teleop_event_success(event)
                    break
                episode_discarded = True
                recorder.discard_episode()
                source.start_episode()
                break

            if done_val:
                logging.info(
                    f"Episode ended (step {step + 1}, success={episode_success})"
                )
                break

        if episode_discarded:
            continue

        if success_only and not episode_success:
            recorder.discard_episode()
            # A keyboard save marks the end of a demo, not its success; only
            # the task's success signal or a device save event does.
            logging.info(f"Episode {attempted_episodes} discarded (not successful)")
        elif recorder.save_episode(success=episode_success):
            saved_episodes += 1
            logging.info(
                f"Episode {attempted_episodes} saved "
                f"(success={episode_success}, total={saved_episodes})"
            )

        source.start_episode()

    return attempted_episodes, saved_episodes


def _robomimic_action_space(wrapped_env: object, env: "AnyEnv") -> object | None:
    get_action_space = getattr(wrapped_env, "get_action_space", None)
    if callable(get_action_space):
        return cast(object, get_action_space())
    return getattr(getattr(env, "unwrapped", env), "single_action_space", None)


def _demo_io_contract(wrapped_env: object) -> Mapping[str, object] | None:
    getter = getattr(wrapped_env, "get_demo_io_contract", None)
    value = getter() if callable(getter) else None
    return cast(Mapping[str, object], value) if isinstance(value, Mapping) else None


def _resolve_success_term(env_cfg: Any, explicit: Any | None) -> Any | None:
    return explicit if explicit is not None else success_term_from_env_cfg(env_cfg)


def _report_collection(
    *,
    output_path: Path,
    attempted: int,
    saved: int,
    requested: int,
    success_only: bool,
) -> None:
    """Fail when nothing was published; warn when fewer demos were saved."""
    from srb.utils import logging

    if saved == 0:
        reason = " (--success_only discarded every episode)" if success_only else ""
        raise RuntimeError(
            f"no episodes were saved after {attempted} attempt(s){reason}; "
            f"no dataset was written to {output_path}"
        )
    if saved < requested:
        logging.warning(
            f"Data collection saved {saved} of {requested} requested episode(s) "
            f"after {attempted} attempt(s). Dataset saved to: {output_path}"
        )
        return
    logging.info(f"Data collection complete. Dataset saved to: {output_path}")


def _episode_info_success(wrapped_env: Any, info: Any) -> bool | None:
    """Read the episode's success from the step info that ended it."""
    if getattr(wrapped_env, "_is_vector_env", True) is False:
        return _terminal_info_success(info)
    return _vector_info_success(info)


def _vector_info_success(info: Any) -> bool | None:
    """Read SRB success from one-row vector step info (collection is single-env)."""
    from srb.core.env.runtime.episode_info import EPISODE_SUCCESS_KEY

    if not isinstance(info, Mapping) or EPISODE_SUCCESS_KEY not in info:
        return None
    import numpy as np

    value = info[EPISODE_SUCCESS_KEY]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    if not isinstance(value, np.ndarray) or value.dtype != np.bool_:
        raise TypeError("vector info['success'] must be a boolean array")
    if value.size != 1:
        raise ValueError(
            f"vector info['success'] must describe one environment, got {value.size}"
        )
    return bool(value.reshape(-1)[0])


def _terminal_info_success(info: Any) -> bool | None:
    """Read an explicit scalar boolean success flag from terminal step info."""
    if not isinstance(info, Mapping) or "success" not in info:
        return None
    import numpy as np

    value = info["success"]
    if not isinstance(value, (bool, np.bool_)):
        raise TypeError("terminal info['success'] must be a boolean scalar")
    return bool(value)


class _FinalizedOnExit:
    """Finalize demos on every exit, dropping unfinished episodes on errors.

    Attach publication failures as notes to the original error. Finalization
    writes demo masks and the ``.srb.json`` manifest.
    """

    def __init__(self, recorder: Any) -> None:
        self._recorder = recorder

    def __enter__(self) -> None:
        return None

    def __exit__(self, exc_type: Any, exc: BaseException | None, tb: Any) -> bool:
        if exc is None:
            self._recorder.finalize()
            return False
        _, error = attempt_cleanup(self._publish_saved_demos)
        if error is not None:
            exc.add_note(f"saved demos were not published: {error!r}")
        return False

    def _publish_saved_demos(self) -> None:
        self._recorder.discard_episode()
        self._recorder.finalize()


def _inference_context() -> Any:
    try:
        import torch
    except ImportError:
        return nullcontext()
    return torch.inference_mode()


def _sim_is_running(sim_app: Any) -> bool:
    if sim_app is None:
        return True
    is_running = getattr(sim_app, "is_running", None)
    return bool(is_running()) if callable(is_running) else True


def _start_source_episode(source: Any) -> None:
    start_episode = getattr(source, "start_episode", None)
    if callable(start_episode):
        start_episode()


def _start_recorder_episode(recorder: Any, env: "AnyEnv") -> None:
    """Start one generic demo and bind it to recorder-owned sidecars."""
    unwrapped = getattr(env, "unwrapped", env)
    for attribute in ("_episode_recorder", "_video_recorder"):
        episode_recorder = getattr(unwrapped, attribute, None)
        if getattr(episode_recorder, "persists_artifacts", True) is False:
            continue
        getter = getattr(episode_recorder, "episode_artifact_reference", None)
        if callable(getter):
            reference = getter(0)
            if isinstance(reference, Mapping) and reference:
                recorder.start_episode(episode_recording=dict(reference))
                return
    recorder.start_episode()


def _source_action(source: Any, obs: Any) -> Any:
    action = getattr(source, "action", None)
    if callable(action):
        return action(obs)
    if callable(source):
        return source(obs)
    raise TypeError("collection action source must define action(obs) or be callable")


def _to_numpy_action(action: Any) -> Any:
    if hasattr(action, "detach"):
        action = action.detach()
    if hasattr(action, "cpu"):
        action = action.cpu()
    if hasattr(action, "numpy"):
        return action.numpy()
    return action


def _single_env_observation(observation: Any) -> Any:
    """Remove the wrapper's ``(1, *shape)`` batch axis for dataset rows."""
    import numpy as np

    if isinstance(observation, Mapping):
        return {
            key: _single_env_observation(value) for key, value in observation.items()
        }
    array = np.asarray(observation)
    if array.ndim == 0 or array.shape[0] != 1:
        raise ValueError(
            "collected observation leaf must carry one environment row, got "
            f"shape {array.shape}"
        )
    return array[0]


def _single_action_shape(action_space: Any, action_dim: int) -> tuple[int, ...]:
    """The shape the recorder expects for one environment's action."""
    import gymnasium

    if isinstance(action_space, gymnasium.spaces.Box):
        return tuple(int(dim) for dim in action_space.shape)
    return (action_dim,)


def _single_env_policy_action(action: Any, single_shape: tuple[int, ...]) -> Any:
    """Remove ``(1, *shape)`` policy batching; recorder rows use ``shape``."""
    shape = getattr(action, "shape", None)
    if shape is not None and tuple(shape) == (1, *single_shape):
        return action[0]
    return action


def _single_env_action(action: Any) -> Any:
    action = _to_numpy_action(action)
    ndim = getattr(action, "ndim", None)
    if ndim is not None and int(ndim) > 1:
        return action[0]
    return action


def _scalar_value(value: Any) -> float:
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "reshape"):
        value = value.reshape(-1)[0]
    if hasattr(value, "item"):
        return float(value.item())
    return float(value)


def _evaluate_success_term(
    *,
    success_term: Any,
    env: "AnyEnv",
    logger: Any,
) -> bool:
    try:
        value = success_term.func(env.unwrapped, **success_term.params)
    except FailureError as exc:
        # A typed failure is authoritative evidence.  Folding it into
        # ``success=False`` would record an integrity/safety/invariant
        # failure as an ordinary unsuccessful episode, so only explicitly
        # fallback-allowed categories may degrade like an untyped hiccup.
        if (
            exc.info.fallback_allowed
            and exc.info.category in FALLBACK_ALLOWED_CATEGORIES
        ):
            logger.warning(
                f"Success term evaluation failed ({exc.info.category.value}): {exc}"
            )
            return False
        raise
    except (RuntimeError, IndexError, KeyError) as exc:
        logger.warning(f"Success term evaluation failed: {exc}")
        return False
    return _truthy_first(value)


def _teleop_collection_request(
    *, event: Any, controls: "TeleopCollectionControls"
) -> str | None:
    """Resolve a step request before recording its terminal flag.

    Teleop events take precedence. Return ``"<source>:<action>"`` or ``None``;
    unconsumed key requests remain pending.
    """

    for action in ("save", "discard", "reset"):
        if _teleop_event_requested(event, action):
            return f"event:{action}"
    if controls.consume_save():
        return "key:save"
    if controls.consume_discard():
        return "key:discard"
    if controls.consume_reset():
        return "key:reset"
    return None


def _teleop_event_requested(event: Any, key: str) -> bool:
    if not hasattr(event, "get"):
        return False
    return bool(event.get(key, False))


def _teleop_event_success(event: Any) -> bool:
    if not hasattr(event, "get"):
        return True
    return bool(event.get("success", True))


def _truthy_first(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "reshape"):
        value = value.reshape(-1)[0]
    if hasattr(value, "item"):
        return bool(value.item())
    try:
        return bool(next(iter(value)))
    except TypeError:
        return bool(value)


__all__ = [
    "BuiltinActionSource",
    "CollectionRunResult",
    "PolicyActionSource",
    "TeleopActionSource",
    "TeleopCollectionControls",
    "run_robomimic_collection",
    "run_robomimic_teleop_collection",
    "success_term_from_env_cfg",
]
