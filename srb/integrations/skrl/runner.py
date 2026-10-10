"""SKRL 2.1 runner registration and SRB configuration translation."""

from __future__ import annotations

import ast
import copy
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any, cast

import torch._dynamo  # noqa: F401
from skrl.resources.noises.torch import GaussianNoise, OrnsteinUhlenbeckNoise
from skrl.trainers.torch import SequentialTrainer
from skrl.utils.runner.torch import Runner

from srb.integrations.skrl.off_policy import SrbDDPG, SrbTD3
from srb.integrations.skrl.rewards import NativeRewardScale
from srb.integrations.skrl.sac import SrbSAC, squashed_gaussian_model

_MISSING = object()

# Share algorithm aliases across translation rules and validation errors.
_SAC_CLASSES = frozenset({"sac", "srb_sac", "srbsac"})
_SPLIT_RATE_CLASSES = _SAC_CLASSES | {"ddpg", "srbddpg", "td3", "srbtd3"}

# SKRL 1.x DDPG/TD3 defaults (1.4.3).
_LEGACY_EXPLORATION_INITIAL_SCALE = 1.0
_LEGACY_EXPLORATION_FINAL_SCALE = 1.0e-3

#: Checkpoint-module name that persists the absolute trainer timestep.
TRAINING_CURSOR_MODULE = "_srb_training_cursor"
_TRAINING_CURSOR_VERSION = 1


def _values_equal(left: Any, right: Any) -> bool:
    """Compare configuration values without invoking tensor truthiness."""

    if isinstance(left, Mapping) and isinstance(right, Mapping):
        return set(left) == set(right) and all(
            _values_equal(left[key], right[key]) for key in left
        )
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        return len(left) == len(right) and all(
            _values_equal(a, b) for a, b in zip(left, right, strict=True)
        )
    try:
        result = left == right
    except (RuntimeError, TypeError, ValueError):
        return left is right
    return bool(result) if isinstance(result, bool) else left is right


def _alias_key(
    section: dict[str, Any],
    old: str,
    new: str,
    *,
    path: str = "agent",
) -> None:
    """Move one legacy key, rejecting a contradictory new value."""

    if old not in section:
        return
    legacy = section.pop(old)
    if new in section:
        if not _values_equal(legacy, section[new]):
            raise ValueError(
                f"conflicting SKRL configuration keys {path}.{old!s} and {path}.{new!s}"
            )
        return
    section[new] = legacy


def _merge_legacy_learning_rates(agent: dict[str, Any], algorithm: str) -> None:
    """Translate SRB's split optimizer-rate names to skrl 2.1 tuples."""

    # Tuning and saved configs still use these split names.
    for old, new in (
        ("learning_rate_policy", "actor_learning_rate"),
        ("learning_rate_value", "critic_learning_rate"),
    ):
        _alias_key(agent, old, new)

    split_names = ("actor_learning_rate", "critic_learning_rate")
    has_split = any(name in agent for name in split_names)
    has_entropy = "entropy_learning_rate" in agent
    if not has_split and not has_entropy:
        return

    if "actor_learning_rate" not in agent or "critic_learning_rate" not in agent:
        if algorithm in _SPLIT_RATE_CLASSES:
            raise ValueError(
                "SKRL split learning rates require both actor_learning_rate "
                "and critic_learning_rate"
            )
        raise ValueError(f"entropy_learning_rate is unsupported for SKRL {algorithm!r}")

    actor = agent.pop("actor_learning_rate")
    critic = agent.pop("critic_learning_rate")
    entropy = agent.pop("entropy_learning_rate", _MISSING)
    expected = [actor, critic]
    if algorithm in _SAC_CLASSES:
        if entropy is _MISSING:
            raise ValueError(
                "SKRL SAC split learning rates require entropy_learning_rate"
            )
        expected.append(entropy)
    elif entropy is not _MISSING:
        raise ValueError(f"entropy_learning_rate is unsupported for SKRL {algorithm!r}")

    current = agent.get("learning_rate", _MISSING)
    if current is _MISSING:
        agent["learning_rate"] = tuple(expected)
        return

    # Accept a scalar only when it matches every split rate.
    if not isinstance(current, (list, tuple)):
        if not all(_values_equal(current, value) for value in expected):
            raise ValueError("conflicting SKRL learning_rate and split optimizer rates")
        agent["learning_rate"] = tuple(expected)
        return
    if not _values_equal(tuple(current), tuple(expected)):
        raise ValueError("conflicting SKRL learning_rate and split optimizer rates")
    agent["learning_rate"] = tuple(current)


def _translate_clip_predicted_values(agent: dict[str, Any]) -> None:
    """Translate the removed boolean value-clipping switch."""

    if "clip_predicted_values" not in agent:
        return
    enabled = agent.pop("clip_predicted_values")
    if not isinstance(enabled, bool):
        raise ValueError("SKRL clip_predicted_values must be boolean")
    expected = agent.get("value_clip", _MISSING)
    if expected is _MISSING:
        agent["value_clip"] = 0.2 if enabled else 0.0
        return
    try:
        expected_value = float(expected)
    except (TypeError, ValueError) as exc:
        raise ValueError("SKRL value_clip must be numeric") from exc
    if enabled and expected_value <= 0:
        raise ValueError(
            "conflicting SKRL clip_predicted_values=True and value_clip<=0"
        )
    if not enabled and expected_value != 0.0:
        raise ValueError(
            "conflicting SKRL clip_predicted_values=False and value_clip!=0"
        )


def _legacy_exploration_scheduler(
    initial: Any,
    final: Any,
    horizon: Any,
) -> Callable[[int, int], float]:
    """Use trainer budget when no horizon is set; return zero after the horizon."""

    try:
        initial_value = float(initial)
        final_value = float(final)
    except (TypeError, ValueError) as exc:
        raise ValueError("SKRL exploration scales must be finite numbers") from exc
    if not math.isfinite(initial_value) or not math.isfinite(final_value):
        raise ValueError("SKRL exploration scales must be finite numbers")
    horizon_value: int | None = None
    if horizon is not None:
        if isinstance(horizon, bool):
            raise ValueError("SKRL exploration timesteps must be an integer")
        try:
            horizon_value = int(horizon)
        except (TypeError, ValueError) as exc:
            raise ValueError("SKRL exploration timesteps must be an integer") from exc
        if horizon_value <= 0:
            raise ValueError("SKRL exploration timesteps must be positive")

    def scheduler(timestep: int, timesteps: int) -> float:
        limit = horizon_value if horizon_value is not None else int(timesteps)
        if timestep > limit:
            return 0.0
        if limit <= 0:
            return initial_value
        return (1.0 - float(timestep) / limit) * (
            initial_value - final_value
        ) + final_value

    return scheduler


def _translate_noise_expression(value: Any) -> Any:
    """Rewrite supported SKRL 1.x positional noise calls for SKRL 2.1."""

    if not isinstance(value, str):
        return value
    try:
        expression = ast.parse(value.strip(), mode="eval").body
    except SyntaxError:
        return value
    if not isinstance(expression, ast.Call) or not isinstance(
        expression.func, ast.Name
    ):
        return value
    name = expression.func.id
    positional = tuple(expression.args)
    if name == "GaussianNoise" and len(positional) == 2:
        if any(keyword.arg is None for keyword in expression.keywords):
            return value
        if expression.keywords:
            # Leave invalid mixed calls for SKRL's native error path.
            return value
        return (
            f"GaussianNoise(mean={ast.unparse(positional[0])}, "
            f"std={ast.unparse(positional[1])})"
        )
    if name == "OrnsteinUhlenbeckNoise" and len(positional) == 3:
        if any(keyword.arg is None for keyword in expression.keywords):
            return value
        keyword_text = ", ".join(
            f"{keyword.arg}={ast.unparse(keyword.value)}"
            for keyword in expression.keywords
            if keyword.arg is not None
        )
        suffix = f", {keyword_text}" if keyword_text else ""
        return (
            f"OrnsteinUhlenbeckNoise(theta={ast.unparse(positional[0])}, "
            f"sigma={ast.unparse(positional[1])}, "
            f"base_scale={ast.unparse(positional[2])}{suffix})"
        )
    return value


def _translate_exploration(agent: dict[str, Any]) -> None:
    """Translate the pre-2.1 nested exploration section."""

    nested = agent.pop("exploration", _MISSING)
    if nested is _MISSING:
        return
    if not isinstance(nested, Mapping):
        raise ValueError("SKRL agent.exploration must be a mapping")
    nested = dict(nested)
    allowed = {"noise", "noise_kwargs", "initial_scale", "final_scale", "timesteps"}
    unknown = sorted(set(nested) - allowed)
    if unknown:
        raise ValueError(
            "unsupported SKRL legacy exploration keys: " + ", ".join(unknown)
        )
    if "noise" not in nested:
        raise ValueError("SKRL legacy exploration requires a noise value")
    if "exploration_noise" in agent:
        if not _values_equal(agent["exploration_noise"], nested["noise"]):
            raise ValueError("conflicting SKRL exploration noise declarations")
    else:
        agent["exploration_noise"] = nested["noise"]
    kwargs = nested.get("noise_kwargs", {})
    if kwargs is None:
        kwargs = {}
    if not isinstance(kwargs, Mapping):
        raise ValueError("SKRL exploration noise_kwargs must be a mapping")
    if "exploration_noise_kwargs" in agent and not _values_equal(
        agent["exploration_noise_kwargs"], kwargs
    ):
        raise ValueError("conflicting SKRL exploration noise kwargs")
    agent.setdefault("exploration_noise_kwargs", dict(kwargs))

    # Preserve SKRL 1.x defaults when fields are omitted.
    initial = nested.get("initial_scale", _LEGACY_EXPLORATION_INITIAL_SCALE)
    final = nested.get("final_scale", _LEGACY_EXPLORATION_FINAL_SCALE)
    horizon = nested.get("timesteps")
    if "exploration_scheduler" in agent:
        raise ValueError(
            "conflicting SKRL legacy exploration and exploration_scheduler"
        )
    # SKRL 2.1's unscheduled noise matches constant unit scale.
    if not (
        _values_equal(initial, 1.0) and _values_equal(final, 1.0) and horizon is None
    ):
        agent["exploration_scheduler"] = _legacy_exploration_scheduler(
            initial, final, horizon
        )


def _translate_model_inputs(models: Any) -> None:
    """Rewrite the removed 1.x ``STATES_ACTIONS`` shorthand."""

    if not isinstance(models, Mapping):
        return
    for model in models.values():
        if not isinstance(model, Mapping):
            continue
        networks = model.get("network")
        if not isinstance(networks, Sequence) or isinstance(networks, (str, bytes)):
            continue
        for entry in networks:
            if not isinstance(entry, Mapping):
                continue
            if entry.get("input") == "STATES_ACTIONS":
                entry["input"] = "concatenate([STATES, ACTIONS])"


def translate_skrl_config(config: Mapping[str, Any]) -> dict[str, Any]:
    """Translate SRB 1.x keys into a copy; reject unsupported SKRL 2.1 keys."""

    if not isinstance(config, Mapping):
        raise TypeError(f"SKRL runner config must be a mapping, got {type(config)!r}")
    translated = copy.deepcopy(dict(config))
    source_agent = translated.get("agent", {})
    if source_agent is None:
        source_agent = {}
    if not isinstance(source_agent, Mapping):
        raise TypeError("SKRL runner config 'agent' must be a mapping")
    agent = dict(source_agent)
    translated["agent"] = agent
    algorithm = str(agent.get("class", "")).strip().lower()

    _alias_key(agent, "lambda", "gae_lambda")
    _translate_clip_predicted_values(agent)
    _alias_key(agent, "value_learning_rate", "learning_rate")
    _alias_key(agent, "value_learning_epochs", "learning_epochs")
    _alias_key(agent, "grad_norm_clip_value", "grad_norm_clip")
    _alias_key(agent, "max_cg_iters", "conjugate_gradient_steps")
    _alias_key(agent, "backtrack_iters", "max_backtrack_steps")
    _alias_key(agent, "policy_update_frequency", "policy_delay")
    _alias_key(agent, "noise_clip", "smooth_regularization_clip")

    if "backtrack_coef" in agent:
        raise ValueError(
            "SKRL 2.1 has no semantics-preserving equivalent for backtrack_coef"
        )

    # Copy the shared preprocessor only when no 2.x observation key exists.
    if "observation_preprocessor" not in agent and "state_preprocessor" in agent:
        agent["observation_preprocessor"] = copy.deepcopy(agent["state_preprocessor"])
        if (
            "state_preprocessor_kwargs" in agent
            and "observation_preprocessor_kwargs" not in agent
        ):
            agent["observation_preprocessor_kwargs"] = copy.deepcopy(
                agent["state_preprocessor_kwargs"]
            )
    _merge_legacy_learning_rates(agent, algorithm)
    _translate_exploration(agent)
    for noise_key in ("exploration_noise", "smooth_regularization_noise"):
        if noise_key in agent:
            agent[noise_key] = _translate_noise_expression(agent[noise_key])
    _translate_model_inputs(translated.get("models"))

    return translated


class TrainingCursorCheckpoint:
    """Persist the absolute completed timestep in SKRL checkpoint modules."""

    def __init__(self, agent: Any) -> None:
        self._agent = agent
        self.restored = False

    def state_dict(self) -> dict[str, int]:
        return {
            "version": _TRAINING_CURSOR_VERSION,
            "timestep": int(getattr(self._agent, "_srb_training_cursor", 0)),
        }

    def load_state_dict(self, state: Any) -> None:
        if not isinstance(state, Mapping):
            raise ValueError("SKRL training cursor checkpoint must be a mapping")
        version = state.get("version")
        timestep = state.get("timestep")
        if version != _TRAINING_CURSOR_VERSION:
            raise ValueError(
                f"unsupported SKRL training cursor checkpoint version {version!r}"
            )
        if not isinstance(timestep, int) or isinstance(timestep, bool) or timestep < 0:
            raise ValueError(
                "SKRL training cursor checkpoint timestep must be a "
                "non-negative integer"
            )
        self._agent._srb_training_cursor = timestep
        self.restored = True


def install_training_cursor(agent: Any) -> TrainingCursorCheckpoint | None:
    """Register the cursor unless separate module files prevent resume."""

    modules = getattr(agent, "checkpoint_modules", None)
    if not isinstance(modules, dict):
        return None
    experiment = getattr(getattr(agent, "cfg", None), "experiment", None)
    if bool(getattr(experiment, "store_separately", False)):
        return None
    existing = modules.get(TRAINING_CURSOR_MODULE)
    if isinstance(existing, TrainingCursorCheckpoint):
        return existing
    agent._srb_training_cursor = int(getattr(agent, "_srb_training_cursor", 0))
    module = TrainingCursorCheckpoint(agent)
    modules[TRAINING_CURSOR_MODULE] = module
    return module


def restored_training_cursor(agent: Any) -> int | None:
    """Return a restored absolute timestep, if available."""

    modules = getattr(agent, "checkpoint_modules", None)
    module = (
        modules.get(TRAINING_CURSOR_MODULE) if isinstance(modules, Mapping) else None
    )
    restored = (
        isinstance(module, TrainingCursorCheckpoint) and module.restored
    ) or bool(getattr(agent, "demo_replay_restored", False))
    if not restored:
        return None
    return int(getattr(agent, "_srb_training_cursor", 0))


class _ResumeAgentView:
    """Shift native trainer callbacks to an absolute resumed timestep."""

    def __init__(self, agent: Any, *, offset: int, timesteps: int) -> None:
        self._agent = agent
        self._offset = offset
        self._timesteps = timesteps

    def __getattr__(self, name: str) -> Any:
        return getattr(self._agent, name)

    def enable_training_mode(self, *args: Any, **kwargs: Any) -> Any:
        return self._agent.enable_training_mode(*args, **kwargs)

    def pre_interaction(self, *, timestep: int, timesteps: int) -> Any:
        del timesteps
        return self._agent.pre_interaction(
            timestep=self._offset + timestep,
            timesteps=self._timesteps,
        )

    def act(
        self,
        observations: Any,
        states: Any,
        *,
        timestep: int,
        timesteps: int,
    ) -> Any:
        del timesteps
        return self._agent.act(
            observations,
            states,
            timestep=self._offset + timestep,
            timesteps=self._timesteps,
        )

    def track_data(self, *args: Any, **kwargs: Any) -> Any:
        return self._agent.track_data(*args, **kwargs)

    def record_transition(self, **kwargs: Any) -> Any:
        shifted = dict(kwargs)
        shifted["timestep"] = self._offset + int(shifted["timestep"])
        shifted["timesteps"] = self._timesteps
        return self._agent.record_transition(**shifted)

    def post_interaction(self, *, timestep: int, timesteps: int) -> Any:
        del timesteps
        absolute = self._offset + timestep
        # Update before serialization so each checkpoint stores its cursor.
        self._agent._srb_training_cursor = absolute + 1
        return self._agent.post_interaction(
            timestep=absolute,
            timesteps=self._timesteps,
        )


def _validate_resume_cursor(cursor: Any) -> int:
    if type(cursor) is not int or cursor < 0:
        raise ValueError("SKRL resume cursor must be a non-negative integer")
    return cursor


def _train_from_cursor(
    trainer: Any,
    cursor: int,
    train: Callable[[], None],
) -> int:
    """Run from an absolute timestep and return the final trainer timestep."""

    if trainer.num_simultaneous_agents != 1:
        if cursor:
            raise RuntimeError(
                "SKRL resume cursor is only supported for one native agent"
            )
        train()
        return cursor
    total_timesteps = int(trainer.cfg.timesteps)
    if cursor >= total_timesteps:
        return cursor
    original_agents = trainer.agents
    original_timesteps = trainer.cfg.timesteps
    trainer.cfg.timesteps = total_timesteps - cursor
    trainer.agents = _ResumeAgentView(
        original_agents,
        offset=cursor,
        timesteps=total_timesteps,
    )
    try:
        train()
    finally:
        trainer.agents = original_agents
        trainer.cfg.timesteps = original_timesteps
    return total_timesteps


class _ResumableSequentialTrainer(SequentialTrainer):
    """Keep SKRL 2.1's trainer loop while adding an absolute resume cursor."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._srb_resume_cursor = 0

    def set_resume_cursor(self, cursor: int) -> None:
        self._srb_resume_cursor = _validate_resume_cursor(cursor)

    def train(self) -> None:
        self._srb_resume_cursor = _train_from_cursor(
            self,
            self._srb_resume_cursor,
            super().train,
        )


class SrbSkrlRunner(Runner):  # type: ignore[misc]
    """SKRL Runner with explicit registration for SRB-owned agents/models."""

    def __init__(self, env: Any, cfg: Mapping[str, Any]) -> None:
        # Translate first to preserve legacy observation/state semantics.
        super().__init__(env, translate_skrl_config(cfg))
        self._srb_resume_cursor = 0
        install_training_cursor(self.agent)

    def run(self, mode: str = "train") -> None:
        """Run SKRL, starting training at the configured absolute cursor."""

        if mode != "train":
            super().run(mode)  # type: ignore[arg-type]
            return
        trainer = self._trainer
        if isinstance(trainer, _ResumableSequentialTrainer):
            trainer.train()
        elif isinstance(trainer, SequentialTrainer):
            _train_from_cursor(trainer, self._srb_resume_cursor, trainer.train)
        elif self._srb_resume_cursor:
            raise RuntimeError("SKRL trainer does not support resume cursors")
        else:
            super().run(mode)  # type: ignore[arg-type]

    def _process_cfg(self, cfg: dict[str, Any]) -> dict[str, Any]:
        """Process configs while retaining SRB reward and noise contracts."""

        # Keep the SRB-only scale out of SKRL's dataclass config.
        source = copy.deepcopy(cfg)
        scale = source.pop("rewards_shaper_scale", None)
        scheduler = source.pop("exploration_scheduler", _MISSING)
        if isinstance(scheduler, str):
            source["exploration_scheduler"] = scheduler
        processed = cast(dict[str, Any], super()._process_cfg(source))
        self._normalise_noise_factories(processed)
        if scheduler is not _MISSING and scheduler is not None:
            if not isinstance(scheduler, str):
                processed["exploration_scheduler"] = scheduler
        if scale is not None:
            try:
                scale_value = float(scale)
            except (TypeError, ValueError) as exc:
                raise ValueError("SKRL rewards_shaper_scale must be numeric") from exc
            if not math.isfinite(scale_value):
                raise ValueError("SKRL rewards_shaper_scale must be finite")
            if "rewards_shaper" in processed and scale_value != 1.0:
                raise ValueError(
                    "SKRL rewards_shaper and rewards_shaper_scale conflict"
                )
            if scale_value != 1.0:
                processed["rewards_shaper"] = NativeRewardScale(scale_value)
        return processed

    @staticmethod
    def _normalise_noise_factories(cfg: dict[str, Any]) -> None:
        """Wrap native noise instances as factories that accept the runtime device."""

        def factory_for(value: Any) -> Any:
            if callable(value):
                return value
            if type(value) not in {GaussianNoise, OrnsteinUhlenbeckNoise}:
                return value
            distribution = getattr(value, "distribution", None)
            loc = getattr(distribution, "loc", None)
            scale = getattr(distribution, "scale", None)
            if (
                not hasattr(loc, "detach")
                or not hasattr(scale, "detach")
                or loc.numel() != 1
                or scale.numel() != 1
            ):
                raise ValueError(
                    "SKRL native noise factories require scalar distribution parameters"
                )
            mean = float(loc.detach().cpu().item())
            std = float(scale.detach().cpu().item())
            if type(value) is GaussianNoise:
                return lambda *, device=None, _mean=mean, _std=std: GaussianNoise(
                    mean=_mean, std=_std, device=device
                )
            if type(value) is OrnsteinUhlenbeckNoise:
                theta = float(value.theta)
                sigma = float(value.sigma)
                base_scale = float(value.base_scale)
                return (
                    lambda *, device=None, _theta=theta, _sigma=sigma, _base_scale=base_scale, _mean=mean, _std=std: (
                        OrnsteinUhlenbeckNoise(
                            theta=_theta,
                            sigma=_sigma,
                            base_scale=_base_scale,
                            mean=_mean,
                            std=_std,
                            device=device,
                        )
                    )
                )
            return value

        # Materialize kwargs so SKRL's runtime device reaches each noise factory.
        for holder, key in (
            (cfg, "exploration_noise"),
            (cfg, "smooth_regularization_noise"),
            (cfg, "noise"),
            (cfg.get("exploration"), "noise"),
        ):
            if isinstance(holder, dict) and key in holder:
                holder[key] = factory_for(holder[key])
                holder.setdefault(f"{key}_kwargs", {})

    def _component(self, name: str) -> Any:
        normalized = name.lower()
        if normalized == "sequentialtrainer":
            return _ResumableSequentialTrainer
        if normalized in {
            "srbsquashedgaussianmixin",
            "srbsquashedgaussian",
        }:
            return squashed_gaussian_model
        if normalized in {"sac", "srbsac"}:
            return SrbSAC
        if normalized in {"ddpg", "srbddpg"}:
            return SrbDDPG
        if normalized in {"td3", "srbtd3"}:
            return SrbTD3
        return super()._component(name)

    def set_resume_cursor(self, cursor: int) -> None:
        """Set the absolute timestep used by a resumed native trainer."""

        cursor = _validate_resume_cursor(cursor)
        trainer = self.trainer
        if isinstance(trainer, _ResumableSequentialTrainer):
            trainer.set_resume_cursor(cursor)
        elif cursor and not isinstance(trainer, SequentialTrainer):
            raise RuntimeError("SKRL trainer does not support resume cursors")
        self._srb_resume_cursor = cursor


def runner_for_config(
    env: Any,
    config: Mapping[str, Any],
) -> Runner:
    """Construct the registered runner for an SRB SKRL agent config."""
    agent_section = config.get("agent", {})
    if not isinstance(agent_section, Mapping):
        raise TypeError("SKRL runner config 'agent' must be a mapping")
    agent_class = str(agent_section.get("class", "")).lower()
    if agent_class in {"srbrecurrentppo", "srbrppo", "ppo_rnn"}:
        # Keep the optional recurrent runner lazy to avoid an import cycle.
        from srb.integrations.skrl.ppo_rnn import SrbPpoRnnRunner

        return SrbPpoRnnRunner(env, config)
    return SrbSkrlRunner(env, config)


__all__ = [
    "TRAINING_CURSOR_MODULE",
    "SrbSkrlRunner",
    "TrainingCursorCheckpoint",
    "install_training_cursor",
    "restored_training_cursor",
    "runner_for_config",
    "translate_skrl_config",
]
