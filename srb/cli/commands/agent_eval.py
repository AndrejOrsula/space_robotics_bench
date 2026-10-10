"""CLI adapter for evaluating trained agents."""

from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Any

from srb.cli.commands import agent_policy as _policy_cli
from srb.core.env.runtime.action_space import (
    action_space_shape as _shared_action_space_shape,
)
from srb.core.env.runtime.action_space import (
    coerce_policy_action as _shared_coerce_policy_action,
)
from srb.core.env.runtime.action_space import (
    coerce_policy_action_for_env as _shared_coerce_policy_action_for_env,
)
from srb.core.env.runtime.action_space import (
    env_action_shape as _shared_env_action_shape,
)
from srb.core.env.runtime.action_space import (
    env_action_space as _shared_env_action_space,
)
from srb.core.env.runtime.action_space import (
    iter_mapping_action as _shared_iter_mapping_action,
)
from srb.core.env.runtime.action_space import (
    reshape_action as _shared_reshape_action,
)
from srb.core.env.runtime.action_space import (
    validate_mapping_action_keys as _shared_validate_mapping_action_keys,
)
from srb.core.env.runtime.step_return import truthy_any as _truthy_any
from srb.experiments.policies import infer_algo_from_model
from srb.integrations.eval_quota import (
    EpisodeQuota,
    effective_eval_seed,
    eval_episode_budget,
    eval_max_steps,
    finite_mean,
    write_eval_result,
)

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv


_resolve_model_artifact_policy_inputs = _policy_cli.resolve_model_artifact_policy_inputs
_resolve_model_artifact_reference = _policy_cli.resolve_model_artifact_reference
_load_policy = _policy_cli.load_policy


def eval_agent(
    algo: str | None = None, model: Path | str | None = None, **kwargs: Any
) -> None:
    """Evaluate a trained agent."""
    from srb.experiments.workflows import run_policy_evaluation

    artifact_reference = _resolve_model_artifact_reference(algo=algo, model=model)
    if (
        artifact_reference is not None
        and kwargs.get("env") is not None
        and kwargs.get("sim_app") is not None
    ):
        try:
            _eval_agent_policy(
                algo=artifact_reference.algo,
                model=artifact_reference.checkpoint_path,
                artifact_reference=artifact_reference,
                **kwargs,
            )
        finally:
            close = getattr(artifact_reference, "close", None)
            if callable(close):
                close()
        return

    algo, model = _resolve_model_artifact_policy_inputs(algo=algo, model=model)
    if algo is None:
        if model is None:
            raise ValueError(
                "Either --algo or --model must be provided for evaluation. "
                "Use --algo to specify the algorithm or --model to auto-detect from checkpoint path."
            )
        algo = infer_algo_from_model(model)
        from srb.utils import logging

        logging.info(f"Inferred algorithm '{algo}' from model path: {model}")

    run_policy_evaluation(algo=algo, model=model, **kwargs)


def _eval_agent_policy(
    *,
    env: "AnyEnv",
    sim_app: "SimulationApp",
    algo: str,
    model: Path | str,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    evaluation_logdir: Path | None = None,
    artifact_reference: Any | None = None,
    **_kwargs: Any,
) -> dict[str, Any] | None:
    import torch

    from srb.utils import logging

    target_episodes = eval_episode_budget()
    model_path = Path(model)
    logging.info(f"Loading policy from: {model_path}")
    load_kwargs: dict[str, Any] = {
        "agent_cfg": agent_cfg,
        "env_cfg": env_cfg,
        "logdir": logdir,
    }
    if artifact_reference is not None:
        load_kwargs["artifact_reference"] = artifact_reference
    policy_fn, start_episode_fn, deterministic = _load_policy(
        algo,
        model_path,
        env,
        **load_kwargs,
    )

    if target_episodes is not None:
        if not deterministic:
            raise ValueError(
                "bounded SRB policy evaluation requires deterministic inference"
            )
        with torch.inference_mode():
            result = _run_bounded_eval(
                policy_fn,
                start_episode_fn,
                env,
                sim_app,
                deterministic=deterministic,
                target_episodes=target_episodes,
                reset_seed=effective_eval_seed(env_cfg),
            )
        write_eval_result(result, evaluation_logdir=evaluation_logdir)
        return result

    observation, _info = env.reset()
    start_episode_fn()
    with torch.inference_mode():
        while sim_app.is_running():
            action = policy_fn(observation)
            env_action = _coerce_policy_action_for_env(action, env)
            observation, reward, terminated, truncated, info = env.step(env_action)
            trace = getattr(
                logging,
                "trace",
                getattr(logging, "debug", lambda *_: None),
            )
            trace(
                f"action: {env_action}\n"
                f"observation: {observation}\n"
                f"reward: {reward}\n"
                f"terminated: {terminated}\n"
                f"truncated: {truncated}\n"
                f"info: {info}\n"
            )
            done_mask = _done_env_mask(terminated, truncated)
            if done_mask is not None:
                # Per-env reset: only start a new episode for the envs that
                # finished (a recurrent policy resets just those; others full-reset
                # via the default None). Avoids wiping running envs at num_envs>1.
                if bool(done_mask.any()):
                    start_episode_fn(done_mask)
            elif _truthy_any(terminated) or _truthy_any(truncated):
                start_episode_fn()
    return None


def _run_bounded_eval(
    policy_fn: Any,
    start_episode_fn: Any,
    env: "AnyEnv",
    sim_app: "SimulationApp",
    *,
    deterministic: bool,
    target_episodes: int,
    reset_seed: Any | None = None,
) -> dict[str, Any]:
    """Run a generic loaded SRB policy over exactly N completed episodes.

    Episodes are selected by a fixed per-env quota of episodes started after
    the reset below (:class:`srb.integrations.eval_quota.EpisodeQuota`), not
    by completion order, so short episodes are not over-represented.
    """
    if target_episodes <= 0:
        raise ValueError(f"target_episodes must be positive, got {target_episodes!r}")
    if not deterministic:
        raise ValueError(
            "bounded SRB policy evaluation requires deterministic inference"
        )

    num_envs = _eval_num_envs(env)
    max_steps = eval_max_steps(env, target_episodes)
    quota = EpisodeQuota(target_episodes, num_envs)
    episode_returns = _zeros(num_envs)
    completed_returns: list[float] = []
    success_values: list[bool | None] = []
    metric_sums = dict.fromkeys(_EVAL_METRIC_NAMES, 0.0)
    metric_counts = dict.fromkeys(_EVAL_METRIC_NAMES, 0)
    if reset_seed is None:
        observation, _info = env.reset()
    else:
        observation, _info = env.reset(seed=reset_seed)
    start_episode_fn()
    step_count = 0
    stop_reason: str | None = None

    while not quota.complete and step_count < max_steps:
        if not bool(sim_app.is_running()):
            stop_reason = "sim_stopped"
            break
        action = policy_fn(observation)
        env_action = _coerce_policy_action_for_env(action, env)
        observation, reward, terminated, truncated, info = env.step(env_action)
        step_count += 1

        reward_values = _per_env_float_values(reward, num_envs)
        if reward_values is None:
            raise ValueError(
                "SRB evaluation expected one reward per environment, "
                f"got an invalid value for {num_envs} environments"
            )
        for env_id, reward_value in enumerate(reward_values):
            episode_returns[env_id] += reward_value
            if not _is_finite(episode_returns[env_id]):
                raise ValueError("SRB evaluation episode return became non-finite")

        done_values, scalar_done = _per_env_done_values(terminated, truncated, num_envs)
        if done_values is None:
            raise ValueError(
                "SRB evaluation expected one done flag per environment, "
                f"got an invalid value for {num_envs} environments"
            )
        done_ids = [env_id for env_id, done in enumerate(done_values) if done]
        if done_ids:
            success_by_env = _per_env_success(info, num_envs)
            metrics_by_name = _per_env_metrics(info, num_envs)
            for env_id in quota.accept(done_ids):
                completed_returns.append(episode_returns[env_id])
                success_values.append(success_by_env[env_id])
                for metric_name, values in metrics_by_name.items():
                    if values is None:
                        continue
                    metric_value = values[env_id]
                    if metric_value is None:
                        continue
                    metric_sums[metric_name] += metric_value
                    if not _is_finite(metric_sums[metric_name]):
                        raise ValueError(
                            "SRB evaluation produced a non-finite episode metric"
                        )
                    metric_counts[metric_name] += 1
            for env_id in done_ids:
                # Rows that finished outside their quota restart too.
                episode_returns[env_id] = 0.0

            if scalar_done and num_envs == 1:
                start_episode_fn()
            else:
                start_episode_fn(_as_bool_array(done_values))

    if quota.complete:
        stop_reason = None
    elif step_count >= max_steps:
        stop_reason = "step_budget"
    elif stop_reason is None:
        stop_reason = "sim_stopped"

    complete = quota.complete
    if not complete:
        from srb.utils import logging

        logging.warning(
            "SRB policy evaluation incomplete: %s/%s episodes after %s/%s steps (%s)",
            len(completed_returns),
            target_episodes,
            step_count,
            max_steps,
            stop_reason,
        )
    has_success = bool(success_values) and all(
        value is not None for value in success_values
    )
    successes = sum(bool(value) for value in success_values) if has_success else None
    result: dict[str, Any] = {
        "success_rate": (
            successes / len(completed_returns)
            if successes is not None and completed_returns
            else None
        ),
        "successes": successes,
        "failures": (
            len(completed_returns) - successes if successes is not None else None
        ),
        "mean_return": (
            finite_mean(completed_returns, "SRB") if completed_returns else None
        ),
        "n_episodes": len(completed_returns),
        "episodes_requested": target_episodes,
        "episodes_completed": len(completed_returns),
        "complete": complete,
        "incomplete": not complete,
        "steps": step_count,
        "max_steps": max_steps,
        "stop_reason": stop_reason,
        **quota.result_fields(),
    }
    result.update(
        {
            output_name: (
                metric_sums[metric_name] / metric_counts[metric_name]
                if metric_counts[metric_name]
                else None
            )
            for metric_name, output_name in _EVAL_METRIC_NAMES.items()
        }
    )
    return result


_EVAL_METRIC_NAMES = {
    "formation_distance_error": "mean_distance_error",
    "formation_orientation_error": "mean_yaw_error",
    "success_hold_steps": "mean_hold_steps_at_done",
}


def _eval_num_envs(env: Any) -> int:
    raw_num_envs = getattr(getattr(env, "unwrapped", env), "num_envs", 1)
    if (
        isinstance(raw_num_envs, bool)
        or not isinstance(raw_num_envs, int)
        or raw_num_envs <= 0
    ):
        raise ValueError(
            f"SRB evaluation requires a positive num_envs, got {raw_num_envs}"
        )
    return raw_num_envs


def _zeros(size: int) -> list[float]:
    return [0.0] * size


def _as_numpy(value: Any) -> Any:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return value


def _per_env_values(value: Any, num_envs: int) -> list[Any] | None:
    import numpy as np

    if isinstance(value, Mapping) or isinstance(value, (str, bytes)):
        return None
    try:
        array = np.asarray(_as_numpy(value))
    except (TypeError, ValueError):
        return None
    if array.ndim == 0:
        return [array.item()] if num_envs == 1 else None
    flattened = array.reshape(-1)
    if flattened.size != num_envs:
        return None
    return flattened.tolist()


def _per_env_float_values(value: Any, num_envs: int) -> list[float] | None:
    import numpy as np

    values = _per_env_values(value, num_envs)
    if values is None:
        return None
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return None
    if array.size != num_envs or not bool(np.isfinite(array).all()):
        raise ValueError("SRB evaluation received a non-finite reward")
    return [float(item) for item in array.tolist()]


def _per_env_done_values(
    terminated: Any, truncated: Any, num_envs: int
) -> tuple[list[bool] | None, bool]:
    terminated_values = _per_env_values(terminated, num_envs)
    truncated_values = _per_env_values(truncated, num_envs)
    scalar_done = _is_scalar_value(terminated) and _is_scalar_value(truncated)
    if terminated_values is None or truncated_values is None:
        return None, scalar_done
    return [
        bool(term) or bool(trunc)
        for term, trunc in zip(terminated_values, truncated_values)
    ], scalar_done


def _is_scalar_value(value: Any) -> bool:
    import numpy as np

    if isinstance(value, Mapping) or isinstance(value, (str, bytes)):
        return False
    try:
        return np.asarray(_as_numpy(value)).ndim == 0
    except (TypeError, ValueError):
        return False


def _as_bool_array(values: list[bool]) -> Any:
    import numpy as np

    return np.asarray(values, dtype=bool)


def _scalar_success(value: Any) -> bool | None:
    value = _as_numpy(value)
    if hasattr(value, "item"):
        try:
            value = value.item()
        except (ValueError, TypeError):
            return None
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value) if value in (0, 1) else None
    if isinstance(value, float):
        return bool(value) if math.isfinite(value) and value in (0.0, 1.0) else None
    return None


def _per_env_success(info: Any, num_envs: int) -> list[bool | None]:
    if isinstance(info, (list, tuple)):
        if len(info) != num_envs:
            return [None] * num_envs
        return [
            _scalar_success(item.get("success")) if isinstance(item, Mapping) else None
            for item in info
        ]
    if not isinstance(info, Mapping) or "success" not in info:
        return [None] * num_envs
    values = _per_env_values(info["success"], num_envs)
    if values is None:
        return [None] * num_envs
    return [_scalar_success(value) for value in values]


def _per_env_metrics(info: Any, num_envs: int) -> dict[str, list[float | None] | None]:
    result: dict[str, list[float | None] | None] = {}
    for metric_name in _EVAL_METRIC_NAMES:
        values: Any = None
        if isinstance(info, (list, tuple)):
            if len(info) == num_envs:
                values = [
                    item.get("episode_metrics", {}).get(metric_name)
                    if isinstance(item, Mapping)
                    and isinstance(item.get("episode_metrics"), Mapping)
                    else None
                    for item in info
                ]
        elif isinstance(info, Mapping):
            metrics = info.get("episode_metrics")
            if isinstance(metrics, Mapping):
                values = metrics.get(metric_name)
                if values is None:
                    values = metrics.get(_EVAL_METRIC_NAMES[metric_name])
        raw_values = _per_env_values(values, num_envs) if values is not None else None
        if raw_values is None:
            result[metric_name] = None
            continue
        converted: list[float | None] = []
        for value in raw_values:
            try:
                numeric = float(value)
            except (TypeError, ValueError, OverflowError):
                converted.append(None)
            else:
                converted.append(numeric if _is_finite(numeric) else None)
        result[metric_name] = converted
    return result


def _is_finite(value: float) -> bool:
    return math.isfinite(float(value))


def _coerce_policy_action_for_env(action: Any, env: "AnyEnv") -> Any:
    return _shared_coerce_policy_action_for_env(action, env)


def _coerce_policy_action(
    action: Any,
    action_space: Any,
    device: Any,
    *,
    np: Any = None,
    torch: Any = None,
) -> Any:
    return _shared_coerce_policy_action(action, action_space, device)


def _iter_policy_mapping_action(
    action: Mapping[Any, Any],
    spaces: Any,
) -> list[tuple[Any, Any, Any | None]]:
    if isinstance(spaces, Mapping):
        _shared_validate_mapping_action_keys(action, spaces)
        return [(key, action[key], subspace) for key, subspace in spaces.items()]
    return _shared_iter_mapping_action(action, spaces)


def _validate_policy_mapping_action_keys(
    action: Mapping[Any, Any], spaces: Mapping[Any, Any]
) -> None:
    _shared_validate_mapping_action_keys(action, spaces)


def _policy_env_action_space(env: "AnyEnv") -> Any:
    return _shared_env_action_space(env)


def _policy_env_action_shape(env: "AnyEnv") -> tuple[int, ...] | None:
    return _shared_env_action_shape(env)


def _policy_action_space_shape(action_space: Any) -> tuple[int, ...] | None:
    return _shared_action_space_shape(action_space)


def _reshape_policy_action(action: Any, target_shape: tuple[int, ...] | None) -> Any:
    return _shared_reshape_action(action, target_shape)


def _done_env_mask(terminated: Any, truncated: Any) -> "Any | None":
    """Return a ``(num_envs,)`` bool mask, or ``None`` for scalar signals.

    Recurrent policies use the mask to reset only completed envs.
    """
    import numpy as np

    def _as_bool(x: Any) -> Any | None:
        if isinstance(x, bool):
            return None
        if hasattr(x, "detach"):
            x = x.detach().cpu().numpy()
        if isinstance(x, np.ndarray):
            return np.asarray(x, dtype=bool).reshape(-1)
        return None

    mask = None
    for part in (_as_bool(terminated), _as_bool(truncated)):
        if part is None:
            continue
        mask = part if mask is None else (mask | part)
    return mask


__all__ = (
    "eval_agent",
    "_coerce_policy_action",
    "_coerce_policy_action_for_env",
    "_eval_agent_policy",
    "_iter_policy_mapping_action",
    "_load_policy",
    "_policy_action_space_shape",
    "_policy_env_action_shape",
    "_policy_env_action_space",
    "_reshape_policy_action",
    "_resolve_model_artifact_policy_inputs",
    "_resolve_model_artifact_reference",
    "_truthy_any",
    "_validate_policy_mapping_action_keys",
)
