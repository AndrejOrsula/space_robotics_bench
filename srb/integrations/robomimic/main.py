# SPDX-License-Identifier: Apache-2.0 OR MIT
"""Robomimic training and evaluation workflows."""

import math
from copy import deepcopy
from pathlib import Path
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    from typing import Any

    import gymnasium
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv, AnyEnvCfg

# Fallback seed when caller and environment config omit one.
DEFAULT_SEED = 42


def run(
    workflow: Literal["learn", "eval"],
    env: "AnyEnv | gymnasium.Env",
    sim_app: "SimulationApp",
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: "dict[str, Any] | None",
    logdir: Path,
    model: "Path | None" = None,
    dataset: "Path | str | None" = None,
    algo: str = "bc",
    horizon: int = 400,
    num_rollouts: int = 50,
    seed: "int | None" = None,
    normalize_actions: bool = False,
    **kwargs,
) -> "dict[str, Any] | None":
    """Run offline training or policy evaluation."""
    kwargs.pop("run_metadata", None)

    from srb.utils import logging
    from srb.utils.path import stamp_dir

    workflow_seed = DEFAULT_SEED if seed is None else seed
    match workflow:
        case "learn":
            if dataset is None:
                dataset = _find_dataset(logdir)
                if dataset is None:
                    raise ValueError(
                        "No training dataset found in logdir; pass --dataset or "
                        "run the 'collect' workflow first."
                    )
            logging.info(f"Training on dataset: {dataset}")

            _run_learn(
                dataset=dataset,
                algo=algo,
                agent_cfg=agent_cfg,
                logdir=logdir,
                normalize_actions=normalize_actions,
                seed=workflow_seed,
                **kwargs,
            )

        case "eval":
            if model is None:
                model = _find_latest_checkpoint(logdir)
                if model is None:
                    raise ValueError(
                        "No model checkpoint found in logdir; pass --model or "
                        "run the 'learn' workflow first."
                    )
            logging.info(f"Evaluating checkpoint: {model}")

            return _run_eval(
                env=env,
                algo=algo,
                sim_app=sim_app,
                env_id=env_id,
                env_cfg=env_cfg,
                logdir=stamp_dir(logdir.joinpath("eval")),
                model=model,
                num_rollouts=num_rollouts,
                horizon=horizon,
                seed=seed,
                **kwargs,
            )
    return None


def _run_learn(
    dataset: "str | Path",
    algo: str,
    agent_cfg: "dict[str, Any] | None",
    logdir: Path,
    normalize_actions: bool = False,
    seed: int = 42,
    num_epochs: "int | None" = None,
    **kwargs,
) -> Path:
    """Run offline training and return the checkpoint directory."""
    from srb.integrations.robomimic.train import run_training
    from srb.utils import logging

    agent_cfg = deepcopy(agent_cfg) if agent_cfg is not None else None
    config_path = None
    if agent_cfg and "config_path" in agent_cfg:
        config_path = agent_cfg.pop("config_path")

    if config_path is None:
        from srb.utils.path import SRB_HYPERPARAMS_DIR

        hyperparams_dir = SRB_HYPERPARAMS_DIR.joinpath("robomimic")
        config_file = hyperparams_dir.joinpath(f"{algo}.json")
        if not config_file.exists():
            algo_base = algo.split("_")[0]
            config_file = hyperparams_dir.joinpath(f"{algo_base}.json")
        if config_file.exists():
            config_path = config_file
            logging.info(f"Using config from: {config_path}")

    return run_training(
        dataset_path=str(dataset),
        algo=algo,
        logdir=logdir,
        config_path=config_path,
        config_overrides=agent_cfg,
        normalize_actions=normalize_actions,
        num_epochs=num_epochs,
        seed=seed,
    )


def _run_eval(
    env: "AnyEnv | gymnasium.Env",
    sim_app: "SimulationApp",
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    logdir: Path,
    model: Path,
    num_rollouts: int = 50,
    horizon: int = 400,
    seed: "int | None" = None,
    evaluation_logdir: "Path | None" = None,
    algo: str = "bc",
    **kwargs,
) -> "dict[str, Any]":
    """Evaluate one environment and return recorded metrics.

    ``SRB_EVAL_EPISODES=N`` targets N episodes, subject to the shared step guard;
    horizon does not cut bounded episodes. Otherwise, run ``num_rollouts`` of
    at most ``horizon`` steps.
    Success uses the ending step's task ``success`` episode-info signal. If
    absent, report a warning and ``None`` success rate. The result records
    ``seed``, then ``env.seed``, then ``DEFAULT_SEED`` in precedence order.
    """
    import random

    import numpy as np
    import torch

    from srb.integrations.eval_quota import (
        EpisodeQuota,
        effective_eval_seed,
        eval_episode_budget,
        eval_max_steps,
        finite_mean,
        write_eval_result,
    )
    from srb.integrations.policies import load_policy
    from srb.utils import logging
    from srb.wrappers.robomimic import RobomimicEnvWrapper

    _reject_caller_action_range(kwargs)
    budget = eval_episode_budget()
    bounded = budget is not None
    if budget is None:
        for name, value in (("num_rollouts", num_rollouts), ("horizon", horizon)):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer, got {value!r}")
    target_episodes: int = num_rollouts if budget is None else budget
    episode_step_limit = None if bounded else horizon
    max_steps = (
        eval_max_steps(env, target_episodes) if bounded else target_episodes * horizon
    )
    if seed is None:
        seed = effective_eval_seed(env_cfg)
    if seed is None:
        seed = DEFAULT_SEED

    # Observations and actions pass through the collector's own projection,
    # so the policy sees what the dataset recorded. The wrapper rejects
    # ``num_envs != 1`` before any policy is loaded.
    wrapped_env = RobomimicEnvWrapper(env)
    policy_fn, start_episode_fn, deterministic = load_policy(
        f"robomimic_{algo}",
        Path(model),
        env,
        env_cfg=env_cfg,
        logdir=logdir,
    )
    if bounded and deterministic is not True:
        raise ValueError(
            "robomimic bounded evaluation requires explicitly deterministic "
            "policy inference"
        )

    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)
    wrapped_env.seed(seed)

    quota = EpisodeQuota(target_episodes, 1)
    results: list[bool | None] = []
    returns: list[float] = []
    horizon_cuts = 0
    step_count = 0
    stop_reason: str | None = None
    with torch.inference_mode():
        while not quota.complete and stop_reason is None:
            if not sim_app.is_running():
                stop_reason = "sim_stopped"
                break
            obs, _info = wrapped_env.reset()
            start_episode_fn()
            episode_return = 0.0
            episode_steps = 0
            success: bool | None = None
            while True:
                if step_count >= max_steps:
                    stop_reason = "step_budget"
                    break
                if not sim_app.is_running():
                    stop_reason = "sim_stopped"
                    break
                action = policy_fn(obs)
                obs, reward, terminated, truncated, info = wrapped_env.step(action)
                step_count += 1
                episode_steps += 1
                episode_return += _env0_float(reward, "reward")
                if not math.isfinite(episode_return):
                    raise ValueError(
                        "robomimic evaluation episode return became non-finite"
                    )
                done = _env0_bool(terminated) or _env0_bool(truncated)
                success = _env0_success(info)
                if done:
                    break
                if episode_step_limit is not None and (
                    episode_steps >= episode_step_limit
                ):
                    horizon_cuts += 1
                    break
            if stop_reason is not None:
                break
            quota.accept([0])
            results.append(success)
            returns.append(episode_return)
            logging.info(
                f"Episode {len(results)}/{target_episodes}: "
                f"return={episode_return:.3f}, success={success}"
            )

    complete = quota.complete
    if complete:
        stop_reason = None
    else:
        logging.warning(
            f"robomimic evaluation incomplete: {len(results)}/{target_episodes} "
            f"episodes after {step_count}/{max_steps} steps ({stop_reason})"
        )
    has_signal = bool(results) and all(value is not None for value in results)
    successes = sum(bool(value) for value in results) if has_signal else None
    if results and not has_signal:
        logging.warning(
            f"robomimic evaluation of {env_id}: the task did not publish a "
            "boolean `success` episode-info signal for every episode; the "
            "success rate is not reported (returns only)."
        )
    result: dict[str, Any] = {
        "success_rate": successes / len(results) if successes is not None else None,
        "successes": successes,
        "failures": len(results) - successes if successes is not None else None,
        "mean_return": finite_mean(returns, "robomimic") if returns else None,
        "n_episodes": len(results),
        "episodes_requested": target_episodes,
        "episodes_completed": len(results),
        "complete": complete,
        "incomplete": not complete,
        "steps": step_count,
        "max_steps": max_steps,
        "stop_reason": stop_reason,
        **quota.result_fields(),
        "success_signal": "episode_info" if has_signal else None,
        "seed": seed,
        "horizon": episode_step_limit,
        "horizon_cut_episodes": horizon_cuts,
        "results": results,
        "returns": returns,
        "checkpoint": str(model),
    }
    write_eval_result(result, evaluation_logdir=evaluation_logdir)
    return result


def _reject_caller_action_range(kwargs: "dict[str, Any]") -> None:
    """Refuse an action range that does not come from the checkpoint."""
    supplied = sorted(
        key
        for key in ("norm_factor_min", "norm_factor_max")
        if kwargs.get(key) is not None
    )
    if supplied:
        raise ValueError(
            f"robomimic {supplied} cannot be supplied by the caller: the "
            "--normalize_actions range is read from the checkpoint's "
            "env_metadata, which records the range the policy was trained on"
        )


def _env0_values(value: "Any") -> "Any":
    import numpy as np

    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value).reshape(-1)


def _env0_float(value: "Any", name: str) -> float:
    values = _env0_values(value)
    if values.size != 1:
        raise ValueError(f"robomimic evaluation expected one {name}, got {values.size}")
    number = float(values[0])
    if not math.isfinite(number):
        raise ValueError(f"robomimic evaluation received a non-finite {name}")
    return number


def _env0_bool(value: "Any") -> bool:
    values = _env0_values(value)
    if values.size != 1:
        raise ValueError(
            f"robomimic evaluation expected one done flag, got {values.size}"
        )
    return bool(values[0])


def _env0_success(info: "Any") -> "bool | None":
    """The task's ``success`` episode info: a boolean or binary value, else None."""
    from collections.abc import Mapping

    from srb.core.env.runtime.episode_info import EPISODE_SUCCESS_KEY

    if not isinstance(info, Mapping) or EPISODE_SUCCESS_KEY not in info:
        return None
    try:
        values = _env0_values(info[EPISODE_SUCCESS_KEY])
    except (TypeError, ValueError):
        return None
    if values.size != 1:
        return None
    value = values[0].item()
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value) if value in (0, 1) else None
    if isinstance(value, float):
        return bool(value) if math.isfinite(value) and value in (0.0, 1.0) else None
    return None


def _find_dataset(logdir: Path) -> "Path | None":
    """Return the newest dataset in the log directory."""
    datasets_dir = logdir.joinpath("datasets")
    if datasets_dir.exists():
        hdf5_files = sorted(
            datasets_dir.glob("*.hdf5"), key=lambda x: x.stat().st_mtime
        )
        if hdf5_files:
            return hdf5_files[-1]

    hdf5_files = sorted(logdir.glob("*.hdf5"), key=lambda x: x.stat().st_mtime)
    if hdf5_files:
        return hdf5_files[-1]

    return None


def _find_latest_checkpoint(logdir: Path) -> "Path | None":
    """Return the newest Robomimic checkpoint in the log directory."""
    for pattern in ["**/*.pth"]:
        checkpoints = sorted(logdir.glob(pattern), key=lambda x: x.stat().st_mtime)
        if checkpoints:
            return checkpoints[-1]

    return None
