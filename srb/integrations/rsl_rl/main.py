from __future__ import annotations

import math
import os
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

import torch
from isaacsim.simulation_app import SimulationApp

from srb.integrations.eval_quota import (
    EpisodeQuota,
    eval_episode_budget,
    eval_max_steps,
    finite_mean,
    write_eval_result,
)
from srb.integrations.rsl_rl.wrapper import RslRlVecEnvWrapper
from srb.utils import logging
from srb.utils.checkpoint import find_rsl_rl_checkpoint
from srb.utils.path import new_eval_logdir

if TYPE_CHECKING:
    from rsl_rl.runners import OnPolicyRunner

    from srb._typing import AnyEnv, AnyEnvCfg


def run(
    workflow: Literal["train", "eval"],
    env: "AnyEnv",
    sim_app: SimulationApp,
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: dict,
    logdir: Path,
    model: Path,
    continue_training: bool | None = None,
    **kwargs,
) -> None:
    evaluation_logdir = kwargs.pop("evaluation_logdir", None)
    kwargs.pop("run_metadata", None)

    from rsl_rl.runners import OnPolicyRunner

    training_logdir = logdir

    # Determine checkpoint path
    from_checkpoint = find_rsl_rl_checkpoint(
        training_logdir, model, workflow, continue_training
    )
    if from_checkpoint:
        logging.info(f"Loading model from {from_checkpoint}")
    elif continue_training:
        logging.info(
            "continue_training requested but no checkpoint found — starting fresh"
        )

    # Special handling for eval workflow
    if workflow == "eval":
        logdir = (
            evaluation_logdir
            if evaluation_logdir is not None
            else new_eval_logdir(training_logdir)
        )

    # Determine device — agent_cfg override (from hyperparam yaml) wins over
    # env_cfg.sim.device; fall back to CUDA if nothing is available.
    device = agent_cfg.pop(
        "device",
        env_cfg.sim.device if env_cfg and hasattr(env_cfg, "sim") else "cuda",
    )

    # Wrap the environment
    clip_actions = agent_cfg.pop("clip_actions", None)
    # Eval loops poll the app themselves and report an interrupted run.
    env = RslRlVecEnvWrapper(  # type: ignore[misc]
        env,
        clip_actions=clip_actions,
        is_running=sim_app.is_running if workflow == "train" else None,
    )

    # Extract max_iterations before passing config to runner
    max_iterations = agent_cfg.pop("max_iterations", 10000)

    # Create the runner
    runner = OnPolicyRunner(
        env,  # type: ignore[misc]
        train_cfg=agent_cfg,
        log_dir=str(logdir),
        device=device,
    )

    # Load checkpoint if needed
    if from_checkpoint:
        runner.load(from_checkpoint)
        logging.info(f"Loaded checkpoint from {from_checkpoint}")

    if workflow == "train":
        try:
            runner.learn(num_learning_iterations=max_iterations)
        except KeyboardInterrupt:
            # The runner saves the final model only after its loop completes.
            if runner.logger.writer is not None:
                it = runner.current_learning_iteration
                runner.save(os.path.join(runner.logger.log_dir, f"model_{it}.pt"))
                runner.logger.stop_logging_writer()
                logging.info("Saved RSL-RL checkpoint after interruption")
            raise
    elif workflow == "eval":
        _eval(runner, env, sim_app, evaluation_logdir=Path(logdir))


def _eval(
    runner: "OnPolicyRunner",
    env: RslRlVecEnvWrapper,
    sim_app: SimulationApp,
    *,
    evaluation_logdir: Path | None = None,
) -> dict[str, Any] | None:
    """Run evaluation loop with a trained policy.

    By default, visualize until the simulator stops. ``SRB_EVAL_EPISODES``
    enables bounded evaluation with a fixed per-environment quota, avoiding
    over-representation of short episodes. Success requires a valid
    per-environment ``success`` signal. Keep ``env.truncate_episodes=true``
    (the default) to ensure every environment reaches a verdict before the
    step budget expires.

    Reset recurrent policy state for each environment when its episode ends.
    """
    n_target = eval_episode_budget()
    policy = runner.get_inference_policy(device=env.device)
    obs = env.get_observations()

    if n_target is None:
        while sim_app.is_running():
            with torch.inference_mode():
                actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            _reset_finished_rows(policy, dones)
        return None

    num_envs = int(getattr(env, "num_envs", 0) or 0)
    if num_envs <= 0:
        raise ValueError(
            f"RSL-RL evaluation requires a positive num_envs, got {num_envs}"
        )
    max_steps = eval_max_steps(env, n_target)
    # The wrapper reset every row on construction, so no episode was already
    # in progress when evaluation began.
    quota = EpisodeQuota(n_target, num_envs)
    episode_returns = [0.0] * num_envs
    completed_returns: list[float] = []
    success_values: list[bool | None] = []
    metric_names = {
        "formation_distance_error": "mean_distance_error",
        "formation_orientation_error": "mean_yaw_error",
        "success_hold_steps": "mean_hold_steps_at_done",
    }
    metric_sums = dict.fromkeys(metric_names, 0.0)
    metric_counts = dict.fromkeys(metric_names, 0)
    step_count = 0
    while sim_app.is_running() and not quota.complete and step_count < max_steps:
        with torch.inference_mode():
            actions = policy(obs)
        obs, rewards, dones, extras = env.step(actions)
        step_count += 1
        reward_values = _per_env_tensor(rewards, num_envs)
        if reward_values is None:
            raise ValueError(
                "RSL-RL evaluation expected one reward per environment, "
                f"got an invalid value for {num_envs} environments"
            )
        reward_values = reward_values.to(dtype=torch.float64, device="cpu")
        if not bool(torch.isfinite(reward_values).all().item()):
            raise ValueError("RSL-RL evaluation received a non-finite reward")
        for env_id, reward in enumerate(reward_values.tolist()):
            episode_returns[env_id] += float(reward)
            if not math.isfinite(episode_returns[env_id]):
                raise ValueError(
                    "RSL-RL evaluation produced a non-finite episode return"
                )

        done_values = _per_env_tensor(dones, num_envs)
        if done_values is None:
            raise ValueError(
                "RSL-RL evaluation expected one done flag per environment, "
                f"got an invalid value for {num_envs} environments"
            )
        done_values = done_values.to(dtype=torch.bool)
        _reset_finished_rows(policy, done_values)
        done_ids = torch.nonzero(done_values.cpu(), as_tuple=False).flatten().tolist()
        if not done_ids:
            continue

        success_by_env = _per_env_success(extras, num_envs)
        metric_by_name = _per_env_metrics(extras, num_envs)
        for env_id in quota.accept(done_ids):
            completed_returns.append(episode_returns[env_id])
            success_values.append(success_by_env[env_id])
            for metric_name, values in metric_by_name.items():
                if values is None or values[env_id] is None:
                    continue
                metric_sums[metric_name] += values[env_id]
                if not math.isfinite(metric_sums[metric_name]):
                    raise ValueError(
                        "RSL-RL evaluation produced a non-finite episode metric"
                    )
                metric_counts[metric_name] += 1
        for env_id in done_ids:
            # The vector environment has already reset this row; start the
            # next episode's return accumulator at zero, whether or not the
            # finished episode counted toward the row's quota.
            episode_returns[env_id] = 0.0

    done_count = quota.completed
    if quota.complete:
        stop_reason = None
    elif step_count >= max_steps:
        stop_reason = "step_budget"
    elif not sim_app.is_running():
        stop_reason = "sim_stopped"
    else:  # pragma: no cover - defensive guard for a future loop change
        stop_reason = "incomplete"

    complete = quota.complete
    if not complete:
        logging.warning(
            f"RSL-RL evaluation incomplete: {done_count}/{n_target} episodes "
            f"after {step_count}/{max_steps} steps ({stop_reason})"
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
            finite_mean(completed_returns, "RSL-RL") if completed_returns else None
        ),
        "n_episodes": done_count,
        "episodes_requested": n_target,
        "episodes_completed": done_count,
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
            for metric_name, output_name in metric_names.items()
        }
    )
    write_eval_result(result, evaluation_logdir=evaluation_logdir)
    return result


def _reset_finished_rows(policy: Any, dones: Any) -> None:
    """Reset recurrent policy state for completed rows, if supported."""
    reset = getattr(policy, "reset", None)
    if not callable(reset):
        return
    mask = torch.as_tensor(dones).reshape(-1).to(dtype=torch.bool)
    if not bool(mask.any().item()):
        return
    with torch.inference_mode():
        reset(dones=mask)


def _per_env_tensor(value: Any, num_envs: int) -> torch.Tensor | None:
    """Convert a vector value without broadcasting a scalar across envs."""
    if isinstance(value, Mapping) or isinstance(value, (str, bytes)):
        return None
    try:
        tensor = torch.as_tensor(value)
    except (TypeError, ValueError, RuntimeError):
        return None
    if tensor.ndim == 0 or tensor.numel() != num_envs:
        return None
    return tensor.detach().reshape(-1)


def _scalar_success(value: Any) -> bool | None:
    """Accept only a finite boolean or binary numeric success value."""
    if isinstance(value, torch.Tensor):
        if value.numel() != 1:
            return None
        value = value.detach().cpu().item()
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value) if value in (0, 1) else None
    if isinstance(value, float):
        return bool(value) if math.isfinite(value) and value in (0.0, 1.0) else None
    return None


def _per_env_success(extras: Any, num_envs: int) -> list[bool | None]:
    """Return task success values, or null values for absent/malformed data."""
    if not isinstance(extras, Mapping) or "success" not in extras:
        return [None] * num_envs
    values = _per_env_tensor(extras["success"], num_envs)
    if values is None:
        return [None] * num_envs
    return [_scalar_success(value) for value in values]


def _per_env_metrics(
    extras: Any, num_envs: int
) -> dict[str, list[float | None] | None]:
    """Extract finite per-environment formation metrics without NaN leakage."""
    metrics = extras.get("episode_metrics") if isinstance(extras, Mapping) else None
    if not isinstance(metrics, Mapping):
        return {
            "formation_distance_error": None,
            "formation_orientation_error": None,
            "success_hold_steps": None,
        }
    values_by_name: dict[str, list[float | None] | None] = {}
    for name in (
        "formation_distance_error",
        "formation_orientation_error",
        "success_hold_steps",
    ):
        values = _per_env_tensor(metrics.get(name), num_envs)
        if values is None:
            values_by_name[name] = None
            continue
        numeric_values: list[float | None] = []
        for value in values.detach().cpu().tolist():
            numeric = float(value)
            numeric_values.append(numeric if math.isfinite(numeric) else None)
        values_by_name[name] = numeric_values
    return values_by_name
