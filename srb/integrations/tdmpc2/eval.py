from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import torch
from termcolor import colored

from srb.integrations.eval_quota import (
    EpisodeQuota,
    eval_episode_budget,
    finite_mean,
    write_eval_result,
)
from srb.integrations.tdmpc2.batched import BatchedTDMPC2
from srb.utils import logging

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb.integrations.tdmpc2.wrapper import SrbTdmpc2Wrapper


def eval_only(
    cfg,
    env: "SrbTdmpc2Wrapper",
    agent: BatchedTDMPC2,
    sim_app: "SimulationApp",
    evaluation_logdir: Path | None = None,
):
    bounded_episodes = eval_episode_budget()
    if bounded_episodes is not None:
        return _run_bounded_eval(
            agent,
            env,
            sim_app,
            cfg=cfg,
            target_episodes=bounded_episodes,
            evaluation_logdir=evaluation_logdir,
        )

    # Preserve legacy diagnostics when no bounded-eval budget is set.
    return _eval_legacy(cfg, env, agent, sim_app)


def _eval_legacy(
    cfg,
    env: "SrbTdmpc2Wrapper",
    agent: BatchedTDMPC2,
    sim_app: "SimulationApp",
):
    num_envs = env.num_envs
    if not isinstance(num_envs, int) or isinstance(num_envs, bool) or num_envs <= 0:
        raise ValueError(f"env.num_envs must be positive, got {num_envs!r}")
    ep_rewards_all: list[np.ndarray] = []
    ep_successes_all: list[list[bool | None]] = []

    # Collect the requested episodes.
    episodes_collected = 0
    target_episodes = int(getattr(cfg, "eval_episodes", 10))
    if target_episodes <= 0:
        raise ValueError(f"cfg.eval_episodes must be positive, got {target_episodes}")
    max_episode_steps = int(getattr(env, "max_episode_steps", 0) or 0)
    if max_episode_steps <= 0:
        raise ValueError(
            "TD-MPC2 evaluation requires env.max_episode_steps > 0 to bound "
            "never-terminating environments"
        )
    # Each vectorized run covers ``num_envs`` episodes.
    runs_needed = (target_episodes + num_envs - 1) // num_envs

    print(
        colored(
            f"Evaluating agent on {cfg.task_title} "
            f"({target_episodes} episodes across {num_envs} envs):",
            "yellow",
            attrs=["bold"],
        )
    )

    for run in range(runs_needed):
        if not sim_app.is_running():
            break

        ep_rewards = torch.zeros(num_envs, device=agent.device)
        ep_successes: list[bool | None] = [None] * num_envs
        finished = torch.zeros(num_envs, dtype=torch.bool, device=agent.device)
        obs = env.reset()
        info: Any = {}
        done = False

        for t in range(max_episode_steps):
            torch.compiler.cudagraph_mark_step_begin()
            t0 = torch.full(
                (num_envs,),
                t == 0,
                dtype=torch.bool,
                device=agent.device,
            )
            action = agent.act(obs, t0=t0, eval_mode=True)
            obs, reward, dones, info = env.step(action)
            dones_b = torch.as_tensor(dones).reshape(-1).to(finished.device).bool()
            active = ~finished
            # Ignore post-autoreset rewards for rows already counted.
            ep_rewards += torch.as_tensor(reward).reshape(-1).to(ep_rewards) * active
            just_finished = dones_b & active
            if just_finished.any():
                success_by_env = _per_env_success(info, num_envs)
                for env_id in torch.nonzero(just_finished, as_tuple=False).flatten():
                    ep_successes[int(env_id)] = success_by_env[int(env_id)]
            finished |= dones_b
            done = bool(finished.all().item())
            if done or not sim_app.is_running():
                break

        if not sim_app.is_running() and not done:
            break
        if not done:
            raise RuntimeError(
                "TD-MPC2 evaluation episode did not terminate within "
                f"env.max_episode_steps={max_episode_steps}"
            )

        take = min(num_envs, target_episodes - episodes_collected)
        ep_rewards_all.append(ep_rewards[:take].cpu().numpy())
        ep_successes_all.append(ep_successes[:take])
        episodes_collected += take

        run_message = (
            f"  Run {run + 1}/{runs_needed}  R: {ep_rewards.mean().item():.01f}  "
        )
        if all(value is not None for value in ep_successes):
            run_message += f"S: {sum(bool(value) for value in ep_successes) / len(ep_successes):.02f}  "
        run_message += f"L: {t + 1}"
        print(
            colored(
                run_message,
                "green",
            )
        )

    if not ep_rewards_all:
        return None
    mean_reward = np.concatenate(ep_rewards_all).mean()
    success_values = [value for row in ep_successes_all for value in row]
    mean_success = (
        float(sum(bool(value) for value in success_values) / len(success_values))
        if success_values and all(value is not None for value in success_values)
        else None
    )
    overall = f"\n  Overall  R: {mean_reward:.01f}"
    if mean_success is not None:
        overall += f"  S: {mean_success:.02f}"
    print(
        colored(
            overall,
            "green",
            attrs=["bold"],
        )
    )
    return {
        "mean_reward": float(mean_reward),
        "mean_success": mean_success,
        "episodes": episodes_collected,
    }


def _eval_max_steps(env: Any, target_episodes: int) -> int:
    """Return a finite guard for environments that never emit a done row."""
    unwrapped = getattr(env, "unwrapped", env)
    raw_length = getattr(env, "max_episode_steps", None)
    if raw_length is None:
        raw_length = getattr(unwrapped, "max_episode_length", 0)
    try:
        max_episode_length = int(raw_length or 0)
    except (OverflowError, TypeError, ValueError):
        max_episode_length = 0
    if max_episode_length <= 0:
        max_episode_length = 1000
    return max_episode_length * target_episodes * 2


def _run_bounded_eval(
    agent: BatchedTDMPC2,
    env: "SrbTdmpc2Wrapper",
    sim_app: "SimulationApp",
    *,
    target_episodes: int,
    cfg: Any = None,
    evaluation_logdir: Path | None = None,
) -> dict[str, Any]:
    """Evaluate deterministic non-MPC inference against per-env quotas.

    MPC remains stochastic in eval mode. Per-row returns and reset masks keep
    asynchronous autoresets independent.
    """
    if (
        isinstance(target_episodes, bool)
        or not isinstance(target_episodes, int)
        or target_episodes <= 0
    ):
        raise ValueError(
            f"target_episodes must be a positive integer, got {target_episodes!r}"
        )
    if _is_mpc(cfg, agent):
        raise ValueError(
            "bounded TD-MPC2 evaluation rejects stochastic MPC; set agent.mpc=false "
            "or unset SRB_EVAL_EPISODES for diagnostic evaluation"
        )

    num_envs = getattr(env, "num_envs", None)
    if isinstance(num_envs, bool) or not isinstance(num_envs, int) or num_envs <= 0:
        raise ValueError(f"env.num_envs must be positive, got {num_envs!r}")

    max_steps = _eval_max_steps(env, target_episodes)
    device = getattr(agent, "device", getattr(env, "device", "cpu"))
    episode_starts = torch.ones(num_envs, dtype=torch.bool, device=device)
    episode_returns = [0.0] * num_envs
    completed_returns: list[float] = []
    success_values: list[bool | None] = []
    obs = _reset_observation(env)
    quota = EpisodeQuota(target_episodes, num_envs)
    step_count = 0
    stop_reason: str | None = None

    while not quota.complete and step_count < max_steps:
        if not bool(sim_app.is_running()):
            stop_reason = "sim_stopped"
            break

        _mark_cudagraph_step_begin()
        action = agent.act(obs, t0=episode_starts, eval_mode=True)
        step_result = env.step(action)
        obs, reward, done, info = _normalise_step_result(step_result, num_envs)
        step_count += 1

        reward_values = _per_env_floats(reward, num_envs)
        if reward_values is None:
            raise ValueError(
                "TD-MPC2 evaluation expected one reward per environment, "
                f"got an invalid value for {num_envs} environments"
            )
        quota_counts = quota.counts
        for env_id, reward_value in enumerate(reward_values):
            if quota_counts[env_id] < quota.targets[env_id]:
                episode_returns[env_id] += reward_value
                if not math.isfinite(episode_returns[env_id]):
                    raise ValueError(
                        "TD-MPC2 evaluation episode return became non-finite"
                    )

        done_values = _per_env_bools(done, num_envs)
        if done_values is None:
            raise ValueError(
                "TD-MPC2 evaluation expected one done flag per environment, "
                f"got an invalid value for {num_envs} environments"
            )
        success_by_env = _per_env_success(info, num_envs)
        accepted_env_ids = set(
            quota.accept(
                env_id for env_id, is_done in enumerate(done_values) if is_done
            )
        )
        for env_id, is_done in enumerate(done_values):
            if not is_done:
                continue
            if env_id in accepted_env_ids:
                completed_returns.append(episode_returns[env_id])
                success_values.append(success_by_env[env_id])
            # The next action sees this row's reset observation and start mask.
            episode_returns[env_id] = 0.0
        episode_starts = torch.as_tensor(done_values, dtype=torch.bool, device=device)

    if quota.complete:
        stop_reason = None
    elif stop_reason is None:
        stop_reason = "step_budget" if step_count >= max_steps else "sim_stopped"

    complete = quota.complete
    if not complete:
        logging.warning(
            f"TD-MPC2 evaluation incomplete: {len(completed_returns)}/"
            f"{target_episodes} episodes after {step_count}/{max_steps} steps "
            f"({stop_reason})"
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
            finite_mean(completed_returns, "TD-MPC2") if completed_returns else None
        ),
        "n_episodes": len(completed_returns),
        "episodes_requested": target_episodes,
        "episodes_completed": len(completed_returns),
        "complete": complete,
        "incomplete": not complete,
        "steps": step_count,
        "max_steps": max_steps,
        "stop_reason": stop_reason,
    }
    result.update(quota.result_fields())
    write_eval_result(result, evaluation_logdir=evaluation_logdir)
    return result


def _is_mpc(cfg: Any, agent: Any) -> bool:
    for owner in (cfg, getattr(agent, "cfg", None)):
        if owner is None:
            continue
        try:
            value = getattr(owner, "mpc")
        except AttributeError:
            if isinstance(owner, Mapping):
                value = owner.get("mpc", False)
            else:
                value = False
        if bool(value):
            return True
    return False


def _reset_observation(env: Any) -> Any:
    result = env.reset()
    if isinstance(result, tuple) and len(result) == 2:
        return result[0]
    return result


def _normalise_step_result(result: Any, num_envs: int) -> tuple[Any, Any, Any, Any]:
    if not isinstance(result, tuple) or len(result) not in (4, 5):
        raise ValueError(
            "TD-MPC2 evaluation expected a four- or five-value environment step"
        )
    if len(result) == 4:
        obs, reward, done, info = result
    else:
        obs, reward, terminated, truncated, info = result
        terminated_values = _per_env_bools(terminated, num_envs)
        truncated_values = _per_env_bools(truncated, num_envs)
        if terminated_values is None or truncated_values is None:
            return obs, reward, None, info
        done = [
            terminated_value or truncated_value
            for terminated_value, truncated_value in zip(
                terminated_values, truncated_values
            )
        ]
    return obs, reward, done, info


def _as_cpu_value(value: Any) -> Any:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu()
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    return value


def _per_env_values(value: Any, num_envs: int) -> list[Any] | None:
    if isinstance(value, Mapping) or isinstance(value, (str, bytes)):
        return None
    try:
        tensor = torch.as_tensor(_as_cpu_value(value))
    except (TypeError, ValueError, RuntimeError):
        return None
    if tensor.ndim == 0:
        return [tensor.item()] if num_envs == 1 else None
    if tensor.numel() != num_envs:
        return None
    return tensor.reshape(-1).tolist()


def _per_env_floats(value: Any, num_envs: int) -> list[float] | None:
    values = _per_env_values(value, num_envs)
    if values is None:
        return None
    converted: list[float] = []
    for raw_value in values:
        try:
            converted_value = float(raw_value)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(converted_value):
            raise ValueError("TD-MPC2 evaluation received a non-finite reward")
        converted.append(converted_value)
    return converted


def _per_env_bools(value: Any, num_envs: int) -> list[bool] | None:
    values = _per_env_values(value, num_envs)
    if values is None:
        return None
    return [bool(value) for value in values]


def _scalar_success(value: Any) -> bool | None:
    value = _as_cpu_value(value)
    if isinstance(value, torch.Tensor):
        if value.numel() != 1:
            return None
        value = value.item()
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return bool(value) if value in (0, 1) else None
    if isinstance(value, (float, np.floating)):
        numeric = float(value)
        return bool(value) if math.isfinite(numeric) and value in (0.0, 1.0) else None
    return None


def _per_env_success(info: Any, num_envs: int) -> list[bool | None]:
    if isinstance(info, (list, tuple)):
        if len(info) != num_envs:
            return [None] * num_envs
        return [
            _scalar_success(item.get("success"))
            if isinstance(item, Mapping) and "success" in item
            else None
            for item in info
        ]
    if not isinstance(info, Mapping) or "success" not in info:
        return [None] * num_envs
    values = _per_env_values(info["success"], num_envs)
    if values is None:
        return [None] * num_envs
    return [_scalar_success(value) for value in values]


def _mark_cudagraph_step_begin() -> None:
    compiler = getattr(torch, "compiler", None)
    marker = getattr(compiler, "cudagraph_mark_step_begin", None)
    if marker is not None:
        marker()
