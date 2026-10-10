"""Load TD-MPC2 checkpoints for policy inference."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np
import torch

from srb.integrations.tdmpc2.config import build_tdmpc2_config, saved_observation_record
from srb.integrations.tdmpc2.preflight import assert_task_supported, normalize_task_id
from srb.integrations.tdmpc2.upstream import assert_upstream_compatible
from srb.utils import logging
from srb.utils.model_artifacts import find_saved_policy_config
from srb.utils.policy_contract import LoadedPolicy, policy_action_to_numpy

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def load_policy(
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
) -> LoadedPolicy:
    """Load checkpoint weights as a collection/evaluation policy."""
    from srb.integrations.tdmpc2.batched import BatchedTDMPC2
    from srb.integrations.tdmpc2.wrapper import SrbTdmpc2Wrapper

    # Preflight before wrapping or constructing the agent.
    assert_task_supported(env=env, env_id=_env_id_of(env))
    assert_upstream_compatible()

    wrapped_env = SrbTdmpc2Wrapper(env)
    cfg = _build_config(
        model_path=model_path,
        env=wrapped_env,
        env_cfg=env_cfg,
        agent_cfg=agent_cfg,
        logdir=logdir,
    )
    agent = BatchedTDMPC2(cfg)
    agent.load(str(model_path))
    logging.info(f"Evaluation load (weights only): {model_path}")

    num_envs = wrapped_env.num_envs
    # Reset planner history per completed row; preserve unfinished peers.
    episode_starts = torch.ones(num_envs, dtype=torch.bool, device=agent.device)

    def start_episode_fn(done_mask=None) -> None:
        """Mark selected vector rows as the next planner sequence start."""
        nonlocal episode_starts
        if done_mask is None:
            episode_starts.fill_(True)
            return
        mask = torch.as_tensor(done_mask, device=agent.device, dtype=torch.bool)
        if mask.numel() != num_envs:
            raise ValueError(
                "TD-MPC2 done_mask must contain one value per environment, "
                f"got shape {tuple(mask.shape)}"
            )
        episode_starts |= mask.reshape(num_envs)

    def policy_fn(obs: Mapping[str, Any]) -> np.ndarray:
        processed_obs = _process_policy_observation(
            wrapped_env,
            obs,
            num_envs=num_envs,
        )
        t0 = episode_starts.clone()
        with torch.no_grad():
            action = agent.act(processed_obs, t0=t0, eval_mode=True)
        episode_starts.zero_()
        return policy_action_to_numpy(action)

    start_episode_fn()
    # MPC samples trajectories and a final elite, even without exploration noise.
    if isinstance(cfg, Mapping):
        mpc = cfg.get("mpc", False)
    else:
        mpc = getattr(cfg, "mpc", False)
    return policy_fn, start_episode_fn, not bool(mpc)


def _build_config(
    *,
    model_path: Path,
    env: Any,
    env_cfg: Any,
    agent_cfg: dict[str, Any] | None,
    logdir: Path | None,
):
    saved_config_path = find_saved_policy_config(model_path)
    task_name = _resolve_task_name(env=env, saved_config=saved_config_path)
    work_dir = Path(logdir or Path(model_path).parent)

    cfg, _merged = build_tdmpc2_config(
        env=env,
        task_name=task_name,
        work_dir=work_dir,
        agent_cfg=agent_cfg,
        saved_config=saved_config_path,
        # No caller seed means keep the saved run's seed.
        seed=_requested_seed(env_cfg),
    )

    # Reject a saved layout that differs from the live state vector.
    saved_observation = saved_observation_record(saved_config_path)
    if saved_observation is not None:
        live = env.state_transform.record()
        if saved_observation.get("layout_fingerprint") != live["layout_fingerprint"]:
            raise ValueError(
                "TD-MPC2 checkpoint observation layout does not match this "
                f"environment: saved {saved_observation} vs live {live}"
            )
    return cfg


def _requested_seed(env_cfg: Any) -> int | None:
    """Seed the caller imposes on the run, or ``None`` to keep the saved one."""
    if env_cfg is None:
        return None
    seed = getattr(env_cfg, "seed", None)
    return None if seed is None else int(seed)


def _env_id_of(env: Any) -> str | None:
    spec = getattr(env, "spec", None) or getattr(
        getattr(env, "unwrapped", None), "spec", None
    )
    return getattr(spec, "id", None)


def _resolve_task_name(*, env: Any, saved_config: Path | None) -> str:
    env_id = _env_id_of(getattr(env, "env", env))
    if env_id:
        # Use the same canonical task id as training.
        return normalize_task_id(env_id) or str(env_id).replace("/", "_")
    if saved_config is not None:
        from omegaconf import OmegaConf

        cfg_task = OmegaConf.load(saved_config).get("task")
        if cfg_task is not None and str(cfg_task).strip() and str(cfg_task) != "???":
            return str(cfg_task)
    return env.unwrapped.__class__.__name__


def _process_policy_observation(
    env: Any,
    obs: Mapping[str, Any],
    *,
    num_envs: int,
) -> torch.Tensor:
    """Project an evaluation observation through the *training* transform."""
    if isinstance(obs, Mapping) and len(obs) == 1 and "obs" in obs:
        obs = obs["obs"]
    state = env.state_from_observation(obs)
    return _ensure_batched_tensor(state, env.device, num_envs)


def _ensure_batched_tensor(value: Any, device: Any, num_envs: int) -> torch.Tensor:
    if not isinstance(value, torch.Tensor):
        value = torch.as_tensor(value)
    if value.dtype == torch.float64:
        value = value.float()
    value = value.to(device)
    if value.shape[:1] == (num_envs,):
        return value
    return value.reshape((num_envs, *value.shape))
