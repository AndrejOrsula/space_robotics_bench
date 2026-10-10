"""Policy loading helpers for RSL-RL checkpoints."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import yaml

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
    """Load an RSL-RL checkpoint as a collection/evaluation policy."""
    import torch
    from rsl_rl.runners import OnPolicyRunner

    from srb.integrations.rsl_rl.wrapper import RslRlVecEnvWrapper

    train_cfg = dict(agent_cfg or _load_saved_agent_cfg(model_path) or {})
    device = train_cfg.pop("device", _resolve_device(env=env, env_cfg=env_cfg))
    clip_actions = train_cfg.pop("clip_actions", None)
    train_cfg.pop("max_iterations", None)

    wrapped_env = RslRlVecEnvWrapper(
        env, clip_actions=clip_actions, reset_on_init=False
    )
    runner = OnPolicyRunner(
        wrapped_env,
        train_cfg=train_cfg,
        log_dir=str(logdir or Path(model_path).parent),
        device=device,
    )
    runner.load(str(model_path), map_location=device)
    policy = runner.get_inference_policy(device=wrapped_env.device)

    def policy_fn(obs: dict[str, Any]) -> np.ndarray:
        with torch.inference_mode():
            # Evaluation mode does not select the distribution mean.
            action = policy(
                wrapped_env.observations_from_value(obs),
                stochastic_output=False,
            )
        action_array = policy_action_to_numpy(action)
        if clip_actions is not None:
            action_array = np.clip(action_array, -clip_actions, clip_actions)
        return action_array

    def start_episode_fn(done_mask: Any | None = None) -> None:
        reset = getattr(policy, "reset", None)
        if not callable(reset):
            # Feed-forward actors need no reset; preserve the shared contract.
            return
        if done_mask is None:
            with torch.inference_mode():
                reset()
            return
        mask = torch.as_tensor(
            done_mask,
            dtype=torch.bool,
            device=wrapped_env.device,
        ).reshape(-1)
        if mask.numel() != wrapped_env.num_envs:
            raise ValueError(
                "RSL-RL done_mask must contain one value per environment, "
                f"got shape {tuple(mask.shape)} for {wrapped_env.num_envs} environments"
            )
        # Reset only completed rows; unfinished hidden states remain intact.
        with torch.inference_mode():
            reset(dones=mask)

    return policy_fn, start_episode_fn, True


def _resolve_device(*, env: "AnyEnv", env_cfg: Any) -> Any:
    sim_cfg = getattr(env_cfg, "sim", None)
    if sim_cfg is not None and hasattr(sim_cfg, "device"):
        return sim_cfg.device
    return getattr(getattr(env, "unwrapped", env), "device", "cuda")


def _load_saved_agent_cfg(model_path: Path) -> dict[str, Any] | None:
    config_path = find_saved_policy_config(model_path)
    if config_path is None:
        return None
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"RSL-RL config must be a mapping: {config_path}")

    run_cfg = data.get("run")
    if isinstance(run_cfg, dict) and isinstance(run_cfg.get("agent_cfg"), dict):
        return dict(run_cfg["agent_cfg"])
    for key in ("agent_cfg", "agent"):
        value = data.get(key)
        if isinstance(value, dict):
            return dict(value)
    return dict(data)
