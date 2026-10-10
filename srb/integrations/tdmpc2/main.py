from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import gymnasium
from omegaconf import OmegaConf

try:
    from termcolor import colored
except ModuleNotFoundError:  # pragma: no cover - optional dependency

    def colored(text, *_, **__):  # type: ignore[override]
        return str(text)


from srb.integrations.tdmpc2.config import build_tdmpc2_config
from srb.integrations.tdmpc2.preflight import assert_task_supported, normalize_task_id
from srb.integrations.tdmpc2.upstream import assert_upstream_compatible
from srb.integrations.tdmpc2.wrapper import SrbTdmpc2Wrapper
from srb.utils import logging
from srb.utils.checkpoint import find_tdmpc2_checkpoint
from srb.utils.path import new_eval_logdir

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv, AnyEnvCfg

_BOUNDED_MPC_MESSAGE = (
    "bounded TD-MPC2 evaluation rejects stochastic MPC; set agent.mpc=false "
    "or unset SRB_EVAL_EPISODES for diagnostic evaluation"
)

_CONTINUE_UNSUPPORTED_MESSAGE = (
    "TD-MPC2 `--continue` (continue_training) is not supported: upstream "
    "TD-MPC2 checkpoints store model weights only ({'model': state_dict}). "
    "Optimizer state, replay buffer, trainer step, RNG state, and the "
    "pretraining flag are not serialized, so reloading one restarts training "
    "with an empty replay and reset schedules -- that is a weights import, not "
    "a training-state resume. Pass `--model <checkpoint>` to import weights "
    "into a fresh run (schedules and replay restart from step 0), or implement "
    "and test a versioned full trainer-state checkpoint first."
)


def _bounded_eval_requested() -> bool:
    """Validate the optional budget before config publication or model load."""
    raw = os.environ.get("SRB_EVAL_EPISODES")
    if raw is None:
        return False
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"SRB_EVAL_EPISODES must be a positive integer, got {raw!r}"
        ) from exc
    if value <= 0:
        raise ValueError(f"SRB_EVAL_EPISODES must be a positive integer, got {raw!r}")
    return True


def _config_mpc_enabled(*configs: object) -> bool:
    for config in configs:
        if config is None:
            continue
        try:
            value = getattr(config, "mpc")
        except AttributeError:
            value = config.get("mpc", False) if isinstance(config, Mapping) else False
        if bool(value):
            return True
    return False


def run(
    workflow: Literal["train", "eval"],
    env: "AnyEnv | gymnasium.Env",
    sim_app: "SimulationApp",
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: dict,
    logdir: Path,
    model: "Path | None",
    continue_training: bool | None = None,
    evaluation_logdir: "Path | None" = None,
    **kwargs,
):
    kwargs.pop("run_metadata", None)

    from srb.integrations.tdmpc2 import apply_tdmpc2_runtime_policy

    apply_tdmpc2_runtime_policy()

    # Preflight runs before config, checkpoint, replay, or agent allocation.
    assert_task_supported(env=env, env_id=env_id)
    assert_upstream_compatible()
    if continue_training:
        raise NotImplementedError(_CONTINUE_UNSUPPORTED_MESSAGE)
    bounded_eval = workflow == "eval" and _bounded_eval_requested()

    # Discover checkpoints in the caller's training root, not the eval output.
    training_logdir = Path(logdir)
    checkpoint_path = find_tdmpc2_checkpoint(training_logdir, model, workflow, False)
    if checkpoint_path is not None and not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    if workflow == "eval":
        logdir = (
            Path(evaluation_logdir)
            if evaluation_logdir is not None
            else new_eval_logdir(training_logdir)
        )
    else:
        logdir = training_logdir
    logdir.mkdir(parents=True, exist_ok=True)
    print(colored(f"Agent logdir: {logdir}", "yellow", attrs=["bold"]))

    # Wrap the environment and build config from one observation transform.
    env = SrbTdmpc2Wrapper(env)
    # Ending-mode records use the canonical task id.
    task_name = normalize_task_id(env_id) or env_id.replace("/", "_")

    cfg, merged_cfg = build_tdmpc2_config(
        env=env,
        task_name=task_name,
        work_dir=logdir,
        agent_cfg=agent_cfg,
        seed=getattr(env_cfg, "seed", None) if env_cfg is not None else None,
    )

    if bounded_eval and _config_mpc_enabled(cfg, merged_cfg):
        raise ValueError(_BOUNDED_MPC_MESSAGE)

    # Save config after preflight and before allocation.
    OmegaConf.save(merged_cfg, logdir / "config.yaml")

    # Create agent.
    from srb.integrations.tdmpc2.batched import BatchedTDMPC2

    agent = BatchedTDMPC2(cfg)

    # Checkpoints contain weights only; replay, optimizers, step, RNG, and
    # pretraining state restart. Log the training step as 0.
    start_step = 0
    if checkpoint_path:
        checkpoint_path = Path(checkpoint_path)
        agent.load(str(checkpoint_path))
        if workflow == "eval":
            logging.info(f"Evaluation load (weights only): {checkpoint_path}")
        else:
            logging.info(
                f"Weights import (weights only): {checkpoint_path}. "
                "Training starts at step 0 with a fresh replay buffer, fresh "
                "optimizers, and reset schedules; this is not a training-state "
                "resume."
            )

    # ------------------------------------------------------------------
    # Dispatch workflow
    # ------------------------------------------------------------------
    if workflow == "train":
        from srb.integrations.tdmpc2.train import train

        train(
            cfg=cfg,
            env=env,
            agent=agent,
            sim_app=sim_app,
            start_step=start_step,
        )
    elif workflow == "eval":
        from srb.integrations.tdmpc2.eval import eval_only

        eval_only(
            cfg=cfg,
            env=env,
            agent=agent,
            sim_app=sim_app,
            evaluation_logdir=logdir,
        )
    else:
        raise ValueError(f"Unknown workflow: {workflow}")
