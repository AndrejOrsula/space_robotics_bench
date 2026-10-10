from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Mapping

import gymnasium
from skrl.utils import set_seed

from srb.integrations.algorithms import validate_demo_training_inputs
from srb.integrations.skrl.off_policy import SrbDDPG, SrbTD3
from srb.integrations.skrl.runner import (
    TRAINING_CURSOR_MODULE,
    restored_training_cursor,
    runner_for_config,
)
from srb.integrations.skrl.sac import SrbSAC
from srb.integrations.skrl.wrapper import SkrlEnvWrapper
from srb.utils import logging
from srb.utils.checkpoint import find_skrl_checkpoint

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv, AnyEnvCfg

_NATIVE_AGENT_CLASSES = {
    "sac": SrbSAC,
    "srbsac": SrbSAC,
    "ddpg": SrbDDPG,
    "srbddpg": SrbDDPG,
    "td3": SrbTD3,
    "srbtd3": SrbTD3,
}
_AGENT_CLASS_ALGORITHMS = {
    agent_class: f"skrl_{algorithm}"
    for agent_class, algorithm in (
        ("sac", "sac"),
        ("srbsac", "sac"),
        ("ddpg", "ddpg"),
        ("srbddpg", "ddpg"),
        ("td3", "td3"),
        ("srbtd3", "td3"),
        ("ppo", "ppo"),
        ("ppo_rnn", "ppo_rnn"),
        ("srbrppo", "ppo_rnn"),
        ("srbrecurrentppo", "ppo_rnn"),
        ("a2c", "a2c"),
        ("rpo", "rpo"),
        ("trpo", "trpo"),
    )
}


def run(
    workflow: Literal["train", "eval"],
    env: "AnyEnv | gymnasium.Env",
    sim_app: SimulationApp,
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: dict,
    logdir: Path,
    model: Path,
    continue_training: bool | None = None,
    algo: str | None = None,
    evaluation_logdir: Path | None = None,
    **kwargs,
):
    if not isinstance(agent_cfg, dict):
        raise TypeError("SKRL agent_cfg must be a mapping")
    agent_section = agent_cfg.get("agent")
    if not isinstance(agent_section, dict):
        raise ValueError("SKRL agent_cfg is missing the 'agent' mapping")
    resolved_algo = _resolve_skrl_algorithm(algo, agent_cfg)
    resume_override = _pop_resume_override(agent_cfg, workflow=workflow)
    run_metadata = kwargs.pop("run_metadata", None)
    demo_dataset = kwargs.pop("demo_dataset", None)
    demo_max_transitions = kwargs.pop("demo_max_transitions", None)
    has_demo_request = demo_dataset is not None or demo_max_transitions is not None
    if has_demo_request:
        validate_demo_training_inputs(
            algo=resolved_algo or "",
            demo_dataset=demo_dataset,
            demo_max_transitions=demo_max_transitions,
        )
        configured_algo = _configured_skrl_algorithm(agent_cfg)
        if configured_algo != resolved_algo:
            raise ValueError(
                "demonstration replay prefill algorithm mismatch: "
                f"configured {configured_algo!r}, requested {resolved_algo!r}"
            )
        if workflow != "train":
            raise ValueError(
                "demonstration replay prefill is only supported for training"
            )

    # Discover checkpoints from the training run, independent of eval output.
    training_logdir = Path(logdir)
    from_checkpoint = find_skrl_checkpoint(
        training_logdir, model, workflow, continue_training
    )
    if from_checkpoint:
        logging.info(f"Loading model from {from_checkpoint}")
    elif continue_training:
        logging.info(
            "continue_training requested but no checkpoint found — starting fresh"
        )
    if resume_override is not None and not from_checkpoint:
        # The persisted Hydra key may outlive its checkpoint; fresh runs start at 0.
        logging.warning(
            f"Ignoring SKRL resume_timestep={resume_override}: no training "
            "checkpoint is being resumed"
        )
        resume_override = None

    # Prepare evaluation output and deterministic agent settings.
    if workflow == "eval":
        if evaluation_logdir is None:
            from srb.utils.path import new_eval_logdir

            evaluation_logdir = new_eval_logdir(training_logdir)
        logdir = Path(evaluation_logdir)
        _cap_eval_timesteps(agent_cfg, env)
        _prepare_eval_config(agent_cfg)
    else:
        logdir = training_logdir

    # Apply environment and experiment settings.
    agent_cfg["seed"] = env_cfg.seed if env_cfg else 0
    set_seed(agent_cfg["seed"])
    experiment = agent_section.setdefault("experiment", {})
    if not isinstance(experiment, dict):
        raise ValueError("SKRL agent.experiment must be a mapping")
    # SKRL 2.1 joins ExperimentCfg paths with ``os.path``; normalize here.
    experiment["directory"] = str(logdir.parent)
    experiment["experiment_name"] = str(logdir)

    # Wrap the environment for SKRL.
    env = SkrlEnvWrapper(env, is_running=sim_app.is_running)  # type: ignore[misc]
    if has_demo_request:
        from srb.integrations.skrl.replay import validate_demo_replay_configuration

        validate_demo_replay_configuration(agent_cfg, env)
    # Build the runner.
    runner = runner_for_config(
        env,  # type: ignore[misc]
        agent_cfg,
    )

    demo_payload = None
    if has_demo_request:
        if not _is_native_demo_agent(runner.agent, agent_cfg, algorithm=resolved_algo):
            raise ValueError(
                "demonstration replay prefill requires the registered native "
                f"SKRL {resolved_algo or 'off-policy'} agent"
            )
        from srb.integrations.skrl.replay import build_demo_replay_payload

        demo_payload = build_demo_replay_payload(
            demo_dataset,
            env,
            demo_max_transitions,
        )
        runner.agent.enable_demo_replay_checkpoint(demo_payload.binding)
        # Demo checkpoints persist the cursor in training state; preserve modules.
        modules = getattr(runner.agent, "checkpoint_modules", None)
        if isinstance(modules, dict):
            modules.pop(TRAINING_CURSOR_MODULE, None)

    # Restore checkpoint state when available.
    if from_checkpoint:
        if _is_native_demo_agent(runner.agent, agent_cfg):
            runner.agent.load(
                from_checkpoint,
                allow_replay=workflow == "train",
            )
        else:
            runner.agent.load(from_checkpoint)  # type: ignore[misc]

    if demo_payload is not None and not bool(
        getattr(runner.agent, "demo_replay_restored", False)
    ):
        runner.agent.prefill_demo_replay(demo_payload)

    if workflow == "train" and from_checkpoint:
        _resume_trainer_cursor(
            runner,
            agent_cfg,
            checkpoint=Path(from_checkpoint),
            override=resume_override,
            explicit_model=bool(model),
        )

    if demo_payload is not None:
        _persist_demo_replay_metadata(
            run_metadata,
            logdir=logdir,
            payload=demo_payload,
            max_transitions=demo_max_transitions,
        )

    # Start training or evaluation.
    try:
        runner.run(mode=workflow)
    except KeyboardInterrupt:
        agent = runner.agent
        interval = getattr(agent, "checkpoint_interval", 0)
        if workflow == "train" and isinstance(interval, int) and interval > 0:
            # Keep the progress made since the last periodic checkpoint.
            timestep = getattr(agent, "_srb_training_cursor", None)
            agent.write_checkpoint(timestep=timestep, timesteps=timestep)
            logging.info("Saved SKRL checkpoint after interruption")
        raise


#: Config key for the absolute timestep of a legacy SKRL checkpoint.
RESUME_TIMESTEP_KEY = "resume_timestep"


def _pop_resume_override(
    agent_cfg: dict[str, Any], *, workflow: Literal["train", "eval"]
) -> int | None:
    """Consume the legacy-checkpoint timestep override; discard it during eval.

    Hydra persists this key in ``.hydra`` when evaluation reuses a training run.
    """

    if RESUME_TIMESTEP_KEY not in agent_cfg:
        return None
    value = agent_cfg.pop(RESUME_TIMESTEP_KEY)
    if value is None or workflow != "train":
        return None
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("SKRL resume_timestep must be a non-negative integer")
    return value


def _resume_trainer_cursor(
    runner: Any,
    agent_cfg: Mapping[str, Any],
    *,
    checkpoint: Path,
    override: int | None,
    explicit_model: bool,
) -> None:
    """Resume from the absolute cursor; require an override for legacy checkpoints.

    Restarting at zero can overwrite checkpoints, repeat warm-up, and add a
    full budget. A persisted cursor overrides stale Hydra ``resume_timestep``.
    """

    cursor = restored_training_cursor(runner.agent)
    if override is not None:
        if cursor is None:
            cursor = override
        elif cursor != override:
            logging.warning(
                f"Ignoring SKRL resume_timestep={override}: {checkpoint} "
                f"persists training cursor {cursor}, which takes precedence"
            )
    if cursor is None:
        raise ValueError(
            f"SKRL checkpoint {checkpoint} has no persisted training cursor (it "
            "predates cursor checkpoints), so --continue_training cannot "
            "resume it at the right timestep. Pass "
            f"+agent.{RESUME_TIMESTEP_KEY}=<N> with the trainer timestep the "
            "checkpoint was written at (N in agent_<N>.pt), or start a new run."
        )
    trainer_cfg = agent_cfg.get("trainer")
    total = trainer_cfg.get("timesteps") if isinstance(trainer_cfg, Mapping) else None
    if isinstance(total, (int, float)) and not isinstance(total, bool):
        if cursor >= int(total):
            if explicit_model:
                # An explicit model with no budget left cannot be fine-tuned.
                raise ValueError(
                    f"SKRL checkpoint {checkpoint} is at timestep {cursor}, "
                    f"which already reaches trainer.timesteps={int(total)}; "
                    "there is nothing to fine-tune. Set trainer.timesteps to "
                    f"the total timestep to reach (above {cursor}), e.g. "
                    "agent.trainer.timesteps=<total>."
                )
            logging.warning(
                f"SKRL checkpoint {checkpoint} is at timestep {cursor}, which "
                f"already reaches trainer.timesteps={int(total)}; nothing is "
                "trained. Raise trainer.timesteps to extend the run."
            )
    if cursor:
        setter = getattr(runner, "set_resume_cursor", None)
        if not callable(setter):
            raise RuntimeError(
                "SKRL resumed training requires a native trainer resume cursor"
            )
        setter(cursor)
        _rearm_off_policy_warmup(runner.agent, cursor=cursor, checkpoint=checkpoint)
    logging.info(f"Resuming SKRL training at timestep {cursor} from {checkpoint}")


def _rearm_off_policy_warmup(agent: Any, *, cursor: int, checkpoint: Path) -> None:
    """Re-arm ``learning_starts`` when resumed off-policy replay is empty.

    Checkpoints omit replay; shift the threshold past the cursor to restore
    warm-up. Rebuilt config avoids compounding on later resumes. Leave random
    action warm-up and populated or demonstration replay unchanged.
    """

    from skrl.agents.torch.ddpg import DDPG, DDPG_RNN
    from skrl.agents.torch.sac import SAC, SAC_RNN
    from skrl.agents.torch.td3 import TD3, TD3_RNN

    if not isinstance(agent, (SAC, SAC_RNN, TD3, TD3_RNN, DDPG, DDPG_RNN)):
        return
    memory = getattr(agent, "memory", None)
    if memory is None or len(memory) > 0:
        return
    warmup = int(agent.cfg.learning_starts)
    agent.cfg.learning_starts = cursor + warmup
    logging.warning(
        f"SKRL checkpoint {checkpoint} carries no replay memory; resuming with "
        f"an empty buffer and re-arming learning_starts until timestep "
        f"{agent.cfg.learning_starts}"
    )


def _configured_agent_class(agent_cfg: Mapping[str, Any]) -> str:
    configured = agent_cfg.get("agent")
    return (
        str(configured.get("class", "")).lower()
        if isinstance(configured, Mapping)
        else ""
    )


def _prepare_eval_config(agent_cfg: dict[str, Any]) -> None:
    """Configure deterministic native-agent evaluation without checkpoints.

    SKRL 2.1 DDPG/TD3 add noise in ``act``; disable it and random warm-up.
    """

    agent = agent_cfg.get("agent")
    if not isinstance(agent, dict):
        raise ValueError("SKRL agent_cfg is missing the 'agent' mapping")
    trainer = agent_cfg.setdefault("trainer", {})
    if not isinstance(trainer, dict):
        raise ValueError("SKRL agent_cfg is missing the 'trainer' mapping")
    if trainer.get("stochastic_evaluation") is True:
        raise ValueError("SKRL evaluation requires trainer.stochastic_evaluation=false")
    trainer["stochastic_evaluation"] = False
    agent["random_timesteps"] = 0
    experiment = agent.setdefault("experiment", {})
    if isinstance(experiment, dict):
        # SKRL's eval loop still writes periodic checkpoints; eval must not.
        experiment["checkpoint_interval"] = 0
    agent_class = str(agent.get("class", "")).lower()
    if agent_class in {"ddpg", "srbddpg", "td3", "srbtd3"}:
        agent["exploration_noise"] = None
        agent["exploration_scheduler"] = None


def _configured_skrl_algorithm(agent_cfg: Mapping[str, Any]) -> str | None:
    """Infer a public SKRL label from the configured agent class."""
    return _AGENT_CLASS_ALGORITHMS.get(_configured_agent_class(agent_cfg))


def _resolve_skrl_algorithm(
    algo: str | None, agent_cfg: Mapping[str, Any]
) -> str | None:
    """Normalize adapter-local or public algorithm names for SKRL."""
    if algo is None:
        return _configured_skrl_algorithm(agent_cfg)
    candidate = str(algo).strip().lower()
    if not candidate.startswith("skrl_"):
        candidate = f"skrl_{candidate}"
    return candidate


def _is_native_demo_agent(
    agent: Any,
    agent_cfg: Mapping[str, Any],
    *,
    algorithm: str | None = None,
) -> bool:
    """Require the configured registered class for demo/checkpoint seams."""
    configured_class = _configured_agent_class(agent_cfg)
    configured_algorithm = _configured_skrl_algorithm(agent_cfg)
    if algorithm is not None and configured_algorithm != algorithm:
        return False
    expected_class = _NATIVE_AGENT_CLASSES.get(configured_class)
    if expected_class is None:
        return False
    if isinstance(agent, expected_class):
        return True
    # Retain the SAC fake-runner seam; require SRB classes for DDPG/TD3.
    return expected_class is SrbSAC and all(
        callable(getattr(agent, name, None))
        for name in (
            "enable_demo_replay_checkpoint",
            "prefill_demo_replay",
        )
    )


def _persist_demo_replay_metadata(
    run_metadata: Mapping[str, Any] | None,
    *,
    logdir: Path,
    payload: Any,
    max_transitions: int,
) -> None:
    """Compatibility delegate for the framework-neutral metadata publisher."""

    from srb.utils.metadata import persist_demo_replay_metadata

    persist_demo_replay_metadata(
        run_metadata,
        logdir=logdir,
        payload=payload,
        max_transitions=max_transitions,
        framework="SKRL",
    )


def _cap_eval_timesteps(agent_cfg: dict, env: "AnyEnv | gymnasium.Env") -> None:
    """Cap SKRL's bounded eval loop for ``SRB_EVAL_EPISODES``.

    Use the per-episode limit and target count with headroom; preserve unset or
    shorter configured budgets.
    """
    import os

    n_target_env = os.environ.get("SRB_EVAL_EPISODES")
    if not n_target_env:
        return
    try:
        n_target = int(n_target_env)
    except (TypeError, ValueError):
        logging.warning(
            f"Ignoring non-integer SRB_EVAL_EPISODES={n_target_env!r}; "
            "leaving the eval step budget unchanged."
        )
        return
    if n_target <= 0:
        return

    trainer_cfg = agent_cfg.get("trainer")
    if not isinstance(trainer_cfg, dict):
        # Custom configs without a trainer section have no budget to cap.
        return

    # Use the unwrapped per-episode limit; 2x headroom matches the rsl_rl guard.
    unwrapped = getattr(env, "unwrapped", env)
    max_episode_length = int(getattr(unwrapped, "max_episode_length", 0) or 0)
    budget = (max_episode_length or 1000) * n_target * 2

    current = trainer_cfg.get("timesteps")
    # Accept int/float budgets, but exclude bool (an int subclass), to shrink only.
    current_is_numeric = isinstance(current, (int, float)) and not isinstance(
        current, bool
    )
    if not current_is_numeric or budget < current:
        trainer_cfg["timesteps"] = budget
        logging.info(
            f"Capping skrl eval to {budget} timesteps for "
            f"SRB_EVAL_EPISODES={n_target} (was {current}). If episodes never "
            "terminate, set env.truncate_episodes=true."
        )
