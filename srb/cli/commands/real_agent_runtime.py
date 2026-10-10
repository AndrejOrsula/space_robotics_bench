"""CLI adapter for ``srb real_agent`` runtime workflows."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Literal, Sequence

from srb.cli.commands import agent_dispatch as _dispatch_cli
from srb.cli.commands import agent_policy as _policy_cli
from srb.cli.commands import real_agent_generation as _generation_cli

_REAL_AGENT_METADATA_KWARG_KEYS = frozenset(
    (
        "model",
        "continue_training",
        "algo",
        "headless",
        "teleop_device",
        "pos_sensitivity",
        "rot_sensitivity",
        "invert_controls",
        "collector",
        "num_episodes",
        "horizon",
        "success_only",
    )
)

_REAL_ENV_NAMESPACE = "srb_real"
_REAL_ENV_TASK_START_CHARS = frozenset(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
)
_REAL_ENV_TASK_CHARS = _REAL_ENV_TASK_START_CHARS | frozenset("0123456789_-")
_REAL_ENV_ADOPTION_KWARG = "_adopt_cleanup_owner"


def _validate_missing_real_env_id(env_id: str) -> str:
    """Return the task component of an exact canonical missing RealEnv ID."""
    parts = env_id.split("/") if isinstance(env_id, str) else ()
    task = parts[1] if len(parts) == 2 else ""
    if (
        len(parts) != 2
        or parts[0] != _REAL_ENV_NAMESPACE
        or not task
        or task[0] not in _REAL_ENV_TASK_START_CHARS
        or any(char not in _REAL_ENV_TASK_CHARS for char in task)
    ):
        raise ValueError(
            f"Invalid RealEnv ID {ascii(env_id)}; expected "
            "exactly 'srb_real/<single-task-component>'."
        )
    return task


def _remove_real_env_adoption_kwarg(env: Any) -> None:
    """Remove the constructor-only callback from Gymnasium's cached specs."""
    current = env
    visited: set[int] = set()
    while current is not None and id(current) not in visited:
        visited.add(id(current))
        spec = getattr(current, "spec", None)
        spec_kwargs = getattr(spec, "kwargs", None)
        if isinstance(spec_kwargs, dict):
            spec_kwargs.pop(_REAL_ENV_ADOPTION_KWARG, None)
        current = getattr(current, "env", None)


def _make_real_agent_env(
    gymnasium: Any,
    *,
    env_id: str,
    hardware: Sequence[str],
    adopt: Any,
) -> Any:
    """Construct through Gym while keeping provisional ownership retryable."""
    provisional: list[Any] = []

    def adopt_provisional(owner: Any) -> None:
        provisional.append(owner)
        adopt(owner)

    env: Any | None = None
    try:
        env = gymnasium.make(
            id=env_id,
            hardware=hardware,
            **{_REAL_ENV_ADOPTION_KWARG: adopt_provisional},
        )
        return env
    finally:
        if env is not None:
            _remove_real_env_adoption_kwarg(env)
        for owner in provisional:
            _remove_real_env_adoption_kwarg(owner)


def execute_runtime_command(
    *,
    real_agent_subcommand: Literal[
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    ],
    **kwargs: Any,
) -> None:
    """Run a real-agent workflow after parser-level validation."""
    _run_real_agent_with_env(
        real_agent_subcommand=real_agent_subcommand,
        **kwargs,
    )


def _build_real_agent_metadata_cli_kwargs(
    *,
    config: str,
    hardware: Sequence[str],
    kwargs: dict[str, Any],
) -> dict[str, Any]:
    """Build the CLI subset persisted in metadata.json for ``srb real_agent``."""
    return {
        "agent_command": "real_agent",
        "config": config,
        "hardware": list(hardware),
        **{k: v for k, v in kwargs.items() if k in _REAL_AGENT_METADATA_KWARG_KEYS},
    }


def _run_real_agent_with_env(
    real_agent_subcommand: Literal[
        "zero",
        "rand",
        "teleop",
        "ros",
        "train",
        "eval",
        "collect",
    ],
    env_id: str,
    config: str,
    hardware: Sequence[str],
    logdir_path: str,
    forwarded_args: Sequence[str] = (),
    **kwargs: Any,
) -> None:
    import gymnasium

    from srb.cli.capabilities import require_hydra

    # ``srb.hydra.real`` imports hydra/omegaconf at module scope and both ship
    # only in the ``cli`` extra, so probe them first: a lean install must get
    # the adapter-owned remediation instead of a raw ``ModuleNotFoundError``.
    require_hydra()

    from srb.cli.commands.real_agent_session import RealAgentSession
    from srb.hydra.real import hydra_task_config
    from srb.interfaces.sim_to_real import RealEnv
    from srb.interfaces.sim_to_real import hardware as srb_real_hw  # noqa: F401
    from srb.interfaces.sim_to_real.env import register_all as _register_real_envs
    from srb.utils import logging
    from srb.utils.cfg import last_logdir, new_logdir
    from srb.utils.path import new_eval_logdir
    from srb.utils.registry import get_srb_tasks

    _register_real_envs()

    if env_id not in get_srb_tasks("srb_real").values():
        env_name = _validate_missing_real_env_id(env_id)
        logging.info(
            f"The RealEnv for {env_name} is not registered. Generating it now..."
        )
        _generation_cli.generate_missing_real_env(
            env_id=env_name,
            hardware=hardware,
            forwarded_args=forwarded_args,
        )
        logging.info(
            f"Generated the missing RealEnv for {env_name}. Please re-run your desired workflow."
        )
        exit(0)

    if real_agent_subcommand == "teleop" and (
        kwargs.get("algo") or kwargs.get("model")
    ):
        raise ValueError(
            "real-agent teleop accepts direct native input only; "
            "--algo and --model are not supported"
        )

    kwargs["algo"], kwargs["model"] = _resolve_real_agent_policy_inputs(
        real_agent_subcommand=real_agent_subcommand,
        algo=kwargs.get("algo"),
        model=kwargs.get("model"),
    )
    native_input_workflow = real_agent_subcommand == "teleop"
    if real_agent_subcommand == "collect":
        from srb.cli.commands.agent_collection import _resolve_collect_source

        native_input_workflow = (
            _resolve_collect_source(
                kwargs.get("collector"), kwargs.get("algo"), kwargs.get("model")
            )
            == "human"
        )
    if algo := kwargs.get("algo"):
        from srb.cli.preflight import require_skrl_algorithm_capability

        require_skrl_algorithm_capability(algo)
    workflow = kwargs.get("algo") or real_agent_subcommand
    logdir_root = Path(logdir_path).resolve()
    reuses_run = True

    if model := kwargs.get("model"):
        model = Path(model).resolve()
        if not (model.exists()):
            raise ValueError(f"Model path does not exist: {model}")
        logdir = model.parent
        while not (
            logdir.parent.name == workflow
            and (logdir.parent.parent.name == env_id.rsplit("/", 1)[-1])
        ):
            _new_parent = logdir.parent
            if logdir == _new_parent:
                logdir = new_logdir(
                    env_id=env_id,
                    workflow=workflow,
                    root=logdir_root,
                    namespace="srb_real",
                )
                model_symlink_path = logdir.joinpath(model.name)
                model_symlink_path.parent.mkdir(parents=True, exist_ok=True)
                if model_symlink_path.is_symlink() or model_symlink_path.exists():
                    model_symlink_path.unlink()
                os.symlink(model, model_symlink_path)
                model = model_symlink_path
                reuses_run = False
                break
            logdir = _new_parent
        kwargs["model"] = model
    elif (
        real_agent_subcommand == "train" and kwargs["continue_training"]
    ) or _policy_cli.should_use_latest_policy_logdir(
        real_agent_subcommand,
        algo=kwargs.get("algo"),
        collector=kwargs.get("collector"),
    ):
        logdir = last_logdir(
            env_id=env_id, workflow=workflow, root=logdir_root, namespace="srb_real"
        )
    else:
        logdir = new_logdir(
            env_id=env_id, workflow=workflow, root=logdir_root, namespace="srb_real"
        )
        reuses_run = False

    from srb.cli.commands.agent_runtime import _metadata_logdir
    from srb.utils.metadata import collect_run_metadata, write_run_metadata

    # Keep a reused training run intact: eval, collect and teleop write their
    # metadata, Hydra config and results to a fresh subdirectory.
    evaluation_logdir = (
        new_eval_logdir(logdir) if real_agent_subcommand == "eval" else None
    )
    run_logdir = _metadata_logdir(
        logdir,
        real_agent_subcommand,
        evaluation_logdir=evaluation_logdir,
        reuses_run=reuses_run,
    )

    metadata_common_kwargs = {
        "env_id": env_id,
        "workflow": workflow,
        "algo": kwargs.get("algo"),
        "logdir": run_logdir,
        "agent_subcommand": real_agent_subcommand,
        "cli_kwargs": _build_real_agent_metadata_cli_kwargs(
            config=config,
            hardware=hardware,
            kwargs=kwargs,
        ),
        "forwarded_args": list(forwarded_args),
    }
    write_run_metadata(collect_run_metadata(**metadata_common_kwargs), run_logdir)

    config_path = None
    if config.upper() == "DEFAULT":
        search_dir = logdir
        while search_dir != search_dir.parent:
            maybe_path = search_dir.joinpath(".hydra", "config.yaml").resolve()
            if maybe_path.exists():
                config_path = maybe_path
                logging.info(f"Using default Hydra config from: {config_path}")
                break
            search_dir = search_dir.parent
    elif config and config.upper() not in ("IGNORE", "NONE", "NULL"):
        config_path = Path(config).expanduser().resolve()
        if not config_path.exists():
            raise FileNotFoundError(f"Hydra config path does not exist: {config_path}")
        logging.info(f"Using custom Hydra config from: {config_path}")
        if config_path.suffix.lower() not in (".yaml", ".yml"):
            logging.warning(
                f"The provided Hydra config path does not appear to be a YAML file: {config_path}"
            )

    # Check Hydra leftovers and forwarded args so the appended default cannot
    # override a user-supplied run directory.
    if not any(
        arg.startswith("hydra.run.dir=") for arg in (*forwarded_args, *sys.argv[1:])
    ):
        sys.argv.extend([f"hydra.run.dir={run_logdir.as_posix()}"])

    @hydra_task_config(
        task_name=env_id,
        agent_cfg_entry_point=f"{kwargs['algo']}_cfg" if kwargs.get("algo") else None,
        config_path=config_path.as_posix() if config_path else None,
    )
    def hydra_main(agent_cfg: dict[str, Any] | None = None) -> None:
        with RealAgentSession(defer_activation=native_input_workflow) as session:
            if native_input_workflow:
                from srb.cli.commands.real_agent_teleop import preflight_real_teleop

                preflight_real_teleop(
                    env_id,
                    teleop_device=kwargs.get("teleop_device"),
                )
                session.activate()
            env: RealEnv = session.construct_env(
                lambda adopt: _make_real_agent_env(
                    gymnasium,
                    env_id=env_id,
                    hardware=hardware,
                    adopt=adopt,
                )
            )  # type: ignore[assignment]
            env.reset()

            def agent_impl(**kwargs: Any) -> None:
                phase2_metadata = collect_run_metadata(
                    **metadata_common_kwargs,
                    env_cfg=None,
                    agent_cfg=agent_cfg,
                    env=env,
                )
                write_run_metadata(phase2_metadata, run_logdir)
                kwargs["run_metadata"] = phase2_metadata
                if evaluation_logdir is not None:
                    kwargs["evaluation_logdir"] = evaluation_logdir

                kwargs.update(
                    {
                        "env_id": env_id,
                        "agent_cfg": agent_cfg,
                        "env_cfg": None,
                    }
                )

                workflow_kwargs = kwargs
                if native_input_workflow or real_agent_subcommand == "ros":
                    workflow_kwargs = {**kwargs, "real_agent_session": session}
                _dispatch_real_agent_workflow(real_agent_subcommand, workflow_kwargs)

            class FakeSimulationApp:
                def is_running(self) -> bool:
                    return session.running

            agent_impl(
                env=env,
                sim_app=FakeSimulationApp(),
                logdir=logdir,
                headless=False,
                **kwargs,
            )

    hydra_main()  # type: ignore[misc]


_dispatch_real_agent_workflow = _dispatch_cli.dispatch_real_agent_workflow


def _resolve_real_agent_policy_inputs(
    *,
    real_agent_subcommand: str,
    algo: str | None,
    model: Path | str | None,
) -> tuple[str | None, Path | str | None]:
    """Resolve real-agent policy inputs before logdir and Hydra setup."""
    algo, model = _policy_cli.resolve_model_artifact_policy_inputs(
        algo=algo,
        model=model,
    )
    algo = _policy_cli.resolve_agent_workflow_algo(
        agent_subcommand=real_agent_subcommand,
        algo=algo,
        model=model,
    )
    return algo, model


_generate_real_agent_subprocess = _generation_cli._generate_real_agent_subprocess


__all__ = ("execute_runtime_command",)
