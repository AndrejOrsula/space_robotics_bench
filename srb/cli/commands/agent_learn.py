"""CLI adapter for offline agent learning workflows."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

from srb.cli.commands import agent_policy as _policy_cli
from srb.experiments.workflows import run_offline_learning as _run_offline_learning
from srb.utils.path import SRB_LOGS_DIR, new_logdir

_maybe_package_model_artifact_from_run = (
    _policy_cli.maybe_package_model_artifact_from_run
)


def learn_from_demonstrations(
    algo: str,
    dataset: str | None = None,
    logdir_path: str = "",
    model_artifact_output: Path | str | None = None,
    model_artifact_checkpoint: Path | str | None = None,
    model_artifact_task: str | None = None,
    model_artifact_embodiment: str | None = None,
    model_artifact_action_mode: str | None = None,
    model_artifact_framework: str | None = None,
    model_artifact_dataset: Path | str | None = None,
    model_artifact_dataset_repo_id: str | None = None,
    model_artifact_dataset_revision: str | None = None,
    model_artifact_repo_id: str | None = None,
    **kwargs: Any,
) -> None:
    """Learn from collected demonstrations using an offline workflow."""
    if not logdir_path:
        logdir_path = str(SRB_LOGS_DIR)

    logdir = Path(logdir_path).resolve()

    env_id = kwargs.pop("env_id", "unknown")
    if "/" in env_id:
        env_id = env_id.rsplit("/", 1)[-1]
    agent_cfg = _offline_agent_cfg(
        kwargs.pop("config", "DEFAULT"), kwargs.pop("hydra_args", ())
    )

    # The logdir must use the raw algo name so that checkpoint discovery in
    # ``srb agent eval``/``collect`` (which resolves ``logs/<env>/<algo>/``)
    # finds the run without an explicit ``--model``.
    workflow_logdir = new_logdir(
        env_id=env_id,
        workflow=algo,
        root=logdir,
    )

    _run_offline_learning(
        algo=algo,
        dataset=dataset,
        logdir=workflow_logdir,
        env=None,
        sim_app=None,
        env_id=env_id,
        env_cfg=None,
        agent_cfg=agent_cfg,
        **kwargs,
    )
    _maybe_package_model_artifact_from_run(
        run_dir=workflow_logdir,
        output=model_artifact_output,
        algo=algo,
        env=None,
        env_cfg=None,
        checkpoint=model_artifact_checkpoint,
        task=model_artifact_task,
        embodiment=model_artifact_embodiment,
        action_mode=model_artifact_action_mode,
        framework=model_artifact_framework,
        training_dataset=model_artifact_dataset or dataset,
        dataset_repo_id=model_artifact_dataset_repo_id,
        dataset_revision=model_artifact_dataset_revision,
        repo_id=model_artifact_repo_id,
    )


def _offline_agent_cfg(
    config: str | None, hydra_args: Sequence[str]
) -> dict[str, Any] | None:
    """Merge the ``agent`` section of ``--cfg`` with ``agent.KEY=VALUE`` overrides."""
    from omegaconf import OmegaConf

    merged = OmegaConf.create()
    if config and config.upper() not in ("DEFAULT", "IGNORE", "NONE", "NULL"):
        loaded = OmegaConf.load(config)
        if not OmegaConf.is_dict(loaded) or "agent" not in loaded:
            raise ValueError(f"--cfg {config} has no 'agent' section")
        merged = OmegaConf.merge(merged, loaded.agent)
    if hydra_args:
        merged = OmegaConf.merge(
            merged,
            OmegaConf.from_dotlist([arg.removeprefix("agent.") for arg in hydra_args]),
        )
    agent_cfg = OmegaConf.to_container(merged, resolve=True)
    return agent_cfg or None


__all__ = (
    "learn_from_demonstrations",
    "_maybe_package_model_artifact_from_run",
)
