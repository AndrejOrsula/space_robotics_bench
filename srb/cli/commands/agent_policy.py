"""CLI adapter for agent policy and model artifact workflows."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

from srb.utils.policy_contract import LoadedPolicy

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def resolve_agent_workflow_algo(
    *,
    agent_subcommand: str,
    algo: str | None,
    model: Path | str | None,
) -> str | None:
    """Resolve the CLI policy algorithm before logdir and Hydra setup."""
    from srb.experiments.policies import resolve_agent_workflow_algo as resolve

    return resolve(
        agent_subcommand=agent_subcommand,
        algo=algo,
        model=model,
    )


def should_use_latest_policy_logdir(
    agent_subcommand: str,
    *,
    algo: str | None,
    collector: str | None = None,
) -> bool:
    """Return whether a policy workflow should reuse the latest training logdir."""
    if not algo:
        return False
    if agent_subcommand in ("eval", "teleop"):
        return True
    if agent_subcommand == "collect":
        return collector in (None, "policy")
    return False


def resolve_model_artifact_policy_inputs(
    *,
    algo: str | None,
    model: Path | str | None,
) -> tuple[str | None, Path | str | None]:
    """Resolve CLI policy inputs when ``model`` points to a model artifact."""
    from srb.experiments.policies import resolve_model_artifact_policy_inputs as resolve

    return resolve(algo=algo, model=model)


def resolve_model_artifact_reference(
    *,
    algo: str | None,
    model: Path | str | None,
) -> Any | None:
    """Return a validated model artifact reference for CLI policy inputs."""
    from srb.experiments.policies import resolve_model_artifact_reference as resolve

    return resolve(algo=algo, model=model)


def maybe_package_model_artifact_from_run(
    *,
    run_dir: Path | str,
    output: Path | str | None,
    algo: str | None,
    env: "AnyEnv | None",
    env_cfg: Any = None,
    checkpoint: Path | str | None = None,
    task: str | None = None,
    embodiment: str | None = None,
    action_mode: str | None = None,
    framework: str | None = None,
    training_dataset: Path | str | None = None,
    dataset_repo_id: str | None = None,
    dataset_revision: str | None = None,
    repo_id: str | None = None,
) -> object | None:
    """Package a model artifact from a completed CLI training run."""
    if output is None:
        return None

    inferred_identity = infer_model_artifact_identity_from_env(
        env=env,
        env_cfg=env_cfg,
    )
    from srb.experiments.models import prepare_model_artifact_from_run
    from srb.utils import logging

    result = prepare_model_artifact_from_run(
        run_dir=Path(run_dir),
        output_dir=Path(output),
        task=task or inferred_identity.get("task"),
        embodiment=embodiment or inferred_identity.get("embodiment"),
        action_mode=action_mode or inferred_identity.get("action_mode"),
        algo=algo,
        checkpoint_path=None if checkpoint is None else Path(checkpoint),
        framework=framework,
        training_dataset_path=(
            None if training_dataset is None else Path(training_dataset)
        ),
        dataset_repo_id=dataset_repo_id,
        dataset_revision=dataset_revision,
        hf_repo_id=repo_id,
    )
    manifest_path = getattr(result, "manifest_path", None)
    if manifest_path is not None:
        logging.info(f"Model artifact saved to: {manifest_path}")
    return result


def infer_model_artifact_identity_from_env(
    *,
    env: "AnyEnv | None",
    env_cfg: Any,
) -> dict[str, str]:
    """Infer artifact identity fields from the current CLI environment context."""
    identity: dict[str, str] = {}
    robot_cfg = getattr(env_cfg, "robot", None)

    task = getattr(env_cfg, "task_name", None)
    if task is None and env is not None:
        task = getattr(getattr(env, "spec", None), "id", None)
    if task is None and env is not None:
        unwrapped = getattr(env, "unwrapped", env)
        task = unwrapped.__class__.__name__
    if task is not None:
        identity["task"] = str(task)

    embodiment = getattr(robot_cfg, "name", None)
    if embodiment is None and robot_cfg is not None:
        embodiment = robot_cfg.__class__.__name__
    if embodiment is not None:
        identity["embodiment"] = str(embodiment)

    action_mode = getattr(robot_cfg, "action_mode", None)
    if action_mode is not None:
        identity["action_mode"] = str(action_mode)

    return identity


def resolve_model_path(
    algo: str,
    model: Path | str | None,
    logdir: Path,
) -> Path:
    """Resolve the model checkpoint path based on algorithm and logdir."""
    from srb.experiments.policies import resolve_model_path as resolve

    return resolve(algo, model, logdir)


def latest_file(
    directory: Path,
    *,
    pattern: str = "*",
    modification_time: bool = False,
) -> Path | None:
    """Return the latest file in a directory using SRB run-layout rules."""
    from srb.experiments.run_layouts import latest_file as latest

    return latest(
        directory,
        pattern=pattern,
        modification_time=modification_time,
    )


def load_policy(
    algo: str,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: Any | None = None,
) -> LoadedPolicy:
    """Load a trained policy for simulation, collection, or validation."""
    from srb.experiments.policies import load_policy as load

    return load(
        algo,
        model_path,
        env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
        artifact_reference=artifact_reference,
    )


def load_policy_from_checkpoint(
    algo: str,
    checkpoint: Path | str,
    *,
    device: str = "cpu",
    env: "AnyEnv | None" = None,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
) -> Callable[[Any], Any]:
    """Load a checkpoint policy for non-interactive validation workflows.

    The validated ``ModelArtifactReference`` (when *checkpoint* is an SRB
    model artifact) is retained through preflight and handed to the policy
    loader so contract metadata is never collapsed to a bare checkpoint path.
    """
    from srb.experiments.policies import load_policy_from_checkpoint as load

    return load(
        algo,
        checkpoint,
        device=device,
        env=env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
    )


_resolve_agent_workflow_algo = resolve_agent_workflow_algo
_should_use_latest_policy_logdir = should_use_latest_policy_logdir
_resolve_model_artifact_policy_inputs = resolve_model_artifact_policy_inputs
_resolve_model_artifact_reference = resolve_model_artifact_reference
_maybe_package_model_artifact_from_run = maybe_package_model_artifact_from_run
_infer_model_artifact_identity_from_env = infer_model_artifact_identity_from_env
_resolve_model_path = resolve_model_path
_latest_file = latest_file
_load_policy = load_policy
_load_policy_from_checkpoint = load_policy_from_checkpoint


__all__ = (
    "LoadedPolicy",
    "infer_model_artifact_identity_from_env",
    "latest_file",
    "load_policy",
    "load_policy_from_checkpoint",
    "maybe_package_model_artifact_from_run",
    "resolve_agent_workflow_algo",
    "resolve_model_artifact_policy_inputs",
    "resolve_model_artifact_reference",
    "resolve_model_path",
    "should_use_latest_policy_logdir",
    "_infer_model_artifact_identity_from_env",
    "_latest_file",
    "_load_policy",
    "_load_policy_from_checkpoint",
    "_maybe_package_model_artifact_from_run",
    "_resolve_agent_workflow_algo",
    "_resolve_model_artifact_policy_inputs",
    "_resolve_model_artifact_reference",
    "_resolve_model_path",
    "_should_use_latest_policy_logdir",
)
