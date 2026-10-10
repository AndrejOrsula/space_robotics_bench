"""CLI adapter for dispatching constructed agent environments to workflows."""

from __future__ import annotations

from typing import Any


def dispatch_agent_workflow(
    agent_subcommand: str,
    kwargs: dict[str, Any],
    *,
    headless: bool = False,
    package_model_artifact_from_run: Any | None = None,
) -> None:
    """Dispatch a simulation workflow after the environment has been constructed."""
    from srb.cli.commands.agent_baseline import (
        random_agent,
        zero_agent,
    )
    from srb.cli.commands.agent_collection import collect_agent
    from srb.cli.commands.agent_eval import eval_agent
    from srb.cli.commands.agent_ros import ros_agent
    from srb.cli.commands.agent_teleop import teleop_agent
    from srb.cli.commands.agent_train import train_agent

    match agent_subcommand:
        case "zero":
            zero_agent(**kwargs)
        case "rand":
            random_agent(**kwargs)
        case "teleop":
            teleop_agent(headless=headless, **kwargs)
        case "ros":
            ros_agent(**kwargs)
        case "train":
            train_agent(**kwargs)
            if package_model_artifact_from_run is not None:
                package_model_artifact_from_run(
                    run_dir=kwargs["logdir"],
                    output=kwargs.get("model_artifact_output"),
                    algo=kwargs.get("algo"),
                    env=kwargs.get("env"),
                    env_cfg=kwargs.get("env_cfg"),
                    checkpoint=kwargs.get("model_artifact_checkpoint"),
                    task=kwargs.get("model_artifact_task"),
                    embodiment=kwargs.get("model_artifact_embodiment"),
                    action_mode=kwargs.get("model_artifact_action_mode"),
                    framework=kwargs.get("model_artifact_framework"),
                    training_dataset=kwargs.get("model_artifact_dataset"),
                    dataset_repo_id=kwargs.get("model_artifact_dataset_repo_id"),
                    dataset_revision=kwargs.get("model_artifact_dataset_revision"),
                    repo_id=kwargs.get("model_artifact_repo_id"),
                )
        case "eval":
            eval_agent(**kwargs)
        case "collect":
            collect_agent(headless=headless, **kwargs)
        case _:
            raise ValueError(f"Unsupported agent workflow: {agent_subcommand}")


def dispatch_real_agent_workflow(
    real_agent_subcommand: str,
    kwargs: dict[str, Any],
) -> None:
    """Dispatch a real-agent workflow after the RealEnv has been constructed."""
    from srb.cli.commands.agent_baseline import (
        random_agent,
        zero_agent,
    )
    from srb.cli.commands.agent_collection import collect_agent
    from srb.cli.commands.agent_eval import eval_agent
    from srb.cli.commands.agent_ros import ros_agent
    from srb.cli.commands.agent_teleop import teleop_agent
    from srb.cli.commands.agent_train import train_agent

    match real_agent_subcommand:
        case "zero":
            zero_agent(**kwargs)
        case "rand":
            random_agent(**kwargs)
        case "teleop":
            teleop_agent(**kwargs)
        case "ros":
            ros_agent(**kwargs)
        case "train":
            train_agent(workflow="train_real", **kwargs)
        case "eval":
            eval_agent(**kwargs)
        case "collect":
            collect_kwargs = dict(kwargs)
            collect_kwargs.setdefault("dataset_split", "real")
            collect_agent(**collect_kwargs)
        case _:
            raise ValueError(
                f"Unsupported real-agent workflow: {real_agent_subcommand}"
            )


_dispatch_agent_workflow = dispatch_agent_workflow
_dispatch_real_agent_workflow = dispatch_real_agent_workflow


__all__ = (
    "dispatch_agent_workflow",
    "dispatch_real_agent_workflow",
    "_dispatch_agent_workflow",
    "_dispatch_real_agent_workflow",
)
