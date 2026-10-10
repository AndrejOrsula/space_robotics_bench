"""CLI adapter for training agents."""

from __future__ import annotations

from typing import Any, Literal

from srb.cli.commands import agent_policy as _policy_cli

_resolve_model_artifact_policy_inputs = _policy_cli.resolve_model_artifact_policy_inputs
_resolve_model_artifact_reference = _policy_cli.resolve_model_artifact_reference


def train_agent(
    algo: str, workflow: Literal["train", "train_real"] = "train", **kwargs: Any
) -> None:
    """Train an agent with online RL workflow adapters."""
    from srb.experiments.workflows import run_online_training

    reference = None
    if kwargs.get("model") is not None:
        reference = _resolve_model_artifact_reference(algo=algo, model=kwargs["model"])
    if reference is None:
        run_online_training(algo=algo, workflow=workflow, **kwargs)
        return

    # Resume from the private snapshot validated during preflight, not from the
    # caller-owned artifact member, which may be replaced afterwards. The
    # snapshot lives only as long as the reference, so hold it for the run.
    kwargs["model"] = reference.load_path
    try:
        run_online_training(algo=algo, workflow=workflow, **kwargs)
    finally:
        reference.close()


__all__ = (
    "train_agent",
    "_resolve_model_artifact_policy_inputs",
    "_resolve_model_artifact_reference",
)
