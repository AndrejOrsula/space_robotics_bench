"""Experiment workflow dispatch for SRB agent commands."""

from __future__ import annotations

from importlib import import_module
from pathlib import Path
from typing import Any, Literal

from srb.core.algorithms import canonical_algorithm_label
from srb.integrations.algorithms import (
    AlgorithmAdapter,
    resolve_algorithm_adapter,
    validate_demo_training_inputs,
)
from srb.utils.torch_runtime import warm_torch_native_stack


def run_online_training(
    *,
    algo: str,
    workflow: Literal["train", "train_real"] = "train",
    **kwargs: Any,
) -> None:
    """Run an online-training workflow for an SRB algorithm label."""
    algo = canonical_algorithm_label(algo)
    adapter = resolve_algorithm_adapter(algo)
    validate_demo_training_inputs(
        algo=algo,
        demo_dataset=kwargs.get("demo_dataset"),
        demo_max_transitions=kwargs.get("demo_max_transitions"),
    )
    if not adapter.online_training:
        raise ValueError(
            f"Algorithm '{algo}' is not supported for online training. "
            "Use 'learn' for imitation learning / offline RL algorithms."
        )
    if workflow == "train_real" and not adapter.real_training:
        raise ValueError(
            f"Algorithm '{algo}' is not supported for real-environment training."
        )
    _run_integration(adapter, workflow=workflow, **kwargs)
    _export_metrics(kwargs.get("logdir"))


def _export_metrics(logdir: Any) -> None:
    """Export TensorBoard scalars beside the run without failing training."""
    if not logdir:
        return
    try:
        from srb.utils.metrics_jsonl import export_tensorboard_scalars

        export_tensorboard_scalars(Path(logdir))
    except (OSError, ValueError, RuntimeError, ImportError) as exc:  # pragma: no cover
        from srb.utils import logging

        logging.warning(f"metrics.jsonl export skipped: {exc}")


def run_policy_evaluation(
    *,
    algo: str,
    model: Path | str | None,
    **kwargs: Any,
) -> None:
    """Run a policy-evaluation workflow for an SRB algorithm label."""
    algo = canonical_algorithm_label(algo)
    adapter = resolve_algorithm_adapter(algo)
    if not adapter.policy_evaluation:
        raise ValueError(f"Algorithm '{algo}' is not supported for evaluation.")
    _run_integration(adapter, workflow="eval", model=model, **kwargs)


def run_offline_learning(
    *,
    algo: str,
    **kwargs: Any,
) -> None:
    """Run an offline-learning workflow for an SRB algorithm label."""
    algo = canonical_algorithm_label(algo)
    adapter = resolve_algorithm_adapter(algo)
    if not adapter.offline_learning:
        raise ValueError(
            f"Algorithm '{algo}' is not supported for offline learning. "
            "Use 'train' for online RL algorithms."
        )
    _run_integration(adapter, workflow="learn", **kwargs)


def _run_integration(
    adapter: AlgorithmAdapter,
    **kwargs: Any,
) -> None:
    if adapter.integration == "skrl":
        run = _import_integration_run(adapter.integration)
        run(**adapter.integration_kwargs(), **kwargs)
        return
    main = _import_integration_main(adapter.integration)
    main.run(**adapter.integration_kwargs(), **kwargs)


def _import_integration_run(integration: str) -> Any:
    """Load an integration's public run entry point after Torch setup."""
    warm_torch_native_stack()
    package = import_module(f"srb.integrations.{integration}")
    run = getattr(package, "run", None)
    if not callable(run):
        raise TypeError(
            f"integration '{integration}' public run entry point must be callable"
        )
    return run


def _import_integration_main(integration: str) -> Any:
    # Load Torch's native libraries before importing framework providers.
    warm_torch_native_stack()
    package = import_module(f"srb.integrations.{integration}")
    main = getattr(package, "main", None)
    if main is not None:
        return main
    return import_module(f"srb.integrations.{integration}.main")
