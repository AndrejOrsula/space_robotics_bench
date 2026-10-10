"""CLI adapter for ``srb real_agent validate``."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Callable

REQUIRED_CHECKPOINT_PLACEHOLDER = "REQUIRED_AT_RUNTIME"


def execute_validate_command(**kwargs: Any) -> int:
    """Run ``srb real_agent validate`` and return its process exit code."""

    from srb.interfaces.sim_to_real.validation import harness as validation_harness
    from srb.utils import validation_spec

    overrides = build_validation_spec_overrides(
        spec_path=kwargs["spec"],
        checkpoint=kwargs.get("checkpoint"),
        episodes=kwargs.get("episodes"),
        storage_root=kwargs.get("storage_root"),
    )
    spec = validation_spec.load_validation_spec(kwargs["spec"], overrides=overrides)
    dry_run = bool(kwargs.get("dry_run", False))

    if not dry_run:
        # Verify one typed deployment identity before environment, hardware,
        # or policy side effects.
        from srb.interfaces.sim_to_real.validation.deployment import (
            ValidationDeploymentError,
        )

        try:
            deployment = verify_validation_deployment(spec)
        except ValidationDeploymentError as exc:
            logging.error("validation deployment rejected: %s", exc)
            return 4
        logging.info("validation deployment: %s", deployment.as_report_dict())

    if dry_run:
        harness = validation_harness.ValidationHarness(
            spec,
            env_factory=None,
            policy_factory=None,
            wandb_enabled=bool(kwargs.get("wandb", False)),
            tensorboard_enabled=not kwargs.get("no_tensorboard", False),
            rerun_web_enabled=not kwargs.get("no_rerun_web", False),
            post_status=kwargs.get("post_status", False),
            dry_run=True,
        )
        return int(harness.run().exit_code)

    # Deployment identity is settled before entering the process owner. Its
    # deferred activation keeps earlier workspace/preflight failures ROS-free.
    from srb.cli.commands.real_agent_session import RealAgentSession

    with RealAgentSession(defer_activation=True) as process:
        runtime: Any | None = None

        def env_factory(current_spec: Any) -> Any:
            if runtime is None:
                raise RuntimeError("validation runtime owner is not bound")
            process.activate()
            return make_validation_env(
                current_spec, adopt_cleanup_owner=runtime.adopt_environment
            )

        harness = validation_harness.ValidationHarness(
            spec,
            env_factory=env_factory,
            policy_factory=load_validation_policy,
            wandb_enabled=bool(kwargs.get("wandb", False)),
            tensorboard_enabled=not kwargs.get("no_tensorboard", False),
            rerun_web_enabled=not kwargs.get("no_rerun_web", False),
            post_status=kwargs.get("post_status", False),
            dry_run=False,
        )
        runtime = harness.bind_runtime_cleanup(process.close)
        process.adopt_cleanup_owner(runtime)
        return int(harness.run().exit_code)


def ensure_real_env_registry() -> None:
    """Register generated ``srb_real/*`` IDs with Gymnasium."""

    from srb.interfaces.sim_to_real.env import register_all

    register_all()
    from srb.interfaces.sim_to_real import hardware as _srb_real_hw  # noqa: F401


def _srb_real_task_ids() -> tuple[str, ...]:
    from srb.utils.registry import get_srb_tasks

    return tuple(get_srb_tasks("srb_real"))


def _registered_real_tasks() -> tuple[str, ...]:
    """Register the generated real environments, then list their ids."""

    ensure_real_env_registry()
    return _srb_real_task_ids()


def verify_validation_deployment(spec: Any) -> Any:
    """Build the typed deployment identity without creating runtime objects.

    Registers generated environments and raises on identity mismatches.
    """

    from srb.interfaces.sim_to_real.validation.deployment import (
        build_validation_deployment,
    )

    return build_validation_deployment(
        spec, registered_real_tasks=_registered_real_tasks
    )


def make_validation_env(
    spec: Any,
    *,
    adopt_cleanup_owner: Callable[[Any], None] | None = None,
) -> Any:
    """Create the real validation environment for a loaded spec.

    Missing real environments raise instead of falling back to simulation,
    which could misreport simulation results as hardware validation. Other
    construction errors propagate for the harness to report as preflight
    failures.
    """

    import gymnasium as gym
    from gymnasium.error import NameNotFound, NamespaceNotFound

    deployment = verify_validation_deployment(spec)
    try:
        if adopt_cleanup_owner is not None:
            from srb.cli.commands.real_agent_runtime import _make_real_agent_env

            return _make_real_agent_env(
                gym,
                env_id=deployment.env_id,
                hardware=deployment.hardware,
                adopt=adopt_cleanup_owner,
            )
        return gym.make(deployment.env_id, hardware=list(deployment.hardware))
    except (NameNotFound, NamespaceNotFound) as exc:
        raise RuntimeError(
            f"Simulation-to-real environment '{deployment.env_id}' is not "
            f"registered, so validation task '{deployment.task}' has no real "
            "environment to run. Generate it first (srb real_agent gen). "
            "Validation never falls back to the base simulation task: a "
            "simulation result must never be reported as a real-validation "
            "result."
        ) from exc


def load_validation_policy(spec: Any, *, env: Any = None) -> Any:
    """Load the validation policy from the spec checkpoint."""

    try:
        from srb.cli.commands import agent_policy
    except ImportError as exc:
        raise NotImplementedError(
            "Policy loading helper not available. "
            "Use --dry-run to smoke-test validation wiring."
        ) from exc
    deployment = verify_validation_deployment(spec)
    try:
        policy = agent_policy.load_policy_from_checkpoint(
            deployment.algo,
            str(deployment.checkpoint),
            env=env,
        )
    except AssertionError as exc:
        message = str(exc)
        if spec.algo == "dreamer" and "assert_trees_all_equal_shapes" in message:
            raise RuntimeError(
                "Dreamer checkpoint is incompatible with this validation "
                "environment's observation/action spaces. Train or pass a "
                f"checkpoint produced for task '{spec.task}'."
            ) from exc
        raise
    return policy


def build_validation_spec_overrides(
    *,
    spec_path: str | Path,
    checkpoint: str | None,
    episodes: int | None,
    storage_root: str | None = None,
) -> list[str]:
    """Build Hydra dotlist overrides for a validation spec."""

    overrides: list[str] = []
    if checkpoint is not None:
        overrides.append(f"checkpoint={checkpoint}")
        artifact_algo = infer_validation_checkpoint_artifact_algo(checkpoint)
        if artifact_algo is not None:
            spec_algo = read_validation_spec_algo(spec_path)
            if spec_algo is None or str(spec_algo).strip() in (
                "",
                REQUIRED_CHECKPOINT_PLACEHOLDER,
            ):
                overrides.append(f"algo={artifact_algo}")
            else:
                from srb.experiments.artifacts import slugify_underscore

                if slugify_underscore(str(spec_algo)) != artifact_algo:
                    raise ValueError(
                        f"validation spec algo {spec_algo!r} does not match "
                        f"model artifact algo {artifact_algo!r}"
                    )
    if episodes is not None:
        overrides.append(f"n_episodes={episodes}")
    if storage_root:
        overrides.append(f"storage_root={storage_root}")
    return overrides


def infer_validation_checkpoint_artifact_algo(checkpoint: str) -> str | None:
    """Return a model artifact's algorithm slug when checkpoint points to one."""

    from srb.experiments.models import try_model_artifact_reference

    artifact_reference = try_model_artifact_reference(checkpoint)
    if artifact_reference is None:
        return None
    return artifact_reference.algo


def read_validation_spec_algo(spec_path: str | Path) -> object | None:
    """Read ``algo`` from a validation YAML file without full spec validation."""

    from omegaconf import OmegaConf

    raw = OmegaConf.to_container(OmegaConf.load(str(spec_path)), resolve=True)
    if not isinstance(raw, dict):
        return None
    return raw.get("algo")


__all__ = [
    "REQUIRED_CHECKPOINT_PLACEHOLDER",
    "build_validation_spec_overrides",
    "ensure_real_env_registry",
    "execute_validate_command",
    "infer_validation_checkpoint_artifact_algo",
    "load_validation_policy",
    "make_validation_env",
    "read_validation_spec_algo",
    "verify_validation_deployment",
]
