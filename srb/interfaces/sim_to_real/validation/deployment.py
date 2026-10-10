"""Verified deployment identity for sim-to-real validation sessions.

Bind task, algorithm, checkpoint, and hardware identity before constructing
the environment, hardware, or policy. Construction imports the generated
environment registry but otherwise has no side effects.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REQUIRED_CHECKPOINT_PLACEHOLDER = "REQUIRED_AT_RUNTIME"
REAL_ENV_NAMESPACE = "srb_real"


class ValidationDeploymentError(ValueError):
    """The validation spec does not identify a valid real deployment."""


@dataclass(frozen=True, slots=True)
class ValidationDeployment:
    """Identity fields for one real validation deployment."""

    task: str
    env_id: str
    algo: str
    checkpoint: Path
    artifact_algo: str | None
    hardware: tuple[str, ...]
    spec_id: str | None

    def as_report_dict(self) -> dict[str, Any]:
        return {
            "task": self.task,
            "env_id": self.env_id,
            "algo": self.algo,
            "checkpoint": str(self.checkpoint),
            "artifact_algo": self.artifact_algo,
            "hardware": list(self.hardware),
            "spec_id": self.spec_id,
        }


def _default_registered_real_tasks() -> Sequence[str]:
    """Get registered ``srb_real/*`` ids, loading generated tasks if needed."""

    from srb.utils.registry import get_srb_tasks

    if not get_srb_tasks(REAL_ENV_NAMESPACE):
        from srb.interfaces.sim_to_real import hardware as _srb_real_hw  # noqa: F401
        from srb.interfaces.sim_to_real.env import register_all

        register_all(namespace=REAL_ENV_NAMESPACE)
    return tuple(get_srb_tasks(REAL_ENV_NAMESPACE))


def _default_artifact_algo(checkpoint: Path) -> str | None:
    from srb.experiments.models import try_model_artifact_reference

    reference = try_model_artifact_reference(checkpoint)
    return None if reference is None else reference.algo


def _normalize_algo(value: object) -> str:
    from srb.experiments.artifacts import slugify_underscore

    return slugify_underscore(str(value))


def build_validation_deployment(
    spec: Any,
    *,
    registered_real_tasks: Callable[[], Sequence[str]] | None = None,
    artifact_algo_for: Callable[[Path], str | None] | None = None,
    checkpoint_exists: Callable[[Path], bool] | None = None,
) -> ValidationDeployment:
    """Validate ``spec`` and return its deployment identity.

    Raises :class:`ValidationDeploymentError` for missing or placeholder
    checkpoints, algorithm mismatches, unregistered tasks, or invalid hardware
    selectors.
    """

    task = str(getattr(spec, "task", "") or "").strip()
    if not task:
        raise ValidationDeploymentError("validation spec has no task")

    checkpoint_value = str(getattr(spec, "checkpoint", "") or "").strip()
    if not checkpoint_value or checkpoint_value == REQUIRED_CHECKPOINT_PLACEHOLDER:
        raise ValidationDeploymentError(
            "spec.checkpoint is the placeholder 'REQUIRED_AT_RUNTIME'; pass "
            "--checkpoint <path> at run time"
        )
    checkpoint = Path(checkpoint_value).expanduser()
    exists = checkpoint_exists or (lambda path: path.exists())
    if not exists(checkpoint):
        raise ValidationDeploymentError(f"checkpoint does not exist: {checkpoint}")

    artifact_algo = (artifact_algo_for or _default_artifact_algo)(checkpoint)
    spec_algo_value = getattr(spec, "algo", None)
    spec_algo = "" if spec_algo_value is None else str(spec_algo_value).strip()
    if spec_algo in ("", REQUIRED_CHECKPOINT_PLACEHOLDER):
        if artifact_algo is None:
            raise ValidationDeploymentError(
                "spec.algo is unset and the checkpoint is not an SRB model "
                "artifact; declare algo in the spec"
            )
        algo = artifact_algo
    else:
        algo = _normalize_algo(spec_algo)
        if artifact_algo is not None and _normalize_algo(artifact_algo) != algo:
            raise ValidationDeploymentError(
                f"validation spec algo {spec_algo!r} does not match model "
                f"artifact algo {artifact_algo!r}"
            )

    hardware_values = getattr(spec, "hardware", None) or ()
    hardware: list[str] = []
    from srb.interfaces.sim_to_real import hardware as _srb_real_hw  # noqa: F401
    from srb.interfaces.sim_to_real.core.hardware import HardwareInterfaceRegistry

    for selector in hardware_values:
        if not isinstance(selector, str) or not selector.strip():
            raise ValidationDeploymentError(
                f"hardware selectors must be non-empty strings, got {selector!r}"
            )
        selector = selector.strip()
        if selector in hardware:
            raise ValidationDeploymentError(f"duplicate hardware selector {selector!r}")
        if HardwareInterfaceRegistry.get_by_name(selector) is None:
            raise ValidationDeploymentError(
                f"hardware selector {selector!r} is not a registered interface"
            )
        hardware.append(selector)

    env_id = f"{REAL_ENV_NAMESPACE}/{task}"
    registered = tuple((registered_real_tasks or _default_registered_real_tasks)())
    if env_id not in registered and task not in registered:
        raise ValidationDeploymentError(
            f"Simulation-to-real environment '{env_id}' is not registered, so "
            f"validation task '{task}' has no real environment to run. Generate "
            "it first (srb real_agent gen). Validation never falls back to the "
            "base simulation task: a simulation result must never be reported "
            "as a real-validation result."
        )

    return ValidationDeployment(
        task=task,
        env_id=env_id,
        algo=algo,
        checkpoint=checkpoint,
        artifact_algo=artifact_algo,
        hardware=tuple(hardware),
        spec_id=getattr(spec, "spec_id", None),
    )


__all__ = [
    "REAL_ENV_NAMESPACE",
    "REQUIRED_CHECKPOINT_PLACEHOLDER",
    "ValidationDeployment",
    "ValidationDeploymentError",
    "build_validation_deployment",
]
