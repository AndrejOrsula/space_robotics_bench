"""Task evaluation and session-abort contracts for real deployments.

RealTaskEvaluator owns task metrics and success criteria; RealEnv owns call
ordering. Session aborts remain distinct from MDP termination and truncation.
Deployment identity is checked before policy inference. Landscaping-specific
evaluation lives in :mod:`srb.interfaces.sim_to_real.validation.landscaping`.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from math import isfinite
from typing import Any, ClassVar, NoReturn, Protocol, cast, runtime_checkable

import numpy as np

from srb.interfaces.sim_to_real.core.hardware import DeploymentCapability
from srb.utils.validation_contract import (
    SESSION_ABORT_OUTCOMES,
    SESSION_OUTCOME_CALIBRATION_MISMATCH,
    SESSION_OUTCOME_MANIFEST_MISMATCH,
    SESSION_OUTCOME_OPERATOR_ABORT,
    SESSION_OUTCOME_SENSOR_INCOMPLETE,
    SESSION_OUTCOME_SENSOR_STALE,
    TIME_LIMIT_TRUNCATION,
)


class RealSessionAbort(RuntimeError):
    """A real session ended outside the MDP; ``outcome`` names why."""

    outcome: ClassVar[str] = ""


class SensorStaleAbort(RealSessionAbort):
    """An accepted-frame source crossed the hard freshness cutoff."""

    outcome: ClassVar[str] = SESSION_OUTCOME_SENSOR_STALE


class SensorIncompleteAbort(RealSessionAbort):
    """A sensor frame was missing, unsynchronized, or not fully covered."""

    outcome: ClassVar[str] = SESSION_OUTCOME_SENSOR_INCOMPLETE


class CalibrationMismatchAbort(RealSessionAbort):
    """The delivered frame's frame/datum/calibration is not the deployed one."""

    outcome: ClassVar[str] = SESSION_OUTCOME_CALIBRATION_MISMATCH


class ManifestMismatchAbort(RealSessionAbort):
    """Deployed task identity differs from the manifest or model artifact."""

    outcome: ClassVar[str] = SESSION_OUTCOME_MANIFEST_MISMATCH


class OperatorAbort(RealSessionAbort):
    """An operator (or safety) stop ended the session."""

    outcome: ClassVar[str] = SESSION_OUTCOME_OPERATOR_ABORT


class _ImmutableMetrics(dict[str, float]):
    """Dict-shaped evaluation snapshot that remains serialization-friendly."""

    def __init__(self, values: Mapping[str, float]) -> None:
        dict.__init__(self, values)

    def __setitem__(self, key: str, value: float) -> NoReturn:
        raise TypeError("real task evaluation metrics are immutable")

    def __delitem__(self, key: str) -> NoReturn:
        raise TypeError("real task evaluation metrics are immutable")

    def clear(self) -> NoReturn:
        raise TypeError("real task evaluation metrics are immutable")

    def pop(self, *args: object) -> NoReturn:
        raise TypeError("real task evaluation metrics are immutable")

    def popitem(self) -> NoReturn:
        raise TypeError("real task evaluation metrics are immutable")

    def setdefault(self, *args: object) -> NoReturn:
        raise TypeError("real task evaluation metrics are immutable")

    def update(self, *args: object, **kwargs: object) -> NoReturn:
        raise TypeError("real task evaluation metrics are immutable")

    def __ior__(self, other: object) -> NoReturn:  # type: ignore[misc]
        raise TypeError("real task evaluation metrics are immutable")


@dataclass(frozen=True, slots=True)
class RealTaskEvaluation:
    """One step's task-owned evaluation of an accepted real frame."""

    reward: float
    success: bool
    metrics: Mapping[str, float]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "reward",
            _finite_real(self.reward, field_name="real task evaluation reward"),
        )
        if type(self.success) is not bool:
            raise TypeError("real task evaluation success must be a boolean")
        object.__setattr__(self, "metrics", _freeze_metrics(self.metrics))


@runtime_checkable
class AcceptedFrameProvider(Protocol):
    """Provide the current accepted frame or raise a session abort.

    Returning ``None`` signals an incomplete frame; RealEnv raises
    :class:`SensorIncompleteAbort` rather than evaluate stale data.
    """

    def accepted_frame(self) -> Any | None: ...


class RealTaskEvaluator(Protocol):
    """Task-owned evaluator passed explicitly to RealEnv.

    Not runtime-checkable: ``isinstance`` would only check property presence.
    RealEnv validates the object in ``_prepare_task_evaluator``.
    """

    @property
    def deployment_capabilities(self) -> frozenset[DeploymentCapability]: ...

    @property
    def horizon_steps(self) -> int: ...

    @property
    def identity(self) -> "DeploymentIdentity": ...

    def reset(self) -> None: ...

    def validate_frame(self, frame: Any) -> None:
        """Validate a pre-dispatch frame without mutating episode state."""
        ...

    def evaluate(self, *, frame: Any, action: np.ndarray) -> RealTaskEvaluation: ...

    def final_info(self) -> dict[str, Any]: ...


#: Fields compared against the loaded model artifact before policy inference.
MODEL_ARTIFACT_IDENTITY_FIELDS: tuple[str, ...] = (
    "task_id",
    "io_schema_fingerprint",
    "target_manifest_sha256",
    "reset_layout_sha256",
    "normalization_policy",
)


@dataclass(frozen=True, slots=True)
class DeploymentIdentity:
    """Deployed task identity that a model artifact must match."""

    task_id: str
    io_schema_fingerprint: str
    target_manifest_sha256: str
    reset_layout_sha256: str
    normalization_policy: str

    def __post_init__(self) -> None:
        for field in MODEL_ARTIFACT_IDENTITY_FIELDS:
            _require_non_empty_string(
                getattr(self, field), field_name=f"deployment identity {field}"
            )

    def as_dict(self) -> dict[str, str]:
        return asdict(self)


def verify_model_artifact_identity(
    deployed: DeploymentIdentity,
    artifact: Mapping[str, Any] | None,
) -> None:
    """Reject startup when model and deployed task identities differ.

    Missing or differing comparable fields raise
    :class:`ManifestMismatchAbort` before ROS acquisition, hardware start or
    policy inference. The artifact may omit ``reset_layout_sha256`` only when
    its target-manifest hash matches the deployed identity.
    """

    if not isinstance(deployed, DeploymentIdentity):
        raise TypeError("deployed task identity must be a DeploymentIdentity")

    if not isinstance(artifact, Mapping):
        raise ManifestMismatchAbort(
            "model artifact declares no task-identity block; a real session "
            "may not load a policy whose task id, IO-schema fingerprint, "
            "target manifest, reset layout, and normalization policy cannot "
            f"be compared against the deployed identity {deployed.as_dict()}"
        )

    differing: list[str] = []
    unverifiable: list[str] = []
    expected = deployed.as_dict()
    for field in MODEL_ARTIFACT_IDENTITY_FIELDS:
        if field not in artifact:
            if field == "reset_layout_sha256" and _manifest_hash_matches(
                artifact, expected
            ):
                continue
            unverifiable.append(f"{field} (missing)")
            continue
        value = artifact[field]
        if type(value) is not str or not value:
            unverifiable.append(f"{field} (must be a non-empty string)")
            continue
        if value != expected[field]:
            differing.append(field)

    if differing or unverifiable:
        details = []
        for field in differing:
            details.append(
                f"{field}: artifact={artifact[field]!r} deployed={expected[field]!r}"
            )
        for field in unverifiable:
            details.append(f"{field}: missing or invalid in the model artifact")
        raise ManifestMismatchAbort(
            "model artifact does not match the deployed task identity; "
            "refusing to start before policy inference (" + "; ".join(details) + ")"
        )


def _manifest_hash_matches(
    artifact: Mapping[str, Any], expected: Mapping[str, str]
) -> bool:
    declared = artifact.get("target_manifest_sha256")
    return (
        type(declared) is str
        and bool(declared)
        and declared == expected["target_manifest_sha256"]
    )


def _require_non_empty_string(value: object, *, field_name: str) -> None:
    if type(value) is not str:
        raise TypeError(f"{field_name} must be a string")
    if not value:
        raise ValueError(f"{field_name} must be non-empty")


def _finite_real(value: object, *, field_name: str) -> float:
    if type(value) not in (int, float):
        raise TypeError(f"{field_name} must be an int or float")
    try:
        numeric = float(cast(int | float, value))
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{field_name} must be finite") from exc
    if not isfinite(numeric):
        raise ValueError(f"{field_name} must be finite")
    return numeric


def _freeze_metrics(metrics: Mapping[str, float]) -> Mapping[str, float]:
    if not isinstance(metrics, Mapping):
        raise TypeError("real task evaluation metrics must be a mapping")
    frozen: dict[str, float] = {}
    for key, value in metrics.items():
        if type(key) is not str:
            raise TypeError("real task evaluation metric keys must be strings")
        frozen[key] = _finite_real(
            value, field_name=f"real task evaluation metric {key!r}"
        )
    return _ImmutableMetrics(frozen)


__all__ = [
    "MODEL_ARTIFACT_IDENTITY_FIELDS",
    "SESSION_ABORT_OUTCOMES",
    "SESSION_OUTCOME_CALIBRATION_MISMATCH",
    "SESSION_OUTCOME_MANIFEST_MISMATCH",
    "SESSION_OUTCOME_OPERATOR_ABORT",
    "SESSION_OUTCOME_SENSOR_INCOMPLETE",
    "SESSION_OUTCOME_SENSOR_STALE",
    "TIME_LIMIT_TRUNCATION",
    "AcceptedFrameProvider",
    "CalibrationMismatchAbort",
    "DeploymentIdentity",
    "ManifestMismatchAbort",
    "OperatorAbort",
    "RealSessionAbort",
    "RealTaskEvaluation",
    "RealTaskEvaluator",
    "SensorIncompleteAbort",
    "SensorStaleAbort",
    "verify_model_artifact_identity",
]
