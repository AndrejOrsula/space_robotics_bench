"""Serializable payloads for validation reports and badges."""

from __future__ import annotations

from typing import Any

from srb.interfaces.sim_to_real.validation.criteria import CriterionResult
from srb.interfaces.sim_to_real.validation.numeric import ensure_finite
from srb.interfaces.sim_to_real.validation.result import SessionResult
from srb.interfaces.sim_to_real.validation.session_workspace import (
    ValidationSessionIdentity,
)
from srb.utils.validation_spec import ValidationSpec


def criterion_to_dict(criterion: CriterionResult) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "status": criterion.status,
        "detail": criterion.detail,
        "optional": criterion.optional,
        "breakdown": dict(criterion.breakdown),
    }
    if criterion.score is not None:
        payload["score"] = criterion.score
    return payload


def summary_payload(
    spec: ValidationSpec,
    result: SessionResult,
    *,
    identity: ValidationSessionIdentity | None = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": result.schema_version,
        "pass_overall": result.pass_overall,
        "hardware_evidence": result.hardware_evidence,
        "exit_code": result.exit_code,
        "task": spec.task,
        "algo": spec.algo,
        "checkpoint": spec.checkpoint,
        "criterion_a": criterion_to_dict(result.criterion_a),
        "criterion_b": criterion_to_dict(result.criterion_b),
        "criterion_c": criterion_to_dict(result.criterion_c),
        "metrics": dict(result.metrics),
        "sim_metrics": (
            None if result.sim_metrics is None else dict(result.sim_metrics)
        ),
        "sim_real_ratio": (
            None if result.sim_real_ratio is None else dict(result.sim_real_ratio)
        ),
        "drift_z": None if result.drift_z is None else dict(result.drift_z),
        "notes": spec.notes,
    }
    if identity is not None:
        payload["session_identity"] = identity.to_dict()
    ensure_finite(payload, context="validation summary")
    return payload


def badge_payload(result: SessionResult) -> dict[str, Any]:
    if not result.hardware_evidence:
        return {
            "schemaVersion": 1,
            "label": "real-eval",
            "message": "non-hardware evidence",
            "color": "red",
        }
    if result.pass_overall:
        return {
            "schemaVersion": 1,
            "label": "real-eval",
            "message": "pass",
            "color": "green",
        }

    criterion = (
        "a"
        if result.criterion_a.status == "fail"
        else "b"
        if result.criterion_b.status == "fail"
        else "c"
        if result.criterion_c.status == "fail"
        else "?"
    )
    return {
        "schemaVersion": 1,
        "label": "real-eval",
        "message": f"fail (criterion {criterion})",
        "color": "red",
    }
