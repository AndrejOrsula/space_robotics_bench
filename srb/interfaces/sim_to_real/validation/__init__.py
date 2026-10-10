"""Real-robot evaluation and validation harness.

Public exports are resolved lazily so importing a small validation submodule
does not pull telemetry, PyTorch, TensorBoard, or other optional runtime sinks
into Isaac/Kit startup.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS: dict[str, str] = {
    "RerunCfg": "srb.utils.validation_spec",
    "SimBaselineRef": "srb.utils.validation_spec",
    "ThresholdsCfg": "srb.utils.validation_spec",
    "ValidationSpec": "srb.utils.validation_spec",
    "load_validation_spec": "srb.utils.validation_spec",
    "ValidationHarness": "harness",
    "CRASH_TERM_REASONS": "outcome",
    "SESSION_ABORT_TERM_REASONS": "outcome",
    "TIME_LIMIT_TRUNCATION_TERM_REASON": "outcome",
    "EpisodeOutcome": "outcome",
    "ValidationEpisodeOutcome": "outcome",
    "episode_metrics": "outcome",
    "is_crash_term_reason": "outcome",
    "is_session_abort_term_reason": "outcome",
    "LANDSCAPING_NORMALIZATION_POLICY": "landscaping",
    "LandscapingAcceptedFrame": "landscaping",
    "LandscapingRealEvaluator": "landscaping",
    "landscaping_session_abort": "landscaping",
    "load_baseline_metrics": "baseline_metrics",
    "CriterionResult": "criteria",
    "evaluate_threshold": "criteria",
    "evaluate_crash": "criteria",
    "evaluate_drift": "criteria",
    "github_status_command": "github_status",
    "post_github_status": "github_status",
    "ValidationSessionWorkspace": "session_workspace",
    "ValidationSessionIdentity": "session_workspace",
    "build_session_workspace": "session_workspace",
    "current_git_sha": "session_workspace",
    "current_code_sha": "session_workspace",
    "validation_config_digest": "session_workspace",
    "validation_artifact_digest": "session_workspace",
    "utc_session_timestamp": "session_workspace",
    "write_spec_snapshot": "session_workspace",
    "call_policy_factory": "policy_factory",
    "ValidationEnvFactory": "runtime_session",
    "ValidationPolicyCallable": "runtime_session",
    "ValidationPolicyFactory": "runtime_session",
    "ValidationRuntimeSession": "runtime_session",
    "ValidationSessionFinalizer": "session_finalizer",
    "ValidationAbortSignal": "episode_runner",
    "ValidationEpisodeRollout": "episode_runner",
    "ValidationEpisodeRunner": "episode_runner",
    "ValidationPolicy": "episode_runner",
    "ValidationTelemetrySink": "episode_runner",
    "SessionResult": "result",
    "build_session_result": "result",
    "criterion_to_dict": "report_payload",
    "summary_payload": "report_payload",
    "badge_payload": "report_payload",
    "load_index_rows": "report_index",
    "write_validation_index": "report_index",
    "aggregate_session_metrics": "session_metrics",
    "iter_episode_end_records": "session_metrics",
    "ValidationRolloutAdapter": "rollout",
    "ValidationStep": "rollout",
    "EPISODES_JSONL": "srb.utils.validation_session_log",
    "build_step_record": "srb.utils.validation_session_log",
    "build_episode_end_record": "srb.utils.validation_session_log",
    "build_event_record": "srb.utils.validation_session_log",
    "write_validation_record": "srb.utils.validation_session_log",
    "validation_log_path": "srb.utils.validation_session_log",
    "iter_validation_records": "srb.utils.validation_session_log",
    "read_validation_episode_records": "srb.utils.validation_session_log",
    "NonFiniteValidationError": "numeric",
    "ensure_finite": "numeric",
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(
        module_name if "." in module_name else f"{__name__}.{module_name}"
    )
    value = getattr(module, name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted((*globals(), *__all__))
