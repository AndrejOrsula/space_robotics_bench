"""Env runtime building blocks, re-exported lazily.

Importing one runtime submodule (e.g. ``delay``) should not pull in every other
runtime module, so names resolve on first attribute access.
"""

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING, Any

_EXPORTS: dict[str, str] = {
    "ActionLayout": "action_layout",
    "ActionLayoutLike": "action_layout",
    "ActionTerm": "action_layout",
    "GenericActionLayout": "action_layout",
    "GenericActionTerm": "action_layout",
    "StructuredActionLayout": "action_layout",
    "AgentCommandCapabilityPolicy": "capabilities",
    "CapabilityDecision": "capabilities",
    "EnvCapability": "capabilities",
    "best_effort_warning": "capabilities",
    "classify_env": "capabilities",
    "decide_agent_capability_support": "capabilities",
    "decide_capability_support": "capabilities",
    "get_agent_command_capability_policy": "capabilities",
    "missing_capability_error": "capabilities",
    "missing_env_requirements": "capabilities",
    "resolve_env_capability": "capabilities",
    "ActionDelay": "delay",
    "DelayActivationDiagnostic": "delay",
    "DelayActivationPlan": "delay",
    "DelayChannelConfig": "delay",
    "DelayDiagnosticLevel": "delay",
    "DelayRange": "delay",
    "DelayRuntime": "delay",
    "DelayRuntimeConfig": "delay",
    "ObservationDelay": "delay",
    "TensorDelayBuffer": "delay",
    "plan_delay_runtime_activation": "delay",
    "EPISODE_METRICS_KEY": "episode_info",
    "EPISODE_SUCCESS_KEY": "episode_info",
    "episode_outcome_info": "episode_info",
    "LifecycleAdapter": "lifecycle",
    "LifecycleSettings": "lifecycle",
    "RuntimeLifecycle": "lifecycle",
    "SrbEnvAdapter": "lifecycle",
    "NumpyObservationLayout": "observation_layout",
    "NumpyObservationTerm": "observation_layout",
    "ObservationLayout": "observation_layout",
    "ObservationLayoutLike": "observation_layout",
    "ObservationSpaceLayout": "observation_layout",
    "ObservationTerm": "observation_layout",
    "build_observation_space_layout": "observation_layout",
    "DirectStepOutcomeAdapter": "outcome",
    "NonFiniteWarning": "outcome",
    "ObservationBuilder": "outcome",
    "ResetOutcome": "outcome",
    "StepOutcome": "outcome",
    "StepOutcomeCore": "outcome",
    "process_reset_outcome": "outcome",
    "process_step_outcome": "outcome",
    "build_action_spaces": "spaces",
    "build_observation_spaces": "spaces",
    "coerce_step_return_to_torch": "step_return",
    "flatten_observations": "step_return",
    "scrub_non_finite_step_return": "step_return",
    "validate_step_return": "step_return",
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(f".{module_name}", __name__), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted({*globals(), *_EXPORTS})


if TYPE_CHECKING:
    from .action_layout import (
        ActionLayout,
        ActionLayoutLike,
        ActionTerm,
        GenericActionLayout,
        GenericActionTerm,
        StructuredActionLayout,
    )
    from .capabilities import (
        AgentCommandCapabilityPolicy,
        CapabilityDecision,
        EnvCapability,
        best_effort_warning,
        classify_env,
        decide_agent_capability_support,
        decide_capability_support,
        get_agent_command_capability_policy,
        missing_capability_error,
        missing_env_requirements,
        resolve_env_capability,
    )
    from .delay import (
        ActionDelay,
        DelayActivationDiagnostic,
        DelayActivationPlan,
        DelayChannelConfig,
        DelayDiagnosticLevel,
        DelayRange,
        DelayRuntime,
        DelayRuntimeConfig,
        ObservationDelay,
        TensorDelayBuffer,
        plan_delay_runtime_activation,
    )
    from .episode_info import (
        EPISODE_METRICS_KEY,
        EPISODE_SUCCESS_KEY,
        episode_outcome_info,
    )
    from .lifecycle import (
        LifecycleAdapter,
        LifecycleSettings,
        RuntimeLifecycle,
        SrbEnvAdapter,
    )
    from .observation_layout import (
        NumpyObservationLayout,
        NumpyObservationTerm,
        ObservationLayout,
        ObservationLayoutLike,
        ObservationSpaceLayout,
        ObservationTerm,
        build_observation_space_layout,
    )
    from .outcome import (
        DirectStepOutcomeAdapter,
        NonFiniteWarning,
        ObservationBuilder,
        ResetOutcome,
        StepOutcome,
        StepOutcomeCore,
        process_reset_outcome,
        process_step_outcome,
    )
    from .spaces import build_action_spaces, build_observation_spaces
    from .step_return import (
        coerce_step_return_to_torch,
        flatten_observations,
        scrub_non_finite_step_return,
        validate_step_return,
    )
