from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

from srb.utils.env_capability import EnvCapability

_MISSING = object()


@dataclass(frozen=True)
class AgentCommandCapabilityPolicy:
    best_effort: bool = False
    required_env_data: tuple[str, ...] = ()


_DEFAULT_AGENT_COMMAND_CAPABILITY_POLICY = AgentCommandCapabilityPolicy()
_AGENT_COMMAND_CAPABILITY_POLICIES: Mapping[str, AgentCommandCapabilityPolicy] = {
    "ros": AgentCommandCapabilityPolicy(
        best_effort=True,
        required_env_data=(
            "action_manager.iter_terms",
            "action_space.shape",
            "cfg",
            "device",
            "num_envs",
            "scene",
        ),
    ),
    "teleop": AgentCommandCapabilityPolicy(best_effort=True),
    "collect": AgentCommandCapabilityPolicy(best_effort=True),
}


def classify_env(env: Any) -> EnvCapability:
    target = env if isinstance(env, type) else getattr(env, "unwrapped", env)

    if getattr(target, "is_srb_env", False):
        return EnvCapability.SRB_NATIVE

    module_name = getattr(target, "__module__", type(target).__module__)
    if module_name.startswith("isaaclab.") or (
        hasattr(target, "scene") and hasattr(target, "sim")
    ):
        return EnvCapability.ISAACLAB_NATIVE

    return EnvCapability.GYMNASIUM_NATIVE


def resolve_env_capability(
    *,
    env_id: str,
    env: Any,
    declared_capability: EnvCapability | str | None = None,
) -> EnvCapability:
    if declared_capability is not None:
        return EnvCapability(declared_capability)
    if (registered_capability := _get_registered_env_capability(env_id)) is not None:
        return registered_capability
    return classify_env(env)


@dataclass(frozen=True)
class CapabilityDecision:
    env_id: str
    command: str
    capability: EnvCapability
    best_effort: bool = False
    missing: tuple[str, ...] = ()

    @property
    def should_fail(self) -> bool:
        return bool(self.missing)

    @property
    def should_warn(self) -> bool:
        return (
            self.best_effort
            and not self.should_fail
            and self.capability is not EnvCapability.SRB_NATIVE
        )

    @property
    def message(self) -> str | None:
        if self.should_fail:
            return missing_capability_error(
                env_id=self.env_id,
                command=self.command,
                capability=self.capability,
                missing=", ".join(self.missing),
            )
        if self.should_warn:
            return best_effort_warning(
                env_id=self.env_id,
                command=self.command,
                capability=self.capability,
            )
        return None


def decide_capability_support(
    *,
    env_id: str,
    command: str,
    capability: EnvCapability,
    best_effort: bool = False,
    missing: Iterable[str] = (),
) -> CapabilityDecision:
    return CapabilityDecision(
        env_id=env_id,
        command=command,
        capability=capability,
        best_effort=best_effort,
        missing=tuple(item for item in missing if item),
    )


def get_agent_command_capability_policy(
    agent_subcommand: str,
) -> AgentCommandCapabilityPolicy:
    return _AGENT_COMMAND_CAPABILITY_POLICIES.get(
        agent_subcommand,
        _DEFAULT_AGENT_COMMAND_CAPABILITY_POLICY,
    )


def decide_agent_capability_support(
    *,
    env_id: str,
    agent_subcommand: str,
    env: Any,
    declared_capability: EnvCapability | str | None = None,
) -> CapabilityDecision:
    policy = get_agent_command_capability_policy(agent_subcommand)
    return decide_capability_support(
        env_id=env_id,
        command=f"agent {agent_subcommand}",
        capability=resolve_env_capability(
            env_id=env_id,
            env=env,
            declared_capability=declared_capability,
        ),
        best_effort=policy.best_effort,
        missing=missing_env_requirements(env, policy.required_env_data),
    )


def missing_env_requirements(env: Any, requirements: Iterable[str]) -> tuple[str, ...]:
    target = env if isinstance(env, type) else getattr(env, "unwrapped", env)
    return tuple(
        requirement
        for requirement in requirements
        if _resolve_attr_path(target, requirement) is _MISSING
    )


def _resolve_attr_path(target: Any, path: str) -> Any:
    current = target
    for attr in path.split("."):
        current = getattr(current, attr, _MISSING)
        if current is _MISSING or current is None:
            return _MISSING
    return current


def _get_registered_env_capability(env_id: str) -> EnvCapability | None:
    try:
        from srb.utils.registry import get_declared_srb_task_capability
    except ImportError:
        return None
    return get_declared_srb_task_capability(env_id)


def best_effort_warning(
    *,
    env_id: str,
    command: str,
    capability: EnvCapability,
) -> str:
    return (
        f"Warning: command '{command}' for environment '{env_id}' is running in "
        f"best-effort mode for capability '{capability.value}'. Falling back to "
        "available Gymnasium/IsaacLab interfaces where SRB-native metadata is "
        "not available."
    )


def missing_capability_error(
    *,
    env_id: str,
    command: str,
    capability: EnvCapability,
    missing: str,
) -> str:
    return (
        f"Command '{command}' for environment '{env_id}' detected capability "
        f"'{capability.value}' but cannot continue because required data is "
        f"missing: {missing}."
    )


__all__ = [
    "AgentCommandCapabilityPolicy",
    "CapabilityDecision",
    "EnvCapability",
    "best_effort_warning",
    "classify_env",
    "decide_agent_capability_support",
    "decide_capability_support",
    "get_agent_command_capability_policy",
    "missing_env_requirements",
    "missing_capability_error",
    "resolve_env_capability",
]
