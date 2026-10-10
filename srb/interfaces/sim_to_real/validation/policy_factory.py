"""Policy factory adapter for validation pre-flight."""

from __future__ import annotations

import inspect
from typing import Any, Callable

import numpy as np

from srb.utils.validation_spec import ValidationSpec

PolicyCallable = Callable[[Any], np.ndarray]
PolicyFactory = Callable[..., PolicyCallable]


def call_policy_factory(
    policy_factory: PolicyFactory,
    spec: ValidationSpec,
    env: Any,
) -> PolicyCallable:
    """Call policy factories with env context when their Interface allows it."""

    mode = _policy_factory_env_mode(policy_factory)
    if mode == "keyword":
        return policy_factory(spec, env=env)
    if mode == "positional":
        return policy_factory(spec, env)
    return policy_factory(spec)


def _policy_factory_env_mode(policy_factory: PolicyFactory) -> str | None:
    try:
        signature = inspect.signature(policy_factory)
    except (TypeError, ValueError):
        return None

    parameters = tuple(signature.parameters.values())
    if any(param.kind == param.VAR_KEYWORD for param in parameters):
        return "keyword"

    env_param = signature.parameters.get("env")
    if env_param is not None:
        if env_param.kind == env_param.POSITIONAL_ONLY:
            return "positional"
        return "keyword"

    if any(param.kind == param.VAR_POSITIONAL for param in parameters):
        return "positional"

    positional_params = [
        param
        for param in parameters
        if param.kind in (param.POSITIONAL_ONLY, param.POSITIONAL_OR_KEYWORD)
    ]
    if len(positional_params) >= 2:
        return "positional"
    return None
