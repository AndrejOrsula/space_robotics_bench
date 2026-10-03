"""Lifecycle helpers for per-env SRB validation specs."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from srb.core.spec.models import EnvSpec

SRB_SPEC_RESET_ERROR = "env.reset() must be called before env.srb_spec is accessed"


def init_srb_spec_state(env: Any) -> None:
    """Ensure an env instance has the SRB spec lifecycle state attrs."""
    if not hasattr(env, "_cached_spec"):
        env._cached_spec = None
    if not hasattr(env, "_post_reset_called"):
        env._post_reset_called = False


def mark_srb_spec_reset(env: Any) -> None:
    """Mark that the env has completed a reset that can populate EnvSpec data."""
    env._post_reset_called = True


def require_srb_spec_reset(env: Any) -> None:
    """Fail closed until reset has populated runtime buffers used by EnvSpec."""
    if not getattr(env, "_post_reset_called", False):
        raise RuntimeError(SRB_SPEC_RESET_ERROR)


def get_cached_srb_spec(env: Any) -> "EnvSpec":
    """Return the cached SRB EnvSpec, building it on first access."""
    require_srb_spec_reset(env)
    if getattr(env, "_cached_spec", None) is None:
        from srb.core.spec import builder

        env._cached_spec = builder.build_env_spec(env)
    return env._cached_spec


def invalidate_cached_srb_spec(env: Any) -> None:
    """Drop the cached SRB EnvSpec without clearing the post-reset marker."""
    env._cached_spec = None


__all__ = [
    "SRB_SPEC_RESET_ERROR",
    "get_cached_srb_spec",
    "init_srb_spec_state",
    "invalidate_cached_srb_spec",
    "mark_srb_spec_reset",
    "require_srb_spec_reset",
]
