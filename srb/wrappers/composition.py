"""Standard SRB environment-wrapper composition."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def wrap_env(env: "AnyEnv") -> "AnyEnv":
    """Apply the standard tuning, action, and observation wrappers."""
    from srb.wrappers import (
        ActionSmoothingWrapper,
        ObservationStackingWrapper,
        TuneReporterWrapper,
    )

    # Report raw transitions before action and observation transforms.
    env = TuneReporterWrapper.wrap(env)
    env = ActionSmoothingWrapper.wrap(env, cfg=env.unwrapped.cfg.action_smoothing)
    env = ObservationStackingWrapper.wrap(
        env, cfg=env.unwrapped.cfg.observation_stacking
    )
    return env


__all__ = ("wrap_env",)
