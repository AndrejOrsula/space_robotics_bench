"""Execute plans from :mod:`srb.core.env.direct_env_finalize_init_plan`."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from srb.core.env.direct_env_finalize_init_plan import (
    DirectEnvFinalizeInitPlan,
    FinalizeInitStep,
)


class DirectEnvFinalizeInitHooks(Protocol):
    def init_step_return(self) -> None: ...

    def init_reward_outcome(self) -> None: ...

    def preinit_recorders(self) -> None: ...

    def update_gym_spaces(self) -> None: ...

    def init_direct_step_outcome_adapter(self) -> None: ...

    def configure_delay_runtime(self) -> None: ...

    def init_episode_recorder(self) -> None: ...


def execute_direct_env_finalize_init_plan(
    plan: DirectEnvFinalizeInitPlan, hooks: DirectEnvFinalizeInitHooks
) -> None:
    for step in plan.steps:
        if step is FinalizeInitStep.INIT_STEP_RETURN:
            hooks.init_step_return()
        elif step is FinalizeInitStep.INIT_REWARD_OUTCOME:
            hooks.init_reward_outcome()
        elif step is FinalizeInitStep.PREINIT_RECORDERS:
            hooks.preinit_recorders()
        elif step is FinalizeInitStep.UPDATE_GYM_SPACES:
            hooks.update_gym_spaces()
        elif step is FinalizeInitStep.INIT_DIRECT_STEP_OUTCOME_ADAPTER:
            hooks.init_direct_step_outcome_adapter()
        elif step is FinalizeInitStep.CONFIGURE_DELAY_RUNTIME:
            hooks.configure_delay_runtime()
        elif step is FinalizeInitStep.INIT_EPISODE_RECORDER:
            hooks.init_episode_recorder()
        else:  # pragma: no cover
            raise RuntimeError(f"Unhandled DirectEnv finalize-init step: {step}")


@dataclass(slots=True)
class TraceDirectEnvFinalizeInitHooks:
    calls: list[FinalizeInitStep] = field(default_factory=list)

    def init_step_return(self) -> None:
        self.calls.append(FinalizeInitStep.INIT_STEP_RETURN)

    def init_reward_outcome(self) -> None:
        self.calls.append(FinalizeInitStep.INIT_REWARD_OUTCOME)

    def preinit_recorders(self) -> None:
        self.calls.append(FinalizeInitStep.PREINIT_RECORDERS)

    def update_gym_spaces(self) -> None:
        self.calls.append(FinalizeInitStep.UPDATE_GYM_SPACES)

    def init_direct_step_outcome_adapter(self) -> None:
        self.calls.append(FinalizeInitStep.INIT_DIRECT_STEP_OUTCOME_ADAPTER)

    def configure_delay_runtime(self) -> None:
        self.calls.append(FinalizeInitStep.CONFIGURE_DELAY_RUNTIME)

    def init_episode_recorder(self) -> None:
        self.calls.append(FinalizeInitStep.INIT_EPISODE_RECORDER)


__all__ = [
    "DirectEnvFinalizeInitHooks",
    "TraceDirectEnvFinalizeInitHooks",
    "execute_direct_env_finalize_init_plan",
]
