"""Finalize-init planning for :class:`~srb.core.env.common.base.direct.impl.DirectEnv`.

This module is intentionally importable without Isaac Lab (and without SRB's
SimForge-dependent config layer) so unit tests can validate initialization
ordering/invariants in a lightweight environment.

The `Module` here is the plan builder: it concentrates the init ordering
knowledge behind a small `Interface` (a plan of named steps).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class FinalizeInitStep(str, Enum):
    INIT_STEP_RETURN = "init_step_return"
    INIT_REWARD_OUTCOME = "init_reward_outcome"
    PREINIT_RECORDERS = "preinit_recorders"
    UPDATE_GYM_SPACES = "update_gym_spaces"
    INIT_DIRECT_STEP_OUTCOME_ADAPTER = "init_direct_step_outcome_adapter"
    CONFIGURE_DELAY_RUNTIME = "configure_delay_runtime"
    INIT_EPISODE_RECORDER = "init_episode_recorder"


@dataclass(frozen=True, slots=True)
class DirectEnvFinalizeInitPlan:
    steps: tuple[FinalizeInitStep, ...]

    def _index(self, step: FinalizeInitStep) -> int | None:
        try:
            return self.steps.index(step)
        except ValueError:
            return None

    def assert_invariants(self) -> None:
        """Validate ordering invariants for the plan.

        Raises:
            ValueError: if the plan violates a load-bearing ordering constraint.
        """

        def _require_before(a: FinalizeInitStep, b: FinalizeInitStep) -> None:
            ia = self._index(a)
            ib = self._index(b)
            if ia is None or ib is None:
                return
            if ia >= ib:
                raise ValueError(
                    f"FinalizeInit invariant violated: {a} must run before {b}"
                )

        # `_update_gym_env_spaces()` may call `_get_observations()` in the legacy
        # workflow, and `_get_observations()` touches `_episode_recorder`.
        _require_before(
            FinalizeInitStep.PREINIT_RECORDERS,
            FinalizeInitStep.UPDATE_GYM_SPACES,
        )

        # Delay runtime consumes action dim / spaces; configure after spaces.
        _require_before(
            FinalizeInitStep.UPDATE_GYM_SPACES,
            FinalizeInitStep.CONFIGURE_DELAY_RUNTIME,
        )

        # Step-return workflow: first step_return must exist before space build.
        _require_before(
            FinalizeInitStep.INIT_STEP_RETURN,
            FinalizeInitStep.UPDATE_GYM_SPACES,
        )

        # Outcome adapter uses reward runtime + observation assembly.
        _require_before(
            FinalizeInitStep.INIT_REWARD_OUTCOME,
            FinalizeInitStep.INIT_DIRECT_STEP_OUTCOME_ADAPTER,
        )
        _require_before(
            FinalizeInitStep.UPDATE_GYM_SPACES,
            FinalizeInitStep.INIT_DIRECT_STEP_OUTCOME_ADAPTER,
        )

        # `init_episode_recorder` overwrites the `_episode_recorder` that
        # `preinit_recorders` set to ``None``, and the legacy space-probing in
        # `update_gym_spaces` -> `_get_observations` relies on it still being
        # ``None``. The recorder must therefore be initialized after both.
        _require_before(
            FinalizeInitStep.PREINIT_RECORDERS,
            FinalizeInitStep.INIT_EPISODE_RECORDER,
        )
        _require_before(
            FinalizeInitStep.UPDATE_GYM_SPACES,
            FinalizeInitStep.INIT_EPISODE_RECORDER,
        )


def build_direct_env_finalize_init_plan(
    *,
    step_return_workflow: bool,
    camera_record: bool,
    event_camera: bool = False,
) -> DirectEnvFinalizeInitPlan:
    """Build the finalize-init plan for a DirectEnv.

    Args:
        step_return_workflow: True if the task overrides ``extract_step_return``.
        camera_record: True if episode recording runtime should be initialized.
        event_camera: True if live event-camera observations are configured.

    Returns:
        A plan describing which steps to run and in what order.
    """

    steps: list[FinalizeInitStep] = []

    if step_return_workflow:
        steps.append(FinalizeInitStep.INIT_STEP_RETURN)

    steps.append(FinalizeInitStep.INIT_REWARD_OUTCOME)
    steps.append(FinalizeInitStep.PREINIT_RECORDERS)
    steps.append(FinalizeInitStep.UPDATE_GYM_SPACES)

    if step_return_workflow:
        steps.append(FinalizeInitStep.INIT_DIRECT_STEP_OUTCOME_ADAPTER)

    steps.append(FinalizeInitStep.CONFIGURE_DELAY_RUNTIME)

    if camera_record or event_camera:
        steps.append(FinalizeInitStep.INIT_EPISODE_RECORDER)

    plan = DirectEnvFinalizeInitPlan(steps=tuple(steps))
    plan.assert_invariants()
    return plan


__all__ = [
    "DirectEnvFinalizeInitPlan",
    "FinalizeInitStep",
    "build_direct_env_finalize_init_plan",
]
