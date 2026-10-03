"""Init planning for :class:`~srb.core.env.common.base.direct.impl.DirectEnv`.

This module is intentionally importable without Isaac Lab (and without SRB's
SimForge-dependent config layer) so unit tests can validate initialization
ordering/invariants in a lightweight environment.

The `Module` here is the plan builder: it concentrates init ordering knowledge
behind a small `Interface` (a plan of named steps).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class DirectEnvInitStep(str, Enum):
    INIT_HUD_FIELDS = "init_hud_fields"
    SUPER_INIT = "super_init"
    INIT_SRB_SPEC_STATE = "init_srb_spec_state"
    APPLY_VISUALS = "apply_visuals"
    INIT_ACTION_MANAGER = "init_action_manager"
    INIT_ASSET_REGISTRIES = "init_asset_registries"
    INIT_PARTICLE_SYSTEMS = "init_particle_systems"
    REAPPLY_VIEWER_POSE = "reapply_viewer_pose"


@dataclass(frozen=True, slots=True)
class DirectEnvInitPlan:
    steps: tuple[DirectEnvInitStep, ...]

    def _index(self, step: DirectEnvInitStep) -> int | None:
        try:
            return self.steps.index(step)
        except ValueError:
            return None

    def assert_invariants(self) -> None:
        """Validate ordering invariants for the plan.

        Raises:
            ValueError: if the plan violates a load-bearing ordering constraint.
        """

        def _require_before(a: DirectEnvInitStep, b: DirectEnvInitStep) -> None:
            ia = self._index(a)
            ib = self._index(b)
            if ia is None or ib is None:
                return
            if ia >= ib:
                raise ValueError(
                    f"DirectEnv init invariant violated: {a} must run before {b}"
                )

        # Super-init must run before anything that reads env runtime state.
        for step in self.steps:
            if step in (
                DirectEnvInitStep.INIT_HUD_FIELDS,
                DirectEnvInitStep.SUPER_INIT,
            ):
                continue
            _require_before(DirectEnvInitStep.SUPER_INIT, step)

        # HUD fields must be available before viewer/reward hooks touch them.
        _require_before(DirectEnvInitStep.INIT_HUD_FIELDS, DirectEnvInitStep.SUPER_INIT)

        # Asset registries must exist before optional registries that depend on them.
        _require_before(
            DirectEnvInitStep.INIT_ASSET_REGISTRIES,
            DirectEnvInitStep.INIT_PARTICLE_SYSTEMS,
        )

        # Viewer pose re-apply is intended as a final polish step.
        for step in self.steps:
            if step is DirectEnvInitStep.REAPPLY_VIEWER_POSE:
                continue
            _require_before(step, DirectEnvInitStep.REAPPLY_VIEWER_POSE)


def build_direct_env_init_plan(
    *,
    actions_enabled: bool,
    particle_systems_enabled: bool,
) -> DirectEnvInitPlan:
    """Build the init plan for a DirectEnv.

    Args:
        actions_enabled: True if an ActionManager should be constructed.
        particle_systems_enabled: True if particle-system registry is non-empty.

    Returns:
        A plan describing which steps to run and in what order.
    """

    steps: list[DirectEnvInitStep] = [
        DirectEnvInitStep.INIT_HUD_FIELDS,
        DirectEnvInitStep.SUPER_INIT,
        DirectEnvInitStep.INIT_SRB_SPEC_STATE,
        DirectEnvInitStep.APPLY_VISUALS,
    ]

    if actions_enabled:
        steps.append(DirectEnvInitStep.INIT_ACTION_MANAGER)

    steps.append(DirectEnvInitStep.INIT_ASSET_REGISTRIES)

    if particle_systems_enabled:
        steps.append(DirectEnvInitStep.INIT_PARTICLE_SYSTEMS)

    steps.append(DirectEnvInitStep.REAPPLY_VIEWER_POSE)

    plan = DirectEnvInitPlan(steps=tuple(steps))
    plan.assert_invariants()
    return plan


__all__ = [
    "DirectEnvInitPlan",
    "DirectEnvInitStep",
    "build_direct_env_init_plan",
]
