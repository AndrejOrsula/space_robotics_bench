"""Execute plans from :mod:`srb.core.env.direct_env_init_plan`."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from srb.core.env.direct_env_init_plan import DirectEnvInitPlan, DirectEnvInitStep


class DirectEnvInitHooks(Protocol):
    def init_hud_fields(self) -> None: ...

    def super_init(self) -> None: ...

    def init_srb_spec_state(self) -> None: ...

    def apply_visuals(self) -> None: ...

    def init_action_manager(self) -> None: ...

    def init_asset_registries(self) -> None: ...

    def init_particle_systems(self) -> None: ...

    def reapply_viewer_pose(self) -> None: ...


def execute_direct_env_init_plan(
    plan: DirectEnvInitPlan, hooks: DirectEnvInitHooks
) -> None:
    for step in plan.steps:
        if step is DirectEnvInitStep.INIT_HUD_FIELDS:
            hooks.init_hud_fields()
        elif step is DirectEnvInitStep.SUPER_INIT:
            hooks.super_init()
        elif step is DirectEnvInitStep.INIT_SRB_SPEC_STATE:
            hooks.init_srb_spec_state()
        elif step is DirectEnvInitStep.APPLY_VISUALS:
            hooks.apply_visuals()
        elif step is DirectEnvInitStep.INIT_ACTION_MANAGER:
            hooks.init_action_manager()
        elif step is DirectEnvInitStep.INIT_ASSET_REGISTRIES:
            hooks.init_asset_registries()
        elif step is DirectEnvInitStep.INIT_PARTICLE_SYSTEMS:
            hooks.init_particle_systems()
        elif step is DirectEnvInitStep.REAPPLY_VIEWER_POSE:
            hooks.reapply_viewer_pose()
        else:  # pragma: no cover
            raise RuntimeError(f"Unhandled DirectEnv init step: {step}")


@dataclass(slots=True)
class TraceDirectEnvInitHooks:
    calls: list[DirectEnvInitStep] = field(default_factory=list)

    def init_hud_fields(self) -> None:
        self.calls.append(DirectEnvInitStep.INIT_HUD_FIELDS)

    def super_init(self) -> None:
        self.calls.append(DirectEnvInitStep.SUPER_INIT)

    def init_srb_spec_state(self) -> None:
        self.calls.append(DirectEnvInitStep.INIT_SRB_SPEC_STATE)

    def apply_visuals(self) -> None:
        self.calls.append(DirectEnvInitStep.APPLY_VISUALS)

    def init_action_manager(self) -> None:
        self.calls.append(DirectEnvInitStep.INIT_ACTION_MANAGER)

    def init_asset_registries(self) -> None:
        self.calls.append(DirectEnvInitStep.INIT_ASSET_REGISTRIES)

    def init_particle_systems(self) -> None:
        self.calls.append(DirectEnvInitStep.INIT_PARTICLE_SYSTEMS)

    def reapply_viewer_pose(self) -> None:
        self.calls.append(DirectEnvInitStep.REAPPLY_VIEWER_POSE)


__all__ = [
    "DirectEnvInitHooks",
    "TraceDirectEnvInitHooks",
    "execute_direct_env_init_plan",
]
