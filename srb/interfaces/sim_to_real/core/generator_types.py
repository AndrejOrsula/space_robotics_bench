from __future__ import annotations

from dataclasses import dataclass

import gymnasium
from pydantic import BaseModel, Field

from srb.core.command_mapping import CommandMapping

# Explicit provenance for the legacy extraction path: spaces sampled from one
# ``extract_step_return()`` with dtype-extrema bounds. Canonical contract-backed
# extraction records the contract id instead and never uses dtype extrema.
LEGACY_SCHEMA_PROVENANCE = "legacy_sampled_dtype_extrema"


class EnvInfo(BaseModel, arbitrary_types_allowed=True):
    single_action_space: gymnasium.Space
    detailed_action_space: gymnasium.spaces.Dict
    action_ordering: dict[str, slice]
    action_rate: float
    action_scale: dict[str, float] = Field(default_factory=dict)
    command_mapping: CommandMapping | None = None

    single_observation_space: gymnasium.Space
    detailed_observation_space: gymnasium.spaces.Dict

    robot: str

    # Deployment identity/readiness. Legacy extraction leaves the
    # defaults; canonical contract-backed extraction fills every field.
    schema_provenance: str = LEGACY_SCHEMA_PROVENANCE
    io_schema_fingerprint: str | None = None
    target_manifest_sha256: str | None = None
    requires_runtime_target_manifest: bool = False
    deployment_requirements: tuple[str, ...] = Field(default_factory=tuple)


@dataclass(frozen=True)
class PyExpr:
    """Raw Python expression inserted verbatim by generator renderer."""

    code: str

    def __str__(self) -> str:
        return self.code

    def __repr__(self) -> str:  # pragma: no cover
        return f"PyExpr({self.code!r})"
