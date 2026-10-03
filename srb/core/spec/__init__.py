"""Typed observation / action / sensor spec API."""

from __future__ import annotations

from srb.core.spec.builder import build_env_spec
from srb.core.spec.decorators import spec, spec_field
from srb.core.spec.lifecycle import (
    get_cached_srb_spec,
    init_srb_spec_state,
    invalidate_cached_srb_spec,
    mark_srb_spec_reset,
    require_srb_spec_reset,
)
from srb.core.spec.models import (
    ActionSpec,
    EnvSpec,
    GroupSpec,
    ObservationSpec,
    SensorEntry,
    SensorSpec,
    TermSpec,
)
from srb.core.spec.registry import ComponentDescriptor, SpecRegistry

__all__: list[str] = [
    "ActionSpec",
    "build_env_spec",
    "ComponentDescriptor",
    "EnvSpec",
    "get_cached_srb_spec",
    "GroupSpec",
    "init_srb_spec_state",
    "invalidate_cached_srb_spec",
    "mark_srb_spec_reset",
    "ObservationSpec",
    "require_srb_spec_reset",
    "SensorEntry",
    "SensorSpec",
    "spec",
    "spec_field",
    "SpecRegistry",
    "TermSpec",
]
