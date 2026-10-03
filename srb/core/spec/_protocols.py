"""SpecProvider Protocol — opt-in for components with dynamic descriptors."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from srb.core.spec.registry import ComponentDescriptor


@runtime_checkable
class SpecProvider(Protocol):
    """Components may provide their descriptor at runtime.

    Builder priority:
        1. SpecProvider — wins
        2. SpecRegistry @spec — fallback for non-provider components
        3. Top-down reflection — last resort, marks provenance="<fallback>"
    """

    def spec(self) -> ComponentDescriptor: ...


@runtime_checkable
class SpecMetadataProvider(Protocol):
    """Optional task metadata merged into ``EnvSpec.metadata`` under task keys.

    The builder stays task-agnostic; without this hook, metadata is empty.
    """

    def srb_spec_metadata(self) -> dict[str, object]: ...
