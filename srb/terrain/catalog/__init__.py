"""Per-body pinned terrain catalogs, loading, and resolution exports."""

from __future__ import annotations

from srb.terrain.catalog.resolve import (
    FLOAT32_BYTES,
    MAX_REGION_SIZE_M,
    POLAR_LAT_DEG,
    CompositionPlan,
    LayerContribution,
    RegionRequest,
    est_fetch_bytes,
    resolve_stacks,
    to_stack_refs,
)

__all__ = [
    "FLOAT32_BYTES",
    "MAX_REGION_SIZE_M",
    "POLAR_LAT_DEG",
    "CompositionPlan",
    "LayerContribution",
    "RegionRequest",
    "est_fetch_bytes",
    "resolve_stacks",
    "to_stack_refs",
]
