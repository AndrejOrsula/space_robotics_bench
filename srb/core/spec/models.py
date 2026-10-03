"""Pydantic types for env spec."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

DType = Literal[
    "float32",
    "float64",
    "int8",
    "int16",
    "int32",
    "int64",
    "uint8",
    "uint16",
    "uint32",
    "uint64",
    "bool",
]


class TermSpec(BaseModel):
    model_config = ConfigDict(frozen=False, arbitrary_types_allowed=False)

    name: str
    shape: tuple[int, ...]
    dtype: DType
    units: str | None = None
    frame: str | None = None
    semantic_label: str | None = None
    min: float | list[float] | None = None
    max: float | list[float] | None = None
    privileged: bool = False
    provenance: str | None = None

    @field_validator("shape")
    @classmethod
    def _shape_positive(cls, v: tuple[int, ...]) -> tuple[int, ...]:
        if not v:
            raise ValueError("shape must be non-empty")
        if any(d <= 0 for d in v):
            raise ValueError(f"shape components must be positive: {v}")
        return v

    @model_validator(mode="after")
    def _bounds_match_shape(self) -> TermSpec:
        # Per-element bounds reshape to ``shape`` during projection; reject a
        # wrong element count here instead of at the projection boundary.
        size = 1
        for extent in self.shape:
            size *= extent
        for bound_name in ("min", "max"):
            bound = getattr(self, bound_name)
            if isinstance(bound, list) and len(bound) != size:
                raise ValueError(
                    f"{bound_name} has {len(bound)} elements; shape {self.shape} "
                    f"needs {size}"
                )
        return self


class GroupSpec(BaseModel):
    name: str
    terms: dict[str, TermSpec]
    schema_version: int = 1


class SensorEntry(BaseModel):
    name: str
    category: str
    output_shape: tuple[int, ...]
    output_dtype: DType
    update_period_s: float | None = None
    provenance: str | None = None
    extras: dict[str, Any] = Field(default_factory=dict)


class ObservationSpec(BaseModel):
    groups: dict[str, GroupSpec]
    schema_version: int = 1


class ActionSpec(BaseModel):
    groups: dict[str, GroupSpec]
    schema_version: int = 1


class SensorSpec(BaseModel):
    sensors: dict[str, SensorEntry]
    schema_version: int = 1


class EnvSpec(BaseModel):
    schema_version: int = 1
    task: str
    observation: ObservationSpec
    action: ActionSpec
    sensor: SensorSpec
    metadata: dict[str, Any] = Field(default_factory=dict)
