"""Pydantic schema for the rocks catalog."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

from srb.utils.artifact_paths import validate_artifact_segment


class Rock(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    url: str
    sha256: str = Field(min_length=64, max_length=64, pattern=r"^[0-9a-fA-F]{64}$")
    format: Literal["glb", "obj", "ply", "stl"]
    target_face_count: int = Field(default=5000, ge=500, le=50000)
    license: str
    attribution: str


class RockSet(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    body: Literal["moon", "mars", "earth"]
    rocks: dict[str, Rock]

    @field_validator("rocks")
    @classmethod
    def _rock_ids_are_path_segments(cls, value: dict[str, Rock]) -> dict[str, Rock]:
        for rock_id in value:
            validate_artifact_segment(rock_id, field="rock id")
        return value


class RocksCatalog(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: int
    sets: dict[str, RockSet]

    @field_validator("sets")
    @classmethod
    def _set_names_are_path_segments(
        cls, value: dict[str, RockSet]
    ) -> dict[str, RockSet]:
        for set_name in value:
            validate_artifact_segment(set_name, field="rock set name")
        return value
