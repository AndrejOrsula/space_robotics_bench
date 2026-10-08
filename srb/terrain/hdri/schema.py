"""Pydantic schema for the HDRI/skydome catalog."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

from srb.utils.artifact_paths import validate_artifact_segment


class SkydomeEntry(BaseModel):
    """Catalog entry for a skydome texture and its bake settings."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    body: Literal["moon", "mars", "earth"]
    url: str
    sha256: str = Field(min_length=64, max_length=64, pattern=r"^[0-9a-fA-F]{64}$")
    format: Literal["png", "hdr", "exr"]
    intensity: float = Field(gt=0.0, allow_inf_nan=False)
    resize_to: int = Field(default=2048, ge=64, le=8192)
    license: str
    attribution: str


class SkydomeCatalog(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: int
    skydomes: dict[str, SkydomeEntry]

    @field_validator("skydomes")
    @classmethod
    def _skydome_names_are_path_segments(
        cls, value: dict[str, SkydomeEntry]
    ) -> dict[str, SkydomeEntry]:
        for name in value:
            validate_artifact_segment(name, field="skydome name")
        return value
