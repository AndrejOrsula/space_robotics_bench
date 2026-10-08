"""Pydantic schema for the lighting catalog."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class LightingPreset(BaseModel):
    """Single named lighting preset.

    All angles in degrees. Sun intensity in lux. Color temperature in K.
    Earthshine fields are optional (None disables the secondary light).
    """

    # Non-finite values would fail JSON serialization only later, at bake time.
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)

    body: Literal["moon", "mars", "earth"]
    sun_elevation_deg: float = Field(ge=-90.0, le=90.0)
    sun_azimuth_deg: float = Field(ge=0.0, lt=360.0)
    sun_intensity_lux: float = Field(ge=0.0)
    sun_color_temperature_k: float = Field(gt=1000.0, le=20000.0)
    sun_angular_diameter_deg: float = Field(gt=0.0, le=5.0)
    earthshine_intensity_lux: float | None = Field(default=None, ge=0.0)
    earthshine_color_temperature_k: float | None = Field(
        default=None, gt=1000.0, le=20000.0
    )
    license: str = "Public Domain"
    attribution: str = "SRB synthesized lighting preset"


class LightingCatalog(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: int
    presets: dict[str, LightingPreset]
