"""Validation specification models and YAML loader.

The models are pure configuration contracts, so datasets and other lower
consumers can load a spec without importing interface runtime modules.
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Sequence

from omegaconf import OmegaConf
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationInfo,
    field_validator,
    model_validator,
)

_TASK_IDENTIFIER_RE = re.compile(
    r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}\Z",
    re.ASCII,
)
_TASK_IDENTIFIER_ERROR = (
    "task must be a single ASCII identifier matching ^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$"
)


def _require_finite(value: float, *, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return value


class ThresholdsCfg(BaseModel):
    model_config = ConfigDict(extra="forbid")
    success_rate_min: float = Field(default=0.5, ge=0.0, le=1.0, strict=True)
    sim_real_ratio_min: float = Field(default=0.7, gt=0.0, strict=True)
    action_smoothness_max_ratio: float = Field(default=1.5, gt=0.0, strict=True)
    lower_is_better_metrics: list[str] = Field(default_factory=list)
    crash_rate_max: float = Field(default=0.0, ge=0.0, le=1.0, strict=True)
    drift_z_max: float = Field(default=3.0, gt=0.0, strict=True)

    @field_validator(
        "success_rate_min",
        "sim_real_ratio_min",
        "action_smoothness_max_ratio",
        "crash_rate_max",
        "drift_z_max",
    )
    @classmethod
    def _finite(cls, value: float, info: ValidationInfo) -> float:
        return _require_finite(value, name=str(info.field_name))


class RerunCfg(BaseModel):
    model_config = ConfigDict(extra="forbid")
    app_id: str = "srb-validation"
    web_port: int = Field(default=9090, ge=1, le=65535, strict=True)
    grpc_port: int = Field(default=9876, ge=1, le=65535, strict=True)
    bind_address: str | None = "0.0.0.0"
    keep_historical_data: bool = True
    keep_scalar_history: bool = True
    record_to_rrd: bool = True


class SimBaselineRef(BaseModel):
    model_config = ConfigDict(extra="forbid")
    wandb_run_id: str | None = None
    local_rrd: str | None = None
    metrics_json: str | None = None


class ValidationSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")
    spec_id: str | None = None
    task: str
    algo: str
    checkpoint: str
    # A session must attempt at least one episode. A zero-episode session
    # already fails criterion (b) ("no completed validation episodes", exit 2),
    # but `--episodes 0` is a caller mistake worth refusing at the front door
    # rather than reporting as a failed validation run.
    n_episodes: int = Field(default=10, ge=1, strict=True)
    max_episode_seconds: float = Field(default=120.0, gt=0.0, strict=True)
    hardware: list[str]
    thresholds: ThresholdsCfg = Field(default_factory=ThresholdsCfg)
    rerun: RerunCfg = Field(default_factory=RerunCfg)
    sim_baseline: SimBaselineRef = Field(default_factory=SimBaselineRef)
    drift_window: int = Field(default=5, ge=1, strict=True)
    storage_root: str = "logs/real_validation"
    metrics_to_track: list[str]
    notes: str = ""

    @model_validator(mode="before")
    @classmethod
    def _discard_legacy_null_fault_injection(cls, value: object) -> object:
        """Accept stored defaults from older specs without exposing the setting."""

        if (
            isinstance(value, Mapping)
            and value.get("fault_injection", object()) is None
        ):
            value = dict(value)
            value.pop("fault_injection")
        return value

    @field_validator("task")
    @classmethod
    def _validate_task_identifier(cls, value: str) -> str:
        if _TASK_IDENTIFIER_RE.fullmatch(value) is None:
            raise ValueError(_TASK_IDENTIFIER_ERROR)
        return value

    @field_validator("max_episode_seconds")
    @classmethod
    def _finite_timeout(cls, value: float) -> float:
        return _require_finite(value, name="max_episode_seconds")


def load_validation_spec(
    path: str | Path, *, overrides: Sequence[str] | None = None
) -> ValidationSpec:
    """Load a YAML file and apply Hydra-style dotpath overrides.

    Overrides take the form ``"field.subfield=value"``. Lists/dicts use the
    standard OmegaConf override syntax.
    """

    cfg = OmegaConf.load(str(path))
    if overrides:
        cfg = OmegaConf.merge(cfg, OmegaConf.from_dotlist(list(overrides)))
    raw = OmegaConf.to_container(cfg, resolve=True)
    if not isinstance(raw, dict):
        raise ValueError(
            f"validation spec YAML must be a mapping, got {type(raw).__name__}"
        )
    return ValidationSpec.model_validate(raw)


__all__ = [
    "RerunCfg",
    "SimBaselineRef",
    "ThresholdsCfg",
    "ValidationSpec",
    "load_validation_spec",
]
