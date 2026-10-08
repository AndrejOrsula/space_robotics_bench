"""Cached YAML loader for the lighting catalog."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import yaml

from srb.terrain.lights.schema import LightingCatalog

DEFAULT_CATALOG_PATH = Path(__file__).parent / "catalog.yaml"


@lru_cache(maxsize=4)
def load_lighting_catalog(path: Path | None = None) -> LightingCatalog:
    p = Path(path) if path is not None else DEFAULT_CATALOG_PATH
    raw = yaml.safe_load(p.read_text())
    return LightingCatalog.model_validate(raw)
