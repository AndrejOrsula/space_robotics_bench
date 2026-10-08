"""Cached YAML loader for the rocks catalog."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import yaml

from srb.terrain.rocks.schema import RocksCatalog

DEFAULT_CATALOG_PATH = Path(__file__).parent / "catalog.yaml"


@lru_cache(maxsize=4)
def load_rocks_catalog(path: Path | None = None) -> RocksCatalog:
    p = Path(path) if path is not None else DEFAULT_CATALOG_PATH
    raw = yaml.safe_load(p.read_text())
    return RocksCatalog.model_validate(raw)
