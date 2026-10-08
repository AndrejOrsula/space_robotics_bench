"""Cached YAML loader for the HDRI catalog."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import yaml

from srb.terrain.hdri.schema import SkydomeCatalog

DEFAULT_CATALOG_PATH = Path(__file__).parent / "catalog.yaml"


@lru_cache(maxsize=4)
def load_hdri_catalog(path: Path | None = None) -> SkydomeCatalog:
    p = Path(path) if path is not None else DEFAULT_CATALOG_PATH
    raw = yaml.safe_load(p.read_text())
    return SkydomeCatalog.model_validate(raw)
