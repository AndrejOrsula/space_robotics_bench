"""Load and cache per-body terrain catalogs from YAML."""

from __future__ import annotations

import os
from copy import deepcopy
from functools import lru_cache
from pathlib import Path

import yaml

from srb.terrain.catalog_schema import SourceCatalog, SourceLayer
from srb.terrain.errors import CatalogMissingError
from srb.terrain.schema import Body

CATALOG_DIR_ENV = "SRB_TERRAIN_CATALOG_DIR"
DEFAULT_CATALOG_DIR = Path(__file__).parent


def catalog_dir() -> Path:
    """Catalog directory, honoring `SRB_TERRAIN_CATALOG_DIR` (mirrors
    `cache.cache_root()`, `cache.py`)."""
    env = os.environ.get(CATALOG_DIR_ENV)
    return Path(env) if env else DEFAULT_CATALOG_DIR


def catalog_path(body: Body) -> Path:
    """Return `<catalog_dir()>/<body>.yaml`."""
    return catalog_dir() / f"{body}.yaml"


def load_source_catalog(body: Body = "moon", path: Path | None = None) -> SourceCatalog:
    """Load a validated catalog, returning an independent model instance."""
    resolved = Path(path if path is not None else catalog_path(body)).resolve()
    return deepcopy(_load_source_catalog_cached(body, resolved))


@lru_cache(maxsize=4)
def _load_source_catalog_cached(
    body: Body = "moon", path: Path | None = None
) -> SourceCatalog:
    """Load and validate a catalog, cached by `(body, path)`.

    Missing files raise `CatalogMissingError`; schema validation errors
    propagate. Empty `layers` is valid. Clear the cache after changing the
    catalog directory or a previously loaded file."""
    resolved = Path(path if path is not None else catalog_path(body)).resolve()
    if not resolved.is_file():
        raise CatalogMissingError(
            f"no source catalog for body {body!r}: {resolved} does not exist; "
            f"pass an existing catalog path or set {CATALOG_DIR_ENV} to its directory"
        )
    raw = yaml.safe_load(resolved.read_text())
    return SourceCatalog.model_validate(raw)


def clear_source_catalog_cache() -> None:
    """Invalidate cached catalogs, including for test isolation."""
    _load_source_catalog_cached.cache_clear()


def layers_by_id(catalog: SourceCatalog) -> dict[str, SourceLayer]:
    """Index a catalog's layers by `layer_id`."""
    return {layer.layer_id: layer for layer in catalog.layers}
