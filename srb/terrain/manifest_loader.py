"""Load and cache the curated-site manifest.

The default manifest ships at ``srb/terrain/manifest.yaml``; tests pass an
explicit path. The result is a fully-validated `Manifest` pydantic model.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml
from pydantic import ValidationError

from srb.terrain.errors import ManifestInvalidError, ManifestSiteUnknownError
from srb.terrain.schema import Manifest, SiteEntry

DEFAULT_MANIFEST_PATH: Path = Path(__file__).resolve().parent / "manifest.yaml"

_CACHE: dict[Path, Manifest] = {}


def clear_manifest_cache() -> None:
    """Drop the cached manifest singletons. Tests use this between runs."""
    _CACHE.clear()


def load_manifest(manifest_path: Optional[Path] = None) -> Manifest:
    """Read + validate the manifest YAML. Cached per absolute path."""
    path = (manifest_path or DEFAULT_MANIFEST_PATH).resolve()
    if path in _CACHE:
        return _CACHE[path]

    try:
        raw = yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise ManifestInvalidError(f"YAML parse error in {path}: {e}") from e

    try:
        manifest = Manifest.model_validate(raw)
    except ValidationError as e:
        raise ManifestInvalidError(
            f"manifest at {path} failed schema validation:\n{e}"
        ) from e

    _CACHE[path] = manifest
    return manifest


def get_site(site_key: str, *, manifest_path: Optional[Path] = None) -> SiteEntry:
    """Resolve a site by key. Raises `ManifestSiteUnknownError` if missing."""
    manifest = load_manifest(manifest_path)
    if site_key not in manifest.sites:
        raise ManifestSiteUnknownError(site_key)
    return manifest.sites[site_key]
