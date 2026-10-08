"""Pipeline: catalog skydome -> texture file + dome.usd on disk."""

from __future__ import annotations

from pathlib import Path

from srb.terrain.hdri.catalog_loader import load_hdri_catalog
from srb.terrain.hdri.fetch import fetch_and_resize
from srb.terrain.hdri.usd_writer import write_dome_usd
from srb.utils.path import (
    SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES,
    SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES,
)

# Cutoff for placement under low_res vs high_res. Mirrors the existing
# convention: anything <= 1024 wide goes to low_res; larger -> high_res.
_HIGH_RES_THRESHOLD = 1024


def _bake_root(resize_to: int) -> Path:
    return (
        SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES
        if resize_to > _HIGH_RES_THRESHOLD
        else SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES
    )


def bake_skydome(name: str) -> Path:
    """Bake the named skydome to ``<low|high>_res/<body>/<name>/dome.usd``.

    Idempotent: re-running is a no-op while ``dome.usd``, the texture and the
    recorded catalog entry are unchanged.
    """
    catalog = load_hdri_catalog()
    if name not in catalog.skydomes:
        raise KeyError(
            f"unknown skydome {name!r}; available: {sorted(catalog.skydomes)}"
        )
    entry = catalog.skydomes[name]

    out_dir = _bake_root(entry.resize_to) / entry.body / name
    out_usd = out_dir / "dome.usd"
    tex_path = out_dir / f"skydome.{entry.format}"

    # Rebake when the catalog entry changed. Bakes without the sidecar predate
    # it and are reused as-is, so read-only or offline caches keep working.
    identity_path = out_dir / "catalog_entry.json"
    identity = entry.model_dump_json()
    if out_usd.is_file() and tex_path.is_file():
        try:
            recorded = identity_path.read_text(encoding="utf-8")
        except FileNotFoundError:
            recorded = identity
        if recorded == identity:
            return out_usd

    out_dir.mkdir(parents=True, exist_ok=True)
    fetch_and_resize(entry, tex_path)
    write_dome_usd(out_usd, texture_path=tex_path, intensity=entry.intensity)
    identity_path.write_text(identity, encoding="utf-8")
    return out_usd
