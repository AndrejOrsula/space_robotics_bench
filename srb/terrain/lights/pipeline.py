"""Pipeline: catalog preset → DistantLight USD on disk."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import cast

from srb.terrain.lights import ephemeris_bridge
from srb.terrain.lights.catalog_loader import load_lighting_catalog
from srb.terrain.lights.schema import LightingPreset
from srb.terrain.lights.usd_writer import write_lights_usd
from srb.utils.path import SRB_ASSETS_DIR_SRB_LIGHTS
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    PublishedGeneration,
)

_STATIC_LIGHTING_IDENTITY_SCHEMA = "srb-static-lighting/1"
_STATIC_LIGHTING_MEMBERS = ("lights.usd", "identity.json")
_LEAF_HASH_CHARS = 16


def _canonical_json(value: object) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _static_lighting_identity(
    preset_name: str, preset: LightingPreset
) -> dict[str, object]:
    """Identity of one static bake: writer version, preset name, preset fields.

    ``writer_version`` is read at call time from
    ``ephemeris_bridge.LIGHTS_WRITER_VERSION`` (see there for when to bump it).
    """
    core: dict[str, object] = {
        "schema": _STATIC_LIGHTING_IDENTITY_SCHEMA,
        "writer_version": ephemeris_bridge.LIGHTS_WRITER_VERSION,
        "preset_name": preset_name,
        "preset": preset.model_dump(mode="json"),
    }
    return {**core, "sha256": hashlib.sha256(_canonical_json(core)).hexdigest()}


def bake_lighting(preset_name: str) -> Path:
    """Return ``lights.usd`` for the catalog preset, baking it on a miss.

    The cache root is ``<lights_root>/<preset_name>/<content hash>/``, keyed by
    the preset's full content, so a catalog edit selects a fresh slot and is
    baked instead of reusing the file baked for the old values. Each root is
    an immutable generation publication (``lights.usd`` + ``identity.json``):
    identical content reuses the published file without rewriting it, and
    concurrent first use, stale staging and an identity mismatch are handled
    under the publication lock.
    """
    catalog = load_lighting_catalog()
    if preset_name not in catalog.presets:
        raise KeyError(
            f"unknown lighting preset {preset_name!r}; available: "
            f"{sorted(catalog.presets)}"
        )
    preset = catalog.presets[preset_name]
    identity = _static_lighting_identity(preset_name, preset)
    root = (
        SRB_ASSETS_DIR_SRB_LIGHTS
        / preset_name
        / str(identity["sha256"])[:_LEAF_HASH_CHARS]
    )
    publication = ArtifactPublication.generation(root)

    def matches(generation: PublishedGeneration) -> bool:
        try:
            stored = generation.read_member("identity.json", json.loads)
        except (PublicationError, OSError, ValueError):
            return False
        return bool(stored == identity)

    try:
        generation: PublishedGeneration | None = cast(
            PublishedGeneration,
            publication.open(required_members=_STATIC_LIGHTING_MEMBERS),
        )
    except (PublicationError, OSError):
        generation = None
    if generation is None or not matches(generation):

        def build(staging: Path) -> None:
            write_lights_usd(staging / "lights.usd", preset=preset)
            (staging / "identity.json").write_bytes(_canonical_json(identity))

        generation = publication.publish_or_reuse(
            build,
            required_members=_STATIC_LIGHTING_MEMBERS,
            reuse_if=matches,
            repair_invalid=True,
        )
    return generation.path / "lights.usd"
