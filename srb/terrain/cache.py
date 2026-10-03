"""Cache layout, key derivation, generation publication, and lock helpers."""

from __future__ import annotations

import errno
import hashlib
import inspect
import json
import math
import os
import shutil
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, Iterator, Optional, Sequence, cast

from filelock import FileLock, Timeout

from srb.terrain.catalog_schema import SourceCatalog, SourceLayer, _url_basename
from srb.terrain.errors import CacheLockTimeoutError
from srb.terrain.fusion.provenance import (
    FusionMetadata,
    format_fusion_attribution,
    fusion_attributions,
    validate_fusion_metadata,
)
from srb.terrain.schema import SiteEntry, StackRef
from srb.terrain.spec import TerrainSpec
from srb.utils.digest import sha256_file
from srb.utils.failures import IntegrityFailureError
from srb.utils.path import SRB_ASSETS_DIR_SRB_SCENERY, SRB_IS_SOURCE_CHECKOUT
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    PublicationNotFoundError,
    PublishedGeneration,
)

DEFAULT_CACHE_ROOT_ENV = "SRB_TERRAIN_CACHE_ROOT"


def _is_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _published_file_sha256(
    path: Path,
    *,
    expected_sha256: str | None = None,
) -> str:
    """Read one descriptor-bound file snapshot through publication identity."""
    with ArtifactPublication.file(path).open_snapshot(
        expected_sha256=expected_sha256,
    ) as snapshot:
        if snapshot is None:
            raise PublicationError(f"published file is missing: {path}")
        return snapshot.sha256


META_FILENAME = "meta.json"
ATTRIBUTION_FILENAME = "ATTRIBUTION.txt"
TERRAIN_USD = "terrain.usd"
MATERIAL_USDA = "material.usda"
TEXTURES_DIR = "textures"
TMP_DIRNAME = "_tmp"
RAW_DIRNAME = "_raw"
LOCK_TIMEOUT_S = 300.0
CACHE_BODIES = ("moon", "mars", "earth")
BAKE_REQUIRED_MEMBERS = (
    META_FILENAME,
    TERRAIN_USD,
    MATERIAL_USDA,
    ATTRIBUTION_FILENAME,
    f"{TEXTURES_DIR}/albedo.png",
    f"{TEXTURES_DIR}/roughness.png",
    f"{TEXTURES_DIR}/normal.png",
    f"{TEXTURES_DIR}/ao.png",
)
STACK_IDENTITY_SCHEMA = "srb-terrain-stack/1"
BAKE_IDENTITY_SCHEMA = "srb-terrain-bake/1"
CATALOG_LAYER_BUNDLE_IDENTITY_SCHEMA = "srb-terrain-layer-bundle/1"
CATALOG_LAYER_BUNDLE_DIRNAME = "bundles"
_BAKE_IDENTITY_KEYS = frozenset(
    {
        "schema",
        "cache_key",
        "dem_stack_sha256",
        "color_stack_sha256",
        "fusion_version",
        "target_gsd_m",
        "deshade",
    }
)


def cache_root() -> Path:
    """Return the terrain cache root.

    ``SRB_TERRAIN_CACHE_ROOT`` takes precedence. Source checkouts use
    ``<repo>/assets/srb_assets/scenery/terrain`` through
    ``SRB_ASSETS_DIR_SRB_SCENERY``; installed wheels use
    ``$XDG_CACHE_HOME/srb/terrain`` or ``~/.cache/srb/terrain``.
    """
    env = os.environ.get(DEFAULT_CACHE_ROOT_ENV)
    if env:
        return Path(env)
    if SRB_IS_SOURCE_CHECKOUT:
        return SRB_ASSETS_DIR_SRB_SCENERY / "terrain"
    xdg_cache = Path(os.environ.get("XDG_CACHE_HOME", "~/.cache")).expanduser()
    if not xdg_cache.is_absolute():
        xdg_cache = Path.home() / ".cache"
    return xdg_cache / "srb" / "terrain"


def raw_cache_path(
    sha256: str,
    *,
    root: Path | None = None,
    suffix: str = ".tif",
) -> Path:
    """Return the content-addressed raw-cache path for a source SHA-256."""
    sha = sha256.lower()
    cache_dir = cache_root() if root is None else root
    return cache_dir / RAW_DIRNAME / sha[:2] / f"{sha}{suffix}"


def raw_bundle_cache_path(
    identity: str,
    *,
    root: Path | None = None,
) -> Path:
    """Return the directory for one descriptor-identified label bundle."""
    if not _is_sha256(identity):
        raise ValueError(f"invalid catalog bundle identity: {identity!r}")
    cache_dir = cache_root() if root is None else root
    return cache_dir / RAW_DIRNAME / CATALOG_LAYER_BUNDLE_DIRNAME / identity


def _catalog_layer_acquisition_record(layer: SourceLayer) -> dict[str, object]:
    primary_filename = _url_basename(layer.urls[0])
    files: list[dict[str, object]] = [
        {
            "url": layer.urls[0],
            "filename": primary_filename,
            "size_bytes": layer.primary_size_bytes,
            "sha256": layer.sha256[primary_filename],
        }
    ]
    files.extend(
        {
            "url": sidecar.url,
            "filename": sidecar.filename,
            "size_bytes": sidecar.size_bytes,
            "sha256": sidecar.sha256,
        }
        for sidecar in layer.sidecars
    )
    return {"files": files, "size_bytes": layer.size_bytes}


def catalog_layer_cache_key(layer: SourceLayer) -> str:
    """Return the acquisition identity for a catalog layer.

    Legacy single-file layers retain their primary SHA key. A sidecar bundle
    gets a versioned digest of the ordered primary-plus-sidecar descriptors.
    """
    primary_filename = _url_basename(layer.urls[0])
    if not layer.sidecars:
        return layer.sha256[primary_filename].lower()
    material = json.dumps(
        {
            "schema": CATALOG_LAYER_BUNDLE_IDENTITY_SCHEMA,
            "acquisition": _catalog_layer_acquisition_record(layer),
        },
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(material.encode("utf-8")).hexdigest()


GEOMETRY_VERSION: int = 2
"""Sampling geometry version; bump when grid spacing, extent, or georeference changes.

Version 2 samples half-open at pixel centres, matching declared ``gsd_m`` and
``transform_a``. Version 1 keys retain their original serialization; version 2
invalidates artifacts made with the offset version-1 grid.
"""


@dataclass(frozen=True, slots=True)
class CacheKey:
    body: str
    site: str
    patch: str
    lod: str
    seed: int
    materials_version: int
    dem_sha256: Optional[str]  # None for curated sites
    patch_size_m: Optional[float]
    patch_center_xy_m: Optional[tuple[float, float]]
    use_ortho: bool
    preset_name: Optional[str]
    # Optional fields preserve v1 serialization; hash12 appends set values.
    dem_stack_hash: Optional[str] = None
    color_stack_hash: Optional[str] = None
    fusion_version: Optional[int] = None
    target_gsd_m: Optional[float] = None
    # Effective manifest/request value; false stays absent to preserve old keys.
    deshade: bool = False
    # False stays absent to preserve keys created before this opt-in existed.
    allow_synthetic: bool = False
    # Resolved preset parameters distinguish bakes with different augmentation.
    augment_stamp: bool = False
    augment_stamp_material: str = ""
    # Preset overrides affect generated augmentation data even without stamping.
    augment_preset_set: bool = False
    # Version 1 preserves old keys; newer geometry versions invalidate them.
    geometry_version: int = GEOMETRY_VERSION

    @classmethod
    def from_spec(
        cls,
        spec: TerrainSpec,
        *,
        materials_version: int,
        preset_name: Optional[str] = None,
        manifest_dem_sha256: Optional[str] = None,
        deshade_override: Optional[bool] = None,
        canonicalize_geometry: bool = False,
    ) -> "CacheKey":
        """Build a cache key using the current geometry version.

        ``canonicalize_geometry`` is for named-site v2 identities.
        ``deshade_override`` supplies the effective manifest/request value;
        when omitted, ``spec.deshade`` is used.
        ``allow_synthetic`` comes from the ``RegionRequest`` when present,
        otherwise from the spec. Augmentation key fields use the same
        effective preset resolver as bake and scene code;
        ``augment_preset_set`` records explicit overrides even when stamping
        is disabled because they still affect generated geometry.
        """
        dem_sha = (
            sha256_file(spec.dem_path)
            if spec.dem_path is not None
            else manifest_dem_sha256
        )
        allow_synthetic = (
            spec.region.allow_synthetic
            if spec.region is not None
            else spec.allow_synthetic
        )
        from srb.terrain.augmentation.database import AUGMENT_DB_VERSION
        from srb.terrain.companion_scene import (
            resolve_effective_augment_preset,
            spec_augment_preset,
        )

        augment_preset_name = spec_augment_preset(spec)
        augment_preset_set = augment_preset_name is not None
        # Resolve before cache lookup so invalid overrides cannot hit stale keys.
        augment_preset = resolve_effective_augment_preset(
            augment_preset_name, spec.body
        )
        augment_stamp_material = (
            f"augment_stamp={AUGMENT_DB_VERSION}:{augment_preset.crater_density_per_m2:g}:"
            f"{augment_preset.crater_radius_range_m[0]:g}:"
            f"{augment_preset.crater_radius_range_m[1]:g}"
        )
        return cls(
            body=spec.body,
            site=spec.cache_site,
            patch=spec.patch,
            lod=spec.lod,
            seed=spec.seed,
            materials_version=materials_version,
            dem_sha256=dem_sha,
            patch_size_m=(
                (
                    float(spec.patch_size_m)
                    if canonicalize_geometry
                    else spec.patch_size_m
                )
                if spec.patch_size_m is not None
                else None
            ),
            patch_center_xy_m=(
                (
                    tuple(float(value) for value in spec.patch_center_xy_m)
                    if canonicalize_geometry
                    else spec.patch_center_xy_m
                )
                if spec.patch_center_xy_m is not None
                else None
            ),
            use_ortho=spec.use_ortho,
            preset_name=preset_name,
            deshade=spec.deshade if deshade_override is None else deshade_override,
            allow_synthetic=allow_synthetic,
            augment_stamp=spec.augment_stamp,
            augment_stamp_material=augment_stamp_material,
            augment_preset_set=augment_preset_set,
        )

    def hash12(self) -> str:
        # Keep v1 field order and serialization stable for existing bakes.
        parts: list[str] = [
            self.body,
            self.site,
            self.patch,
            self.lod,
            str(self.seed),
            str(self.materials_version),
            self.dem_sha256 or "",
            str(self.patch_size_m) if self.patch_size_m is not None else "",
            str(self.patch_center_xy_m) if self.patch_center_xy_m is not None else "",
            str(int(self.use_ortho)),
            self.preset_name or "",
        ]
        # Append optional fields only when set, preserving v1 serialization.
        if self.dem_stack_hash is not None:
            parts.append(f"dem_stack_hash={self.dem_stack_hash}")
        if self.color_stack_hash is not None:
            parts.append(f"color_stack_hash={self.color_stack_hash}")
        if self.fusion_version is not None:
            parts.append(f"fusion_version={self.fusion_version}")
        if self.target_gsd_m is not None:
            parts.append(f"target_gsd_m={self.target_gsd_m}")
        if self.deshade:
            parts.append("deshade=1")
        if self.allow_synthetic:
            parts.append("synthetic=1")
        # Overrides affect generated augmentation data even without stamping.
        if self.augment_stamp or self.augment_preset_set:
            parts.append(self.augment_stamp_material)
        # Version 1 remains byte-identical; current geometry invalidates old grids.
        if self.geometry_version != 1:
            parts.append(f"geometry_version={self.geometry_version}")
        material = "|".join(parts)
        return hashlib.sha256(material.encode("utf-8")).hexdigest()[:12]


def named_single_source_key(
    legacy_key: str,
    *,
    patch_size_m: float,
    patch_center: tuple[float, float],
    ortho_sha256: str | None,
    attribution: str,
) -> str:
    """Return v2 identity for a resolved, named single-source bake."""
    record = {
        "schema": "srb-terrain-cache-key/2",
        "legacy_key": legacy_key,
        "patch_size_m": float(patch_size_m),
        "patch_center": [float(value) for value in patch_center],
        "ortho_sha256": ortho_sha256,
        "attribution": attribution,
    }
    material = json.dumps(
        record,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(material.encode("utf-8")).hexdigest()[:12]


def with_named_single_source_identity(
    paths: CachePaths,
    spec: TerrainSpec,
    site_entry: SiteEntry,
    *,
    ortho_path: Path | None = None,
) -> CachePaths:
    """Apply the separately versioned identity for a resolved named v1 site."""
    patch = site_entry.patches.get(spec.patch)
    if patch is None:
        from srb.terrain.errors import ManifestInvalidError

        raise ManifestInvalidError(f"site '{spec.site}' has no patch '{spec.patch}'")
    effective_size = (
        spec.patch_size_m if spec.patch_size_m is not None else patch.size_m
    )
    effective_center = (
        spec.patch_center_xy_m
        if spec.patch_center_xy_m is not None
        else patch.center_xy_m or patch.center_latlon
    )
    assert effective_center is not None
    ortho_sha256 = None
    if spec.use_ortho:
        if ortho_path is not None:
            ortho_sha256 = sha256_file(Path(ortho_path))
        elif site_entry.sources.ortho is not None:
            ortho_sha256 = site_entry.sources.ortho.sha256
    dem_source = site_entry.sources.dem
    if dem_source is None:
        raise ValueError("named single-source identity requires sources.dem")
    key = named_single_source_key(
        paths.key,
        patch_size_m=effective_size,
        patch_center=effective_center,
        ortho_sha256=ortho_sha256,
        attribution=(spec.attribution or dem_source.attribution),
    )
    return CachePaths(root=paths.root, body=paths.body, site=paths.site, key=key)


def stack_hash(refs: Sequence[StackRef], catalog: SourceCatalog) -> str:
    """Hash ordered refs and their validated artifact-affecting layer fields.

    The versioned canonical digest ignores mapping order but changes when ref
    order or any included ref/layer field changes. Provenance fields are
    included; legacy single-file size and download-consent fields are not.
    Sidecar descriptors, including sizes, are included in order. Raises
    ``KeyError`` for a ref missing from the catalog. The schema marker
    invalidates generations created before the current fields were covered.
    """
    layers_by_id = {layer.layer_id: layer for layer in catalog.layers}
    entries: list[dict[str, object]] = []
    for ref in refs:
        layer = layers_by_id[ref.layer_id]
        layer_payload = layer.model_dump(
            mode="json",
            exclude={
                "allow_large_download",
                "size_bytes",
                "primary_size_bytes",
                "sidecars",
            },
        )
        if layer.sidecars:
            layer_payload["acquisition"] = _catalog_layer_acquisition_record(layer)
        entries.append(
            {
                "ref": ref.model_dump(mode="json"),
                "layer": layer_payload,
            }
        )
    material = json.dumps(
        {"schema": STACK_IDENTITY_SCHEMA, "entries": entries},
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(material.encode("utf-8")).hexdigest()


def patch_key(
    spec: TerrainSpec,
    *,
    materials_version: int,
    preset_name: Optional[str] = None,
    manifest_dem_sha256: Optional[str] = None,
    deshade_override: Optional[bool] = None,
    canonicalize_geometry: bool = False,
) -> str:
    return CacheKey.from_spec(
        spec,
        materials_version=materials_version,
        preset_name=preset_name,
        manifest_dem_sha256=manifest_dem_sha256,
        deshade_override=deshade_override,
        canonicalize_geometry=canonicalize_geometry,
    ).hash12()


@dataclass(frozen=True, slots=True)
class CachePaths:
    root: Path
    body: str
    site: str
    key: str
    _generation: PublishedGeneration | None = field(
        default=None, compare=False, repr=False
    )

    @property
    def baked(self) -> Path:
        return self.root / self.body / self.site / self.key

    @property
    def current_generation(self) -> Path:
        """Return one immutable generation, or the cache root on a miss.

        The fallback keeps expected paths useful for diagnostics and cache-miss
        commands. Readiness is decided only by :func:`published_generation`, which
        validates the publication manifest before returning true.
        """
        if self._generation is not None:
            return self._generation.path
        generation = published_generation(self)
        return self.baked if generation is None else generation.path

    def snapshot(self) -> "CachePaths":
        """Bind all member properties to one validated generation snapshot."""
        if self._generation is not None:
            return self
        return replace(self, _generation=require_published_generation(self))

    def with_generation(self, generation: PublishedGeneration) -> "CachePaths":
        """Return paths whose members resolve to ``generation`` only."""
        return replace(self, _generation=generation)

    @property
    def tmp(self) -> Path:
        return self.root / TMP_DIRNAME / self.key

    @property
    def raw(self) -> Path:
        return self.root / RAW_DIRNAME

    @property
    def terrain_usd(self) -> Path:
        return self.current_generation / TERRAIN_USD

    @property
    def material_usda(self) -> Path:
        return self.current_generation / MATERIAL_USDA

    @property
    def textures_dir(self) -> Path:
        return self.current_generation / TEXTURES_DIR

    @property
    def meta_json(self) -> Path:
        return self.current_generation / META_FILENAME

    @property
    def attribution_txt(self) -> Path:
        return self.current_generation / ATTRIBUTION_FILENAME

    @property
    def lock_path(self) -> Path:
        return self.root / f".lock_{self.key}"


@dataclass(frozen=True, slots=True)
class CacheBodyStats:
    body: str
    entries: int
    sites: int


@dataclass(frozen=True, slots=True)
class CacheStats:
    root: Path
    exists: bool
    total_size_bytes: int
    body_stats: tuple[CacheBodyStats, ...]
    stale_tmp_dirs: int


def cache_paths(
    spec: TerrainSpec,
    *,
    materials_version: int,
    preset_name: Optional[str] = None,
    manifest_dem_sha256: Optional[str] = None,
    deshade_override: Optional[bool] = None,
    canonicalize_geometry: bool = False,
) -> CachePaths:
    return CachePaths(
        root=cache_root(),
        body=spec.body,
        site=spec.cache_site,
        key=patch_key(
            spec,
            materials_version=materials_version,
            preset_name=preset_name,
            manifest_dem_sha256=manifest_dem_sha256,
            deshade_override=deshade_override,
            canonicalize_geometry=canonicalize_geometry,
        ),
    )


def cache_paths_from_baked_dir(baked_dir: Path) -> CachePaths:
    """Recover the cache key from a key or published-generation directory.

    Public consumers may hold either ``<key>/`` or
    ``<key>/generations/<id>/`` (an immutable snapshot); both forms resolve
    to the same publication root before metadata updates.
    """
    directory = Path(baked_dir)
    key_dir = (
        directory.parent.parent if directory.parent.name == "generations" else directory
    )
    if len(key_dir.parents) < 3:
        raise ValueError(f"invalid terrain baked directory: {baked_dir}")
    return CachePaths(
        root=key_dir.parents[2],
        body=key_dir.parent.parent.name,
        site=key_dir.parent.name,
        key=key_dir.name,
    )


def published_generation(paths: CachePaths) -> PublishedGeneration | None:
    """Return the validated current terrain generation, if one exists.

    A publication that has never acquired a current generation is an ordinary
    cache miss. Once a current pointer exists, malformed pointers, member
    tampering, and terrain-specific identity mismatches are integrity failures;
    they must not silently trigger a rebuild or optional-runtime fallback.
    """
    try:
        if paths._generation is not None:
            return _validate_published_generation(paths, paths._generation)
        return _open_published_generation(paths)
    except PublicationNotFoundError:
        return None
    except IntegrityFailureError:
        raise
    except (PublicationError, OSError, TypeError, ValueError) as exc:
        raise IntegrityFailureError(
            code="terrain.cache.integrity",
            message=f"terrain cache publication failed validation: {exc}",
            component="terrain.cache",
            details={
                "path": str(paths.baked),
                "cause": type(exc).__name__,
            },
            fallback_allowed=False,
            evidence_valid=False,
        ) from exc


def require_published_generation(paths: CachePaths) -> PublishedGeneration:
    """Return a validated terrain generation or raise a publication error."""
    generation = published_generation(paths)
    if generation is None:
        raise PublicationNotFoundError(
            f"terrain cache has no published generation: {paths.baked}"
        )
    return generation


def _open_published_generation(paths: CachePaths) -> PublishedGeneration:
    # Readers use the canonical generation lock only.  They may safely retain
    # an immutable generation snapshot while a group-locked writer promotes a
    # new pointer or removes the cache key.
    publication = ArtifactPublication.generation(paths.baked)
    generation = cast(
        PublishedGeneration,
        publication.open(required_members=BAKE_REQUIRED_MEMBERS),
    )

    return _validate_published_generation(paths, generation)


def _validate_published_generation(
    paths: CachePaths,
    generation: PublishedGeneration,
) -> PublishedGeneration:
    """Validate terrain-specific metadata for an already locked generation."""

    # Fusion bakes advertise a provenance raster in metadata. It is a
    # required member of that generation, not an optional sidecar: otherwise
    # a metadata-only or mixed v2 generation could look ready after the common
    # v1 members validate.
    meta = _read_generation_json_member(generation, META_FILENAME)
    if not isinstance(meta, dict):
        raise PublicationError("terrain metadata is not an object")
    _validate_augmentation_reference(generation, meta)
    if "fusion" not in meta:
        _validate_illumination_reference(paths, meta)
        return generation
    fusion = meta["fusion"]
    fusion_metadata = validate_fusion_metadata(fusion, source="meta.fusion")
    identity_fusion_version, identity_target_gsd_m = _validate_bake_identity(
        paths, meta.get("bake_identity")
    )
    if identity_fusion_version != fusion_metadata["fusion_version"]:
        raise PublicationError(
            "terrain fusion metadata fusion_version does not match bake_identity"
        )
    if identity_target_gsd_m != fusion_metadata["grid"]["gsd_m"]:
        raise PublicationError(
            "terrain fusion metadata grid.gsd_m does not match "
            "bake_identity target_gsd_m"
        )
    for raster_name in (
        fusion_metadata["provenance_tif"],
        fusion_metadata["uncertainty_tif"],
    ):
        if raster_name is not None and raster_name not in generation.member_hashes:
            raise PublicationError(
                f"terrain generation missing required member: {raster_name}"
            )
    _validate_fusion_attribution(generation, meta, fusion_metadata)
    _validate_illumination_reference(paths, meta)
    return generation


def _read_generation_json_member(
    generation: PublishedGeneration,
    member: str,
) -> object:
    path = generation.path / member
    try:
        return generation.read_member(member, json.loads)
    except PublicationError:
        raise
    except (OSError, TypeError, ValueError, UnicodeError) as exc:
        raise PublicationError(
            f"terrain generation member is invalid JSON: {path}"
        ) from exc


def _read_generation_text_member(
    generation: PublishedGeneration,
    member: str,
) -> str:
    path = generation.path / member

    def decode_utf8(payload: bytes) -> str:
        return payload.decode("utf-8")

    try:
        return generation.read_member(member, decode_utf8)
    except PublicationError:
        raise
    except (OSError, UnicodeError) as exc:
        raise PublicationError(
            f"terrain generation member is invalid UTF-8 text: {path}"
        ) from exc


def _validate_fusion_attribution(
    generation: PublishedGeneration,
    meta: dict[str, Any],
    fusion: FusionMetadata,
) -> None:
    expected_attributions = fusion_attributions(fusion)
    attribution = meta.get("attribution")
    if not isinstance(attribution, list) or not all(
        isinstance(item, str) for item in attribution
    ):
        raise PublicationError("fused terrain attribution metadata is not a list")
    if attribution != expected_attributions:
        raise PublicationError(
            "fused terrain attribution metadata does not match fusion contributors"
        )
    attribution_text = _read_generation_text_member(generation, ATTRIBUTION_FILENAME)
    if attribution_text != format_fusion_attribution(fusion):
        raise PublicationError(
            "fused terrain ATTRIBUTION.txt does not match fusion contributors"
        )


def _validate_augmentation_reference(
    generation: PublishedGeneration,
    meta: dict[str, Any],
) -> None:
    augmentation = meta.get("augmentation_db")
    if augmentation is None:
        return
    if not isinstance(augmentation, dict):
        raise PublicationError("terrain augmentation metadata is not an object")

    from srb.terrain.augmentation.database import AUGMENT_DB_VERSION, SCHEMA_ID

    file_name = augmentation.get("file")
    if file_name != "augmentation.npz":
        raise PublicationError("terrain augmentation metadata has an invalid file")
    if augmentation.get("schema") != SCHEMA_ID:
        raise PublicationError("terrain augmentation metadata has an invalid schema")
    db_version = augmentation.get("db_version")
    if isinstance(db_version, bool) or db_version != AUGMENT_DB_VERSION:
        raise PublicationError("terrain augmentation metadata has an invalid version")
    if file_name not in generation.member_hashes:
        raise PublicationError(
            "terrain generation is missing its declared augmentation database"
        )

    counts = augmentation.get("counts")
    count_names = (
        "craters_requested",
        "craters_kept",
        "rocks_requested",
        "rocks_kept",
    )
    if not isinstance(counts, dict):
        raise PublicationError("terrain augmentation metadata has invalid counts")
    for count_name in count_names:
        count = counts.get(count_name)
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise PublicationError(
                f"terrain augmentation metadata has invalid {count_name}"
            )
    if counts["craters_kept"] > counts["craters_requested"]:
        raise PublicationError("terrain augmentation crater counts are inconsistent")
    if counts["rocks_kept"] > counts["rocks_requested"]:
        raise PublicationError("terrain augmentation rock counts are inconsistent")


def _validate_illumination_reference(
    paths: CachePaths,
    meta: dict[str, Any],
) -> None:
    """Validate an out-of-band illumination product referenced by metadata."""
    illumination = meta.get("illumination")
    if illumination is None:
        return
    if not isinstance(illumination, dict):
        raise PublicationError("terrain illumination metadata is not an object")

    file_name = illumination.get("file")
    if (
        not isinstance(file_name, str)
        or not file_name
        or Path(file_name).name != file_name
        or Path(file_name).is_absolute()
    ):
        raise PublicationError("terrain illumination metadata has an invalid file")

    expected_sha256 = illumination.get("sha256")
    if not _is_sha256(expected_sha256):
        raise PublicationError("terrain illumination metadata has no valid sha256")
    content_sha256 = illumination.get("content_sha256", expected_sha256)
    if not _is_sha256(content_sha256) or content_sha256 != expected_sha256:
        raise PublicationError(
            "terrain illumination metadata content_sha256 does not match sha256"
        )
    publication = illumination.get("publication")
    if not isinstance(publication, dict):
        raise PublicationError(
            "terrain illumination metadata has no publication identity"
        )
    if (
        publication.get("kind") != "immutable-file"
        or publication.get("sha256") != expected_sha256
    ):
        raise PublicationError("terrain illumination publication identity mismatch")

    actual_sha256 = _published_file_sha256(
        paths.baked / file_name,
        expected_sha256=expected_sha256,
    )
    if actual_sha256 != expected_sha256:
        raise PublicationError(
            f"terrain illumination product hash mismatch: {paths.baked / file_name}"
        )


def _validate_bake_identity(paths: CachePaths, value: object) -> tuple[int, float]:
    """Validate the self-identifying input record required by v2 bakes."""
    if not isinstance(value, dict):
        raise PublicationError("fused terrain metadata has no bake_identity")
    if frozenset(value) != _BAKE_IDENTITY_KEYS:
        raise PublicationError("fused terrain bake_identity fields are invalid")
    if value.get("schema") != BAKE_IDENTITY_SCHEMA:
        raise PublicationError("fused terrain bake_identity schema is unsupported")
    if value.get("cache_key") != paths.key:
        raise PublicationError("fused terrain bake_identity cache_key mismatch")
    for field_name in ("dem_stack_sha256", "color_stack_sha256"):
        digest = value.get(field_name)
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(char not in "0123456789abcdef" for char in digest)
        ):
            raise PublicationError(
                f"fused terrain bake_identity has invalid {field_name}"
            )
    fusion_version = value.get("fusion_version")
    if isinstance(fusion_version, bool) or not isinstance(fusion_version, int):
        raise PublicationError("fused terrain bake_identity has invalid fusion_version")
    if fusion_version <= 0:
        raise PublicationError("fused terrain bake_identity has invalid fusion_version")
    target_gsd_m = value.get("target_gsd_m")
    if isinstance(target_gsd_m, bool) or not isinstance(target_gsd_m, (int, float)):
        raise PublicationError("fused terrain bake_identity has invalid target_gsd_m")
    if not math.isfinite(float(target_gsd_m)) or float(target_gsd_m) <= 0.0:
        raise PublicationError("fused terrain bake_identity has invalid target_gsd_m")
    if not isinstance(value.get("deshade"), bool):
        raise PublicationError("fused terrain bake_identity has invalid deshade")
    return fusion_version, float(target_gsd_m)


def publish_baked_generation(
    paths: CachePaths,
    builder: Callable[[Path], object],
    *,
    required_members: Sequence[str] = BAKE_REQUIRED_MEMBERS,
    reuse_if_ready: bool = False,
) -> PublishedGeneration:
    """Publish one complete terrain generation through the shared contract."""
    publication = ArtifactPublication.generation(
        paths.baked,
        lock_path=paths.lock_path,
    )
    if reuse_if_ready:

        def reuse_check(generation: PublishedGeneration) -> bool:
            _validate_published_generation(paths, generation)
            return True

        return publication.publish_or_reuse(
            builder,
            required_members=required_members,
            reuse_if=reuse_check,
        )
    return cast(
        PublishedGeneration,
        publication.publish(builder, required_members=required_members),
    )


def remove_baked_cache(paths: CachePaths, *, lock_held: bool = False) -> bool:
    """Remove one baked cache key, returning whether anything was removed."""
    publication = ArtifactPublication.generation(
        paths.baked,
        lock_path=None if lock_held else paths.lock_path,
    )
    return (
        publication.remove_unpublished_root(
            allowed_members=(
                *BAKE_REQUIRED_MEMBERS,
                "augmentation.npz",
                "provenance.tif",
            ),
        )
        or publication.remove()
    )


def clean_terrain_cache(
    *,
    root: Path | None = None,
    raw: bool,
    baked: bool,
    all_artifacts: bool,
    target: str | None = None,
    bodies: tuple[str, ...] = CACHE_BODIES,
) -> tuple[Path, ...]:
    """Remove requested terrain cache artifacts and return removed paths.

    Baked entries are independent generation publications.  Delete each cache
    key through its publication lock, then remove only empty site/body
    containers.  A concurrent publisher can therefore recreate a container,
    but cleanup never recursively removes a publication it does not own.
    """
    cache_dir = cache_root() if root is None else root
    removed: list[Path] = []

    if raw or all_artifacts:
        raw_dir = cache_dir / RAW_DIRNAME
        with raw_cache_cleanup_lock(cache_dir):
            if _remove_dir(raw_dir):
                removed.append(raw_dir)

    if baked or all_artifacts:
        if target:
            if target in {".", ".."} or "/" in target or "\\" in target:
                raise ValueError(
                    f"terrain cache target must be one site name: {target!r}"
                )
            for body in bodies:
                site_dir = cache_dir / body / target
                if not site_dir.resolve().is_relative_to(cache_dir.resolve()):
                    raise ValueError(
                        f"terrain cache target resolves outside cache root: {target!r}"
                    )
                removed.extend(_clean_baked_site(cache_dir, body, target, site_dir))
        else:
            for body in bodies:
                body_dir = cache_dir / body
                removed.extend(_clean_baked_body(cache_dir, body, body_dir))

    return tuple(removed)


def _clean_baked_site(
    cache_dir: Path,
    body: str,
    site: str,
    site_dir: Path,
) -> tuple[Path, ...]:
    """Delete key publications under one site, never the site recursively."""
    if site_dir.is_symlink() or not site_dir.is_dir():
        return ()

    removed_keys: list[Path] = []
    for key_dir in tuple(sorted(site_dir.iterdir())):
        if key_dir.is_symlink() or not key_dir.is_dir():
            continue
        paths = CachePaths(
            root=cache_dir,
            body=body,
            site=site,
            key=key_dir.name,
        )
        if remove_baked_cache(paths):
            removed_keys.append(key_dir)

    return (site_dir,) if _rmdir_if_empty(site_dir) else tuple(removed_keys)


def _clean_baked_body(
    cache_dir: Path,
    body: str,
    body_dir: Path,
) -> tuple[Path, ...]:
    """Delete site publications under one body, then remove it if empty."""
    if body_dir.is_symlink() or not body_dir.is_dir():
        return ()

    removed_children: list[Path] = []
    for site_dir in tuple(sorted(body_dir.iterdir())):
        if site_dir.is_symlink() or not site_dir.is_dir():
            continue
        removed_children.extend(
            _clean_baked_site(cache_dir, body, site_dir.name, site_dir)
        )

    return (body_dir,) if _rmdir_if_empty(body_dir) else tuple(removed_children)


def _rmdir_if_empty(path: Path) -> bool:
    """Remove exactly one empty container; propagate unexpected I/O errors."""
    try:
        path.rmdir()
    except FileNotFoundError:
        return False
    except OSError as exc:
        if exc.errno in {errno.ENOTEMPTY, errno.EEXIST}:
            return False
        raise
    return True


def inspect_cache(
    root: Path | None = None,
    *,
    bodies: tuple[str, ...] = CACHE_BODIES,
) -> CacheStats:
    """Return size and entry counts for the terrain cache layout."""
    cache_dir = cache_root() if root is None else root
    body_stats: list[CacheBodyStats] = []

    if cache_dir.is_dir():
        total_size_bytes = _dir_size_bytes(cache_dir)
        tmp_root = cache_dir / TMP_DIRNAME
        stale_tmp_dirs = (
            sum(1 for child in tmp_root.iterdir() if child.is_dir())
            if tmp_root.is_dir()
            else 0
        )
    else:
        total_size_bytes = 0
        stale_tmp_dirs = 0

    for body in bodies:
        body_dir = cache_dir / body
        if not body_dir.is_dir():
            body_stats.append(CacheBodyStats(body=body, entries=0, sites=0))
            continue
        sites = tuple(sorted(p for p in body_dir.iterdir() if p.is_dir()))
        entries = sum(1 for site in sites for key in site.iterdir() if key.is_dir())
        body_stats.append(CacheBodyStats(body=body, entries=entries, sites=len(sites)))

    return CacheStats(
        root=cache_dir,
        exists=cache_dir.is_dir(),
        total_size_bytes=total_size_bytes,
        body_stats=tuple(body_stats),
        stale_tmp_dirs=stale_tmp_dirs,
    )


def update_meta(
    paths: CachePaths,
    mutator: Callable[[dict[str, Any]], None],
) -> None:
    """Mutate current metadata while owning the complete sorted lock set."""

    def updater(staging: Path, current: PublishedGeneration) -> None:
        raw = _read_generation_json_member(current, META_FILENAME)
        if not isinstance(raw, dict):
            raise PublicationError("terrain metadata is not an object")
        payload = dict(raw)
        mutator(payload)
        _write_metadata_update(paths, payload, staging, current)

    ArtifactPublication.generation(
        paths.baked,
        lock_path=paths.lock_path,
    ).update(
        updater,
        required_members=BAKE_REQUIRED_MEMBERS,
    )


def read_meta(paths: CachePaths) -> dict[str, Any]:
    generation = require_published_generation(paths)
    meta = _read_generation_json_member(generation, META_FILENAME)
    if not isinstance(meta, dict):
        raise PublicationError("terrain metadata is not an object")
    return meta


def _write_metadata_update(
    paths: CachePaths,
    payload: dict[str, Any],
    staging: Path,
    current: PublishedGeneration,
) -> None:
    """Copy the current validated generation and replace only its metadata."""
    _validate_published_generation(paths, current)
    _validate_illumination_reference(paths, payload)
    _copy_generation_members(current, staging)
    (staging / META_FILENAME).write_text(json.dumps(payload, indent=2, sort_keys=True))


def _copy_generation_members(
    source: PublishedGeneration,
    destination: Path,
) -> None:
    """Copy descriptor snapshots already bound to one generation manifest."""
    for relative in sorted(source.member_hashes):
        target = destination / relative
        source.copy_member_to(relative, target)


def _remove_dir(path: Path, *, ignore_errors: bool = False) -> bool:
    if not path.is_dir():
        return False
    shutil.rmtree(path, ignore_errors=ignore_errors)
    return True


def _dir_size_bytes(path: Path) -> int:
    total = 0
    for child in path.rglob("*"):
        if child.is_file():
            try:
                total += child.stat().st_size
            except OSError:
                pass
    return total


class _LegacyPersistentCacheFileLock(FileLock):
    """Release a POSIX filelock descriptor without unlinking its lock path."""

    def _release(self) -> None:
        import fcntl

        descriptor = cast(int, self._context.lock_file_fd)
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        self._context.lock_file_fd = None
        try:
            os.close(descriptor)
        except OSError:
            pass


_FILELOCK_SUPPORTS_PERSISTENT_PATH = (
    "preserve_lock_file" in inspect.signature(FileLock).parameters
)


def _persistent_cache_file_lock(path: Path) -> FileLock:
    """Construct a lock whose inode survives release across filelock versions."""
    if _FILELOCK_SUPPORTS_PERSISTENT_PATH:
        return FileLock(
            str(path),
            timeout=LOCK_TIMEOUT_S,
            preserve_lock_file=True,
        )
    if os.name == "posix":
        return _LegacyPersistentCacheFileLock(str(path), timeout=LOCK_TIMEOUT_S)
    return FileLock(str(path), timeout=LOCK_TIMEOUT_S)


RAW_LOCK_FILENAME = "_raw.lock"
RAW_ACQUISITION_LOCK_DIRNAME = "_raw_locks"


def raw_cache_lock_path(root: Path | None = None) -> Path:
    """Lock file guarding the shared raw source cache (``<root>/_raw``)."""
    cache_dir = cache_root() if root is None else root
    return cache_dir / RAW_LOCK_FILENAME


@contextmanager
def raw_cache_lock(root: Path | None = None) -> Iterator[None]:
    """Hold the legacy exclusive raw-cache lock for raw-cache operations.

    Keep this serialized API for raw-cache users that do not have a
    content-addressed acquisition key. Acquisition and verification use
    ``raw_cache_acquisition_lock`` instead.
    """
    cache_dir = cache_root() if root is None else root
    cache_dir.mkdir(parents=True, exist_ok=True)
    lock = _persistent_cache_file_lock(raw_cache_lock_path(cache_dir))
    try:
        with lock:
            yield
    except Timeout as e:
        raise CacheLockTimeoutError(
            f"timed out waiting for raw cache lock on {raw_cache_lock_path(cache_dir)}"
        ) from e


@contextmanager
def raw_cache_cleanup_lock(root: Path | None = None) -> Iterator[None]:
    """Exclude raw acquisition and verification while removing ``_raw``."""
    cache_dir = cache_root() if root is None else root
    cache_dir.mkdir(parents=True, exist_ok=True)
    with _raw_cache_gate(cache_dir, exclusive=True):
        yield


@contextmanager
def raw_cache_acquisition_lock(
    root: Path | None,
    artifact_sha256: str,
) -> Iterator[None]:
    """Protect one raw artifact under cleanup admission, then its own lock.

    The lock path is outside ``_raw`` and persists after raw cleanup, so no
    waiter can hold an unlinked inode while a later caller locks a replacement.
    Acquisition waits have no fixed timeout; ArtifactPublication retains its
    independent finite publication timeout.
    """
    digest = artifact_sha256.lower()
    if not _is_sha256(digest):
        raise ValueError("raw acquisition lock requires a SHA-256 digest")
    cache_dir = cache_root() if root is None else root
    cache_dir.mkdir(parents=True, exist_ok=True)
    with _raw_cache_gate(cache_dir, exclusive=os.name != "posix"):
        lock_dir = cache_dir / RAW_ACQUISITION_LOCK_DIRNAME
        lock_dir.mkdir(parents=True, exist_ok=True)
        lock = _persistent_acquisition_file_lock(lock_dir / f"{digest}.lock")
        with lock:
            yield


@contextmanager
def _raw_cache_gate(cache_dir: Path, *, exclusive: bool) -> Iterator[None]:
    lock_path = raw_cache_lock_path(cache_dir)
    if os.name == "posix":
        import fcntl

        descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(
                descriptor,
                fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH,
            )
            yield
        finally:
            try:
                fcntl.flock(descriptor, fcntl.LOCK_UN)
            finally:
                os.close(descriptor)
        return

    # Platforms without shared flock support keep serialization, but keyed
    # acquisition and cleanup still wait without a fixed timeout.
    lock = _persistent_acquisition_file_lock(lock_path)
    with lock:
        yield


def _persistent_acquisition_file_lock(path: Path) -> FileLock:
    """Create an inode-stable artifact lock with an interruptible unbounded wait."""
    if _FILELOCK_SUPPORTS_PERSISTENT_PATH:
        return FileLock(str(path), timeout=-1, preserve_lock_file=True)
    if os.name == "posix":
        return _LegacyPersistentCacheFileLock(str(path), timeout=-1)
    return FileLock(str(path), timeout=-1)
