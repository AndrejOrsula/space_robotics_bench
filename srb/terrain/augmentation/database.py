"""Deterministic crater and rock placements (schema ``srb_augdb/1``).

Placements derive from patch data, preset, seed, and slug. SHA-256 seeding,
fixed draw-ahead resampling, and sorted metadata serialization keep generation
deterministic. The compressed NPZ is published with the terrain generation.
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import NoReturn, Optional

import numpy as np

from srb.terrain.augmentation.presets import AugmentPreset
from srb.utils.failures import IntegrityFailureError
from srb.utils.publication import (
    ArtifactPublication,
    PublicationError,
    PublishedGeneration,
)
from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_AUGMENTATION_ROWS,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
    ResourceLimitError,
    checked_shape_nbytes,
    inspect_npz_archive,
    require_bounded_int,
)

AUGMENT_DB_VERSION = 4
SCHEMA_ID = "srb_augdb/1"

# Isolate this RNG stream; changing the salt changes placements. Treat it as
# cache-format state and bump AUGMENT_DB_VERSION instead.
_AUGMENT_STREAM_SALT = 761

# Draw this multiple of candidates once, then keep the first valid rows.
# Retrying until full would make RNG consumption depend on acceptance rate.
_OVERSAMPLE = 4

# Number of SimForge variants indexed by each row's `asset_idx`.
_ROCK_ASSET_SLOTS = 8

__all__ = [
    "AUGMENT_DB_VERSION",
    "SCHEMA_ID",
    "AugmentationDb",
    "generate_augmentation_db",
    "sample_height_m",
    "save_augmentation_db",
    "load_augmentation_db_file",
    "load_augmentation_db",
]


@dataclass(frozen=True)
class AugmentationDb:
    """Versioned synthetic placements generated deterministically from a seed.

    ``craters`` is float32 ``(Nc, 4)``: ``[x_m, y_m, radius_m, depth_m]``.
    ``rocks`` is float32 ``(Nr, 6)``: ``[x_m, y_m, z_m, scale, yaw_rad,
    asset_idx]``. XY use patch-local mesh coordinates (east, north), centered
    on the patch. Rock ``z_m`` uses the baked mesh's centered Z datum, so
    coordinates land directly on its surface; without DEM and GSD it is zero.
    Empty arrays retain shapes ``(0, 4)`` and ``(0, 6)``. V1 files are upgraded
    on read by inserting a zero ``z_m`` column and recording
    ``meta["upgraded_from"] = 1``; unsupported stale versions are rejected.
    """

    craters: np.ndarray
    rocks: np.ndarray
    meta: dict

    def validate(self) -> None:
        if self.meta.get("schema") != SCHEMA_ID:
            raise ValueError(
                f"AugmentationDb schema mismatch: {self.meta.get('schema')!r} "
                f"!= {SCHEMA_ID!r}"
            )
        for name, arr, cols in (("craters", self.craters, 4), ("rocks", self.rocks, 6)):
            if arr.dtype != np.float32 or arr.ndim != 2 or arr.shape[1] != cols:
                raise ValueError(
                    f"{name} must be float32 (N, {cols}), got dtype={arr.dtype} "
                    f"shape={arr.shape}"
                )
            if not np.all(np.isfinite(arr)):
                raise ValueError(f"{name} contains non-finite values")


def _slug_digest_ints(slug: str) -> list[int]:
    """Split the full SHA-256 slug digest into 32-bit big-endian integers."""
    digest = hashlib.sha256(slug.encode("utf-8")).digest()
    return [int.from_bytes(digest[i : i + 4], "big") for i in range(0, len(digest), 4)]


def _rng_for(seed: int, slug: str) -> np.random.Generator:
    return np.random.default_rng(
        [_AUGMENT_STREAM_SALT, AUGMENT_DB_VERSION, int(seed), *_slug_digest_ints(slug)]
    )


def sample_height_m(
    dem: np.ndarray, gsd_m: float, x_m: np.ndarray, y_m: np.ndarray
) -> np.ndarray:
    """Bilinear DEM height at centered mesh coordinates (x east, y north).

    Mesh coordinates map by ``x=(col-(w-1)/2)*gsd`` and
    ``y=((h-1)/2-row)*gsd``.
    """
    arr = np.asarray(dem, dtype=np.float64)
    h, w = arr.shape
    col = np.asarray(x_m, dtype=np.float64) / gsd_m + 0.5 * (w - 1)
    row = 0.5 * (h - 1) - np.asarray(y_m, dtype=np.float64) / gsd_m
    col = np.clip(col, 0.0, w - 1.0)
    row = np.clip(row, 0.0, h - 1.0)
    c0 = np.floor(col).astype(np.int64)
    r0 = np.floor(row).astype(np.int64)
    c1 = np.minimum(c0 + 1, w - 1)
    r1 = np.minimum(r0 + 1, h - 1)
    fc = col - c0
    fr = row - r0
    top = arr[r0, c0] * (1.0 - fc) + arr[r0, c1] * fc
    bot = arr[r1, c0] * (1.0 - fc) + arr[r1, c1] * fc
    return (top * (1.0 - fr) + bot * fr).astype(np.float32)


def _slope_deg_grid(dem: np.ndarray, gsd_m: Optional[float]) -> np.ndarray:
    """Return a deterministic slope proxy in degrees; unknown spacing uses 1 m."""
    spacing = float(gsd_m) if gsd_m is not None else 1.0
    d_row, d_col = np.gradient(np.asarray(dem, dtype=np.float64), spacing)
    return np.degrees(np.arctan(np.hypot(d_row, d_col)))


def _nearest_pixel_slope(
    slope: np.ndarray,
    x_m: np.ndarray,
    y_m: np.ndarray,
    gsd_m: Optional[float],
) -> np.ndarray:
    """Nearest-pixel slope lookup using centered DEM mesh coordinates."""
    h, w = slope.shape
    spacing = float(gsd_m) if gsd_m is not None else 1.0
    col = np.clip(np.round(x_m / spacing + 0.5 * (w - 1)).astype(np.int64), 0, w - 1)
    row = np.clip(np.round(0.5 * (h - 1) - y_m / spacing).astype(np.int64), 0, h - 1)
    return slope[row, col]


def _draw_positions(
    rng: np.random.Generator,
    count: int,
    patch_size_m: float,
    slope: Optional[np.ndarray],
    max_slope_deg: float,
    gsd_m: Optional[float],
) -> np.ndarray:
    """Draw up to ``count`` positions; slope filtering uses fixed draw-ahead.

    A single ``count * _OVERSAMPLE`` batch keeps RNG consumption independent
    of the acceptance rate.
    """
    candidate_count = count if slope is None else count * _OVERSAMPLE
    checked_shape_nbytes(
        (candidate_count, 2),
        np.dtype(np.float64),
        resource="terrain augmentation candidate positions",
    )
    half = patch_size_m / 2.0
    if slope is None:
        return rng.uniform(-half, half, size=(count, 2))
    candidates = rng.uniform(-half, half, size=(count * _OVERSAMPLE, 2))
    slopes = _nearest_pixel_slope(slope, candidates[:, 0], candidates[:, 1], gsd_m)
    valid = candidates[slopes <= max_slope_deg]
    return valid[:count]


def _place_craters(
    rng: np.random.Generator,
    preset: AugmentPreset,
    patch_size_m: float,
    requested: int,
    slope: Optional[np.ndarray],
    max_slope_deg: float,
    gsd_m: Optional[float],
) -> tuple[np.ndarray, int, int]:
    if requested <= 0:
        return np.zeros((0, 4), dtype=np.float32), requested, 0
    xy = _draw_positions(rng, requested, patch_size_m, slope, max_slope_deg, gsd_m)
    kept = xy.shape[0]
    radius = rng.uniform(
        preset.crater_radius_range_m[0], preset.crater_radius_range_m[1], size=kept
    )
    depth = 0.2 * radius
    arr = np.column_stack([xy[:, 0], xy[:, 1], radius, depth]).astype(np.float32)
    return arr, requested, kept


def _place_rocks(
    rng: np.random.Generator,
    preset: AugmentPreset,
    patch_size_m: float,
    requested: int,
    slope: Optional[np.ndarray],
    max_slope_deg: float,
    gsd_m: Optional[float],
) -> tuple[np.ndarray, int, int]:
    if requested <= 0:
        return np.zeros((0, 5), dtype=np.float32), requested, 0
    xy = _draw_positions(rng, requested, patch_size_m, slope, max_slope_deg, gsd_m)
    kept = xy.shape[0]
    scale = rng.uniform(
        preset.rock_size_range_m[0], preset.rock_size_range_m[1], size=kept
    )
    yaw = rng.uniform(0.0, 2.0 * np.pi, size=kept)
    asset_idx = rng.integers(0, _ROCK_ASSET_SLOTS, size=kept)
    arr = np.column_stack([xy[:, 0], xy[:, 1], scale, yaw, asset_idx]).astype(
        np.float32
    )
    return arr, requested, kept


def generate_augmentation_db(
    *,
    slug: str,
    body: str,
    seed: int,
    patch_size_m: float,
    gsd_m: Optional[float],
    preset: AugmentPreset,
    dem: Optional[np.ndarray] = None,
    max_slope_deg: float = 25.0,
    mesh_z_origin_m: Optional[float] = None,
) -> AugmentationDb:
    """Generate a deterministic augmentation database.

    Identical inputs produce byte-identical arrays, independent of process
    hash randomization and global NumPy RNG state.

    ``dem``, when given, is used both for a slope-exclusion proxy and
    (with ``gsd_m``) to bilinearly sample rock ``z_m``.

    ``mesh_z_origin_m`` is the centered mesh origin subtracted from sampled
    heights, placing rocks on the mesh surface rather than the raw DEM datum.
    It defaults to zero.
    """
    craters_requested = require_bounded_int(
        int(round(preset.crater_density_per_m2 * patch_size_m**2)),
        resource="terrain augmentation crater rows",
        maximum=MAX_AUGMENTATION_ROWS,
    )
    rocks_requested = require_bounded_int(
        int(round(preset.rock_density_per_m2 * patch_size_m**2)),
        resource="terrain augmentation rock rows",
        maximum=MAX_AUGMENTATION_ROWS,
    )
    rng = _rng_for(seed, slug)

    slope: Optional[np.ndarray] = None
    dem_stats: Optional[dict] = None
    dem_arr: Optional[np.ndarray] = None
    if dem is not None:
        dem_arr = np.asarray(dem)
        checked_shape_nbytes(
            dem_arr.shape,
            dem_arr.dtype,
            resource="terrain augmentation DEM",
        )
        if gsd_m is not None:
            # Use the inscribed square extent: placements sample a centered
            # square, and checking only row count rejects tall non-square DEMs.
            implied_size_m = min(dem_arr.shape[0], dem_arr.shape[1]) * float(gsd_m)
            if abs(implied_size_m - patch_size_m) > 0.5 * patch_size_m:
                raise ValueError(
                    f"dem shape {dem_arr.shape} with gsd_m={gsd_m} implies a "
                    f"{implied_size_m:.3f} m patch, which differs from "
                    f"patch_size_m={patch_size_m} by more than 50%"
                )
        if not np.isfinite(dem_arr).all():
            raise ValueError(
                "terrain augmentation DEM must contain only finite heights"
            )
        slope = _slope_deg_grid(dem_arr, gsd_m)
        dem_stats = {
            "shape": [int(s) for s in dem_arr.shape],
            "min_m": float(np.min(dem_arr)),
            "max_m": float(np.max(dem_arr)),
        }

    craters, craters_requested, craters_kept = _place_craters(
        rng, preset, patch_size_m, craters_requested, slope, max_slope_deg, gsd_m
    )
    rocks, rocks_requested, rocks_kept = _place_rocks(
        rng, preset, patch_size_m, rocks_requested, slope, max_slope_deg, gsd_m
    )

    # Derive z after placement so adding this column consumes no RNG draws.
    # Subtract the mesh origin to put rocks on the centered mesh datum.
    _z_origin = 0.0 if mesh_z_origin_m is None else float(mesh_z_origin_m)
    if dem_arr is not None and gsd_m is not None:
        z = sample_height_m(dem_arr, float(gsd_m), rocks[:, 0], rocks[:, 1])
        if _z_origin != 0.0:
            z = (z.astype(np.float64) - _z_origin).astype(np.float32)
    else:
        z = np.zeros(rocks.shape[0], dtype=np.float32)
    rocks = np.column_stack(
        [rocks[:, 0], rocks[:, 1], z, rocks[:, 2], rocks[:, 3], rocks[:, 4]]
    ).astype(np.float32)

    meta = {
        "schema": SCHEMA_ID,
        "db_version": AUGMENT_DB_VERSION,
        "seed": int(seed),
        "slug": str(slug),
        "body": str(body),
        "preset": str(preset.body),
        "counts": {
            "craters_requested": craters_requested,
            "craters_kept": craters_kept,
            "rocks_requested": rocks_requested,
            "rocks_kept": rocks_kept,
        },
        "provenance": "synthetic",
        "generated_from": {
            "patch_size_m": float(patch_size_m),
            "gsd_m": None if gsd_m is None else float(gsd_m),
            "max_slope_deg": float(max_slope_deg),
            "dem_stats": dem_stats,
            "mesh_z_origin_m": _z_origin,
        },
    }
    return AugmentationDb(craters=craters, rocks=rocks, meta=meta)


def save_augmentation_db(db: AugmentationDb, path) -> Path:
    """Write ``db`` as one validated, atomically published npz artifact."""
    db.validate()
    require_bounded_int(
        db.craters.shape[0],
        resource="terrain augmentation crater rows",
        maximum=MAX_AUGMENTATION_ROWS,
    )
    require_bounded_int(
        db.rocks.shape[0],
        resource="terrain augmentation rock rows",
        maximum=MAX_AUGMENTATION_ROWS,
    )
    checked_shape_nbytes(
        db.craters.shape,
        db.craters.dtype,
        resource="terrain augmentation crater database",
    )
    checked_shape_nbytes(
        db.rocks.shape,
        db.rocks.dtype,
        resource="terrain augmentation rock database",
    )
    path = Path(path)
    meta_json = json.dumps(db.meta, sort_keys=True)
    meta_bytes = len(meta_json.encode("utf-8"))
    if meta_bytes > MAX_METADATA_ATTRIBUTE_BYTES:
        raise ResourceLimitError(
            "terrain augmentation metadata",
            limit=MAX_METADATA_ATTRIBUTE_BYTES,
            observed=meta_bytes,
        )
    arrays = {
        "craters": db.craters,
        "rocks": db.rocks,
        "meta": np.str_(meta_json),
    }

    def writer(staging: Path) -> None:
        # NumPy appends ``.npz``; keep and remove its scratch archive beside
        # the stage so it cannot be mistaken for the final cache path.
        archive_staging = staging.with_name(f"{staging.name}.npz")
        try:
            np.savez_compressed(archive_staging, **arrays)
            shutil.copyfile(archive_staging, staging)
        finally:
            archive_staging.unlink(missing_ok=True)

    return ArtifactPublication.file(path).publish(writer)


def _raise_augmentation_integrity(
    path: Path,
    reason: str,
    cause: BaseException | None = None,
) -> NoReturn:
    details: dict[str, object] = {"path": str(path), "reason": reason}
    if cause is not None:
        details["cause_type"] = f"{type(cause).__module__}.{type(cause).__qualname__}"
    error = IntegrityFailureError(
        code="terrain.augmentation.publication_integrity",
        message=f"invalid terrain augmentation publication at {path}: {reason}",
        component="terrain.augmentation",
        details=details,
        fallback_allowed=False,
        evidence_valid=False,
    )
    if cause is None:
        raise error
    raise error from cause


def load_augmentation_db(baked_dir) -> Optional[AugmentationDb]:
    """Read the augmentation sidecar from a baked terrain directory.

    Return ``None`` when legacy metadata or its augmentation field is absent.
    Once publication or augmentation metadata exists, malformed pointers,
    hashes, metadata, or payloads raise :class:`IntegrityFailureError` so
    corrupt data cannot be mistaken for optional absence.
    """
    baked_dir = Path(baked_dir)
    generation_hashes: dict[str, str] | None = None
    published_generation: PublishedGeneration | None = None
    current_path = baked_dir / "current.json"
    has_generation_state = os.path.lexists(current_path) or os.path.lexists(
        baked_dir / "generations"
    )
    generation_root: Path | None = None
    expected_generation_dir: Path | None = None
    if baked_dir.parent.name == "generations":
        generation_root = baked_dir.parent.parent
        expected_generation_dir = baked_dir.absolute()
    elif has_generation_state:
        generation_root = baked_dir

    if generation_root is not None:
        try:
            opened_generation = ArtifactPublication.generation(generation_root).open(
                required_members=("meta.json",)
            )
        except (OSError, PublicationError, TypeError, ValueError) as exc:
            _raise_augmentation_integrity(
                generation_root, "generation publication validation failed", exc
            )
        if not isinstance(opened_generation, PublishedGeneration):
            _raise_augmentation_integrity(
                generation_root, "publication did not return a generation"
            )
        published_generation = opened_generation
        if (
            expected_generation_dir is not None
            and published_generation.path != expected_generation_dir
        ):
            _raise_augmentation_integrity(
                baked_dir, "terrain directory is not the current generation"
            )
        baked_dir = published_generation.path
        generation_hashes = dict(published_generation.member_hashes)

    meta_path = baked_dir / "meta.json"
    try:
        if published_generation is not None:
            meta = published_generation.read_member("meta.json", json.loads)
        else:
            meta = ArtifactPublication.file(meta_path).read(json.loads)
            if meta is None:
                return None
    except IntegrityFailureError:
        raise
    except (OSError, PublicationError, TypeError, ValueError, UnicodeError) as exc:
        _raise_augmentation_integrity(meta_path, "metadata read failed", exc)
    if not isinstance(meta, dict):
        _raise_augmentation_integrity(meta_path, "metadata root is not an object")
    if "augmentation_db" not in meta:
        return None
    block = meta["augmentation_db"]
    if not isinstance(block, dict):
        _raise_augmentation_integrity(
            meta_path, "augmentation_db metadata is not an object"
        )
    file_value = block.get("file")
    if not isinstance(file_value, str) or not file_value:
        _raise_augmentation_integrity(
            meta_path, "augmentation_db.file is not a non-empty string"
        )
    member = Path(file_value)
    if member.is_absolute() or member == Path(".") or ".." in member.parts:
        _raise_augmentation_integrity(
            meta_path, "augmentation_db.file escapes the terrain generation"
        )
    db_path = baked_dir / member
    expected_sha256: str | None = None
    if generation_hashes is not None:
        expected_sha256 = generation_hashes.get(member.as_posix())
        if expected_sha256 is None:
            _raise_augmentation_integrity(
                db_path, "referenced database is absent from the generation manifest"
            )
    try:
        return load_augmentation_db_file(
            db_path,
            expected_sha256=expected_sha256,
            _generation=published_generation,
        )
    except IntegrityFailureError:
        raise
    except Exception as exc:
        _raise_augmentation_integrity(db_path, "database validation failed", exc)


def load_augmentation_db_file(
    path,
    *,
    expected_sha256: str | None = None,
    _generation: PublishedGeneration | None = None,
) -> AugmentationDb:
    """Read and validate an augmentation NPZ, upgrading v1 rocks in memory.

    V1 lacks ``z_m`` and is upgraded with a zero column at index 2, recording
    ``meta["upgraded_from"] = 1``. Other stale versions are rejected because
    their rock heights may use an obsolete datum and cannot be recomputed
    without the original bake DEM.
    """
    path = Path(path)

    def parse(payload: bytes) -> AugmentationDb:
        # ArtifactPublication.read supplies one descriptor-verified byte
        # snapshot. Parsing that snapshot through BytesIO prevents a writer
        # from changing an npz member between metadata and array reads.
        source = io.BytesIO(payload)
        declarations = inspect_npz_archive(
            source,
            resource="terrain augmentation database",
            max_archive_bytes=MAX_IN_MEMORY_FILE_BYTES,
            max_decoded_bytes=MAX_ARRAY_BYTES,
        )
        required = {"craters", "rocks", "meta"}
        if "meta" not in declarations:
            raise ValueError(f"{path}: augmentation database has no meta key")
        if set(declarations) != required:
            raise ValueError(
                f"{path}: augmentation database keys must be {sorted(required)}, "
                f"got {sorted(declarations)}"
            )
        for name in ("craters", "rocks"):
            shape = declarations[name].shape
            row_count = shape[0] if shape else 0
            if row_count > MAX_AUGMENTATION_ROWS:
                raise ResourceLimitError(
                    f"terrain augmentation {name} rows",
                    limit=MAX_AUGMENTATION_ROWS,
                    observed=row_count,
                )
        meta_bytes = declarations["meta"].decoded_bytes
        if meta_bytes > MAX_METADATA_ATTRIBUTE_BYTES:
            raise ResourceLimitError(
                "terrain augmentation metadata",
                limit=MAX_METADATA_ATTRIBUTE_BYTES,
                observed=meta_bytes,
            )
        source.seek(0)
        with np.load(source, allow_pickle=False) as data:
            if "meta" not in data:
                raise ValueError(f"{path}: not an srb_augdb file (no meta key)")
            meta = json.loads(str(data["meta"]))
            if not isinstance(meta, dict):
                raise ValueError(f"{path}: metadata is not an object")
            if meta.get("schema") != SCHEMA_ID:
                raise ValueError(
                    f"{path}: unsupported schema {meta.get('schema')!r} "
                    f"(expected {SCHEMA_ID})"
                )
            if "craters" not in data or "rocks" not in data:
                raise ValueError(f"{path}: missing craters/rocks arrays")
            # Preserve on-disk dtype; validate() must reject wrong-dtype arrays.
            craters = np.asarray(data["craters"])
            rocks = np.asarray(data["rocks"])
        db_version = meta.get("db_version")
        if isinstance(db_version, bool) or not isinstance(db_version, int):
            raise ValueError(  # noqa: TRY004 - malformed file content, not API type
                f"{path}: augmentation db_version must be an integer, "
                f"got {db_version!r}"
            )
        if db_version > AUGMENT_DB_VERSION:
            raise ValueError(
                f"{path}: augmentation db_version={db_version} is newer than "
                f"the supported AUGMENT_DB_VERSION={AUGMENT_DB_VERSION}"
            )
        if db_version == 1 and rocks.ndim == 2 and rocks.shape[1] == 5:
            z = np.zeros(rocks.shape[0], dtype=rocks.dtype)
            rocks = np.column_stack(
                [rocks[:, 0], rocks[:, 1], z, rocks[:, 2], rocks[:, 3], rocks[:, 4]]
            ).astype(rocks.dtype)
            meta = dict(meta)
            meta["upgraded_from"] = 1
        elif db_version < AUGMENT_DB_VERSION:
            # Heights from stale versions cannot be safely upgraded without
            # the source DEM; reject rather than serve a mismatched datum.
            raise ValueError(
                f"{path}: augmentation db sidecar has db_version="
                f"{db_version!r}, older than the current "
                f"AUGMENT_DB_VERSION={AUGMENT_DB_VERSION} and not auto-"
                "upgradeable (only db_version=1 is) -- rock z_m on this file "
                "may be on a stale Z datum (e.g. db_version=2 predates the "
                "mesh-datum correction) and would silently misplace rocks. "
                "Re-bake the site (clear its cache entry) to regenerate a "
                "current sidecar."
            )
        db = AugmentationDb(craters=craters, rocks=rocks, meta=meta)
        db.validate()
        return db

    if _generation is not None:
        member = path.relative_to(_generation.path).as_posix()
        return _generation.read_member(member, parse)
    with ArtifactPublication.file(path).open_snapshot(
        expected_sha256=expected_sha256,
        max_bytes=MAX_IN_MEMORY_FILE_BYTES,
    ) as snapshot:
        if snapshot is None:
            raise FileNotFoundError(path)
        return parse(snapshot.file.read())
