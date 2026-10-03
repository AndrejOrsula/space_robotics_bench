"""Read verified COG windows and cache them as patch COGs.

Production reads require a complete locally verified source: GDAL `/vsicurl`
cannot enforce the transport's per-redirect host policy. Injected openers are
the network-free test seam. Byte budgets count decompressed window bytes,
since rasterio/GDAL expose no per-read wire-byte count.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional
from urllib.parse import urlparse

import numpy as np
import rasterio
from rasterio.io import MemoryFile
from rasterio.transform import Affine

from srb.terrain import cache as cache_mod
from srb.terrain.errors import (
    ChecksumMismatchError,
    NetworkError,
    NodataAllError,
    SourceTooLargeError,
    UnreadableFileError,
)
from srb.terrain.ingest.dem import Dem
from srb.terrain.ingest.window import (
    plan_window,
    read_planned_window,
    read_planned_window_multiband,
    reprojected_source,
)
from srb.terrain.sources.acquisition import MAX_UNAPPROVED_SOURCE_BYTES
from srb.utils.publication import ArtifactPublication, sha256_file
from srb.utils.resource_budget import (
    MAX_IN_MEMORY_FILE_BYTES,
    ResourceLimitError,
    checked_shape_nbytes,
    require_file_size,
)

# PGDA rejects HEAD and has no directory indexes.
PGDA_GDAL_ENV: dict[str, str] = {
    "CPL_VSIL_CURL_USE_HEAD": "NO",
    "GDAL_DISABLE_READDIR_ON_OPEN": "EMPTY_DIR",
    "CPL_VSIL_CURL_ALLOWED_EXTENSIONS": ".tif,.tiff,.TIF,.TIFF,.vrt",
    "GDAL_HTTP_MAX_RETRY": "3",
    "GDAL_HTTP_RETRY_DELAY": "2",
}

Opener = Callable[[str], "rasterio.DatasetReader"]


def vsicurl_url(url: str, *, allowed_hosts: frozenset[str]) -> str:
    """Build a `/vsicurl/` path after validating its HTTP(S) scheme and host.

    Recheck here because GDAL bypasses `BaseHttpProvider`; production readers
    still refuse direct network access.
    """
    parsed = urlparse(url)
    if parsed.scheme not in ("http", "https"):
        raise NetworkError(f"/vsicurl requires an http(s) URL, got: {url}")
    if parsed.hostname not in allowed_hosts:
        raise NetworkError(f"host not allowlisted for /vsicurl access: {url}")
    return f"/vsicurl/{url}"


def window_cache_key(
    layer_id: str,
    content_sha256: str,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: float | None,
    dst_crs: Optional[str] = None,
    pad_mode: Optional[str] = None,
    band_mode: Literal["single", "multiband"] = "single",
    verification_mode: Literal["stream", "verified"] = "stream",
    pad_m: float = 0.0,
    resampling: Literal["bilinear", "nearest"] = "bilinear",
    declared_crs: str | None = None,
    body: str | None = None,
) -> str:
    """Return a 16-character SHA-256 key for the complete window read identity.

    The source pin prevents stale windows after re-pinning. Destination CRS,
    padding, resampling, band/verification mode, and serializer versions
    prevent cache hits across reads that can produce different results.
    """
    # Exact serialized bytes define immutable publication identity.
    serializer_identity = [
        "srb-patch-cog/1",
        str(rasterio.__version__),
        str(rasterio.__gdal_version__),
    ]
    material_fields: list[object] = [
        layer_id,
        content_sha256,
        list(center_xy_m),
        size_m,
        target_gsd_m,
        dst_crs,
        pad_mode,
        pad_m,
        resampling,
        serializer_identity,
    ]
    # Keep modes disjoint when their shape, dtype, or integrity contract differs.
    if band_mode != "single" or verification_mode != "stream":
        material_fields.extend([band_mode, verification_mode])
    # Cache hits skip the declared-CRS check, so a changed claim must miss.
    if declared_crs is not None or body is not None:
        material_fields.append(["declared_crs", declared_crs, body])
    material = json.dumps(material_fields)
    return hashlib.sha256(material.encode("utf-8")).hexdigest()[:16]


def patch_cog_path(layer_id: str, key: str, *, root: Path | None = None) -> Path:
    """Return the cache path for one layer/window key."""
    cache_dir = cache_mod.cache_root() if root is None else root
    return cache_dir / cache_mod.RAW_DIRNAME / "_windows" / layer_id / f"{key}.tif"


@dataclass
class WindowByteBudget:
    """Track approximate window bytes, with a promotion hint and hard ceiling."""

    promote_hint_bytes: int = 200 * 1024 * 1024
    hard_ceiling_bytes: int = MAX_UNAPPROVED_SOURCE_BYTES  # == 1 << 30
    transferred: int = 0

    def record(self, n_bytes: int, *, url: str = "<window>") -> None:
        """Accumulate bytes; raise `SourceTooLargeError` above the hard ceiling."""
        self.transferred += n_bytes
        if self.transferred > self.hard_ceiling_bytes:
            raise SourceTooLargeError(
                url, limit=self.hard_ceiling_bytes, observed=self.transferred
            )

    @property
    def should_promote(self) -> bool:
        return self.transferred >= self.promote_hint_bytes

    def hint(self, layer_id: str) -> str:
        return (
            f"windowed reads of '{layer_id}' have moved "
            f"~{self.transferred / 1e6:.0f} MB; consider "
            f"`srb asset download layer {layer_id}` to fetch it once"
        )


def _verified_or_remote_path(
    url: str,
    *,
    allowed_hosts: frozenset[str],
    verified_local_artifact: Path | None,
    expected_sha256: str,
    opener: Opener | None,
) -> str:
    """Return a verified local source or a test-only synthetic remote path."""
    if verified_local_artifact is None:
        if opener is None:
            raise NetworkError(
                "direct GDAL /vsicurl network access is disabled: acquire the "
                "source through fetch_catalog_layer() and pass its verified "
                "local artifact"
            )
        return vsicurl_url(url, allowed_hosts=allowed_hosts)

    path = Path(verified_local_artifact).expanduser().resolve()
    if not path.is_file():
        raise UnreadableFileError(f"verified terrain artifact not found: {path}")
    require_file_size(path, resource=f"verified terrain artifact {path}")
    actual = sha256_file(path)
    expected = expected_sha256.lower()
    if actual != expected:
        raise ChecksumMismatchError(url=str(path), expected=expected, actual=actual)
    return str(path)


def read_window_remote(
    url: str,
    *,
    layer_id: str,
    content_sha256: str,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: float | None = None,
    pad_m: float = 0.0,
    allowed_hosts: frozenset[str],
    gdal_env: Mapping[str, str] | None = None,
    budget: WindowByteBudget | None = None,
    cache: bool = True,
    cache_root: Path | None = None,
    opener: Opener | None = None,
    dst_crs: Optional[str] = None,
    resampling: Literal["bilinear", "nearest"] = "bilinear",
    declared_crs: Optional[str] = None,
    body: Optional[str] = None,
    pad_mode: Literal["edge", "nan"] = "edge",
    verified_local_artifact: Path | None = None,
) -> Dem:
    """Read and optionally cache a verified source window.

    Production requires `verified_local_artifact` matching `content_sha256`;
    its full-file digest is checked before cache lookup. Omission is for
    injected network-free test openers. Only immutable cached snapshots are
    reused; `declared_crs` is checked on misses.
    """
    source_path = _verified_or_remote_path(
        url,
        allowed_hosts=allowed_hosts,
        verified_local_artifact=verified_local_artifact,
        expected_sha256=content_sha256,
        opener=opener,
    )
    key = window_cache_key(
        layer_id,
        content_sha256,
        center_xy_m,
        size_m,
        target_gsd_m,
        dst_crs,
        pad_mode,
        pad_m=pad_m,
        resampling=resampling,
        verification_mode=(
            "verified" if verified_local_artifact is not None else "stream"
        ),
        declared_crs=declared_crs,
        body=body,
    )
    dest = patch_cog_path(layer_id, key, root=cache_root)

    if cache and _patch_cog_has_immutable_identity(dest):
        return _load_patch_cog_single(dest)

    active_opener: Opener = (
        opener if opener is not None else (lambda p: rasterio.open(p))
    )
    env = {**PGDA_GDAL_ENV, **(gdal_env or {})}

    with rasterio.Env(**env):
        ds = active_opener(source_path)
        try:
            dem = _read_window_from_dataset(
                ds,
                center_xy_m=center_xy_m,
                size_m=size_m,
                target_gsd_m=target_gsd_m,
                pad_m=pad_m,
                dst_crs=dst_crs,
                resampling=resampling,
                declared_crs=declared_crs,
                layer_id=layer_id,
                body=body,
                pad_mode=pad_mode,
            )
        finally:
            ds.close()

    # GDAL does not expose wire bytes; account for materialized pixels.
    n_bytes = int(dem.heights.size * dem.heights.itemsize)
    active_budget = budget if budget is not None else WindowByteBudget()
    active_budget.record(n_bytes, url=url)

    if cache:
        # Serialize with `srb clean --raw` over the shared raw-cache tree.
        with cache_mod.raw_cache_lock(
            cache_mod.cache_root() if cache_root is None else cache_root
        ):
            _write_patch_cog(dem, dest)

    return dem


def read_window_remote_multiband(
    url: str,
    *,
    layer_id: str,
    content_sha256: str,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: float | None = None,
    pad_m: float = 0.0,
    allowed_hosts: frozenset[str],
    gdal_env: Mapping[str, str] | None = None,
    budget: WindowByteBudget | None = None,
    cache: bool = True,
    cache_root: Path | None = None,
    opener: Opener | None = None,
    dst_crs: Optional[str] = None,
    resampling: Literal["bilinear", "nearest"] = "bilinear",
    declared_crs: Optional[str] = None,
    body: Optional[str] = None,
    verified_local_artifact: Path | None = None,
) -> Dem:
    """Read and optionally cache a verified window with all source bands.

    Preserves native dtype and overlap extent; padding would upcast byte DN
    and alter fusion radiometry. Production verifies the full local source
    before cache lookup; injected openers are test-only.
    """
    source_path = _verified_or_remote_path(
        url,
        allowed_hosts=allowed_hosts,
        verified_local_artifact=verified_local_artifact,
        expected_sha256=content_sha256,
        opener=opener,
    )
    key = window_cache_key(
        layer_id,
        content_sha256,
        center_xy_m,
        size_m,
        target_gsd_m,
        dst_crs,
        band_mode="multiband",
        pad_m=pad_m,
        resampling=resampling,
        verification_mode=(
            "verified" if verified_local_artifact is not None else "stream"
        ),
        declared_crs=declared_crs,
        body=body,
    )
    dest = patch_cog_path(layer_id, key, root=cache_root)

    if cache and _patch_cog_has_immutable_identity(dest):
        return _load_patch_cog_multiband(dest)

    active_opener: Opener = (
        opener if opener is not None else (lambda p: rasterio.open(p))
    )
    env = {**PGDA_GDAL_ENV, **(gdal_env or {})}

    with rasterio.Env(**env):
        ds = active_opener(source_path)
        try:
            dem = _read_window_from_dataset_multiband(
                ds,
                center_xy_m=center_xy_m,
                size_m=size_m,
                target_gsd_m=target_gsd_m,
                pad_m=pad_m,
                dst_crs=dst_crs,
                resampling=resampling,
                declared_crs=declared_crs,
                layer_id=layer_id,
                body=body,
            )
        finally:
            ds.close()

    active_budget = budget if budget is not None else WindowByteBudget()
    active_budget.record(int(dem.heights.nbytes), url=url)

    if cache:
        # Serialize with `srb clean --raw` over the shared raw-cache tree.
        with cache_mod.raw_cache_lock(
            cache_mod.cache_root() if cache_root is None else cache_root
        ):
            _write_patch_cog(dem, dest)

    return dem


def _read_window_from_dataset(
    ds: "rasterio.DatasetReader",
    *,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: float | None,
    pad_m: float,
    fill_nodata_threshold: float = 0.10,
    dst_crs: Optional[str] = None,
    resampling: Literal["bilinear", "nearest"] = "bilinear",
    declared_crs: Optional[str] = None,
    layer_id: Optional[str] = None,
    body: Optional[str] = None,
    pad_mode: Literal["edge", "nan"] = "edge",
) -> Dem:
    """Read a planned window from an open dataset using shared ingest logic.

    The caller owns `ds`; this context closes only a `WarpedVRT` it creates.
    """
    with reprojected_source(
        ds,
        dst_crs=dst_crs,
        resampling=resampling,
        declared_crs=declared_crs,
        layer_id=layer_id,
        body=body,
    ) as read_ds:
        plan = plan_window(
            read_ds,
            center_xy_m=center_xy_m,
            size_m=size_m,
            target_gsd_m=target_gsd_m,
            pad_m=pad_m,
            source_desc="remote dataset",
        )
        return read_planned_window(
            read_ds,
            plan,
            fill_nodata_threshold=fill_nodata_threshold,
            source_desc="<remote window>",
            pad_mode=pad_mode,
        )


def _read_window_from_dataset_multiband(
    ds: "rasterio.DatasetReader",
    *,
    center_xy_m: tuple[float, float],
    size_m: float,
    target_gsd_m: float | None,
    pad_m: float,
    dst_crs: Optional[str] = None,
    resampling: Literal["bilinear", "nearest"] = "bilinear",
    declared_crs: Optional[str] = None,
    layer_id: Optional[str] = None,
    body: Optional[str] = None,
) -> Dem:
    """Read every source band using the shared multiband window contract."""
    with reprojected_source(
        ds,
        dst_crs=dst_crs,
        resampling=resampling,
        declared_crs=declared_crs,
        layer_id=layer_id,
        body=body,
    ) as read_ds:
        plan = plan_window(
            read_ds,
            center_xy_m=center_xy_m,
            size_m=size_m,
            target_gsd_m=target_gsd_m,
            pad_m=pad_m,
            source_desc="remote dataset",
        )
        return read_planned_window_multiband(read_ds, plan)


def _patch_cog_identity_path(path: Path) -> Path:
    """Return the immutable identity marker used by ``ArtifactPublication``."""
    return path.with_name(f".{path.name}.publication.sha256")


def _patch_cog_has_immutable_identity(path: Path) -> bool:
    """Only trust patch snapshots that have an immutable identity marker."""
    return path.is_file() and _patch_cog_identity_path(path).is_file()


def _patch_cog_payload(dem: Dem) -> bytes:
    """Serialize the exact patch-COG bytes that will be published."""
    transform = Affine(*dem.transform_a)
    heights = dem.heights
    if heights.ndim == 2:
        count = 1
        band_first = heights[None, ...]
    elif heights.ndim == 3:
        count = heights.shape[2]
        band_first = np.moveaxis(heights, -1, 0)
    else:
        raise ValueError(
            f"remote patch cache requires 2-D or 3-D pixels, got {heights.shape}"
        )
    checked_shape_nbytes(
        band_first.shape,
        band_first.dtype,
        resource="terrain patch COG decoded pixels",
        max_bytes=MAX_IN_MEMORY_FILE_BYTES,
    )

    nodata = None if np.isnan(dem.nodata) else dem.nodata
    with MemoryFile() as memory:
        with rasterio.Env(GDAL_TIFF_INTERNAL_MASK="YES"):
            with memory.open(
                driver="GTiff",
                height=heights.shape[0],
                width=heights.shape[1],
                count=count,
                dtype=heights.dtype,
                crs=dem.crs,
                transform=transform,
                nodata=nodata,
            ) as dst:
                dst.write(band_first)
                if dem.valid_mask is not None:
                    dst.write_mask(np.where(dem.valid_mask, 255, 0).astype(np.uint8))
        payload = memory.read(MAX_IN_MEMORY_FILE_BYTES + 1)
        if len(payload) > MAX_IN_MEMORY_FILE_BYTES:
            raise ResourceLimitError(
                "terrain patch COG bytes",
                limit=MAX_IN_MEMORY_FILE_BYTES,
                observed=len(payload),
            )
        return payload


def _write_patch_cog(
    dem: Dem,
    dest: Path,
    *,
    immutable_identity: bool = True,
) -> None:
    """Publish a patch COG; immutable identity hashes its serialized bytes.

    `content_sha256` identifies the source, not this derived window; `False`
    permits replacement at noncanonical destinations.
    """
    payload = _patch_cog_payload(dem)
    publication = ArtifactPublication.file(dest)
    if immutable_identity:
        publication.publish(
            lambda staging: staging.write_bytes(payload),
            expected_sha256=hashlib.sha256(payload).hexdigest(),
            immutable_identity=True,
        )
    else:
        publication.publish(lambda staging: staging.write_bytes(payload))


def _load_patch_cog_multiband(path: Path) -> Dem:
    """Load one immutable cached remote-patch snapshot."""

    if not _patch_cog_has_immutable_identity(path):
        raise UnreadableFileError(f"cached patch has no immutable identity: {path}")

    def parse(payload: bytes) -> Dem:
        with MemoryFile(payload) as memory, memory.open() as ds:
            return _dem_from_multiband_dataset(ds)

    loaded = ArtifactPublication.file(path).read(parse)
    if loaded is None:
        raise UnreadableFileError(f"cached patch not found: {path}")
    return loaded


def _dem_from_multiband_dataset(ds: "rasterio.DatasetReader") -> Dem:
    """Decode a multi-band patch dataset that is already open."""
    checked_shape_nbytes(
        (ds.count, ds.height, ds.width),
        np.result_type(*(np.dtype(dtype) for dtype in ds.dtypes)),
        resource="terrain cached multiband patch pixels",
    )
    checked_shape_nbytes(
        (ds.height, ds.width),
        np.dtype(np.uint8),
        resource="terrain cached multiband patch validity mask",
    )
    raw = ds.read()
    # Always `(H, W, C)`, even for `C == 1`: the miss path
    # (`read_planned_window_multiband`) returns that shape and callers
    # (`_require_dem(..., expected_ndim=3)`) reject a 2-D cache hit.
    heights = np.ascontiguousarray(np.moveaxis(raw, 0, -1))
    t = ds.transform
    nodata = ds.nodatavals[0] if ds.nodatavals else None
    nodata_f = (
        float(nodata) if nodata is not None and not np.isnan(nodata) else float("nan")
    )
    return Dem(
        heights=heights,
        gsd_m=float((ds.res[0] + ds.res[1]) / 2.0),
        nodata=nodata_f,
        crs=str(ds.crs) if ds.crs else None,
        transform_a=(t.a, t.b, t.c, t.d, t.e, t.f),
        valid_mask=ds.dataset_mask() > 0,
    )


def _load_patch_cog_single(path: Path) -> Dem:
    """Load a single-band cache snapshot with ``load_dem`` semantics."""

    if not _patch_cog_has_immutable_identity(path):
        raise UnreadableFileError(f"cached patch has no immutable identity: {path}")

    def parse(payload: bytes) -> Dem:
        with MemoryFile(payload) as memory:
            try:
                ds = memory.open()
            except rasterio.errors.RasterioIOError as exc:
                raise UnreadableFileError(
                    f"rasterio could not open {path}: {exc}"
                ) from exc
            with ds:
                checked_shape_nbytes(
                    (ds.height, ds.width),
                    np.dtype(ds.dtypes[0]),
                    resource=f"terrain cached patch decoded pixels {path}",
                )
                checked_shape_nbytes(
                    (ds.height, ds.width),
                    np.dtype(np.float32),
                    resource=f"terrain cached patch float32 pixels {path}",
                )
                band = ds.read(1).astype(np.float32, copy=False)
                nodata = (
                    ds.nodatavals[0]
                    if ds.nodatavals and ds.nodatavals[0] is not None
                    else float("nan")
                )
                nodata_f = (
                    float(nodata)
                    if nodata is not None and not np.isnan(nodata)
                    else float("nan")
                )
                if np.isnan(nodata_f):
                    mask = np.isnan(band)
                else:
                    mask = band == nodata_f
                if mask.all():
                    raise NodataAllError(f"all pixels are nodata in {path}")
                if mask.any():
                    band = band.copy()
                    band[mask] = np.nan
                ax, by = ds.res
                if abs(ax - by) / max(ax, by, 1e-9) > 1e-3:
                    raise UnreadableFileError(
                        f"non-square pixels in {path}: x={ax}, y={by}"
                    )
                t = ds.transform
                return Dem(
                    heights=band,
                    gsd_m=float((ax + by) / 2.0),
                    nodata=nodata_f,
                    crs=str(ds.crs) if ds.crs else None,
                    transform_a=(t.a, t.b, t.c, t.d, t.e, t.f),
                    valid_mask=np.isfinite(band),
                )

    loaded = ArtifactPublication.file(path).read(parse)
    if loaded is None:
        raise UnreadableFileError(f"cached patch not found: {path}")
    return loaded
