"""Plan, validate, and fetch manifest-backed terrain sources."""

from __future__ import annotations

import os
import zipfile
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from urllib.parse import unquote, urljoin, urlparse

from srb.terrain import cache as cache_mod
from srb.terrain.catalog_schema import SourceLayer, _url_basename
from srb.terrain.errors import (
    ChecksumMismatchError,
    LargeDownloadConsentRequiredError,
    ManifestInvalidError,
    NetworkError,
    SourceTooLargeError,
    TerrainError,
)
from srb.terrain.schema import Manifest, SiteEntry, SourceEntry
from srb.terrain.sources.base import BaseProvider
from srb.terrain.sources.base_http import BaseHttpProvider
from srb.terrain.sources.local import LocalProvider
from srb.utils.digest import sha256_file
from srb.utils.publication import ArtifactPublication
from srb.utils.resource_budget import (
    MAX_ARCHIVE_MEMBERS,
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_EXTERNAL_FILE_BYTES,
    ByteBudget,
    ResourceLimitError,
    require_file_size,
)

# Providers whose ``fetch`` enforces ``max_bytes`` before or while copying.
_SIZE_BOUNDED_PROVIDERS = (BaseHttpProvider, LocalProvider)

PLACEHOLDER_SHA256 = "f" * 64
MAX_UNAPPROVED_SOURCE_BYTES = 1 << 30
MAX_ZIP_MEMBER_BYTES: int = 8 * (1 << 30)  # per-member decompressed ceiling
MAX_APPROVED_SOURCE_BYTES = MAX_EXTERNAL_FILE_BYTES
MAX_ZIP_TOTAL_BYTES = MAX_DECODED_AGGREGATE_BYTES
LARGE_DOWNLOAD_CONSENT_ENV = "SRB_TERRAIN_LARGE_OK"
_TRUTHY = frozenset({"1", "true", "yes", "on"})

ProviderGetter = Callable[[str], BaseProvider]
HeadRequester = Callable[..., object]


class CachedSourceStatus(str, Enum):
    PLACEHOLDER = "placeholder"
    MISSING = "missing"
    MISMATCH = "mismatch"
    INVALID_PAYLOAD = "invalid_payload"
    VALID = "valid"


_MAX_HEAD_REDIRECTS = 5
_REDIRECT_CODES = frozenset({301, 302, 303, 307, 308})


class SourceUrlStatus(str, Enum):
    LOCAL_MISSING = "local_missing"
    UNSUPPORTED_SCHEME = "unsupported_scheme"
    PROVIDER_REJECTED = "provider_rejected"
    HEAD_FAILED = "head_failed"
    HTTP_ERROR = "http_error"
    HTML_PAYLOAD = "html_payload"
    TOO_LARGE = "too_large"
    REDIRECT_REJECTED = "redirect_rejected"
    VALID = "valid"


@dataclass(frozen=True, slots=True)
class CuratedSourcePlan:
    site_key: str
    role: str
    source: SourceEntry
    raw_path: Path

    @property
    def label(self) -> str:
        return f"{self.site_key}.{self.role}"

    @property
    def sha256(self) -> str:
        return self.source.sha256.lower()

    @property
    def is_placeholder_sha(self) -> bool:
        return self.sha256 == PLACEHOLDER_SHA256


@dataclass(frozen=True, slots=True)
class CuratedSiteAcquisition:
    site_key: str
    site_entry: SiteEntry
    dem: CuratedSourcePlan
    ortho: CuratedSourcePlan | None = None

    @property
    def sources(self) -> tuple[CuratedSourcePlan, ...]:
        if self.ortho is None:
            return (self.dem,)
        return (self.dem, self.ortho)

    @property
    def placeholder_labels(self) -> tuple[str, ...]:
        return tuple(
            source.label for source in self.sources if source.is_placeholder_sha
        )

    @property
    def raw_dem_path(self) -> Path:
        return self.dem.raw_path


@dataclass(frozen=True, slots=True)
class CachedSourceCheck:
    plan: CuratedSourcePlan
    status: CachedSourceStatus
    actual_sha256: str | None = None
    error: str | None = None

    @property
    def label(self) -> str:
        return self.plan.label


@dataclass(frozen=True, slots=True)
class SourceUrlCheck:
    plan: CuratedSourcePlan
    status: SourceUrlStatus
    issue: str | None = None

    @property
    def label(self) -> str:
        return self.plan.label


def plan_curated_site_sources(
    site_key: str,
    site_entry: SiteEntry,
    *,
    use_ortho: bool = True,
    root: Path | None = None,
) -> CuratedSiteAcquisition:
    dem_source = site_entry.sources.dem
    if dem_source is None:
        raise ValueError(
            f"{site_key}: v2 dem_stack sites use catalog layer acquisition, "
            "not plan_curated_site_sources"
        )
    dem = _source_plan(
        site_key,
        "dem",
        dem_source,
        root=root,
    )
    ortho = (
        _source_plan(site_key, "ortho", site_entry.sources.ortho, root=root)
        if use_ortho and site_entry.sources.ortho is not None
        else None
    )
    return CuratedSiteAcquisition(
        site_key=site_key,
        site_entry=site_entry,
        dem=dem,
        ortho=ortho,
    )


def iter_manifest_sources(
    manifest: Manifest,
    *,
    root: Path | None = None,
) -> Iterator[CuratedSourcePlan]:
    for site_key, site_entry in manifest.sites.items():
        if site_entry.sources.dem is None:
            continue
        yield from plan_curated_site_sources(
            site_key,
            site_entry,
            use_ortho=True,
            root=root,
        ).sources


def fetch_curated_site_sources(
    site_key: str,
    site_entry: SiteEntry,
    *,
    use_ortho: bool = True,
    root: Path | None = None,
    provider_getter: ProviderGetter | None = None,
) -> tuple[Path, Path | None]:
    plan = plan_curated_site_sources(
        site_key,
        site_entry,
        use_ortho=use_ortho,
        root=root,
    )
    if plan.placeholder_labels:
        labels = ", ".join(plan.placeholder_labels)
        raise ManifestInvalidError(f"{labels}: placeholder sha256")

    dem_path = fetch_curated_source(
        plan.dem,
        provider_getter=provider_getter,
    )
    ortho_path = (
        fetch_curated_source(plan.ortho, provider_getter=provider_getter)
        if plan.ortho is not None
        else None
    )
    return dem_path, ortho_path


def fetch_curated_source(
    plan: CuratedSourcePlan,
    *,
    provider_getter: ProviderGetter | None = None,
    max_unapproved_bytes: int = MAX_UNAPPROVED_SOURCE_BYTES,
) -> Path:
    """Fetch through the provider with the preflight byte limit applied to the stream;
    explicit large-source approval remains bounded by the external-file ceiling.
    """
    if plan.is_placeholder_sha:
        raise ManifestInvalidError(f"{plan.label}: placeholder sha256")

    scheme = urlparse(plan.source.url).scheme
    if plan.source.provider == "local":
        if scheme not in ("", "file"):
            raise ManifestInvalidError(
                f"{plan.label}: local provider requires a file path or file:// URL"
            )
    elif scheme not in ("http", "https"):
        raise ManifestInvalidError(
            f"{plan.label}: provider {plan.source.provider!r} requires an HTTP(S) URL"
        )

    if provider_getter is None:
        from srb.terrain.sources.registry import get_provider

        provider_getter = get_provider

    provider = provider_getter(plan.source.provider)
    # Cleanup admission precedes the digest lock; unrelated payloads can fetch
    # together while cleanup cannot remove `_raw` mid-download.
    with cache_mod.raw_cache_acquisition_lock(
        _raw_cache_root(plan.raw_path), plan.sha256
    ):
        if isinstance(provider, _SIZE_BOUNDED_PROVIDERS):
            return provider.fetch(
                plan.source.url,
                plan.sha256,
                plan.raw_path,
                max_bytes=_source_max_bytes(
                    plan.source, max_unapproved_bytes=max_unapproved_bytes
                ),
            )
        return provider.fetch(plan.source.url, plan.sha256, plan.raw_path)


def _raw_cache_root(raw_path: Path) -> Path:
    for parent in raw_path.parents:
        if parent.name == cache_mod.RAW_DIRNAME:
            return parent.parent
    return cache_mod.cache_root()


def check_cached_source(plan: CuratedSourcePlan) -> CachedSourceCheck:
    if plan.is_placeholder_sha:
        return CachedSourceCheck(
            plan=plan,
            status=CachedSourceStatus.PLACEHOLDER,
        )
    with cache_mod.raw_cache_acquisition_lock(
        _raw_cache_root(plan.raw_path), plan.sha256
    ):
        return _check_cached_source_locked(plan)


def _check_cached_source_locked(plan: CuratedSourcePlan) -> CachedSourceCheck:
    if not plan.raw_path.exists():
        return CachedSourceCheck(plan=plan, status=CachedSourceStatus.MISSING)

    actual = sha256_file(plan.raw_path)
    if actual != plan.sha256:
        return CachedSourceCheck(
            plan=plan,
            status=CachedSourceStatus.MISMATCH,
            actual_sha256=actual,
        )

    try:
        BaseProvider._reject_obvious_html_payload(
            plan.raw_path,
            url=plan.source.url,
        )
    except TerrainError as exc:
        return CachedSourceCheck(
            plan=plan,
            status=CachedSourceStatus.INVALID_PAYLOAD,
            error=str(exc),
        )

    return CachedSourceCheck(plan=plan, status=CachedSourceStatus.VALID)


def check_cached_sources(
    manifest: Manifest,
    *,
    root: Path | None = None,
) -> tuple[CachedSourceCheck, ...]:
    return tuple(
        check_cached_source(plan) for plan in iter_manifest_sources(manifest, root=root)
    )


def check_source_url(
    plan: CuratedSourcePlan,
    *,
    provider_getter: ProviderGetter | None = None,
    head: HeadRequester | None = None,
    max_unapproved_bytes: int = MAX_UNAPPROVED_SOURCE_BYTES,
) -> SourceUrlCheck:
    source = plan.source
    parsed = urlparse(source.url)

    if source.provider == "local":
        if parsed.scheme not in ("", "file"):
            return SourceUrlCheck(
                plan=plan,
                status=SourceUrlStatus.UNSUPPORTED_SCHEME,
                issue=f"{plan.label}: local provider requires a file URL: {source.url}",
            )
        raw_path = parsed.path or parsed.netloc
        # Match LocalProvider: ``file://`` URLs percent-encode reserved characters.
        path = Path(unquote(raw_path) if parsed.scheme == "file" else raw_path)
        if not path.is_file():
            return SourceUrlCheck(
                plan=plan,
                status=SourceUrlStatus.LOCAL_MISSING,
                issue=f"{plan.label}: local source not found: {path}",
            )
        return SourceUrlCheck(plan=plan, status=SourceUrlStatus.VALID)

    if parsed.scheme not in ("http", "https"):
        return SourceUrlCheck(
            plan=plan,
            status=SourceUrlStatus.UNSUPPORTED_SCHEME,
            issue=f"{plan.label}: unsupported URL scheme: {source.url}",
        )

    if provider_getter is None:
        from srb.terrain.sources.registry import get_provider

        provider_getter = get_provider
    provider = provider_getter(source.provider)
    accepts_url = getattr(provider, "_accepts_url", None)
    if accepts_url is not None and not accepts_url(source.url):
        return SourceUrlCheck(
            plan=plan,
            status=SourceUrlStatus.PROVIDER_REJECTED,
            issue=(
                f"{plan.label}: provider {source.provider} does not accept URL: "
                f"{source.url}"
            ),
        )

    if head is None:
        head = _head_source_url

    # Follow redirects by hand so the provider vets each hop before it is contacted.
    response_url = source.url
    try:
        for _ in range(_MAX_HEAD_REDIRECTS + 1):
            response = head(response_url, allow_redirects=False, timeout=10.0)
            location = getattr(response, "headers", {}).get("location")
            if getattr(response, "status_code", 0) not in _REDIRECT_CODES or not (
                location
            ):
                break
            close = getattr(response, "close", None)
            if close is not None:
                close()
            response_url = urljoin(response_url, location)
            if accepts_url is not None and not accepts_url(response_url):
                return SourceUrlCheck(
                    plan=plan,
                    status=SourceUrlStatus.REDIRECT_REJECTED,
                    issue=(
                        f"{plan.label}: provider {source.provider} does not "
                        f"accept redirect target: {response_url}"
                    ),
                )
        else:
            raise RuntimeError(f"more than {_MAX_HEAD_REDIRECTS} redirects")
    except Exception as exc:  # noqa: BLE001 - provider client errors become checks
        return SourceUrlCheck(
            plan=plan,
            status=SourceUrlStatus.HEAD_FAILED,
            issue=f"{plan.label}: URL check failed for {source.url}: {exc}",
        )

    try:
        status_code = getattr(response, "status_code", 0)
        headers = getattr(response, "headers", {})

        if status_code >= 400:
            return SourceUrlCheck(
                plan=plan,
                status=SourceUrlStatus.HTTP_ERROR,
                issue=f"{plan.label}: HTTP {status_code} checking {source.url}",
            )

        content_type = headers.get("content-type", "").lower()
        if content_type.startswith(("text/html", "application/xhtml+xml")):
            return SourceUrlCheck(
                plan=plan,
                status=SourceUrlStatus.HTML_PAYLOAD,
                issue=(
                    f"{plan.label}: URL returned HTML catalog content, "
                    f"not raster bytes: {source.url}"
                ),
            )

        content_length = _content_length_bytes(headers)
        if (
            content_length is not None
            and content_length > max_unapproved_bytes
            and not source.allow_large_download
        ):
            return SourceUrlCheck(
                plan=plan,
                status=SourceUrlStatus.TOO_LARGE,
                issue=(
                    f"{plan.label}: URL source is too large for default "
                    f"validation ({content_length} bytes > "
                    f"{max_unapproved_bytes} bytes); set allow_large_download: "
                    "true only for intentional large rasters"
                ),
            )

    finally:
        close = getattr(response, "close", None)
        if close is not None:
            close()

    return SourceUrlCheck(plan=plan, status=SourceUrlStatus.VALID)


def check_source_urls(
    manifest: Manifest,
    *,
    root: Path | None = None,
    provider_getter: ProviderGetter | None = None,
    head: HeadRequester | None = None,
    max_unapproved_bytes: int = MAX_UNAPPROVED_SOURCE_BYTES,
) -> tuple[SourceUrlCheck, ...]:
    return tuple(
        check_source_url(
            plan,
            provider_getter=provider_getter,
            head=head,
            max_unapproved_bytes=max_unapproved_bytes,
        )
        for plan in iter_manifest_sources(manifest, root=root)
    )


def _source_plan(
    site_key: str,
    role: str,
    source: SourceEntry,
    *,
    root: Path | None,
) -> CuratedSourcePlan:
    return CuratedSourcePlan(
        site_key=site_key,
        role=role,
        source=source,
        raw_path=cache_mod.raw_cache_path(source.sha256, root=root),
    )


def _head_source_url(url: str, **kwargs):
    import requests

    return requests.head(url, **kwargs)


def _source_max_bytes(
    source: SourceEntry,
    *,
    max_unapproved_bytes: int = MAX_UNAPPROVED_SOURCE_BYTES,
) -> int:
    return (
        MAX_APPROVED_SOURCE_BYTES
        if source.allow_large_download
        else max_unapproved_bytes
    )


# v1 `SourceEntry` approval remains `allow_large_download` alone;
# it needs no environment opt-in.
def large_download_consented(env: Mapping[str, str] | None = None) -> bool:
    """Read only the named environment variable. Accept explicit affirmative values;
    deny unset, blank, or unrecognized values. Never prompt or block.
    """
    source = os.environ if env is None else env
    raw = source.get(LARGE_DOWNLOAD_CONSENT_ENV)
    if raw is None:
        return False
    return raw.strip().lower() in _TRUTHY


def layer_max_bytes(
    layer: SourceLayer,
    *,
    max_unapproved_bytes: int = MAX_UNAPPROVED_SOURCE_BYTES,
    env: Mapping[str, str] | None = None,
) -> int:
    """Raise the ordinary limit only when both layer-level approval and operator
    environment consent are present; v1 curated sources retain their separate policy.
    """
    if layer.allow_large_download and large_download_consented(env):
        return MAX_APPROVED_SOURCE_BYTES
    return max_unapproved_bytes


def check_layer_consent(
    layer: SourceLayer,
    est_bytes: int,
    *,
    max_unapproved_bytes: int = MAX_UNAPPROVED_SOURCE_BYTES,
    env: Mapping[str, str] | None = None,
) -> None:
    """Raise when the estimate exceeds the applicable finite allowance; never prompt."""
    limit = layer_max_bytes(layer, max_unapproved_bytes=max_unapproved_bytes, env=env)
    if est_bytes > MAX_APPROVED_SOURCE_BYTES:
        raise SourceTooLargeError(
            layer.urls[0],
            limit=MAX_APPROVED_SOURCE_BYTES,
            observed=est_bytes,
            observed_desc="declared by the catalog (no bytes read)",
        )
    if est_bytes > limit:
        raise LargeDownloadConsentRequiredError(
            layer.layer_id, est_bytes=est_bytes, limit=limit
        )


def allowed_hosts_for_layer(
    layer: SourceLayer,
    *,
    url: str | None = None,
    provider_getter: ProviderGetter | None = None,
) -> frozenset[str]:
    """Resolve and validate the provider URL, then return its single hostname as the COG
    read allowlist.
    """
    target_url = layer.urls[0] if url is None else url

    if provider_getter is None:
        from srb.terrain.sources.registry import get_provider

        provider_getter = get_provider

    provider = provider_getter(layer.provider)
    accepts_url = getattr(provider, "_accepts_url", None)
    if accepts_url is not None and not accepts_url(target_url):
        raise NetworkError(
            f"{layer.layer_id}: provider {layer.provider!r} does not accept "
            f"its own declared URL: {target_url}"
        )

    host = urlparse(target_url).hostname
    if not host:
        raise NetworkError(f"{layer.layer_id}: url has no hostname: {target_url}")
    return frozenset({host})


@dataclass(frozen=True, slots=True)
class CatalogLayerPlan:
    layer: SourceLayer
    url: str
    raw_path: Path
    member_paths: dict[str, Path]
    bundle_path: Path | None = None

    @property
    def archive_sha256(self) -> str:
        return self.layer.sha256[_url_basename(self.url)]

    @property
    def is_placeholder_sha(self) -> bool:
        primary_or_member_placeholder = any(
            value.lower() == PLACEHOLDER_SHA256 for value in self.layer.sha256.values()
        )
        sidecar_placeholder = any(
            sidecar.sha256.lower() == PLACEHOLDER_SHA256
            for sidecar in self.layer.sidecars
        )
        return primary_or_member_placeholder or sidecar_placeholder


def _url_suffix(value: str) -> str:
    return Path(_url_basename(value)).suffix


def plan_catalog_layer(
    layer: SourceLayer, *, root: Path | None = None
) -> CatalogLayerPlan:
    """Build cache destinations without network access or consent checks; fetch-time
    code owns those refusals.
    """
    url = layer.urls[0]
    archive_key = _url_basename(url)
    archive_sha = layer.sha256[archive_key]
    if layer.sidecars:
        bundle_path = cache_mod.raw_bundle_cache_path(
            cache_mod.catalog_layer_cache_key(layer), root=root
        )
        return CatalogLayerPlan(
            layer=layer,
            url=url,
            raw_path=bundle_path / archive_key,
            member_paths={
                sidecar.filename: bundle_path / sidecar.filename
                for sidecar in layer.sidecars
            },
            bundle_path=bundle_path,
        )

    raw_path = cache_mod.raw_cache_path(archive_sha, root=root, suffix=_url_suffix(url))

    member_paths: dict[str, Path] = {}
    if layer.access == "zip_member":
        for member_name, member_sha in layer.sha256.items():
            if member_name == archive_key:
                continue
            member_paths[member_name] = cache_mod.raw_cache_path(
                member_sha, root=root, suffix=_url_suffix(member_name)
            )

    return CatalogLayerPlan(
        layer=layer, url=url, raw_path=raw_path, member_paths=member_paths
    )


def _is_unsafe_zip_member_name(name: str) -> bool:
    """Reject absolute paths, drive prefixes, and parent traversal. Destinations use
    pinned digests rather than archive names; retain this guard against unsafe future
    extraction changes.
    """
    if not name:
        return True
    if len(name) >= 2 and name[1] == ":" and name[0].isalpha():
        return True  # drive prefix, e.g. "C:evil.tif" / "C:/evil.tif"
    normalized = name.replace("\\", "/")
    if normalized.startswith("/"):
        return True  # absolute path
    return ".." in normalized.split("/")  # path traversal


def _check_zip_member_size(
    member_name: str, info: zipfile.ZipInfo, *, url: str
) -> None:
    """Reject a member above the decompressed-size ceiling from ZIP metadata, before
    reading its bytes.
    """
    if info.file_size > MAX_ZIP_MEMBER_BYTES:
        raise SourceTooLargeError(
            f"{url}!{member_name}",
            limit=MAX_ZIP_MEMBER_BYTES,
            observed=info.file_size,
            # This is a metadata-only check (ZipInfo.file_size, read from
            # the archive's central directory) -- nothing is streamed or
            # decompressed here, so the error's default "streamed and
            # counting" wording would be actively wrong; override it.
            observed_desc="declared in the archive's metadata (no bytes read)",
        )


# Earlier verified members stay cached if a later extraction fails;
# the failing member remains unpublished.
def _extract_zip_members(plan: CatalogLayerPlan, archive_path: Path) -> None:
    """Validate all pinned member names, presence, and declared sizes before extraction.
    Then stream only pinned members into staged, hash-verified publications under
    aggregate and per-member limits.
    """
    require_file_size(
        archive_path,
        resource=f"{plan.layer.layer_id} compressed ZIP",
        max_bytes=MAX_APPROVED_SOURCE_BYTES,
    )
    with zipfile.ZipFile(archive_path) as zf:
        infos = zf.infolist()
        if len(infos) > MAX_ARCHIVE_MEMBERS:
            raise ResourceLimitError(
                f"{plan.layer.layer_id} ZIP members",
                limit=MAX_ARCHIVE_MEMBERS,
                observed=len(infos),
            )
        infos_by_name = {info.filename: info for info in infos}

        to_extract: list[tuple[str, zipfile.ZipInfo, str, Path]] = []
        decoded_budget = ByteBudget(
            f"{plan.layer.layer_id} cumulative ZIP decoded bytes",
            MAX_ZIP_TOTAL_BYTES,
        )
        for member_name, dest in plan.member_paths.items():
            if _is_unsafe_zip_member_name(member_name):
                raise ManifestInvalidError(
                    f"{plan.layer.layer_id}: pinned zip member {member_name!r} "
                    "has an unsafe path (contains '..', is absolute, or has a "
                    "drive prefix); refusing to extract"
                )
            info = infos_by_name.get(member_name)
            if info is None:
                raise ManifestInvalidError(
                    f"{plan.layer.layer_id}: pinned zip member {member_name!r} "
                    f"not found in archive {plan.url}"
                )
            _check_zip_member_size(member_name, info, url=plan.url)
            decoded_budget.consume(info.file_size)
            to_extract.append((member_name, info, plan.layer.sha256[member_name], dest))

        for member_name, info, expected_sha, dest in to_extract:

            def writer(
                staging: Path,
                *,
                _info=info,
                _member=member_name,
                _expected=expected_sha,
            ) -> None:
                with zf.open(_info) as src, staging.open("wb") as out:
                    written = 0
                    while chunk := src.read(1 << 20):
                        written += len(chunk)
                        if written > _info.file_size or written > MAX_ZIP_MEMBER_BYTES:
                            raise ResourceLimitError(
                                f"{plan.layer.layer_id} ZIP member {_member!r}",
                                limit=min(_info.file_size, MAX_ZIP_MEMBER_BYTES),
                                observed=written,
                            )
                        out.write(chunk)
                BaseProvider._verify_or_remove(
                    staging, _expected, f"{plan.url}!{_member}"
                )

            ArtifactPublication.file(dest).publish(
                writer,
                expected_sha256=expected_sha.lower(),
                reuse_if_valid=True,
            )


def _verify_downloaded_bundle_file(
    path: Path,
    *,
    expected_size_bytes: int,
    expected_sha256: str,
    source_url: str,
) -> None:
    if path.is_symlink() or not path.is_file():
        raise ManifestInvalidError(
            f"{source_url}: staged download is not a regular file: {path.name}"
        )
    actual_size = path.stat().st_size
    if actual_size != expected_size_bytes:
        path.unlink(missing_ok=True)
        raise ManifestInvalidError(
            f"{source_url}: size mismatch for {path.name}: expected "
            f"{expected_size_bytes} bytes, got {actual_size}"
        )
    BaseProvider._verify_or_remove(path, expected_sha256.lower(), source_url)


def _bundle_files(
    plan: CatalogLayerPlan,
) -> tuple[Path, list[tuple[str, str, int, str, Path]]]:
    bundle_path = plan.bundle_path
    primary_size_bytes = plan.layer.primary_size_bytes
    if bundle_path is None or primary_size_bytes is None:
        raise ManifestInvalidError(
            f"{plan.layer.layer_id}: invalid sidecar bundle fetch plan"
        )
    if bundle_path.is_symlink():
        raise ManifestInvalidError(
            f"{plan.layer.layer_id}: refusing symlink bundle target {bundle_path}"
        )
    files: list[tuple[str, str, int, str, Path]] = [
        (
            plan.url,
            plan.raw_path.name,
            primary_size_bytes,
            plan.archive_sha256,
            plan.raw_path,
        )
    ]
    files.extend(
        (
            sidecar.url,
            sidecar.filename,
            sidecar.size_bytes,
            sidecar.sha256,
            plan.member_paths[sidecar.filename],
        )
        for sidecar in plan.layer.sidecars
    )
    return bundle_path, files


def _published_bundle_is_valid(plan: CatalogLayerPlan) -> bool:
    """Reuse a bundle only when every declared member passes the same size and hash
    checks as a new download.
    """
    bundle_path, files = _bundle_files(plan)
    if bundle_path.is_symlink() or not bundle_path.is_dir():
        return False
    for url, filename, expected_size, expected_sha, destination in files:
        if destination != bundle_path / filename:
            return False
        try:
            _verify_downloaded_bundle_file(
                destination,
                expected_size_bytes=expected_size,
                expected_sha256=expected_sha,
                source_url=url,
            )
        except (ManifestInvalidError, ChecksumMismatchError, OSError):
            return False
    return True


def _fetch_catalog_bundle(
    plan: CatalogLayerPlan,
    *,
    provider: BaseProvider,
    max_bytes: int,
) -> None:
    """Download and verify every bundle member in staging, then atomically publish the
    complete directory.
    """
    bundle_path, files = _bundle_files(plan)

    transfer_budget = ByteBudget(
        f"{plan.layer.layer_id} cumulative bundle bytes",
        max_bytes,
    )
    for _url, _filename, expected_size, _expected_sha, _destination in files:
        transfer_budget.consume(expected_size)

    def builder(staging: Path) -> None:
        if staging.is_symlink() or not staging.is_dir():
            raise ManifestInvalidError(
                f"{plan.layer.layer_id}: invalid bundle staging directory"
            )
        for url, filename, expected_size, expected_sha, destination in files:
            if destination != bundle_path / filename:
                raise ManifestInvalidError(
                    f"{plan.layer.layer_id}: bundle filename/path mismatch"
                )
            staged_destination = staging / filename
            if isinstance(provider, _SIZE_BOUNDED_PROVIDERS):
                provider.fetch(
                    url,
                    expected_sha,
                    staged_destination,
                    max_bytes=max_bytes,
                )
            else:
                provider.fetch(url, expected_sha, staged_destination)
            _verify_downloaded_bundle_file(
                staged_destination,
                expected_size_bytes=expected_size,
                expected_sha256=expected_sha,
                source_url=url,
            )

    # ArtifactPublication owns a hidden same-parent staging directory, file
    # and parent fsyncs where supported, and one atomic directory promotion.
    # Downloads target that staging directory directly, so no cross-device
    # rename or caller-visible partial bundle is involved.
    bundle_path.parent.mkdir(parents=True, exist_ok=True)
    ArtifactPublication.directory(bundle_path).publish(builder)


def fetch_catalog_layer(
    plan: CatalogLayerPlan,
    *,
    provider_getter: ProviderGetter | None = None,
    env: Mapping[str, str] | None = None,
    est_bytes: int | None = None,
) -> Path:
    """Check placeholder hashes and this layer's own consent before I/O, then fetch
    under the derived finite limit; `CompositionPlan.consented` never substitutes for
    this check. Catalog COG estimates are clamped up to the full declared size
    because this path downloads the whole artifact; v1 curated-source policy remains
    unchanged.
    """
    layer = plan.layer
    if plan.is_placeholder_sha:
        raise ManifestInvalidError(f"{layer.layer_id}: placeholder sha256")

    effective_est_bytes = layer.size_bytes if est_bytes is None else est_bytes
    if layer.access == "cog_remote":
        effective_est_bytes = max(effective_est_bytes, layer.size_bytes)
    elif layer.sidecars:
        effective_est_bytes = max(effective_est_bytes, layer.size_bytes)

    check_layer_consent(
        layer,
        effective_est_bytes,
        env=env,
    )

    if provider_getter is None:
        from srb.terrain.sources.registry import get_provider

        provider_getter = get_provider

    provider = provider_getter(layer.provider)
    max_bytes = layer_max_bytes(layer, env=env)
    # Hold cleanup admission and this content-addressed artifact lock across
    # fetch, bundle validation, and extraction. Order matches v1 acquisition.
    with cache_mod.raw_cache_acquisition_lock(
        _raw_cache_root(plan.raw_path), plan.archive_sha256
    ):
        if layer.sidecars:
            # A previously published bundle whose every member still
            # passes the size + SHA-256 check is reused as-is (under the same
            # lock); anything less triggers a full refetch and republication.
            if not _published_bundle_is_valid(plan):
                _fetch_catalog_bundle(plan, provider=provider, max_bytes=max_bytes)
            return plan.raw_path

        if isinstance(provider, _SIZE_BOUNDED_PROVIDERS):
            # max_bytes is ALWAYS threaded explicitly
            # here, preserving the layer-specific ordinary/consented limit.
            archive_path = provider.fetch(
                plan.url, plan.archive_sha256, plan.raw_path, max_bytes=max_bytes
            )
        else:
            # Other providers do not accept `max_bytes`; the size check below
            # still bounds what they produced.
            archive_path = provider.fetch(plan.url, plan.archive_sha256, plan.raw_path)
        require_file_size(
            archive_path,
            resource=f"{layer.layer_id} source",
            max_bytes=max_bytes,
        )

        if layer.access == "zip_member":
            _extract_zip_members(plan, archive_path)

    return archive_path


def _content_length_bytes(headers) -> int | None:
    raw = headers.get("content-length")
    if raw is None:
        return None
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


__all__ = [
    "LARGE_DOWNLOAD_CONSENT_ENV",
    "MAX_UNAPPROVED_SOURCE_BYTES",
    "MAX_ZIP_MEMBER_BYTES",
    "PLACEHOLDER_SHA256",
    "CachedSourceCheck",
    "CachedSourceStatus",
    "CatalogLayerPlan",
    "CuratedSiteAcquisition",
    "CuratedSourcePlan",
    "SourceUrlCheck",
    "SourceUrlStatus",
    "allowed_hosts_for_layer",
    "check_cached_source",
    "check_cached_sources",
    "check_layer_consent",
    "check_source_url",
    "check_source_urls",
    "fetch_catalog_layer",
    "fetch_curated_site_sources",
    "fetch_curated_source",
    "iter_manifest_sources",
    "large_download_consented",
    "layer_max_bytes",
    "plan_catalog_layer",
    "plan_curated_site_sources",
]
