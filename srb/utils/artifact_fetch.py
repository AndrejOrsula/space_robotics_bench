"""Shared artifact download, verification, and cache publication.

Callers provide the artifact, errors, and bound downloader. Source failures may
try ``fallback_urls``; locally classified errors and hash mismatches are fatal.
"""

from __future__ import annotations

import errno
import inspect
import os
import tempfile
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Protocol
from urllib.parse import urlparse

from srb.utils.digest import sha256_file
from srb.utils.publication import FilePublication, PublicationIntegrityError
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    ResourceLimitError,
    require_bounded_int,
    require_file_size,
)

# Markers for unpinned defaults that :func:`fetch_artifact` must reject.
_PLACEHOLDER_URL_PREFIX = "srb-placeholder://"
_PLACEHOLDER_SHA256 = "0" * 64
_DEFAULT_DOWNLOAD_TIMEOUT_S = 30.0
# Hub downloads redirect to region-specific CDN hosts under this domain.
_HUGGINGFACE_REDIRECT_DOMAIN = ".hf.co"


class ArtifactLike(Protocol):
    """Minimum structural interface for fetchable artifacts.

    Caller dataclasses satisfy this by duck typing. Kind-specific fields belong
    to those dataclasses and are read only by code that knows the artifact kind.
    """

    @property
    def name(self) -> str: ...

    @property
    def url(self) -> str: ...

    @property
    def sha256(self) -> str: ...


def is_placeholder(artifact: ArtifactLike) -> bool:
    """True if either ``url`` or ``sha256`` carries the placeholder marker."""
    return (
        artifact.url.startswith(_PLACEHOLDER_URL_PREFIX)
        or artifact.sha256 == _PLACEHOLDER_SHA256
    )


def compute_sha256(path: Path, *, chunk: int = 1 << 20) -> str:
    """Stream a file through SHA256, returning the hex digest."""
    return sha256_file(path, chunk_size=chunk)


def _urllib_download(
    url: str,
    target: Path,
    *,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
) -> None:
    """Download via urllib to an owned file descriptor.

    Callers pass their bound reference to preserve caller-module patch points.
    """
    import urllib.error
    import urllib.request

    byte_limit = require_bounded_int(
        max_bytes,
        resource="artifact download max_bytes",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    parsed = urlparse(url)
    if parsed.scheme not in ("http", "https") or not parsed.hostname:
        raise ValueError(f"artifact URL must be absolute HTTP(S): {url}")
    initial_scheme = parsed.scheme
    initial_host = parsed.hostname
    from_hub = initial_host == "huggingface.co"

    class SafeRedirectHandler(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, req, fp, code, msg, headers, newurl):
            target_url = urlparse(newurl)
            if target_url.scheme not in ("http", "https"):
                raise urllib.error.HTTPError(
                    newurl, code, "redirect uses unsupported scheme", headers, fp
                )
            if initial_scheme == "https" and target_url.scheme != "https":
                raise urllib.error.HTTPError(
                    newurl, code, "refusing HTTPS downgrade redirect", headers, fp
                )
            target_host = target_url.hostname or ""
            if target_host != initial_host and not (
                from_hub and target_host.endswith(_HUGGINGFACE_REDIRECT_DOMAIN)
            ):
                raise urllib.error.HTTPError(
                    newurl, code, "redirect target host is not allowlisted", headers, fp
                )
            redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
            if redirected is not None and not _is_huggingface_url(newurl):
                # Do not forward the Hub token to CDN or altered Hub origins.
                redirected.remove_header("Authorization")
            return redirected

    headers = {}
    if _is_huggingface_url(url):
        token = _huggingface_token()
        if token is not None:
            headers["Authorization"] = f"Bearer {token}"

    req = urllib.request.Request(url, headers=headers)
    opener = urllib.request.build_opener(SafeRedirectHandler())
    try:
        with opener.open(req, timeout=_DEFAULT_DOWNLOAD_TIMEOUT_S) as src:
            declared = src.headers.get("Content-Length")
            if declared is not None:
                try:
                    declared_size = int(declared)
                except ValueError as exc:
                    raise ValueError(
                        f"invalid artifact Content-Length: {declared!r}"
                    ) from exc
                if declared_size < 0:
                    raise ValueError(f"invalid artifact Content-Length: {declared!r}")
                if declared_size > byte_limit:
                    raise ResourceLimitError(
                        "artifact download",
                        limit=byte_limit,
                        observed=declared_size,
                    )
            transferred = 0
            with open(target, "wb") as sink:
                while buf := src.read(1 << 20):
                    transferred += len(buf)
                    if transferred > byte_limit:
                        raise ResourceLimitError(
                            "artifact download",
                            limit=byte_limit,
                            observed=transferred,
                        )
                    sink.write(buf)
                sink.flush()
                os.fsync(sink.fileno())
    except BaseException:
        Path(target).unlink(missing_ok=True)
        raise


def _is_huggingface_url(url: str) -> bool:
    # Only HTTPS requests to the Hub may carry the token; a plain-HTTP URL
    # would send it in cleartext.
    parsed = urlparse(url)
    if parsed.scheme != "https" or parsed.hostname != "huggingface.co":
        return False
    # Do not send credentials to a URL whose authority embeds its own userinfo.
    if parsed.username is not None or parsed.password is not None:
        return False
    try:
        return parsed.port in (None, 443)
    except ValueError:
        return False


def _huggingface_token() -> str | None:
    """Return HF token from env or standard local CLI cache, if available."""
    for env_name in ("HF_TOKEN", "HUGGING_FACE_HUB_TOKEN"):
        token = os.environ.get(env_name, "").strip()
        if token:
            return token

    for path in _huggingface_token_paths():
        try:
            token = path.expanduser().read_text().strip()
        except OSError:
            continue
        if token:
            return token
    return None


def _huggingface_token_paths() -> tuple[Path, ...]:
    paths: list[Path] = []
    if hf_home := os.environ.get("HF_HOME"):
        paths.append(Path(hf_home) / "token")
    if xdg_cache := os.environ.get("XDG_CACHE_HOME"):
        paths.append(Path(xdg_cache) / "huggingface" / "token")
    paths.extend(
        (
            Path.home() / ".cache" / "huggingface" / "token",
            Path.home() / ".huggingface" / "token",
        )
    )
    return tuple(dict.fromkeys(paths))


def _is_local_download_error(error: Exception) -> bool:
    """Return whether retrying another source cannot fix a download failure.

    Invalid input, resource limits, and local filesystem errors are fatal;
    other failures are treated as source-specific and may be retried.
    """
    if isinstance(error, (ResourceLimitError, ValueError)):
        return True
    if isinstance(
        error,
        (PermissionError, IsADirectoryError, NotADirectoryError, FileExistsError),
    ):
        return True
    return isinstance(error, OSError) and error.errno in {
        errno.EACCES,
        errno.EPERM,
        errno.ENOSPC,
        errno.EROFS,
        errno.EDQUOT,
    }


class PlaceholderError(RuntimeError):
    """Base for unpinned-artifact errors; callers supply kind-specific messages."""

    def __init__(self, artifact: ArtifactLike, message: str) -> None:
        self.artifact = artifact
        super().__init__(message)


class HashMismatchError(RuntimeError):
    """Base for SHA256 mismatch errors; callers supply kind-specific messages."""

    def __init__(
        self,
        *,
        expected: str,
        actual: str,
        path: Path,
        message: str,
    ) -> None:
        self.expected = expected
        self.actual = actual
        self.path = path
        super().__init__(message)


def fetch_artifact[ArtifactT: ArtifactLike](
    artifact: ArtifactT,
    *,
    cache_dir: Path,
    downloader: Callable[[str, Path], None],
    force: bool,
    placeholder_exc: Callable[[ArtifactT], PlaceholderError],
    mismatch_exc: Callable[..., HashMismatchError],
    fallback_urls: Sequence[str] = (),
    lock_timeout_s: float = 30.0,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
) -> Path:
    """Fetch and verify an artifact in ``cache_dir``, returning its cached path.

    A matching cache entry skips download. A mismatch never replaces the published file,
    and each filename is claimed by its first verified SHA256. Placeholder validation
    precedes cache lookup.

    Try ``artifact.url`` first, then ``fallback_urls`` after source-specific failures.
    Failures classified as local are fatal. Verify the first successful download once; a
    hash mismatch is fatal and never triggers another source.

    Callers provide all keyword arguments except ``fallback_urls``.
    """
    if is_placeholder(artifact):
        raise placeholder_exc(artifact)
    byte_limit = require_bounded_int(
        max_bytes,
        resource="artifact max_bytes",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    try:
        downloader_parameters = inspect.signature(downloader).parameters.values()
    except (TypeError, ValueError):
        downloader_accepts_byte_limit = False
    else:
        downloader_accepts_byte_limit = any(
            (
                parameter.name == "max_bytes"
                and parameter.kind is not inspect.Parameter.POSITIONAL_ONLY
            )
            or parameter.kind is inspect.Parameter.VAR_KEYWORD
            for parameter in downloader_parameters
        )

    cache = cache_dir.expanduser()
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / artifact.name
    # Reject names whose resolved target escapes the cache directory.
    if not target.resolve().is_relative_to(cache.resolve()):
        raise ValueError(f"artifact name {artifact.name!r} escapes the cache directory")

    def download(staging: Path) -> None:
        download_error: Exception | None = None
        for url in (artifact.url, *fallback_urls):
            temp_fd, temp_name = tempfile.mkstemp(
                prefix=f".{staging.name}.download.",
                dir=staging.parent,
            )
            os.close(temp_fd)
            temporary = Path(temp_name)
            try:
                try:
                    if downloader_accepts_byte_limit:
                        downloader(  # type: ignore[call-arg]
                            url, temporary, max_bytes=byte_limit
                        )
                    else:
                        downloader(url, temporary)
                except Exception as exc:  # noqa: BLE001 - fallback contract
                    if _is_local_download_error(exc):
                        raise
                    download_error = exc
                    continue
                require_file_size(
                    temporary,
                    resource=f"artifact {artifact.name!r}",
                    max_bytes=byte_limit,
                )
                os.replace(temporary, staging)
                return
            finally:
                try:
                    temporary.unlink()
                except OSError:
                    pass
        assert download_error is not None
        raise download_error

    try:
        return FilePublication(target, lock_timeout_s=lock_timeout_s).publish(
            download,
            expected_sha256=artifact.sha256,
            reuse_if_valid=not force,
            immutable_identity=True,
        )
    except PublicationIntegrityError as exc:
        raise mismatch_exc(
            expected=exc.expected_sha256,
            actual=exc.actual_sha256,
            path=target,
        ) from exc


__all__ = [
    "_PLACEHOLDER_SHA256",
    "_PLACEHOLDER_URL_PREFIX",
    "ArtifactLike",
    "HashMismatchError",
    "PlaceholderError",
    "_urllib_download",
    "compute_sha256",
    "fetch_artifact",
    "is_placeholder",
]
