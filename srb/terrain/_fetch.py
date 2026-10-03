"""Shared HTTP and file:// fetch and SHA-256 helpers for external assets."""

from __future__ import annotations

from pathlib import Path
from urllib.parse import unquote, urljoin, urlparse

from srb.terrain.errors import FetchError, NetworkError, SourceTooLargeError
from srb.utils.digest import sha256_file as _sha256_file
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    require_bounded_int,
    require_file_size,
)


def sha256_file(path: Path) -> str:
    """Return the lowercase SHA-256 hex digest of ``path`` in 1 MB chunks."""
    return _sha256_file(path, chunk_size=1 << 20)


def download_to(
    url: str,
    dst: Path,
    *,
    max_bytes: int = MAX_EXTERNAL_FILE_BYTES,
    allowed_hosts: frozenset[str] | None = None,
    max_redirects: int = 5,
) -> None:
    """Stream a file:// or HTTP(S) URL to ``dst``.

    Raises ``FetchError`` for unsupported schemes or missing local files, and
    ``NetworkError`` for network failures.
    """
    byte_limit = require_bounded_int(
        max_bytes,
        resource="download max_bytes",
        minimum=1,
        maximum=MAX_EXTERNAL_FILE_BYTES,
    )
    redirect_limit = require_bounded_int(
        max_redirects,
        resource="download max_redirects",
        maximum=20,
    )
    parsed = urlparse(url)
    dst.parent.mkdir(parents=True, exist_ok=True)

    if parsed.scheme == "file":
        src = Path(unquote(parsed.path or parsed.netloc))
        try:
            require_file_size(src, resource="local download", max_bytes=byte_limit)
        except ValueError as exc:
            raise FetchError(str(exc)) from exc
        try:
            with src.open("rb") as source, dst.open("wb") as output:
                transferred = 0
                while chunk := source.read(1 << 20):
                    transferred += len(chunk)
                    if transferred > byte_limit:
                        raise SourceTooLargeError(
                            url,
                            limit=byte_limit,
                            observed=transferred,
                        )
                    output.write(chunk)
        except BaseException:
            dst.unlink(missing_ok=True)
            raise
        return

    if parsed.scheme in ("http", "https"):
        import requests

        initial_host = parsed.hostname
        if not initial_host:
            raise FetchError(f"URL has no hostname: {url}")
        hosts = frozenset({initial_host}) if allowed_hosts is None else allowed_hosts
        if initial_host not in hosts:
            raise FetchError(f"initial host is not allowlisted: {initial_host}")
        active_url = url
        response = None
        try:
            redirects = 0
            while True:
                response = requests.get(
                    active_url,
                    stream=True,
                    timeout=30,
                    allow_redirects=False,
                )
                if response.status_code not in (301, 302, 303, 307, 308):
                    break
                location = response.headers.get("Location")
                response.close()
                response = None
                if redirects >= redirect_limit:
                    raise NetworkError(f"too many redirects fetching {active_url}")
                if not location:
                    raise NetworkError(
                        f"redirect with no Location fetching {active_url}"
                    )
                next_url = urljoin(active_url, location)
                target = urlparse(next_url)
                if target.scheme not in ("http", "https"):
                    raise NetworkError(
                        f"redirect target uses unsupported scheme: {next_url}"
                    )
                if parsed.scheme == "https" and target.scheme != "https":
                    raise NetworkError(
                        f"refusing HTTPS downgrade redirect: {active_url} -> {next_url}"
                    )
                if target.hostname not in hosts:
                    raise NetworkError(
                        f"redirect target host is not allowlisted: {next_url}"
                    )
                active_url = next_url
                redirects += 1

            response.raise_for_status()
            declared = response.headers.get("Content-Length")
            if declared is not None:
                try:
                    declared_size = int(declared)
                except ValueError as exc:
                    raise NetworkError(
                        f"invalid Content-Length fetching {active_url}: {declared!r}"
                    ) from exc
                if declared_size < 0:
                    raise NetworkError(
                        f"invalid Content-Length fetching {active_url}: {declared!r}"
                    )
                if declared_size > byte_limit:
                    raise SourceTooLargeError(
                        active_url,
                        limit=byte_limit,
                        observed=declared_size,
                        observed_desc="declared by Content-Length (no body read)",
                    )
            try:
                transferred = 0
                with dst.open("wb") as f:
                    for chunk in response.iter_content(chunk_size=1 << 20):
                        if not chunk:
                            continue
                        transferred += len(chunk)
                        if transferred > byte_limit:
                            raise SourceTooLargeError(
                                active_url,
                                limit=byte_limit,
                                observed=transferred,
                            )
                        f.write(chunk)
            except BaseException:
                dst.unlink(missing_ok=True)
                raise
        except requests.RequestException as e:
            raise NetworkError(f"network error fetching {active_url}: {e}") from e
        finally:
            if response is not None:
                response.close()
        return

    raise FetchError(f"unsupported URL scheme: {parsed.scheme!r}")
