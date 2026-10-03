"""Common HTTP fetch logic for real providers (USGS, PDS, OpenPlanetary).

Downloads stream into a unique ``ArtifactPublication`` stage.  Retries reuse
that stage but truncate it before each attempt; no destination-side ``.part``
or sidecar is shared between fetches.
"""

from __future__ import annotations

import time
from abc import abstractmethod
from pathlib import Path
from typing import ClassVar
from urllib.parse import urljoin

import requests

from srb.terrain.errors import (
    Http4xxError,
    Http5xxError,
    NetworkError,
    SourceTooLargeError,
)
from srb.terrain.sources.base import BaseProvider
from srb.utils.publication import ArtifactPublication
from srb.utils.resource_budget import MAX_EXTERNAL_FILE_BYTES, ResourceLimitError

DEFAULT_TIMEOUT_S = 60.0
DEFAULT_CHUNK_SIZE = 1 << 20  # 1 MiB


class BaseHttpProvider(BaseProvider):
    """Base for HTTP-backed providers.

    The publication writer owns the complete download stage and holds the
    destination lock for the whole retry loop.  This makes a reader observe
    either the old complete file or the new complete file, while a failed or
    crashed writer can leave no deterministic staging sibling behind.
    """

    name: ClassVar[str] = "http"

    def __init__(
        self,
        *,
        max_retries: int = 3,
        retry_backoff_s: float = 2.0,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        chunk_size: int = DEFAULT_CHUNK_SIZE,
    ) -> None:
        self.max_retries = max_retries
        self.retry_backoff_s = retry_backoff_s
        self.timeout_s = timeout_s
        self.chunk_size = chunk_size

    @abstractmethod
    def _accepts_url(self, url: str) -> bool:
        """Return True if this provider should handle ``url``."""

    def fetch(
        self,
        url: str,
        sha256: str,
        dest: Path,
        *,
        max_bytes: int | None = MAX_EXTERNAL_FILE_BYTES,
    ) -> Path:
        effective_max_bytes = _effective_download_limit(max_bytes)
        publication = ArtifactPublication.file(dest)

        def writer(staging: Path) -> None:
            if not self._accepts_url(url):
                raise NetworkError(f"{self.name} does not accept URL: {url}")

            last_exc: Exception | None = None
            for attempt in range(self.max_retries + 1):
                try:
                    # _stream opens this API-owned stage with ``wb`` for every
                    # attempt, so a transient partial response cannot mix
                    # with the next response.
                    self._stream(
                        url,
                        staging,
                        sha256=sha256,
                        max_bytes=effective_max_bytes,
                    )
                    self._reject_obvious_html_payload(staging, url=url)
                    self._verify_or_remove(staging, sha256, url)
                    return
                except (Http5xxError, NetworkError) as exc:
                    last_exc = exc
                    if attempt < self.max_retries:
                        time.sleep(self.retry_backoff_s * (2**attempt))

            assert last_exc is not None
            raise last_exc

        # The expected digest is checked by FilePublication after the writer
        # validates the terrain-specific error contract. ``reuse_if_valid``
        # preserves the no-network cache-hit behavior.
        return publication.publish(
            writer,
            expected_sha256=sha256.lower(),
            reuse_if_valid=True,
        )

    def _stream(
        self,
        url: str,
        tmp: Path,
        *,
        sha256: str,
        max_bytes: int | None = MAX_EXTERNAL_FILE_BYTES,
    ) -> None:
        """Stream one complete response into ``tmp``.

        ``sha256`` remains part of the seam signature for provider/test
        compatibility; checksum validation belongs to ``fetch`` after the
        response is closed.
        """
        del sha256
        effective_max_bytes = _effective_download_limit(max_bytes)
        active_url = url

        def get_response(request_url: str) -> requests.Response:
            try:
                return requests.get(
                    request_url,
                    stream=True,
                    timeout=self.timeout_s,
                    allow_redirects=False,
                )
            except requests.exceptions.RequestException as exc:
                raise NetworkError(
                    f"connection error fetching {request_url}: {exc}"
                ) from exc

        response = get_response(active_url)
        redirects_followed = 0
        try:
            while response.status_code in (301, 302, 303, 307, 308):
                location = response.headers.get("Location")
                response.close()
                if redirects_followed >= 5:
                    raise NetworkError(f"too many redirects fetching {active_url}")
                if not location:
                    raise NetworkError(f"redirect with no Location for {active_url}")
                next_url = urljoin(active_url, location)
                if not self._accepts_url(next_url):
                    raise NetworkError(
                        f"{self.name} does not accept redirect target: "
                        f"{next_url} (from {active_url})"
                    )
                active_url = next_url
                response = get_response(active_url)
                redirects_followed += 1

            if 400 <= response.status_code < 500:
                raise Http4xxError(f"HTTP {response.status_code} fetching {active_url}")
            if response.status_code >= 500:
                raise Http5xxError(f"HTTP {response.status_code} fetching {active_url}")

            total_bytes = 0
            with tmp.open("wb") as output:
                for chunk in response.iter_content(chunk_size=self.chunk_size):
                    if not chunk:
                        continue
                    total_bytes += len(chunk)
                    if total_bytes > effective_max_bytes:
                        raise SourceTooLargeError(
                            active_url,
                            limit=effective_max_bytes,
                            observed=total_bytes,
                        )
                    output.write(chunk)
        except requests.exceptions.RequestException as exc:
            raise NetworkError(f"stream error fetching {active_url}: {exc}") from exc
        finally:
            response.close()


def _effective_download_limit(max_bytes: int | None) -> int:
    if max_bytes is None:
        return MAX_EXTERNAL_FILE_BYTES
    if isinstance(max_bytes, bool) or not isinstance(max_bytes, int):
        raise TypeError("max_bytes must be a positive integer or None")
    if max_bytes <= 0:
        raise ValueError("max_bytes must be positive")
    if max_bytes > MAX_EXTERNAL_FILE_BYTES:
        raise ResourceLimitError(
            "HTTP download limit",
            limit=MAX_EXTERNAL_FILE_BYTES,
            observed=max_bytes,
        )
    return max_bytes
