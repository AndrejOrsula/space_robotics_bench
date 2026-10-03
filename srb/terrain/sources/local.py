"""Local provider: copies from ``file://`` URLs.

Used for tests, fixtures, and user-supplied DEMs (the user passes
`dem_path=...` and the pipeline rewrites it to a `file://` URL).
"""

from __future__ import annotations

from pathlib import Path
from urllib.parse import unquote, urlparse

from srb.terrain.errors import ChecksumMismatchError, NetworkError
from srb.terrain.sources.base import BaseProvider
from srb.utils.publication import ArtifactPublication
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    ResourceLimitError,
    require_file_size,
)


class LocalProvider(BaseProvider):
    name = "local"

    def fetch(
        self,
        url: str,
        sha256: str,
        dest: Path,
        *,
        max_bytes: int | None = MAX_EXTERNAL_FILE_BYTES,
    ) -> Path:
        # Reuse a valid cached payload.
        if dest.is_file() and self._hash_file(dest) == sha256.lower():
            return dest

        if max_bytes is None:
            max_bytes = MAX_EXTERNAL_FILE_BYTES
        parsed = urlparse(url)
        if parsed.scheme not in ("file", ""):
            raise NetworkError(f"LocalProvider only handles file:// URLs, got {url}")

        raw_path = parsed.path or parsed.netloc
        # ``file://`` URLs percent-encode reserved characters (``Path.as_uri``).
        src = Path(unquote(raw_path) if parsed.scheme == "file" else raw_path)
        if not src.is_file():
            raise NetworkError(f"local source not found: {src}")
        # Reject oversized sources before hashing or copying them.
        require_file_size(
            src, resource=f"local terrain source {src}", max_bytes=max_bytes
        )

        self._reject_obvious_html_payload(src, url=url)
        actual = self._hash_file(src)
        if actual != sha256.lower():
            raise ChecksumMismatchError(url=url, expected=sha256, actual=actual)

        # Copy caller-owned input into the publication stage; never mutate it.
        def writer(staging: Path) -> None:
            _copy_bounded(src, staging, max_bytes=max_bytes)
            self._verify_or_remove(staging, sha256, url)

        ArtifactPublication.file(dest).publish(
            writer,
            expected_sha256=sha256.lower(),
            reuse_if_valid=True,
        )
        return dest


def _copy_bounded(src: Path, dst: Path, *, max_bytes: int) -> None:
    """Copy ``src`` while enforcing ``max_bytes`` in case it grew after checks."""
    copied = 0
    with src.open("rb") as reader, dst.open("wb") as writer:
        for chunk in iter(lambda: reader.read(1 << 20), b""):
            copied += len(chunk)
            if copied > max_bytes:
                raise ResourceLimitError(
                    f"local terrain source {src}", limit=max_bytes, observed=copied
                )
            writer.write(chunk)
