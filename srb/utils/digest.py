"""SHA-256 helpers for in-memory payloads and streamed files.

File hashing follows symlinks and has no size limit. For untrusted files, use
``srb.utils.publication.sha256_file``, which rejects symlinks and caps input.
"""

from __future__ import annotations

import hashlib
from pathlib import Path


def sha256_bytes(data: bytes | bytearray | memoryview) -> str:
    """Return the SHA-256 hex digest of an in-memory payload."""
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path | str, *, chunk_size: int = 1 << 20) -> str:
    """Return the SHA-256 hex digest of ``path``, streaming in 1 MiB blocks.

    Follows symlinks and has no size limit; use the publication helper for
    untrusted files.
    """
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()
