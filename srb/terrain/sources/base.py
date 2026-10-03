"""Base class and shared helpers for terrain source providers."""

from __future__ import annotations

import hashlib
from abc import ABC, abstractmethod
from pathlib import Path
from typing import ClassVar

from srb.terrain.errors import ChecksumMismatchError, InvalidSourcePayloadError


class BaseProvider(ABC):
    """Abstract base for source providers."""

    name: ClassVar[str] = "base"

    @abstractmethod
    def fetch(self, url: str, sha256: str, dest: Path) -> Path:
        """Fetch ``url`` to ``dest`` and verify against ``sha256``."""

    # ------------------------------------------------------------------
    # helpers shared by providers

    @staticmethod
    def _hash_file(path: Path) -> str:
        h = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()

    @classmethod
    def _verify_or_remove(cls, dest: Path, expected: str, url: str) -> None:
        actual = cls._hash_file(dest)
        if actual != expected.lower():
            try:
                dest.unlink()
            finally:
                raise ChecksumMismatchError(url=url, expected=expected, actual=actual)

    @classmethod
    def _reject_obvious_html_payload(cls, path: Path, *, url: str) -> None:
        """Reject catalog/error HTML saved under a terrain-raster filename."""
        try:
            with path.open("rb") as f:
                head = f.read(512)
        except OSError:
            return
        probe = head.lstrip().lower()
        if probe.startswith((b"<!doctype html", b"<html")) or b"<html" in probe[:128]:
            raise InvalidSourcePayloadError(
                f"source {url} returned HTML, not terrain raster bytes"
            )
