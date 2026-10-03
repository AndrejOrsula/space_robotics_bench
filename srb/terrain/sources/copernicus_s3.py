"""Provider for public Copernicus GLO-30/GLO-90 DEM S3 buckets."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_COPERNICUS_S3_HOSTS: frozenset[str] = frozenset(
    {"copernicus-dem-30m.s3.amazonaws.com", "copernicus-dem-90m.s3.amazonaws.com"}
)


class CopernicusS3Provider(BaseHttpProvider):
    """Fetches Earth Copernicus GLO-30/GLO-90 DEM products from the public S3 buckets."""

    name = "copernicus_s3"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return (
            parsed.scheme in ("http", "https")
            and parsed.hostname in _COPERNICUS_S3_HOSTS
        )
