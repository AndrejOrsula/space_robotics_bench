"""Provider for public USGS 3DEP DEM products on S3."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_USGS_3DEP_HOSTS: frozenset[str] = frozenset({"prd-tnm.s3.amazonaws.com"})


class Usgs3DepProvider(BaseHttpProvider):
    """Fetches Earth 3DEP DEM products from the public USGS 3DEP S3 bucket."""

    name = "usgs_3dep"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return (
            parsed.scheme in ("http", "https") and parsed.hostname in _USGS_3DEP_HOSTS
        )
