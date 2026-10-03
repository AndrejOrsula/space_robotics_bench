"""Provider for NASA GSFC's PGDA lunar products."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_PGDA_HOSTS: frozenset[str] = frozenset({"pgda.gsfc.nasa.gov"})


class PgdaProvider(BaseHttpProvider):
    """Fetches lunar polar DEM/ortho products from NASA GSFC PGDA."""

    name = "pgda"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return parsed.scheme in ("http", "https") and parsed.hostname in _PGDA_HOSTS
