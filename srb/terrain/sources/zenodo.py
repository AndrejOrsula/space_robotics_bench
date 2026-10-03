"""Provider for data products hosted on Zenodo."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_ZENODO_HOSTS: frozenset[str] = frozenset({"zenodo.org", "www.zenodo.org"})


class ZenodoProvider(BaseHttpProvider):
    """Fetches archived data products hosted on Zenodo."""

    name = "zenodo"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return parsed.scheme in ("http", "https") and parsed.hostname in _ZENODO_HOSTS
