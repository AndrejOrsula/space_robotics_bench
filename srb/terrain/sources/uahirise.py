"""Provider for public University of Arizona HiRISE products."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_UAHIRISE_HOSTS: frozenset[str] = frozenset(
    {"hirise.lpl.arizona.edu", "www.uahirise.org", "uahirise.org"}
)


class UahiriseProvider(BaseHttpProvider):
    """Fetches Mars HiRISE DTM/ortho products from UAHiRISE."""

    name = "uahirise"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return parsed.scheme in ("http", "https") and parsed.hostname in _UAHIRISE_HOSTS
