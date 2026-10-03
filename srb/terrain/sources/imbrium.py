"""Imbrium's MIT LOLA archive provider.

Accepts `imbrium.mit.edu` over HTTPS only. PGDA links to this archive over
HTTP, but TLS is available; the HTTPS-only policy also rejects downgrade
redirects because `BaseHttpProvider` checks every redirect target.
"""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_IMBRIUM_HOSTS: frozenset[str] = frozenset({"imbrium.mit.edu"})


class ImbriumProvider(BaseHttpProvider):
    """Fetches LOLA archive products from `imbrium.mit.edu`, HTTPS only."""

    name = "imbrium"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        # `"https"` only -- see this module's docstring. Not a typo for the
        # `("http", "https")` tuple every sibling provider uses.
        return parsed.scheme == "https" and parsed.hostname in _IMBRIUM_HOSTS
