"""Provider for OpenPlanetary hosts and its public S3 mirror."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider


class OpenPlanetaryProvider(BaseHttpProvider):
    """Fetches data from OpenPlanetary services and their S3 mirror."""

    name = "openplanetary"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        if parsed.scheme not in ("http", "https"):
            return False
        host = parsed.hostname or ""
        return (
            host == "openplanetary.org"
            or host.endswith(".openplanetary.org")
            or host == "openplanetary.co"
            or host.endswith(".openplanetary.co")
            or host == "openplanetary.s3.amazonaws.com"
        )
