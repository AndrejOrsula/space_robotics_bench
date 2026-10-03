"""Provider for public USGS Astrogeology products."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_USGS_HOSTS: frozenset[str] = frozenset(
    {
        "asc-astropedia.s3.us-west-2.amazonaws.com",
        "asc-pds-services.s3.us-west-2.amazonaws.com",
        "astrogeo-ard.s3.us-west-2.amazonaws.com",
        "astrogeology.usgs.gov",
        "pilot.wr.usgs.gov",
        "planetarymaps.usgs.gov",
    }
)


class UsgsAstroProvider(BaseHttpProvider):
    """Fetches DTMs / orthos from USGS Astrogeology services."""

    name = "usgs_astro"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return parsed.scheme in ("http", "https") and parsed.hostname in _USGS_HOSTS
