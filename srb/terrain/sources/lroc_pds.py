"""Provider for direct LROC and ShadowCam PDS downloads."""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider

_LROC_HOSTS: frozenset[str] = frozenset(
    {
        "pds.lroc.im-ldi.com",
        "data.lroc.im-ldi.com",
        "pds.shadowcam.im-ldi.com",
        "pds.mcp.nasa.gov",
    }
)


class LrocPdsProvider(BaseHttpProvider):
    """Fetches Lunar DEM products from the LROC PDS mirror."""

    name = "lroc_pds"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return parsed.scheme in ("http", "https") and parsed.hostname in _LROC_HOSTS
