"""NASA PDS Geosciences Node provider for `pds-geosciences.wustl.edu`.

Fetches payloads as-is. Callers can convert legacy PDS3 `.IMG` files with
`srb.terrain.sources.pds_img.img_to_geotiff` before terrain ingestion.
"""

from __future__ import annotations

from urllib.parse import urlparse

from srb.terrain.sources.base_http import BaseHttpProvider


class PdsGeosciencesProvider(BaseHttpProvider):
    """Fetches Lunar / Martian data from NASA PDS Geosciences Node."""

    name = "pds_geosciences"

    def _accepts_url(self, url: str) -> bool:
        parsed = urlparse(url)
        return (
            parsed.scheme in ("http", "https")
            and parsed.hostname == "pds-geosciences.wustl.edu"
        )
