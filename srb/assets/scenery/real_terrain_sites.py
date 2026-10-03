"""Generate one body-prefixed ``RealTerrain`` shortcut per bundled site."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml

from srb.assets.scenery.real_terrain import RealTerrain
from srb.core.domain import Domain
from srb.terrain.schema import Manifest


def _camel(snake: str) -> str:
    """`apollo17` -> `Apollo17`, `jezero_v2` -> `JezeroV2`."""
    return "".join(part.capitalize() for part in snake.split("_"))


_BODY_PREFIX_AND_DOMAIN: dict[str, tuple[str, Domain]] = {
    "moon": ("LunarSite", Domain.MOON),
    "mars": ("MartianSite", Domain.MARS),
    "earth": ("EarthSite", Domain.EARTH),
}


def _make_shortcut(site_key: str, body: str) -> type[RealTerrain]:
    prefix, domain = _BODY_PREFIX_AND_DOMAIN[body]
    name = f"{prefix}{_camel(site_key)}"

    cls = type(
        name,
        (RealTerrain,),
        {
            "__module__": __name__,
            "DOMAINS": (domain,),
            "body": body,
            "site": site_key,
            "__annotations__": {
                "body": Literal["moon", "mars", "earth"],
                "site": str,
            },
        },
    )
    return cls


# Load the bundled manifest directly: runtime manifest paths may be overridden,
# but these import-time shortcuts always represent shipped sites.
_MANIFEST_FILE = Path(__file__).resolve().parents[2] / "terrain" / "manifest.yaml"
_manifest = Manifest.model_validate(yaml.safe_load(_MANIFEST_FILE.read_text()))
for _site_key, _site_entry in _manifest.sites.items():
    _cls = _make_shortcut(_site_key, _site_entry.body)
    globals()[_cls.__name__] = _cls

# Export only generated shortcuts.
__all__ = [
    name
    for name in globals()
    if name.startswith(("LunarSite", "MartianSite", "EarthSite"))
]

# Avoid leaking loop locals; pop also handles an empty manifest.
for _name in ("_site_key", "_site_entry", "_cls", "_manifest"):
    globals().pop(_name, None)
del _name
