"""Symbolic-name → provider registry.

Used by the orchestrator to resolve a manifest's `provider:` field. Providers
are singletons (no per-call state).
"""

from __future__ import annotations

from srb.terrain.sources.base import BaseProvider
from srb.terrain.sources.copernicus_s3 import CopernicusS3Provider
from srb.terrain.sources.imbrium import ImbriumProvider
from srb.terrain.sources.local import LocalProvider
from srb.terrain.sources.lroc_pds import LrocPdsProvider
from srb.terrain.sources.openplanetary import OpenPlanetaryProvider
from srb.terrain.sources.pds_geosciences import PdsGeosciencesProvider
from srb.terrain.sources.pgda import PgdaProvider
from srb.terrain.sources.uahirise import UahiriseProvider
from srb.terrain.sources.usgs_3dep import Usgs3DepProvider
from srb.terrain.sources.usgs_astro import UsgsAstroProvider
from srb.terrain.sources.zenodo import ZenodoProvider

_REGISTRY: dict[str, BaseProvider] = {
    "local": LocalProvider(),
    "lroc_pds": LrocPdsProvider(),
    "usgs_astro": UsgsAstroProvider(),
    "pds_geosciences": PdsGeosciencesProvider(),
    "openplanetary": OpenPlanetaryProvider(),
    "pgda": PgdaProvider(),
    "zenodo": ZenodoProvider(),
    "imbrium": ImbriumProvider(),
    "uahirise": UahiriseProvider(),
    "copernicus_s3": CopernicusS3Provider(),
    "usgs_3dep": Usgs3DepProvider(),
}

KNOWN_PROVIDERS: frozenset[str] = frozenset(_REGISTRY.keys())


def get_provider(name: str) -> BaseProvider:
    if name not in _REGISTRY:
        raise KeyError(f"unknown provider: {name!r}; known: {sorted(KNOWN_PROVIDERS)}")
    return _REGISTRY[name]
