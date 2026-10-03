"""Source providers — fetch raw inputs (DEM, ortho) into the content-addressed cache."""

from srb.terrain.sources.acquisition import (
    LARGE_DOWNLOAD_CONSENT_ENV,
    MAX_UNAPPROVED_SOURCE_BYTES,
    MAX_ZIP_MEMBER_BYTES,
    CachedSourceCheck,
    CachedSourceStatus,
    CatalogLayerPlan,
    CuratedSiteAcquisition,
    CuratedSourcePlan,
    SourceUrlCheck,
    SourceUrlStatus,
    check_cached_source,
    check_cached_sources,
    check_layer_consent,
    check_source_url,
    check_source_urls,
    fetch_catalog_layer,
    fetch_curated_site_sources,
    fetch_curated_source,
    iter_manifest_sources,
    large_download_consented,
    layer_max_bytes,
    plan_catalog_layer,
    plan_curated_site_sources,
)
from srb.terrain.sources.base import BaseProvider
from srb.terrain.sources.copernicus_s3 import CopernicusS3Provider
from srb.terrain.sources.imbrium import ImbriumProvider
from srb.terrain.sources.local import LocalProvider
from srb.terrain.sources.lroc_pds import LrocPdsProvider
from srb.terrain.sources.openplanetary import OpenPlanetaryProvider
from srb.terrain.sources.pds_geosciences import PdsGeosciencesProvider
from srb.terrain.sources.pgda import PgdaProvider
from srb.terrain.sources.registry import KNOWN_PROVIDERS, get_provider
from srb.terrain.sources.uahirise import UahiriseProvider
from srb.terrain.sources.usgs_3dep import Usgs3DepProvider
from srb.terrain.sources.usgs_astro import UsgsAstroProvider
from srb.terrain.sources.zenodo import ZenodoProvider

__all__ = [
    "BaseProvider",
    "CachedSourceCheck",
    "CachedSourceStatus",
    "CatalogLayerPlan",
    "CopernicusS3Provider",
    "CuratedSiteAcquisition",
    "CuratedSourcePlan",
    "ImbriumProvider",
    "LARGE_DOWNLOAD_CONSENT_ENV",
    "MAX_UNAPPROVED_SOURCE_BYTES",
    "MAX_ZIP_MEMBER_BYTES",
    "LocalProvider",
    "LrocPdsProvider",
    "OpenPlanetaryProvider",
    "PdsGeosciencesProvider",
    "PgdaProvider",
    "UahiriseProvider",
    "Usgs3DepProvider",
    "ZenodoProvider",
    "SourceUrlCheck",
    "SourceUrlStatus",
    "UsgsAstroProvider",
    "KNOWN_PROVIDERS",
    "check_cached_source",
    "check_cached_sources",
    "check_layer_consent",
    "check_source_url",
    "check_source_urls",
    "fetch_catalog_layer",
    "fetch_curated_site_sources",
    "fetch_curated_source",
    "get_provider",
    "iter_manifest_sources",
    "large_download_consented",
    "layer_max_bytes",
    "plan_catalog_layer",
    "plan_curated_site_sources",
]
