"""Isaac Lab asset wrapper for baked real terrain and companion assets."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar, Literal, Optional, Sequence

from pydantic import Field

from srb.core.asset import AssetBaseCfg, AssetVariant, Terrain
from srb.core.domain import Domain
from srb.core.sim import CollisionPropertiesCfg, UsdFileCfg
from srb.terrain import bake_or_load
from srb.terrain.augmentation.presets import get_augment_preset
from srb.terrain.augmentation.simforge_ext import build_real_terrain_augment_cfg
from srb.terrain.companion_scene import (
    attach_terrain_companions_arbitrated,
    build_real_terrain_companion_assets,
)
from srb.terrain.surface_source import (
    build_real_terrain_runtime_asset,
    terrain_auto_fetch_default,
    validate_real_terrain_selection,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg
    from srb.core.ephemeris.env_support import TerrainEphemerisFacts


def _make_real_terrain_multi_asset_spawn(spawn_cfgs: list[Any]) -> Any:
    """Wrap per-patch USD configs in Layer B for Isaac-free surface policy.

    Set the compatible spawner eagerly for direct consumers; it accepts both
    env-scoped template paths and concrete stacked paths.
    """
    from isaaclab.sim.spawners.wrappers import MultiAssetSpawnerCfg

    from srb.core.sim.spawners.multi_asset_compat import spawn_multi_asset_compat

    cfg = MultiAssetSpawnerCfg(assets_cfg=spawn_cfgs, random_choice=False)
    cfg.func = spawn_multi_asset_compat
    return cfg


def build_real_terrain_asset_cfg(
    *,
    body: Literal["moon", "mars", "earth"],
    site: str | None,
    auto_fetch: bool,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    size_m: float | None = None,
) -> Any:
    """Build the Isaac-dependent asset config after surface policy admits it."""
    return RealTerrain(
        body=body,
        site=site,
        auto_fetch=auto_fetch,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        size_m=size_m,
    ).asset_cfg


class RealTerrain(Terrain, scenery_metaclass=True):
    """Parametric baked terrain with optional companion assets.

    Baked USD uses ``patch_size_m`` and ``lod``; inherited SimForge-only
    ``scale``, ``texture_resolution``, ``density``, and ``flat_area_size`` do
    not affect it. SimForge augmentation can consume those inherited fields.
    """

    DOMAINS: ClassVar[Sequence[Domain]] = (Domain.MOON, Domain.MARS, Domain.EARTH)
    ASSET_VARIANT: ClassVar[AssetVariant] = AssetVariant.DATASET

    body: Literal["moon", "mars", "earth"]
    site: Optional[str] = None
    dem_path: Optional[Path] = None
    site_name: Optional[str] = None
    attribution: Optional[str] = None

    # Region selection is exclusive with site/DEM selection and requires all three.
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    size_m: Optional[float] = None
    # Included in the v2 cache key and fusion input; v1 bakes, including stack sites, fail.
    deshade: bool = False

    # Affects region admission and the terrain cache key.
    allow_synthetic: bool = False

    # Requires a preset with nonzero crater density.
    augment_stamp: bool = False

    # Region form only: seed generates centers, or locations supplies them.
    # Patches are assigned uniformly across environments and remain on stage.
    num_locations: int = 1
    locations: Optional[list[tuple[float, float]]] = None
    location_spread_m: Optional[float] = None

    patch: str = "default"
    patch_size_m: Optional[float] = None
    patch_center_xy_m: Optional[tuple[float, float]] = None

    lod: Literal["low", "medium", "high"] = "medium"
    seed: int = 0
    use_ortho: bool = True
    materials_version: Optional[int] = None

    # "database" uses recorded rock transforms; "simforge" uses SimForge scatter.
    augment: Literal["none", "simforge", "database"] = "none"
    augment_preset: Optional[str] = None
    # Ignored outside database augmentation.
    augment_max_rocks: int = 256
    sky: Optional[str] = "auto"
    lighting: Optional[str] = "auto"
    rocks: Optional[str] = "auto"

    auto_fetch: bool = Field(default_factory=terrain_auto_fetch_default)
    preset_name: Optional[str] = None
    attach_companions_enabled: bool = True

    asset_cfg: Any = None

    @classmethod
    def declines_instantiation(cls) -> bool:
        """Reject known-bad sites before construction can fetch or bake."""
        fields = getattr(cls, "model_fields", None)

        def default(name: str) -> object:
            if fields is not None and name in fields:
                return fields[name].default
            return getattr(cls, name, None)

        site = default("site")
        body = default("body")
        if not isinstance(site, str) or body not in ("moon", "mars", "earth"):
            return False

        from srb.terrain.availability import (
            TerrainAvailabilityStatus,
            assess_terrain_site,
        )
        from srb.terrain.errors import TerrainError

        try:
            availability = assess_terrain_site(site, body=body)
        except (TerrainError, ValueError):
            return False
        return availability.status == TerrainAvailabilityStatus.KNOWN_BAD_MANIFEST

    def model_post_init(self, __context: Any) -> None:
        validate_real_terrain_selection(
            body=self.body,
            lod=self.lod,
            augment=self.augment,
            site=self.site,
            dem_path=self.dem_path,
            lat_deg=self.lat_deg,
            lon_deg=self.lon_deg,
            size_m=self.size_m,
            patch_size_m=self.patch_size_m,
            patch_center_xy_m=self.patch_center_xy_m,
            num_locations=self.num_locations,
            locations=self.locations,
            location_spread_m=self.location_spread_m,
        )

        super().model_post_init(__context)

        self.asset_cfg = build_real_terrain_runtime_asset(
            body=self.body,
            site=self.site,
            dem_path=self.dem_path,
            site_name=self.site_name,
            attribution=self.attribution,
            patch=self.patch,
            patch_size_m=self.patch_size_m,
            patch_center_xy_m=self.patch_center_xy_m,
            lod=self.lod,
            seed=self.seed,
            use_ortho=self.use_ortho,
            materials_version=self.materials_version,
            auto_fetch=self.auto_fetch,
            preset_name=self.preset_name,
            bake=bake_or_load,
            make_asset_cfg=AssetBaseCfg,
            make_usd_spawn=UsdFileCfg,
            make_collision_props=CollisionPropertiesCfg,
            lat_deg=self.lat_deg,
            lon_deg=self.lon_deg,
            size_m=self.size_m,
            deshade=self.deshade,
            allow_synthetic=self.allow_synthetic,
            augment_stamp=self.augment_stamp,
            augment_preset=self.augment_preset,
            num_locations=self.num_locations,
            locations=self.locations,
            location_spread_m=self.location_spread_m,
            make_multi_asset_spawn=_make_real_terrain_multi_asset_spawn,
        )

    @property
    def primary_baked_usd_path(self) -> str:
        """Return the primary patch USD; multi-location terrain uses variant child 0."""
        spawn = self.asset_cfg.spawn
        usd_path = getattr(spawn, "usd_path", None)
        if usd_path is None:
            usd_path = spawn.assets_cfg[0].usd_path
        return str(usd_path)

    def ephemeris_site_facts(self) -> "TerrainEphemerisFacts | None":
        """Return terrain georeference facts for environment ephemeris, if known."""
        region_coords: tuple[float, float] | None = None
        if self.site is None:
            if self.lat_deg is not None and self.lon_deg is not None:
                region_coords = (self.lat_deg, self.lon_deg)
            elif self.locations:
                region_coords = self.locations[0]
            else:
                return None

        # Multi-location spawns keep paths on their children; the primary patch
        # carries the persisted grid convergence.
        baked_usd = (
            self.primary_baked_usd_path
            if getattr(self.asset_cfg, "spawn", None) is not None
            else None
        )

        from srb.core.ephemeris.env_support import (
            TerrainEphemerisFacts,
            region_terrain_ephemeris_facts,
        )
        from srb.terrain.lights.ephemeris_bridge import _read_north_convergence

        north_convergence_deg = _read_north_convergence(baked_usd)
        if self.site is None:
            assert region_coords is not None
            lat_deg, lon_deg = region_coords

            from srb.terrain.catalog.resolve import RegionRequest

            region_key = RegionRequest(
                body=self.body,
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                size_m=self.size_m,
            ).slug
            return region_terrain_ephemeris_facts(
                body=self.body,
                region_key=region_key,
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                north_convergence_deg=north_convergence_deg,
            )

        from srb.terrain.manifest_loader import get_site

        site_entry = get_site(self.site)
        return TerrainEphemerisFacts(
            body=self.body,
            site_key=self.site,
            lat_deg=site_entry.coords.lat,
            lon_deg=site_entry.coords.lon,
            north_convergence_deg=north_convergence_deg,
        )

    @property
    def illumination_product_path(self) -> Optional[Path]:
        """Return the verified illumination sidecar, if available."""
        usd_path = Path(self.primary_baked_usd_path)
        candidate = usd_path.parent / "illumination.npz"
        generation_parent = usd_path.parent.parent.name == "generations"
        if not os.path.lexists(candidate) and generation_parent:
            # Follow the stable cache key, not the currently live generation.
            candidate = usd_path.parent.parent.parent / "illumination.npz"
        if not os.path.lexists(candidate):
            return None

        try:
            from srb.terrain.cache import (
                _published_file_sha256,
                cache_paths_from_baked_dir,
                read_meta,
            )
            from srb.utils.publication import ArtifactPublication, PublicationError

            meta: dict[str, Any] | None
            if generation_parent:
                meta = read_meta(cache_paths_from_baked_dir(usd_path.parent))
            else:
                meta = ArtifactPublication.file(candidate.parent / "meta.json").read(
                    json.loads
                )
            if not isinstance(meta, dict):
                return None
            illumination = meta.get("illumination")
            if not isinstance(illumination, dict):
                return None
            expected_sha256 = illumination.get("sha256")
            content_sha256 = illumination.get("content_sha256", expected_sha256)
            publication = illumination.get("publication")
            if (
                illumination.get("file") != candidate.name
                or not isinstance(expected_sha256, str)
                or content_sha256 != expected_sha256
                or not isinstance(publication, dict)
                or publication.get("kind") != "immutable-file"
                or publication.get("sha256") != expected_sha256
            ):
                return None
            return (
                candidate
                if _published_file_sha256(candidate) == expected_sha256
                else None
            )
        except (OSError, PublicationError, TypeError, ValueError):
            return None

    @property
    def augmentation_db_path(self) -> Optional[Path]:
        """Return the primary patch's bake-time augmentation sidecar, if present."""
        candidate = Path(self.primary_baked_usd_path).parent / "augmentation.npz"
        return candidate if candidate.is_file() else None

    def companions(self) -> dict[str, Any]:
        """Build configured companions and augmentation assets.

        Keys include resolved ``sky``, ``lighting``, and ``rocks``; augmentation
        adds ``augment_rocks`` or ``augment_rocks_db`` when it produces assets.
        """
        return build_real_terrain_companion_assets(
            body=self.body,
            site=self.site,
            sky=self.sky,
            lighting=self.lighting,
            rocks=self.rocks,
            augment=self.augment,
            augment_preset=self.augment_preset,
            baked_usd=self.primary_baked_usd_path,
            seed=self.seed,
            augment_max_rocks=self.augment_max_rocks,
            get_preset=get_augment_preset,
            build_augment=build_real_terrain_augment_cfg,
        )

    def attach_companions(self, env_cfg: "AnyEnvCfg") -> None:
        """Attach companions to the scene and arbitrate ephemeris drives."""
        if self.site:
            context = self.site
        elif self.lat_deg is None and not self.locations:
            # Use the user-facing name to scope warnings for local DEMs.
            context = self.site_name or "<local dem>"
        else:
            # Use the normalized slug so cache-equivalent requests share one
            # warning; explicit locations use their first center.
            from srb.terrain.catalog.resolve import RegionRequest

            lat, lon = (
                (self.lat_deg, self.lon_deg)
                if self.lat_deg is not None
                else self.locations[0]
            )
            context = RegionRequest(
                body=self.body,
                lat_deg=lat,
                lon_deg=lon,
                size_m=self.size_m,
                lod=self.lod,
            ).slug
        attach_terrain_companions_arbitrated(
            env_cfg, self.companions(), context=context
        )

    def handle_companion_attach_failure(
        self,
        env_cfg: "AnyEnvCfg",
        err: Exception,
    ) -> None:
        """Apply terrain policy when companion attachment fails."""
        from srb.terrain.companion_scene import handle_companion_attach_failure

        handle_companion_attach_failure(env_cfg, self, err)
