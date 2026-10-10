# Environment Configuration — Terrain Stacks

Preview terrain coverage without downloading products:

```bash
srb asset stack --body moon --lat -89.46 --lon 137.3 --size-m 2000
srb asset stack --body moon --lat -89.46 --lon 137.3 --size-m 2000 --json
```

## The catalog model

V2 fuses a gap-free DEM base with finer partial DEM insets, sunlit/PSR colour, masks, and quality layers for arbitrary regions or declared sites. V1 pairs one DEM with one ortho. Target Isaac Sim 6.1.0 / Isaac Lab 3.0.0 ([requirements](../getting_started/requirements.md)).

### Cache and catalog compatibility

Per-body catalogs (`srb/terrain/catalog/<body>.yaml`, loaded by `srb.terrain.catalog.loader`) pin product URLs, SHA-256, footprint, native GSD, license, `priority_class`, and curated consent. `srb.terrain.catalog.resolve.resolve_stacks` intersects footprints and returns prioritized DEM, sunlit/PSR colour, mask, and quality stacks in a `CompositionPlan`.

Preview reports only the `--deshade` request (`on (request)` or `off`); manifest `deshade: true` can still enable bake correction. Use catalog layers in a manifest `version: 2` site's `dem_stack`/`color_stack`, with the v1 `RealTerrain(body=..., site=...)` or `srb asset download terrain <site>` interface, or request any region.

| Setting / behavior | Default or limit |
| --- | --- |
| `SRB_TERRAIN_CATALOG_DIR` | Replaces shipped catalogs with a hand-built catalog directory. |
| `SRB_TERRAIN_CACHE_ROOT` | Overrides cache root. Checkouts use SRB scenery cache; wheels use `$XDG_CACHE_HOME/srb/terrain`, or `~/.cache/srb/terrain` if XDG is unset or relative. |
| Cache identity | Named single-source sites use effective patch geometry, enabled ortho, and attribution; equivalent integer/float geometry spellings share identity. Pre-0.1.0 SRB bakes are not reused: download or bake again. `srb asset doctor` preserves seed and coordinate precision. |
| Cleanup | Raw archives share content identity, so `--raw`/`--all` cleanup requires omitting site; baked cleanup is site-specific. Clean incomplete unpublished bakes; inspect corrupt published pointers. |
| Concurrent raw acquisition (POSIX) | Different sources download concurrently; identical content waits for active acquisition. Raw cleanup waits for fetch and verification. |
| Catalog admission | YAML uses `body`/`layers`; rejects unknown catalog/alignment keys, malformed footprint coordinates/rings, and polar caps outside `(0, 90]`. Polygon topology unchecked. Alignment offsets: `dx_m`, `dy_m`, and `dz_m`. |
| Region / raster values | Antimeridian coordinates are rounded; signed zero is normalized. Target GSD must be positive and finite. Integer colour is normalized by dtype maximum; float loaders supply unit-scale values (photometric class has no scale/offset). Mixed stacks are normalized before compositing. |
| Failed fetch | Consent refusal or source-size limits abort the stack; degraded data is not cached. Cached-window identity includes padding distance and resampling method, preserving admitted nodata margins on reread. |

### Catalog status per body

| Body | Layers and curated families |
| --- | --- |
| moon (19) | `pgda_ldem_83s_10mpp` / `pgda_ldem_80s_20mpp` (LOLA polar DEM); NAC DTM: `lroc_nac_dtm_shackrdge02`, `lroc_nac_dtm_nobile03`, `lroc_nac_dtm_malapert02`, `lroc_nac_dtm_malapert03`; `usgs_astro_haworth_sfs_dem_1m`; A3CLR Connecting Ridge/Haworth DEM/ortho pairs: `zenodo_sfs_a3clr06_connridge_dem_5m` / `zenodo_sfs_a3clr06_connridge_omos_5m`, `zenodo_sfs_a3clr02_haworth_dem_5m` / `zenodo_sfs_a3clr02_haworth_omos_5m`; `lroc_shadowcam_dtm_faustini_6m` + `lroc_shadowcam_cmosaic_faustini01_20m`; `lroc_nac_roi_haworth_lo1_20m`; `lroc_wac_emp_643nm_south_100m`; `imbrium_ldam_50s_1000m`; `pgda_lpsr_85s_60m`; and `lroc_nac_pole_south_cm065_p892s2250_1m` + `lroc_nac_pole_south_cm135_p892s2250_1m`. The last pair are band-1 `P892S2250` tiles, one per subsolar epoch, `reference_only: true`, resolvable only by explicit `layer_id`, never bake input. |
| mars (2) | `usgs_astro_hirise_jezero_dtm_1m` + `usgs_astro_hirise_jezero_ortho_1m` (HiRISE Jezero). |
| earth (2) | `copernicus_glo30_dryvalleys_s78e161` (polar GLO-30); `usgs_3dep_nm_southeast_x45y364_1m` (non-polar 1 m 3DEP). |

Moon catalog details:

- LDAM is a detached PDS3 `.LBL`/`.IMG` pair from HTTPS-only `imbrium`; `cap:50S` is its inscribed extent, and nominal 1000 m/pixel holds at the pole.
- At overlap, priority-3 643 nm WAC_EMP takes precedence over priority-4 1064 nm LDAM, possibly creating a spectral seam.
- No shipped Moon DEM covers the 50S–60S annulus where LDAM would be the only colour.
- Only band-1 NAC_POLE_SOUTH_CM `P892S2250` (8,276,814,528 B) is pinned per epoch; other tiles (8–31 GB) are not.
- A3CLR DEMs/orthos cover only Connecting Ridge and Haworth. See [Adding a v2 source layer](../contributing/adding_terrain_sites.md#adding-a-v2-source-layer) to curate more.

### Named site: Jezero v2

The manifest retains v1 `jezero` and adds `jezero_v2`: center `(18.44, 77.44)`, 2,000 m `medium` patch, pinned HiRISE DTM DEM base and ortho colour base.

```python
RealTerrain(body="mars", site="jezero_v2")
```

```bash
srb asset download terrain jezero_v2 --lod medium
```

Native `jezero_v2` bake is not validated against pinned HiRISE products.

### Providers

Catalog providers are exact-host allowlists in `srb/terrain/sources/*.py`. Shipped catalogs use:

| Provider | Body | Allowed hosts | Auth |
| --- | --- | --- | --- |
| `pgda` | moon | `pgda.gsfc.nasa.gov` | none |
| `lroc_pds` | moon | `pds.lroc.im-ldi.com`, `data.lroc.im-ldi.com`, `pds.shadowcam.im-ldi.com`, `pds.mcp.nasa.gov` | none |
| `zenodo` | moon | `zenodo.org`, `www.zenodo.org` | none |
| `imbrium` | moon | `imbrium.mit.edu` | none |
| `usgs_astro` | moon, mars | `astrogeo-ard.s3.us-west-2.amazonaws.com`, `asc-astropedia.s3.us-west-2.amazonaws.com`, `asc-pds-services.s3.us-west-2.amazonaws.com`, `astrogeology.usgs.gov`, `pilot.wr.usgs.gov`, `planetarymaps.usgs.gov` | none |
| `copernicus_s3` | earth | `copernicus-dem-30m.s3.amazonaws.com`, `copernicus-dem-90m.s3.amazonaws.com` | none |
| `usgs_3dep` | earth | `prd-tnm.s3.amazonaws.com` | none |

`imbrium` (LOLA team's MIT archive) accepts HTTPS only despite PGDA's HTTP link; TLS works, so plaintext is refused. Every redirect hop rechecks `_accepts_url`; HTTPS-to-HTTP downgrade is a hard error. Mars/Earth providers use unauthenticated public HTTPS GETs; they do not configure API-key or SigV4 authentication headers. Signed query-string URLs are not explicitly rejected. Shipped Mars HiRISE layers are USGS ARD COGs via `usgs_astro`; the registered `uahirise` provider (`hirise.lpl.arizona.edu`, `www.uahirise.org`, `uahirise.org`) has no shipped layer.

## Region requests

1. Configure `RealTerrain` with all three region fields; see [region identity](#region-identity-the-slug).
1. Preview coverage using `srb asset stack`; see [CLI](#cli).
1. Bake with `srb asset download terrain` or use the same `RealTerrain` config in an environment.

```python
from srb.assets.scenery import RealTerrain
terrain = RealTerrain(body="moon", lat_deg=-89.66, lon_deg=0.0, size_m=2000.0)
```

### Region identity: the slug

Set `lat_deg`, `lon_deg`, and `size_m` together. Partial input raises `ValueError` naming missing fields. Region fields cannot be combined with `site`, `dem_path`, `patch_size_m`, or `patch_center_xy_m`.

Region size and resolved center define the bake footprint. `lod` maps through. `size_m` is capped at 8000 m (`MAX_REGION_SIZE_M`); split larger areas.

Hydra dotted overrides work on any env with `RealTerrain` scenery:

```bash
srb agent teleop -e _ground \
  env.scenery.lat_deg=-89.66 env.scenery.lon_deg=0.0 env.scenery.size_m=2000
```

Hydra registers these keys (no `+`) for any task whose scenery accepts a variant (`Scenery | AssetVariant`). Resolver derives omitted `env.scenery.body` from `env.domain`; set it explicitly to request another body:

```bash
srb agent teleop -e _ground \
  env.domain=moon env.scenery.body=moon \
  env.scenery.lat_deg=-89.66 env.scenery.lon_deg=0.0 env.scenery.size_m=2000
```

Python construction, `srb asset download terrain --lat --lon --size-m`, and `srb asset stack --lat --lon --size-m` take the same region coordinates; `download` bakes, `stack` only previews its source stack. All coordinates are required; Hydra rejects unknown scenery keys.

The machine-independent slug replaces a manifest site name:

```text
region_{body}_{lat_deg:+08.4f}_{lon_deg:+09.4f}_{int(round(size_m))}m
# e.g. region_moon_-89.6600_+000.0000_2000m
```

Normalization wraps longitude to `[-180, 180)` (`180` = `-180`), converts signed zero to `0.0`, rounds latitude/longitude to `1e-4` degrees (about 3 m on Moon), and size to whole metres. Requests within these quanta share region and cache entry.

### Bodies and canonical CRSs

`Body = Literal["moon", "mars", "earth"]` is supported across requests, catalogs, providers, materials, physics, lighting, augment presets, companions, and scenery. `srb.terrain.ingest.planetary_crs.canonical_crs_for` supplies polar/geographic CRSs. `normalize_crs` matches raw DEM headers by projection parameters first, authority code as fast path.

| Body | South polar | North polar | Geographic |
| --- | --- | --- | --- |
| moon | `IAU_2015:30135` | `IAU_2015:30130` | `IAU_2015:30100` |
| mars | `IAU_2015:49935` | `IAU_2015:49930` | `IAU_2015:49900` |
| earth | `EPSG:3031` | `EPSG:3413` | `EPSG:4326` |

Mars uses IAU_2015 sphere radius `3396190.0 m` (not the often-quoted 3389500 m ellipsoid figure). Earth polar CRSs use WGS84 semi-major axis `6378137.0 m`.

`EPSG:3413` (`lon_0=-45`) matches by authority code only, unlike `EPSG:3031`; see `_CANONICAL_PARAM_SIGNATURES` in `planetary_crs.py`. Polar stereographic `lat_ts`/`k_0` are scale-normalized. WKT equivalent to EPSG:3031 (standard parallel `-71`) matches; unscaled south-polar WGS84 differs by about 2.7%. Unscaled south-polar WGS84 falls back to `matched_by="body_default"` (`EPSG:4326`) and retains its CRS, so reprojection is not skipped. Shipped layers are unscaled polar or geographic; local user DEMs may be affected.

### Body-aware, non-polar-capable resolution

At `|lat_deg| >= 60°`, requests use canonical polar stereographic. Elsewhere, they use a per-request local stereographic CRS for consistent footprint comparisons.

`local_stereographic_crs(body, lat_deg, lon_deg)` uses the quantized `1e-4°` center: `+proj=stere +lat_0=<lat> +lon_0=<lon> +R=<body radius> +units=m +no_defs` (Earth: `+ellps=WGS84`). The projection center is `(0, 0)`. Distortion within 8 km is negligible for meshes, containment, and offsets.

Its nonzero center is not a canonical signature, so `normalize_crs` reports `matched_by="body_default"`; bake `georef` preserves the proj string rather than the authority ID. The helper rejects `|lat_deg| >= 90 - 1e-9`. Poles use the canonical polar CRS to avoid Earth's unscaled oblique-stereographic scale difference.

### What happens when a region can't resolve

No covering footprint returns an empty plan and warning; environments fall back to procedural terrain, CLI errors, and baking an empty DEM plan raises `RegionUnresolvedError`:

| Consumer | Outcome |
| --- | --- |
| `RealTerrain` environment | Warn once with slug and reason; procedural fallback, training continues. |
| `srb asset download terrain --lat ...` | Print reason, exit 1; no procedural fallback. If catalog has no layers, error names catalog file and dormancy state. |

### Footprints: the `bbox:` form

Catalog `footprint` (see [footprint forms](../contributing/adding_terrain_sites.md#footprint--three-forms-and-the-crs-requirement)) accepts `cap:<lat><N|S>`, projected-metre WKT `POLYGON(...)`/`MULTIPOLYGON(...)`, or geographic degrees:

```text
bbox:<lonmin>,<latmin>,<lonmax>,<latmax>
```

Use `bbox:` for non-polar and most Mars/Earth products. WKT needs a shared canonical CRS, while local stereographic varies by request.

Longitudes in `[-360, 360]` are accepted and normalized once to `[-180, 180)`; common 0–360°E metadata works unchanged. If normalized `lonmin > lonmax`, the box wraps the antimeridian and contains `lon >= lonmin or lon <= lonmax`.

Set `latmin <= latmax`. Inversion raises `pydantic.ValidationError` naming the footprint at catalog load, before bytes move. Containment projects sample points back via `xy_to_latlon` in the resolved CRS, then checks the degree box.

## CLI

```bash
# Region target excludes positional site, --all, and site-only --patch/--patch-size/--center/--no-ortho:
srb asset download terrain --lat -89.66 --lon 0.0 --size-m 2000 \
  [--body moon] [--lod medium] [--deshade] [--force]
# Preview without downloading:
srb asset stack --body moon --lat -89.66 --lon 0.0 --size-m 2000 [--deshade]
# Assess without baking; doctor/info recognize region-derived cache entries:
srb asset doctor terrain [--deshade]
```

Region downloads use env `bake_or_load` for consent, cache, and provenance. Doctor/availability `download_command` reproduces `--deshade` when needed for its separate cache entry.

## Per-env location variety

Region requests can bake K distinct patches (one USD each) for env assignment; manifest sites and local `dem_path` have fixed footprints. Use `num_locations` + `location_spread_m` for seeded offsets or `locations` for explicit centers:

```python
from srb.assets.scenery import RealTerrain
terrain = RealTerrain(body="moon", lat_deg=-89.66, lon_deg=0.0, size_m=2000.0,
                      num_locations=8, location_spread_m=3000.0)
terrain = RealTerrain(body="moon", size_m=2000.0,
                      locations=[(-89.66, 0.0), (-89.40, 15.0), (-89.80, -20.0)])
```

Pre-bake serially with same consent/cache/provenance behavior:

```bash
srb asset download terrain --lat -89.66 --lon 0.0 --size-m 2000 \
  --num-locations 8 --location-spread-m 3000
```

Output is `<slug>: <path>` per location in derivation order. Variety flags require region form (like `--patch-size`/`--center`); invalid use or `--num-locations < 1` exits 2. `locations` sets K at any latitude, requires non-empty centers and `size_m`, shares `size_m`, `lod`, and `deshade`, and excludes `lat_deg`/`lon_deg` center and `num_locations != 1`; `location_spread_m` must be positive for multiple derived locations. On `RegionUnresolvedError`, print slug and stop; skip later locations, retaining earlier bakes.

### Semantics

| Aspect | Behavior |
| --- | --- |
| Primary patch | Index 0 is requested center verbatim (or `locations[0]`). `primary_baked_usd_path` anchors companions, SimForge augmentation, and other single-path readers. |
| Derivation | Indexes 1..K-1 use `numpy.random.default_rng([905, seed])` with a fixed salt independent of other seed consumers. They are sampled uniformly in a disk (`r = location_spread_m · √u`) in the resolved CRS: canonical polar near poles, local stereographic elsewhere. Spread defaults to `size_m`. |
| Prefix stability | RNG does not depend on K; increasing K preserves earlier derived slugs and cache entries. |
| Uniqueness | All K slugs must differ after `1e-4°`/whole-metre normalization. Collision (small spread or duplicate explicit centers) raises `RegionInvalidError`, names duplicate slugs, suggests larger spread or deduplication; no dedupe/re-jitter. |
| Spawning | K=1 uses one `UsdFileCfg`; K>1 uses `MultiAssetSpawnerCfg(func=spawn_multi_asset_compat)` at env-scoped prim path. `PerEnvironmentCloneStrategy` assigns env `i` location `i % K`; all locations used when `num_envs >= K`. `env.stack=true` spawns only index 0. |
| Bake cost | All K bake serially/eagerly at construction or CLI; no lazy/parallel path. Only `min(num_envs, K)` spawn; K=8, two envs still bakes eight and spawns two. |
| Clone budget | Region spawner is never shrunk. Larger K uses more of `MAX_CLONE_PLAN_COMBINATIONS = 65536`, reducing other multi-variant spawner budget and shrinking SimForge `num_assets`. |

## Deshade (illumination-corrected colour)

Set `deshade=True` to remove capture-time topographic shading from sunlit colour during fusion. For each sunlit layer, divide by a fused-DEM hillshade at catalog `sun_azimuth_deg`/`sun_elevation_deg`; preserve mean brightness and NaN nodata. Never deshade PSR. Missing sun angles or fused DEM leave colour unchanged and add named warning to `meta.json` fusion `warnings`.

| Surface | Setting |
| --- | --- |
| Manifest | Site `deshade: true` in `srb/terrain/manifest.yaml`. |
| Python | `RealTerrain(..., deshade=True)`. |
| CLI | `--deshade` for `srb asset download terrain`, `stack`, or `doctor`. |

Effective value is manifest OR request: request `True` enables, never disables manifest `True`. It enters v2 cache identity, so on/off bakes coexist; default `False` leaves key unchanged. V1 single-source `sources.dem` sites cannot deshade: `DeshadeUnsupportedError`; CLI checks manifest before fetching bytes.

## Consent

| Request | Requirement and failure |
| --- | --- |
| v2 catalog layer estimated over 1 GiB (`MAX_UNAPPROVED_SOURCE_BYTES`) | Curated `allow_large_download: true` AND fetch-time `SRB_TERRAIN_LARGE_OK=1`; else `LargeDownloadConsentRequiredError`, no prompt (`check_layer_consent`, `srb/terrain/sources/acquisition.py`). |
| `cog_remote` | Gate full `size_bytes`, as full source downloads; see [Current limits](#current-limits). |
| Region request | Same gate for each source's bytes. |
| v1 manifest site | Site `allow_large_download: true` alone. |
| `srb asset stack` preview | “Consent required” is descriptive (`--allow-large` OR env var); preview ignores layer `allow_large_download`, fetches nothing, grants no authorization. |

## Companions and the ephemeris sun

`BaseEnvCfg._add_scenery` calls `attach_companions()` for `RealTerrain` when `attach_companions_enabled=True` (default); set false to opt out. Curated-site companions can supply sky, lighting, and rocks. Region requests have no manifest entry and therefore have no companions. Presentation catalogs are `srb/terrain/hdri/catalog.yaml`, `srb/terrain/lights/catalog.yaml`, and `srb/terrain/rocks/catalog.yaml`; source DEM catalogs do not supply companions. No shipped site declares one; no sky dome or rock set ships, so shipped curated-site envs use the default sun/skydome. `jezero_late_morning` is unused procedural static lighting in `srb/terrain/lights`; on Jezero it renders much darker than the default sun.

`arbitrate_companion_scene` resolves companions against enabled `env.ephemeris`:

| Priority / condition | Effect |
| --- | --- |
| 1. `env.ephemeris.enabled` and `drive_sunlight` | Ephemeris owns sun; suppress companion lighting (warn once). If `drive_skydome`, suppress companion sky. Rocks attach. |
| 2. Otherwise, companion lighting exists | Companion owns sun: `scene.sunlight=None`; skip four `randomize_sunlight_*` randomizers, warning once if configured. `scene.lighting` and `scene.sunlight` are mutually exclusive. Manifest `lighting: "ephemeris"` without enabled env ephemeris uses its baked preset. |
| 3. Companion sky exists | Replace domain-default skydome and clear `randomize_skydome_orientation`, unless operator set non-default string (e.g. `env.skydome=high_res`) or disabled it (`null`/`false`); operator wins. |

Each suppression warns once with both conflicting knobs. Region requests silently attach no curated companions; `env.ephemeris` supplies real sun there. If any site companion fails to build/attach (e.g. offline), the entire set is dropped with a one-time warning; there is no per-product fallback. Attached lighting owns the sun and changes curated-site rendering. Opt out via `attach_companions_enabled=False` or scenery `sky=None`/`lighting=None`. Non-`"ephemeris"` `lighting=` presets are procedural.

## Illumination sidecar

`illumination.npz` (`srb_illum/2`) is an out-of-band sidecar imported by `srb ephemeris import`, not bake input or a cache-key field. `RealTerrain.illumination_product_path` points beside the primary/index-0 baked USD (see [Per-env location variety](#per-env-location-variety)); masked-spawn code in `srb/core/mdp/illumination.py` and the `illumination_fraction_at` reward helper consume it. See [Illumination Products](../reference/ephemeris_products.md) for the schema and importer.

## Augmentation database

Bakes can write deterministic crater and rock placements (`augmentation.npz`, `srb_augdb/1`), optionally stamp craters into the DEM, and scatter rocks with `RealTerrain.augment="database"`. See [Terrain stack augmentation](../development/real_terrain_assets.md#terrain-stack-augmentation) for the schema, determinism, cache identity, and scatter behaviour.

## Synthetic super-resolution layer slot

`radiometry: synthetic` reserves an opt-in schema/resolver slot for model-generated DEM/colour; it sorts after every real candidate:

| Use | Behavior |
| --- | --- |
| Footprint search, default | Exclude synthetic before candidate selection; real layers resolve without warning. |
| Footprint search, admitted | Candidates sort by `(is_synthetic, priority_class, native_gsd_m, layer_id)`, putting synthetic last even below real priority 9 (e.g. synthetic priority 0/GSD 0.5 m). |
| v2 manifest `dem_stack`/`color_stack` | Explicit `layer_id` implicitly admits synthetic. Opt-in filtering also applies to footprint resolution and declared-site mask/quality/PSR-colour candidates. |

`RegionRequest.allow_synthetic=False` and `TerrainSpec.allow_synthetic=False` are threaded like deshade. This setting is not in the region slug; `CacheKey.hash12()` appends `synthetic=1` only when true because pixels can change. `TerrainSpec.allow_synthetic=True` with `region.allow_synthetic=False` raises at construction (the effective value comes from `RegionRequest`). No shipped catalog has synthetic layers or an integrated model; this feature is unrelated to rock augmentation.

## Current limits

| Limit | Effect / remedy |
| --- | --- |
| Coverage is narrow | Few pinned products; see [Catalog status per body](#catalog-status-per-body). |
| `cog_remote` | Compatibility label only: download complete source via redirect-validating, size-bounded provider; verify pinned SHA-256; read local window. Direct GDAL `/vsicurl` is refused. First use needs full-source disk space; see [Consent](#consent). Later bakes reuse content-addressed cache. Fusion `meta.json` records `bytes_verified: true` per contributing layer. |
| Published cache readers | Readers hold no lock. Publish retains only new and previous generations; two newer publishes of one entry may delete files used by a long-running process. Restart after re-baking. |
| Region cache lookup | Key needs resolved stack hashes, so each call loads catalog/resolves footprints before cache check (K times for K locations). |
