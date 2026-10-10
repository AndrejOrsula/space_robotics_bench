# Adding Terrain Sites + Catalog Entries

## Adding a new terrain site

1. Choose a public-domain or compatibly licensed DEM. Record its URL, expected SHA-256, CRS (such as `IAU_Moon_2015`), and ground sampling distance in meters.

1. Add an entry to `srb/terrain/manifest.yaml`:

   ```yaml
   sites:
     my_site:
       body: moon  # moon, mars, or earth
       name: "My Site (display name)"
       coords: { lat: 20.5, lon: 30.7 }
       sources:
         dem:
           provider: usgs_astro  # or lroc_pds, pds_geosciences, openplanetary, local
           url: "https://planetarymaps.usgs.gov/mosaic/<product>.tif"  # host must be on the provider's allowlist
           sha256: "<64-char hex>"
           crs: "IAU_Moon_2015"
           gsd_m: 2.0
           license: "Public Domain (NASA/USGS)"
           attribution: "USGS Astrogeology / LROC NAC"
       patches:
         default: { center_xy_m: [256, 256], size_m: 200, lod: medium }
       sky: null  # sky dome name, or null
       lighting: lunar_default  # null disables lighting
       rocks: null  # rock set name, or null
       augment_preset: lunar_default
   ```

1. Validate and bake:

   ```bash
   srb asset validate-manifest
   srb asset download terrain my_site
   ```

1. In the SRB/Isaac Sim Python environment, verify the import-time shortcut. `real_terrain_sites.py` imports Isaac Lab and generates `LunarSiteMySite`, `MartianSiteMySite`, or `EarthSiteMySite`:

   ```python
   from srb.assets.scenery.real_terrain_sites import LunarSiteMySite
   t = LunarSiteMySite()
   assert t.asset_cfg.spawn.usd_path
   ```

## Adding a v2 source layer

Add a reviewed, pinned layer to `srb/terrain/catalog/<body>.yaml` (for example, `moon.yaml`), not the v1 manifest. Download the product, calculate its SHA-256, and review its footprint. Shipped layers are listed in [Terrain Stacks → Catalog status per body](../config/terrain_stacks.md#catalog-status-per-body).

For detached PDS3 `.LBL`/`.IMG` products such as `imbrium_ldam_50s_1000m`, set `access: download`. List every extra file's URL, exact filename, SHA-256, and size in `SourceLayer.sidecars`; set `primary_size_bytes`. Acquisition publishes atomically, and GDAL follows `^IMAGE` from the label. See [Terrain Stacks](../config/terrain_stacks.md) for resolver, cache, and acquisition contracts.

### Authoring Mars/Earth entries

Mars and Earth use Moon's `SourceLayer` schema. Follow [Terrain Stacks footprint rules](../config/terrain_stacks.md#footprints-the-bbox-form). Region requests need no manifest entry: use `RealTerrain(body="<requested body>", lat_deg=..., lon_deg=..., size_m=...)` or `srb asset download terrain --body mars --lat 18.4 --lon 77.5 --size-m 2000` (add `--deshade` to deshade). See [Terrain Stacks](../config/terrain_stacks.md) for region identity, normalization, and fallback.

### Schema

`srb.terrain.catalog_schema.SourceLayer` validates `srb/terrain/catalog/<body>.yaml`:

```yaml
body: moon
layers:
  - layer_id: ldem_87s # unique within catalog
    body: moon
    kind: dem # dem | color | mask | quality
    product_family: "LOLA/Kaguya merged polar DEM"
    provider: pgda # ProviderName
    urls: ["https://pgda.gsfc.nasa.gov/data/LOLA_GDR/CYLINDRICAL/IMG/ldem_87s_10m.tif"]
    sha256: { "ldem_87s_10m.tif": "<64-char hex>" }
    native_gsd_m: 10.0
    footprint: "cap:83S"
    alignment: { source: assumed_zero }
    license: "Public Domain (NASA/GSFC)"
    attribution: "NASA GSFC PGDA"
    size_bytes: 1_400_000_000
    access: cog_remote # cog_remote | download | zip_member
    priority_class: 1 # lower wins; default 100
    allow_large_download: false
```

Catalog providers use exact-host allowlists:

| Provider | Allowed hosts | Typical body |
| --- | --- | --- |
| `pgda` | `pgda.gsfc.nasa.gov` | moon |
| `zenodo` | `zenodo.org`, `www.zenodo.org` | any; zip-member archives |
| `uahirise` | `hirise.lpl.arizona.edu`, `www.uahirise.org`, `uahirise.org` | mars |
| `copernicus_s3` | `copernicus-dem-30m.s3.amazonaws.com`, `copernicus-dem-90m.s3.amazonaws.com` | earth |
| `usgs_3dep` | `prd-tnm.s3.amazonaws.com` | earth |
| `imbrium` | `imbrium.mit.edu` (HTTPS only) | moon; PDS3 label/image products |

V1 providers: `usgs_astro`, `lroc_pds`, `pds_geosciences`, `openplanetary`, `local`. Allowlists are closed; add hosts in `srb/terrain/sources/*.py`. `uahirise`, `copernicus_s3`, and `usgs_3dep` accept unauthenticated public HTTPS GET only; authenticated products need another provider. `usgs_astro` covers Mars CTX and HRSC-MOLA.

### `priority_class` — lower wins

Set curated priorities (default `100`): DEM `1..5`, sunlit color `1..4`, PSR color `1..4`. Resolver sorts `(is_synthetic, priority_class, native_gsd_m, layer_id)`; GSD alone cannot break ties. Lowest fully covering entry is `role="base"`. See [synthetic-layer admission and sorting](../config/terrain_stacks.md#synthetic-super-resolution-layer-slot).

### `radiometry: synthetic` — the super-resolution slot

Use only for model-generated DEM/color. Footprint resolution excludes synthetic candidates unless `RegionRequest.allow_synthetic=True` or `TerrainSpec.allow_synthetic=True`; admitted candidates sort after real ones. Explicit declared-site `dem_stack`/`color_stack` layer IDs bypass synthetic and coverage filtering; body, frame, and `max_gsd_m` filters still apply. Opt-in adds `synthetic=1` to `CacheKey.hash12()`; default `False` leaves it unchanged. No shipped layer is synthetic; SRB has no model. See [the resolver contract](../config/terrain_stacks.md#synthetic-super-resolution-layer-slot).

### `access` — how a layer's bytes are fetched

Use `cog_remote` to fetch/hash the full source before local windowing, `download` for one whole file, or `zip_member` for pinned members of a whole Zenodo archive. See [Current limits](../config/terrain_stacks.md#current-limits).

### sha256 keys — the basename convention (read this exactly)

`sha256` keys use `urls[0]` basename, `urllib.parse.urlsplit(url).path.rsplit("/", 1)[-1]` (no query/fragment). `SourceLayer._validate_sha_map_matches_access` requires:

| Access | Required keys |
| --- | --- |
| `cog_remote` or `download` | Exactly one key equal to `basename(urls[0])`; missing, extra, or mismatched keys fail catalog loading. |
| `zip_member` | Archive key `basename(urls[0])` and at least one extracted-member key using its in-zip path. |

```yaml
access: zip_member
urls: ["https://zenodo.org/records/<id>/files/<archive>.zip"]
sha256:
  "<archive>.zip": "<64 hex>"
  "dtm/<region>_dem.tif": "<64 hex>"
```

Fetch reads `CatalogLayerPlan.archive_sha256` via `sha256[basename(urls[0])]`; malformed keys raise `KeyError`. `srb asset stack` rejects invalid catalogs before planning. Placeholder `"f" * 64` is refused before archive/member transfer, as in v1 (`PLACEHOLDER_SHA256`, `srb/terrain/sources/acquisition.py`). See [SHA-256 backfill workflow](#sha256-backfill-workflow).

### `allow_large_download` + `SRB_TERRAIN_LARGE_OK`

Set `allow_large_download: true` only after reviewing size. A download above 1 GiB also requires `SRB_TERRAIN_LARGE_OK=1` at fetch time. See [Terrain Stacks → Consent](../config/terrain_stacks.md#consent).

### `mask_encoding` — required in practice for every `kind: mask` layer

Mask values are product-specific DNs. Without `mask_encoding`, the default `None` passes raw values to fusion; `srb.terrain.fusion.color.psr_mask` treats finite nonzero values as masked. A nonzero unmasked DN can cover a region; this once marked the entire lunar south pole permanently shadowed.

The `pgda_lpsr_85s_60m` example in `srb/terrain/catalog/moon.yaml` has raw int16 values `20000` (permanently shadowed), `-32768` (nodata), and typically `-20000` (not shadowed):

```yaml
  kind: mask
  mask_encoding:
    masked_values: [20000]     # -> 1.0 (inside mask)
    nodata_values: [-32768]    # -> NaN (no observation)
                               # everything else -> 0.0 (outside mask)
```

Encoding maps `-20000` to `0.0`; without it, finite nonzero `-20000` makes **99.99998%** of the raster appear masked. `srb.terrain.ingest.mask.decode_mask_dn` decodes before arrays reach `fusion/`.

- Values use exact equality against pixels read as floats: list actual DNs, not ranges or thresholds. Non-finite and duplicate values fail at load.
- `SourceLayer` uses `extra="forbid"`; a typo such as `mask_encodding:` fails validation instead of silently restoring the bug.
- Check URL bytes and histogram. PGDA documents `{0, 1, 255}` uint8, but the pinned `.TIF` serves raw int16. Keep values synchronized with `srb/core/ephemeris/importers.py::_read_pgda_lpsr`, which decodes the product independently.

### `footprint` — three forms, and the CRS requirement

Use `cap:<lat><N|S>` for spherical caps, WKT `POLYGON(...)`/`MULTIPOLYGON(...)` in projected meters for polar layers, or geographic-degree `bbox:<lonmin>,<latmin>,<lonmax>,<latmax>` for non-polar layers. `cap:`/WKT and test regions must share a projected CRS; `bbox:` stays in degrees. Requests at `|lat| >= 60` use canonical polar CRSs (`moon_south`/`moon_north`, `IAU_2015:30135`/`:30130`, plus Mars/Earth equivalents); others use local stereographic. No covering footprint yields an empty plan with a warning; baking that empty DEM plan raises `RegionUnresolvedError`. See [footprint and antimeridian rules](../config/terrain_stacks.md#footprints-the-bbox-form) and [canonical CRSs](../config/terrain_stacks.md#bodies-and-canonical-crss).

### Validate

Preview both plan formats:

```bash
srb asset stack --body moon --lat -89.46 --lon 137.3 --size-m 2000
srb asset stack --body moon --lat -89.46 --lon 137.3 --size-m 2000 --json
```

Malformed SHA-256 shape, footprint, duplicate `layer_id`, or provider raises `pydantic.ValidationError` at load, before planning.

## Adding a v2 site with a declared stack

1. Confirm each referenced layer exists in `srb/terrain/catalog/<body>.yaml`; local testing may use a hand-built catalog. Catalog layers pin data; manifest sites name layers to fuse, like v1 `dem`/`ortho`.

1. Set manifest `version: 2` before declaring stacks. Existing v1 sites remain valid: `Sources` requires exactly one of `dem` or `dem_stack` per site regardless of manifest version.

   ```yaml
   version: 2
   sites:
     apollo17: { ... }  # existing v1 site, unchanged
   ```

1. Declare one `role: base` and zero or more `role: inset` entries in `dem_stack`; optional `color_stack` uses the same shape. Use these instead of v1 `sources.dem`/`sources.ortho`:

   ```yaml
   sites:
     connecting_ridge:
       body: moon
       name: "Connecting Ridge (v2 fused)"
       coords: { lat: -89.46, lon: 137.3 }
       sources:
         dem_stack:
           - { layer_id: ldem_87s, role: base }
           - { layer_id: nac_dtm_esall_cr1, role: inset }
         color_stack:
           - { layer_id: cm_avg_87s, role: base }
           - { layer_id: shadowcam_cmosaic_cr1, role: inset }
       patches:
         default: { center_xy_m: [0, 10309.97], size_m: 2000, lod: medium }
         # Or: { center_latlon: [-89.66, 0.0], size_m: 2000, lod: medium }
       sky: null  # sky dome name, or null
       lighting: lunar_default
       rocks: null
       deshade: true # optional; default illumination-corrected color
   ```

   `deshade: true` divides sunlit color by fused-DEM hillshade using catalog `sun_azimuth_deg`/`sun_elevation_deg`; PSR layers are never deshaded, and missing angles warn and skip. `RealTerrain(..., deshade=True)` enables undeclared deshading but cannot disable manifest `deshade: true`; the effective value enters the cache key. See [Deshade](../config/terrain_stacks.md#deshade-illumination-corrected-colour).

   Each `layer_id` must exist in its body's catalog; unknown ids fail at resolve and name the missing layer. `role` selects the base regardless of list order. `StackRef.max_gsd_m` excludes coarse candidates with a warning.

1. Validate and bake as for [Adding a new terrain site](#adding-a-new-terrain-site). The generated shortcut is `LunarSiteConnectingRidge`; CLI and `RealTerrain` constructor are unchanged. `bake_or_load` detects declared stacks (`sources.dem is None`) and routes to fusion.

1. Inspect outputs: `meta.json` contains a `fusion` block, `provenance.tif` sits beside `terrain.usd`, and `ATTRIBUTION.txt` has one block per contributing layer. [Real Terrain Assets → the fusion engine](../development/real_terrain_assets.md#real-terrain-v2-the-fusion-engine) documents contents, mesh/texture budget tiers, and QC/degradation: demoted or unreachable insets warn, drop, and bake succeeds; an unreachable base raises because v1 has no fallback.

The `connecting_ridge` IDs (`ldem_87s`, `nac_dtm_esall_cr1`, `cm_avg_87s`, `shadowcam_cmosaic_cr1`) are illustrative, not shipped. See `jezero_v2` in `srb/terrain/manifest.yaml` and [Terrain Stacks → Named site: Jezero v2](../config/terrain_stacks.md#named-site-jezero-v2).

## Other curated catalogs

### Adding a new HDRI skydome

Add an entry to `srb/terrain/hdri/catalog.yaml`:

```yaml
skydomes:
  my_dome:
    body: moon
    url: "https://example.org/dome.hdr"
    sha256: "<64-char hex>"
    format: hdr        # or png, exr
    intensity: 1.0
    resize_to: 2048
    license: "Public Domain"
    attribution: "Source name"
```

EXR requires optional `imageio[freeimage]` (`pip install srb[terrain-extras]`); PNG and HDR need no extra.

### Adding a new rock set

Add an entry to `srb/terrain/rocks/catalog.yaml`:

```yaml
sets:
  my_set:
    body: moon
    rocks:
      rock_a:
        url: "https://example.org/rock_a.glb"
        sha256: "<64-char hex>"
        format: glb       # or obj, ply, stl
        target_face_count: 5000
        license: "Public Domain"
        attribution: "Source"
```

Pipeline decimates each rock to `target_face_count` (default 5000), welds vertices, recenters at the centroid, and scales AABB diagonal to 1.0. Caller scales at scatter time.

### Adding a new lighting preset

Add a preset to `srb/terrain/lights/catalog.yaml` (pure config; no fetch):

```yaml
presets:
  my_preset:
    body: moon
    sun_elevation_deg: 30
    sun_azimuth_deg: 90
    sun_intensity_lux: 130000
    sun_color_temperature_k: 5800
    sun_angular_diameter_deg: 0.53
    earthshine_intensity_lux: 200      # optional
    earthshine_color_temperature_k: 7500
    license: "Public Domain"
    attribution: "Synthesized from ephemeris X"
```

Derive sun pose from the mission timestamp and site coordinates using an ephemeris, or synthesize an artistic look.

## Shared workflows

### sha256 backfill workflow

1. Replace placeholder `"f" * 64` with the downloaded file's real hash:

   ```bash
   curl -L "<url>" | sha256sum
   ```

1. Update the manifest, rerun `srb asset validate-manifest`, and confirm `srb asset download terrain <site>` succeeds.

### Common mistakes

- **Symptom:** ingest raises `MissingCRSError` or `DemLoadError`.
  **Cause:** the CRS string is misspelled.
  **Fix:** use `IAU_Moon_2015` or `IAU_Mars_2015`.
- **Symptom:** ingest raises `EmptyHeightmapError`.
  **Cause:** the patch lies wholly outside the DEM extent. Samples beyond the DEM edge are clamped.
  **Fix:** choose a smaller `patch_size_m` or another `center_xy_m`.
- **Symptom:** baking is refused for license compliance.
  **Cause:** user-supplied DEM has no `attribution`.
  **Fix:** add its attribution.
- **Symptom:** a companion catalog reference is stale.
  **Cause:** `sky:`, `lighting:`, or `rocks:` names no catalog entry.
  **Fix:** use a valid catalog name or `null` to disable it.

Next: [Testing](../development/testing.md)
