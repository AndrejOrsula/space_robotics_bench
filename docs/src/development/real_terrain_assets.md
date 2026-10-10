# Real Terrain Assets

1. Add a baked NASA/USGS/ESA digital elevation model (DEM) to a scene. Choose `body="moon"`, `"mars"`, or `"earth"`, then select a manifest site, shortcut, or coordinate region and assign `asset_cfg`:

   ```python
   from srb.assets.scenery import RealTerrain
   from srb.assets.scenery.real_terrain_sites import (
       LunarSiteApollo17,
       MartianSiteJezero,
       MartianSiteJezeroV2,
   )

   terrain = RealTerrain(body="moon", site="apollo17")
   terrain = LunarSiteApollo17()
   terrain = MartianSiteJezero()
   terrain = MartianSiteJezeroV2()  # Declared HiRISE DTM/ortho
   terrain = RealTerrain(body="moon", lat_deg=-89.66, lon_deg=0.0, size_m=2000.0)
   env_cfg.scene.terrain = terrain.asset_cfg
   ```

   `RealTerrain(body=..., site=...)` resolves manifest entries; shortcut classes select specific sites; coordinate regions need no manifest. For curation and offline baking, see [Adding Terrain Sites](../contributing/adding_terrain_sites.md).

1. Configure companions or per-environment variation (optional):

   ```python
   terrain = LunarSiteApollo17()
   terrain.attach_companions(env_cfg)  # Manifest defaults; shipped site has none
   terrain = LunarSiteApollo17(lighting="apollo17_noon", rocks=None)
   terrain = RealTerrain(body="moon", site="apollo17", augment="simforge")
   ```

   By default, `BaseEnvCfg._add_scenery` attaches companions when `attach_companions_enabled=True`; set it to `False` to opt out. Ephemeris and skydome drives arbitrate lighting. Fallback-eligible companion attachment failures skip all companion products and emit one warning; integrity, safety, invalid-input, and unknown failures are re-raised and may abort setup. `augment="simforge"` adds per-environment variation, such as differently seeded rocks, over the static baked terrain. The default augment preset depends on body (`lunar_default` for Moon). If `simforge_foundry` cannot be imported, `SimforgeMissingError` is raised.

1. To bake a local DEM, provide the required license attribution:

   ```python
   terrain = RealTerrain(
       body="moon", dem_path="/data/my_lunar_dem.tif", site_name="my_polar_crater",
       attribution="Acme Surveying, 2026", patch_size_m=300, use_ortho=False,
   )
   ```

   For a local DEM, clear a shortcut's `site` with `site=None`, or construct `RealTerrain(body=..., dem_path=...)`.

1. Inspect, download, bake, or manage assets with the CLI:

   ```bash
   srb asset list terrain
   srb asset list terrain --body moon
   srb asset list rocks
   srb asset list hdri
   srb asset list lights
   srb asset info terrain apollo17
   srb asset download terrain apollo17
   srb asset download terrain --all --body moon
   srb asset download terrain --lat -89.66 --lon 0.0 --size-m 2000 --body moon --deshade
   srb asset bake terrain --dem /data/my.tif --body moon --site-name my_local --attribution "Me, 2026"
   srb asset cache-stats
   srb asset clean terrain apollo17 --baked
   srb asset clean terrain --all
   srb asset validate-manifest
   srb asset prefetch-fixtures
   ```

   Coordinate downloads target arbitrary v2 regions without a manifest. They require all three coordinate flags and reject site targets, `--all`, and site-specific flags.

| Option | Default or constraint |
|---|---|
| Selection | Exactly one (mixing is rejected): manifest `site`, local `dem_path`, or complete `lat_deg`/`lon_deg`/`size_m` region. |
| `lod` | `medium`; valid tiers `low`, `medium`, `high`. |
| `use_ortho` | `True`; disable with `False` for a local DEM without ortho. |
| `augment` | `"none"` by default; `"simforge"` requires `simforge_foundry`. |
| Auto-fetch | On by default; set `SRB_TERRAIN_AUTO_FETCH=0` to require pre-baking. |

## Ephemeris-driven lighting

Sites with `lighting: ephemeris`, `coords`, and curated `epoch` bake Sun and lunar Earthshine via real SPICE (`srb/terrain/lights/ephemeris_bridge.py`). A `query()` uses persisted `georef.north_convergence_deg`: `az_grid = az_true + γ`; listed azimuth is raw true-north.

| Cache | Path and invalidation |
|---|---|
| Ephemeris | `.../srb_assets/lights/ephemeris/<body>_<site>_<epoch>_<identity>/generations/<generation id>/lights.usd`; `<identity>` is the request SHA-256 prefix (16 hex). Epoch, grid correction, base-preset edits, or lights-writer changes make a new file. |
| Static preset | `.../srb_assets/lights/<preset>/<content hash>/generations/<generation id>/lights.usd`; keyed by lights-writer version and full catalog content (same 16-character prefix); edits need no cache deletion. |

Missing pinned NAIF kernels warn once and use body-default static lighting only when the lighting bridge's `on_missing` policy is `fallback`; `on_missing: error` fails instead. This fallback does not cover `augment="database"`: a missing or empty augmentation database raises an error. `augment="simforge"` uses SimForge directly. `srb ephemeris download` enables SPICE. See [Ephemeris → Terrain lighting](../config/ephemeris.md#terrain-lighting) for manifest fields, fallback, and `srb asset list` provenance.

## Real-terrain v2: source catalog and stack resolution

Preview region coverage offline; an uncovered request returns an empty plan. Text mode prints `no v2 source coverage for <body> at (<lat>, <lon>) — falling back to the v1 single-source path`; JSON mode returns the empty plan without that message.

```bash
srb asset stack --body moon --lat -89.46 --lon 137.3 --size-m 2000
srb asset stack --body moon --lat -89.46 --lon 137.3 --size-m 2000 --json
```

Manifest `dem_stack` (optional `color_stack`) and arbitrary regions use `RealTerrain`, download, and bake fusion APIs; regions resolve by footprint. [Terrain Stacks](../config/terrain_stacks.md) covers schema, coverage, normalization, fallback, CRS, deshade, cache, companions, and consent; [Catalog status per body](../config/terrain_stacks.md#catalog-status-per-body) lists layers/access. [Adding Terrain Sites](../contributing/adding_terrain_sites.md#adding-a-v2-source-layer) explains layer curation and [declared stacks](../contributing/adding_terrain_sites.md#adding-a-v2-site-with-a-declared-stack).

## Real-terrain v2: the fusion engine

A `dem_stack` has one gap-free full-patch `role: base` plus optional finer partial `role: inset` layers; `color_stack` follows suit. Fusion produces one DEM and colour raster:

1. Warp layers to the patch grid with `fusion.grid.warp_to_grid`.
1. Co-register DEM insets using catalog shifts or estimates; warn and drop insets with excessive QC residuals.
1. Feather-blend by priority: finest/highest-priority inset wins within its data boundary, blending to the next layer within `blend_len_m` of the true boundary. Clamp `blend_len_m` to 50–200 m; declared footprint without data does not count. No global weighted average.
1. Composite colour first-finite-wins in sorted-stack order, then normalize radiometry against the anchor layer.

Fused bakes add these beside v1 `terrain.usd` and textures:

| Output | Contents |
|---|---|
| `provenance.tif` | Same-size `uint8` GeoTIFF: `0` means no contributor; `1..N` is winning layer index matching `meta["fusion"]` `index`. |
| `meta.json["fusion"]` | V2 only: `fusion_version`; `grid` (`crs`, `center_xy_m`, `size_m`, `gsd_m`, `pixels`); `dem_layers`/`color_layers` (`layer_id`, `role`, `kind`, `index`, `area_fraction`, `native_gsd_m`, `upsampled`, `radiometry`, `alignment`, `license`, `attribution`); loaded-mask `contributors` affect colour but own no raster index; `provenance_tif`/`uncertainty_tif` filename or `null`; `warnings`. V1 has no `fusion`; `meta["georef"]` is unchanged. |
| `meta.json["bake_identity"]` | Versioned fused-cache identity: cache key, canonical DEM and colour/auxiliary stack hashes, fusion version, target GSD, effective deshade. Required for a ready fused generation. |
| `ATTRIBUTION.txt` | One `layer_id`/`license`/`attribution` block per contributing base, surviving inset, and loaded mask, in blend order. V1's single `attribution + "\n"` applies only to v1 single-source bakes. |

**Budgets (v2 only).** V1 uses stride-based LOD meshes and uncapped textures. V2 quadric-decimates to face budgets and caps texture/AO resolution. Skirts and solid-shell collision geometry remain undecimated, adding a small perimeter-scaled face count; shipped meshes may slightly exceed budget without decimation failure.

| LOD | Max grid faces | Max texture side (px) |
|---|---:|---:|
| `high` | 4,000,000 | 8192 |
| `medium` | 1,000,000 | 4096 |
| `low` | 250,000 | 2048 |

AO is computed at ≤2048 px then upsampled at every LOD. See `srb/terrain/fusion/budgets.py`.

## Caching and reproducibility

Bake identity includes `(body, site, patch, lod, seed, materials_version, dem_sha256, ...)`; matching inputs reuse `assets/srb_assets/scenery/terrain/<body>/<site>/<patch_key>/`. Set `SRB_TERRAIN_CACHE_ROOT=/custom/path` to override the root.

## Terrain stack augmentation

### Augmentation database (`srb_augdb/1`)

Bake can generate deterministic synthetic crater/rock placements from fused terrain via `srb.terrain.augmentation.database.generate_augmentation_db`; compressed sidecar `augmentation.npz` (`save_augmentation_db`). Schema `srb_augdb/1`, `AUGMENT_DB_VERSION=4`, stores float32 arrays:

| Array | Shape | Columns |
| --- | --- | --- |
| `craters` | `(Nc, 4)` | `x_m, y_m, radius_m, depth_m` |
| `rocks` | `(Nr, 6)` | `x_m, y_m, z_m, scale, yaw_rad, asset_idx` |

XY are patch-local mesh-frame metres (grid east/north, patch-center origin; `mesh_frame {"center_xy": true}`). Rock Z is a bilinear DEM sample or `0.0` without DEM, in mesh Z datum after center-pixel origin subtraction (`heightmap_to_mesh(..., z_origin="center")`, `mesh_frame {"z_origin": "center"}`), so rocks lie on the mesh. `load_augmentation_db_file` upgrades v1 in memory (inserting zero `z_m` at column 2; `meta["upgraded_from"]=1`); older versions raise `ValueError` and require rebaking. Zero-density kinds are empty `(0,4)`/`(0,6)`, never `None`. Metadata includes schema/version, seed, slug, body, preset, requested/kept counts, synthetic provenance, and `generated_from` (`patch_size_m`, `gsd_m`, `max_slope_deg`, `dem_stats`; the last is `None` without DEM).

`bake_or_load` supplies fused `dem.heights`, `dem.gsd_m`. Placements are uniform over patch; reject when local `np.gradient` slope proxy exceeds `max_slope_deg` (default 25°). Presets define `rock_density_per_m2`, `rock_size_range_m`, `crater_density_per_m2`, `crater_radius_range_m`; body defaults have zero crater density. RNG is `np.random.default_rng([761, AUGMENT_DB_VERSION, seed, *sha256(slug)-ints])`, independent of Python `hash()`, dict order, global `np.random`; same seed/version/slug/preset/DEM yields byte-identical arrays across machines/processes. Salt, RNG, placement-rule, or schema changes require version bump; version is part of seed. Draw `_OVERSAMPLE=4` candidates and keep first N passing in order to fix RNG use; no retries, so steep terrain may yield fewer. `meta["counts"]` records requested/kept.

Both v1/v2 bakes write sidecar only if effective preset has nonzero rock or crater density. Moon/Mars defaults have rocks; all-zero Earth `EARTH_DEFAULT` has no sidecar/metadata. `_bake` writes to temp before atomic promotion publishing `provenance.tif`; `meta.json` records `augmentation_db: {"file":"augmentation.npz","schema":"srb_augdb/1","db_version":4,"counts":{...}}`. Cache hits do not regenerate. With nonzero density, generation/write or stamping failure aborts publication. Explicit preset overrides and stamping params/version enter cache identity. `load_augmentation_db(baked_dir)` returns `AugmentationDb | None`; missing metadata/file returns `None`, invalid present file raises `IntegrityFailureError`. `RealTerrain.augmentation_db_path`, like illumination path, is beside primary USD and exposes index 0 for multi-location terrain.

To stamp craters, set `TerrainSpec.augment_stamp=True` or `RealTerrain.augment_stamp`. `_bake` cuts generated craters into DEM after nodata repair and before `heightmap_to_mesh`: a parabolic bowl to rim R plus a raised-cosine collar returning to grade at `1.4 R`, with height `0.06 * depth`. Stamping changes the mesh, materials, and collision; rock Z resamples the stamped surface. Stamping is off by default and requires a nonzero crater-density preset. Shipped defaults use zero; `lunar_cratered`/`mars_cratered` copy body defaults with density `0.002` and radius `(1.0, 8.0)`, or use a custom preset. Select with `RealTerrain.augment_preset`, passed as `TerrainSpec.augment_preset`. `_bake` uses `spec.augment_preset or default_augment_preset_for_body(spec.body)` for generation/stamping. An unknown preset raises `KeyError`; a body mismatch raises `ValueError`. `CacheKey.from_spec` resolves via `companion_scene.resolve_effective_augment_preset`, shared with bake and the `companions()` SimForge/database layer; errors name the value and choices. `hash12()` adds `augment_stamp={db_version}:{density:g}:{radius_min:g}:{radius_max:g}` when stamping or explicitly overriding the preset; the default key is unchanged. Craters deform the DEM, not separate prims; spawn rocks with `augment="database"`.

#### Explicit rock scatter — `augment="database"`

`RealTerrain.augment`: `"none"` (default), `"simforge"`, `"database"`. SimForge gets count/seed through `SimforgeAssetCfg`/`spawn_simforge_assets` and hides transforms. Database reads `(Nr,6)` sidecar rows `[x_m,y_m,z_m,scale,yaw_rad,asset_idx]`, deterministically spawning one static prim per row.

- `database_scatter` generates eight (`_ROCK_ASSET_SLOTS=8`) variant USDs once via SimForge `asset.generator_type(...).generate_subprocess(...)`, without scattering; subprocess cache stores them. Row variant is `asset_idx % len(variants)`.
- `rock_pose(row)` returns `(pos, quat_xyzw)`: position `(x_m,y_m,z_m)` and yaw-only xyzw quaternion `(0,0,sin(yaw/2),cos(yaw/2))` (w-last), matching Isaac Lab `AssetBaseCfg.InitialStateCfg.rot`; identity `(0,0,0,1)`.
- `RealTerrain.augment_max_rocks: int = 256`; `select_rows` takes first N in draw order, never random sample.
- With `augment="database"`, missing sidecar/metadata or zero rocks raises an error; terrain companion setup does not silently continue without the requested database scatter.
- `simforge_ext._build_rock_generator` raises `ValueError` outside Moon/Mars. Earth zero-density shortcut in `build_real_terrain_augment_cfg` and database empty-rock degrade avoid the guard.

## Disabling auto-fetch

For pre-baked or air-gapped CI:

```bash
export SRB_TERRAIN_AUTO_FETCH=0
```

Then `RealTerrain(...)` raises `TerrainNotBakedError` with the exact `srb asset download` command.

## Troubleshooting

| Error | Cause | Remedy |
|---|---|---|
| `ManifestSiteUnknownError` | Bad site | `srb asset list terrain` |
| `LicenseMissingError` | Missing attribution | Pass `attribution="..."` |
| `ChecksumMismatchError` | Upstream DEM changed | `srb asset validate-manifest --check-shas` |
| `SimforgeMissingError` | Package absent | `pip install simforge_foundry` |
| `TerrainNotBakedError` | Auto-fetch off; cache miss | Run the shown `srb asset download` command |
