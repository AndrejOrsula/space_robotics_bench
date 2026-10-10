# Illumination Products

`table` synthesizes `EphemerisState` from imported terrain-patch rasters, without live SPICE queries or NAIF kernels. It provides environment lighting, radiometry, and surface eclipse geometry.

## Schema `srb_illum/2`

One `np.savez_compressed` archive (`SCHEMA_ID="srb_illum/2"`, `product_version=2`) stores `meta` JSON and rasters on a north-up, axis-aligned, pixel-center Real-Terrain-v2 patch grid; include at least one tier:

| Tier/key | Dtype and shape | Meaning |
| --- | --- | --- |
| Epoch `sunlit` | `bool (T,H,W)` | Per-epoch line-of-sight mask. |
| Epoch `sun_fraction` | `float32 (T,H,W)`, NaN holes | Continuous illuminated-disk fraction. |
| Epoch `times_utc` | `(T,)` ISO-8601 | Epoch axis. |
| Epoch `sun_azel` | `(T,2) float64` | True-north compass azimuth and patch-center elevation, degrees. |
| Epoch `sun_angular_diameter_deg` | `(T,) float64` | Sun angular diameter. |
| Optional epoch `earth_los` | `bool (T,H,W)` | Source Earth visibility. |
| Optional epoch `earth_azel` | `(T,2) float64` | Source Earth geometry. |
| Static `psr` | `uint8 (H,W)`, `{0,1,255}` | Permanently-shadowed region; 255 nodata. |
| Static `lit_fraction` | `float32 (H,W)`, NaN holes | Time-averaged illuminated fraction. |
| Static `max_sun_el` | `float32 (H,W)`, NaN holes | Maximum elevation reached. |

Epoch keys are required together; static keys are independently optional. At least one tier is required. `srb_illum/1` is unsupported (`load_product`: `legacy schema 'srb_illum/1' requires explicit migration`); re-import its source. Float rasters: bilinear (including `1×N`/`N×1`); bool/`uint8`: nearest-neighbor. Out-of-bounds `psr`: `PSR_NODATA` (`255`). `patch_center_xy_m` anchors local `(x,y)` (east,north; origin at center) to the pixel transform; `local_xy_to_rowcol_f` and `rowcol_to_local_xy` are inverses.

Stored `sun_azel`/`earth_azel` use query conventions: true-north compass azimuth and patch-center elevation. Importers correct source-grid azimuth/elevation once; table queries apply `site.north_yaw_deg` before `geometry.direction_world()`.

## Importers

Importers reproject to north-up `ProductGrid`; azimuth uses `az_true = az_grid − γ` with source-CRS convergence `γ` at its reference center. Elevation uses first-order curvature/parallax `el_site = el + degrees(s_par/body_radius_m)`, where `s_par` is along-azimuth center offset in source-grid CRS. This matches lunarlab `masks.ramp_elevation`, not full spherical recomputation. `zarr`, `rasterio`, `pyproj` load lazily; module import remains Isaac-free.

| Source | Function | Requirements |
| --- | --- | --- |
| lunarlab `spice_maps` `epochs.zarr` | `import_lunarlab_zarr` | `zarr`/`numcodecs`. |
| lunarlab legacy `save_masks` `.npz` | `import_lunarlab_npz` | NumPy only. |
| PGDA product-69 LPSR GeoTIFF | `import_pgda_lpsr` | Base `rasterio`; Moon-only, static tier. |

1. Use `--baked-dir` to import into a baked patch. It reads the grid, CRS, patch center, and site latitude/longitude from baked-patch `meta.json`. Output defaults to `<baked-dir>/illumination.npz`.

   New cache generations record an `illumination` block in `meta.json`: filename, schema, SHA-256, source, epoch count/start/step, static keys, and import timestamp. Bake metadata is the georeference authority, so `--baked-dir` with `--lat`/`--lon` exits `SystemExit(2)`.

1. Without a bake, specify `--crs`, `--center-xy`, `--size-m`, `--gsd-m`, `--lat`, `--lon`, and `--out`.

   `--size-m` must be an integer multiple of `--gsd-m`. Otherwise, `SystemExit(2)` reports both values and the remainder, preventing subpixel center skew.

   Lunarlab accepts `--body {moon,mars,earth}`, default `moon`. `pgda-lpsr` is Moon-only and rejects `--epoch-range`/`--epoch-stride` (`SystemExit(2)`; stride default `1`) because it has no epoch axis.

For examples, see the [CLI reference](cli_ephemeris.md):

```bash
srb ephemeris import --source lunarlab-zarr \
  --input /data/lunarlab/<product>/epochs.zarr \
  --baked-dir <terrain-cache>/moon/<site>/<patch-key>

srb ephemeris import --source pgda-lpsr \
  --input /data/pgda/lpsr_shackleton.tif \
  --crs "IAU_2015:30135" --center-xy 0.0,10309.969398639505 \
  --size-m 4000 --gsd-m 5.0 --lat -89.66 --lon 0.0 \
  --out /tmp/shackleton_illumination.npz
```

`srb[spice]` and `srb[all]` include `zarr>=2.16,<4` and `numcodecs` for lunarlab zarr. Docker installs the frozen all-extras export, then the editable project with `--no-deps`.

Without the extra, `import_lunarlab_zarr` raises `ImportError` naming it. `.npz` and PGDA need no extra (`rasterio` is a base dependency).

| Decode detail | Behavior |
| --- | --- |
| Packed masks | lunarlab `sunlit`/`earth_los`: `np.unpackbits(arr, axis=-1)[..., :W]`. |
| `sun_fraction` | `uint8` decodes as `q / 254.0`; `q == 255` becomes NaN. |
| `earth_los` | lunarlab's DSN-union station-view mask, not single-site Earth-center line of sight; imported unchanged. |
| PGDA LPSR | `int16`: PSR iff `DN == 20000`; nodata `DN == -32768`. Canonical `uint8 {0,1,255}` passes through; other dtypes raise `ValueError`. |
| PSR fractions | Neither source supplies continuous PSR fraction; `psr` remains hard `{0,1,255}`, never interpolated as probability. |

## Table provider

`TableProvider.query()` uses `set_table_product(path)` or `SRB_EPHEMERIS_TABLE_PRODUCT`. It caches at most one parsed file by identity/mtime; re-imports are visible without restart.

Epoch keys are required. Static-only files raise `ValueError` (“the table provider needs per-epoch sun az/el”). The provider selects the nearest `epoch_index()` within half an epoch step (3600 s with fewer than two epochs). Farther queries raise `ValueError`; no interpolation is used.

Table mode supports surface sites only. Combining table mode with `env.ephemeris.orbit` fails validation before Kit boot.

`epoch_et` uses `approx_et_seconds()` with fixed TT−UTC `69.184 s` (32.184 s + 37 leap seconds since 2016-12-31). It is accurate to \<2 ms from 2017, wrong earlier, and must change if a leap second is added. No downstream calculation uses it.

Table mode skips kernel checks and works without kernels. Provider queries validate a private product snapshot; mutable public `IlluminationProduct` methods validate inputs.

SPICE kernel hash reuse is keyed by process/file identity and invalidated on replacement/fork. Failed furnishing clears partial state, and the next query waits for uncertain cleanup. Offset-aware epochs normalize to UTC before SPICE parsing.

On the Moon, Earth requested only for eclipse is optional. `earth_azel` enables it; without it, the factor stays `1.0`, with one warning per body/provider. Reset queries use this optional behavior when Earth is added only for eclipse.

Explicit `earth` in `env.ephemeris.bodies` requires Earth geometry. Missing data raises the typed availability error, including for config-time lighting.

## Consumers

`srb/core/mdp/illumination.py` exposes static-tier MDP tools. Epoch cubes feed table queries and `sunlit_epoch_windows`, not per-step rewards/observations.

| Term/helper | Inputs and behavior |
| --- | --- |
| `reset_root_state_uniform_illumination_masked` | Reset event; uses `pose_range`, `velocity_range`, `asset_cfg`; `mask="psr"` (default) selects `psr==1`, while `mask="lit_fraction"` selects `lit_fraction>=threshold` (`threshold=0.5`); nodata/NaN are excluded. `invert=false` by default and complements only valid cells. Uniform valid-cell choice, jitter ±`gsd/2`; other pose/velocity sampling matches `reset_root_state_uniform_poisson_disk_2d`. Empty candidates raise `ValueError`. Optional `product_path`. |
| `illumination_fraction_at` | Torch helper sampling `lit_fraction` (default) or `max_sun_el` at `positions_w`, converting world to patch-local through `env.scene.env_origins`; NaN→0.0. Optional `product_path`. These float rasters are reward-ready. `psr` is excluded because its out-of-bounds sentinel is 255; use `IlluminationProduct.sample_static` directly. |

Both prefer explicit `product_path`, then `env.unwrapped.cfg._scenery.illumination_product_path`. If unresolved, `ValueError` names `srb ephemeris import`.

`RealTerrain.illumination_product_path` derives from `primary_baked_usd_path` (index-0/requested-center child), so K>1 terrain variants share that sidecar. It resolves only when the file matches the SHA-256 recorded in that block.

The product is outside terrain `CacheKey`; re-importing at a fixed key does not invalidate or detect stale illumination data.

`sunlit_epoch_windows(product, min_lit_fraction=...)` creates `EpochSpec.utc_windows`.

Grid mode measures valid non-NaN `sunlit` cells. Point mode (`xy_local`) uses continuous `sun_fraction`. Consecutive passing epochs merge into ISO `(start_utc,end_utc)` pairs; windows shorter than `min_duration_s` (default `0`) are dropped.

Assign directly:

```python
from srb.core.ephemeris import EpochSpec
from srb.core.ephemeris.products import load_product, sunlit_epoch_windows

product = load_product("illumination.npz")
windows = sunlit_epoch_windows(product, min_lit_fraction=0.6)  # grid mode
cfg.ephemeris.epoch = EpochSpec(utc=None, utc_windows=windows)
```

Windows exclude `utc`/`utc_range`. Each reset chooses a window proportional to `max(duration_s, 1.0)` and samples uniformly; zero-duration windows weigh 1.

With `time_scale>0`, each reset draws a new start, but progression ignores window bounds and may leave the curriculum window during a long episode.
