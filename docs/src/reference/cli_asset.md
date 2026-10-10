# `srb asset` — Manage Assets

Run `srb asset <action> [options]`. Hydra-style `key=value` overrides report stray tokens in a named error (exit 2).

## Actions

- `list` and `info` discover assets; `doctor` diagnoses them.
- `download` fetches assets; `bake` bakes a local DEM.
- `clean`, `cache-stats`, and `prefetch-fixtures` manage caches; `validate-manifest` checks manifests; `stack` previews offline.

## Kind Choices

| `KIND` | Assets |
| :--- | :--- |
| `terrain` | Planetary DEMs/orthophotos. |
| `rocks` | Procedural/photogrammetric sets. |
| `hdri` | High Dynamic Range Image skydomes. |
| `lights` | Curated presets. |

## `srb asset list`

```bash
srb asset list KIND [--body {moon,mars,earth}]
```

`--body` filters by celestial body.

## `srb asset info`

```bash
srb asset info KIND TARGET
```

`TARGET`: site, rock set, skydome, or lighting preset.

## `srb asset doctor`

```bash
srb asset doctor terrain [TARGET] [options]
```

Optional arguments: `TARGET` and `--body`.

`--body` filters by celestial body. With `TARGET`, it rejects a site on another body.

| Option | Default or meaning |
| :--- | :--- |
| `--patch PATCH` | `default`. |
| `--lod {low,medium,high}` | `medium`. |
| `--patch-size SIZE` | Float, meters. |
| `--center X,Y` | Meters, e.g. `100.0,200.0`. |
| `--no-ortho` | Skip orthophoto check/generation. |
| `--seed SEED` | Procedural seed, `0`. |
| `--deshade` | Corrected variant; separate cache. |

## `srb asset download`

```bash
srb asset download KIND [TARGET] [options]
```

Terrain accepts a curated `TARGET` or a region (`--lat`, `--lon`, `--size-m`), never both.

Regions also accept `--body`, `--deshade`, `--num-locations`, and `--location-spread-m`.

| Option | Default or meaning |
| :--- | :--- |
| `TARGET` | Asset; omit with `--all` or region. |
| `--all` | Fetch/bake all registered terrain sites, rock sets, or HDRIs. Lights require a `TARGET`. |
| `--body {moon,mars,earth}` | For regions, defaults to `moon`; with `--all`, filters terrain sites, rock sets, or HDRIs. |
| `--patch PATCH` | Default `default`. |
| `--lod {low,medium,high}` | Default `medium`. |
| `--patch-size SIZE` | Float, meters. |
| `--center X,Y` | Meters. |
| `--no-ortho` | Skip orthophoto baking. |
| `--seed SEED` | Procedural seed, default `0`. |
| `--lat LAT --lon LON --size-m SIZE` | Region: coordinates in degrees, size in meters; require all three; omit `TARGET`/`--all`. |
| `--deshade` | Corrected terrain color; manifest deshading applies if omitted. |
| `--num-locations N` | Region patches, default `1`; above 1: deterministic offsets. |
| `--location-spread-m SIZE` | Offset radius, meters; for `--num-locations > 1`, default is region size. |
| `--force` | Re-download/re-bake valid cached entries. |
| `--debug` | Verbose pipeline logs and traceback per failed item. |

Non-terrain kinds reject terrain-only options and `--force`. `--lod medium` and `--num-locations 1` act like omission.

## `srb asset bake`

```bash
srb asset bake terrain --dem DEM_PATH --body {moon,mars,earth} --site-name NAME --attribution ATTR [options]
```

| Option | Default or meaning |
| :--- | :--- |
| `--dem DEM_PATH` | User DEM path, absolute/relative (e.g. GeoTIFF). |
| `--body {moon,mars,earth}` | Body. |
| `--site-name NAME` | Custom-site name. |
| `--attribution ATTR` | Metadata attribution string. |
| `--patch-size SIZE` | Float, meters. |
| `--lod {low,medium,high}` | Mesh LOD, default `medium`. |
| `--seed SEED` | Procedural rock/displacement seed, default `0`. |
| `--debug` | Verbose mesh logs. |

## `srb asset clean`

```bash
srb asset clean KIND [TARGET] [options]
```

| Argument/option | Default or meaning |
| :--- | :--- |
| `KIND` | `terrain`, `rocks`, `hdri`, `lights`; only `terrain` supported. Others exit 2: `clean: unsupported kind: '<kind>'; use 'terrain'` |
| `TARGET` | Optional; clean only its baked cache. |
| `--raw` | Remove shared raw archives; omit target. |
| `--baked` | Remove baked cache publications and their required files; raw archives remain unless `--raw` is also selected. |
| `--all` | Remove all kind cache entries; omit target. |

Pass one of `--raw`, `--baked`, or `--all` (otherwise exit 2). Raw archives share content identity. `TARGET` with `--raw`/`--all` is a usage error (exit 2). Clean one site's baked cache: `srb asset clean terrain TARGET --baked`.

## `srb asset validate-manifest`

```bash
srb asset validate-manifest [--manifest MANIFEST_PATH] [--check-urls] [--check-shas]
```

| Option | Default or meaning |
| :--- | :--- |
| `--manifest MANIFEST_PATH` | Custom YAML; default: packaged. |
| `--check-urls` | HTTP-check mirrors and links. |
| `--check-shas` | Check downloaded raw dataset SHA256 against manifest. |

## `srb asset cache-stats`

```bash
srb asset cache-stats
```

Prints cache root, total size, entries/sites per body, stale temporary-directory count.

## `srb asset stack`

```bash
srb asset stack --body {moon,mars,earth} --lat LAT --lon LON --size-m SIZE [options]
```

| Option | Default or meaning |
| :--- | :--- |
| `--body {moon,mars,earth}` | Required body. |
| `--lat LAT`, `--lon LON` | Required coordinates, degrees. |
| `--size-m SIZE` (`--size`) | Region size in meters; max 8000 m; larger requests refused. |
| `--lod {low,medium,high}` | `medium`. |
| `--target-gsd GSD` | Target ground-sample distance, meters. |
| `--catalog PATH` | YAML catalog override (mainly fixture testing). |
| `--allow-large` | Shows the preview as if large downloads were approved; does not download. |
| `--deshade` | Text preview reports the request; it does not resolve corrected output. |
| `--json` | JSON instead of table; serializes the plan and omits the `--deshade` request. |

“Consent required” is descriptive; fetch rechecks per-layer consent at `srb.terrain.sources.acquisition.check_layer_consent`. “Consent required: none” from `--allow-large` or `SRB_TERRAIN_LARGE_OK=1` does not affect fetches.

## `srb asset prefetch-fixtures`

```bash
srb asset prefetch-fixtures
```

Pre-bakes every manifest terrain site at `lod=low`.

## Examples

```bash
srb asset list terrain --body moon
srb asset info terrain jezero
srb asset download terrain jezero --lod high
srb asset bake terrain \
  --dem /path/to/my_dem.tif \
  --body moon \
  --site-name Shackleton_Ridge \
  --attribution "LOLA / Kaguya Team" \
  --lod high
srb asset doctor terrain jezero --lod high
srb asset clean terrain --baked
srb asset cache-stats
```
