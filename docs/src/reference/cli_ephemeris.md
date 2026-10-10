# `srb ephemeris` — Real Sun/Planet Geometry (Optional Feature)

## Usage

```bash
srb ephemeris <action> [options]
```

Install `srb[spice]` for live queries. Help and completion need neither the extra nor Isaac Sim; `download`/`info` load `spiceypy` and `numpy`; `import` needs no `spiceypy` ([details](../config/ephemeris.md)).

## Actions

| Action | Purpose |
| :--- | :--- |
| `download` | Fetches ~43 MB of pinned kernels ([bundle](../config/ephemeris.md#kernel-bundle)). |
| `info` | Prints body azimuth, elevation, distance, angular diameter, and irradiance scale. |
| `import` | Imports illumination as an `srb_illum/2` sidecar ([Lunarlab/PGDA details](ephemeris_products.md)). |

### `info` Options

| Option | Required | Description |
| :--- | :--- | :--- |
| `--body {moon,mars,earth}` | Yes | Observer. |
| `--lat LAT` | Yes | Latitude, `[-90, 90]`°. |
| `--lon LON` | Yes | Longitude, °. |
| `--alt ALT` | No (default `0.0`) | Height above mean radius, m. |
| `--utc UTC` | Yes | ISO-8601 epoch; `de440s`: 1850–2149. |
| `--bodies BODIES` | No (default `sun,earth`) | Choices: `sun,earth,moon,mars,venus`; Sun included; no duplicates/unknowns. |

### `import` Options

| Option | Required | Description |
| :--- | :--- | :--- |
| `--source {lunarlab-zarr,lunarlab-npz,pgda-lpsr}` | Yes | Format. |
| `--input PATH` | Yes | Source. |
| `--baked-dir PATH` | No | Target baked patch. |
| `--out PATH` | Yes\* | Output. |
| `--crs CRS` | Yes\* | Target CRS. |
| `--center-xy CX,CY` | Yes\* | Center. |
| `--size-m SIZE` | Yes\* | Patch size. |
| `--lat LAT` / `--lon LON` | Yes\* | Site coordinates. |
| `--gsd-m GSD` | Yes\* | Grid spacing. |
| `--epoch-range T0,T1` | No | UTC range. |
| `--epoch-stride N` | No (default `1`) | Epoch sampling interval. |
| `--body {moon,mars,earth}` | No (default `moon`) | Terrain body; PGDA-LPSR supports Moon only. |

- Without `--baked-dir`, starred options are required. Baked metadata supplies CRS, patch size, site, and default output. Grid spacing uses source resolution unless `--gsd-m` overrides it; `--out` overrides the output path. Do not pass `--lat`/`--lon` with it (status 2). See [importer rules](ephemeris_products.md#importers).

## Examples

```bash
srb ephemeris download
```

Shackleton:

```bash
srb ephemeris info --body moon --lat -89.66 --lon 0.0 \
  --utc 2026-11-01T12:00:00 --bodies sun,earth
```

```text
sun: az=279.043° el=1.512° dist=149685302 km diam=0.559°
earth: az=64.201° el=2.551° dist=384412 km diam=1.916°
irradiance_scale=1.0021
```

Apollo 17:

```bash
srb ephemeris info --body moon --lat 20.19 --lon 30.77 \
  --utc 1972-12-11T19:54:57 --bodies sun
```

Jezero:

```bash
srb ephemeris info --body mars --lat 18.44 --lon 77.45 \
  --utc 2021-02-18T20:55:00 --bodies sun
```

Lunarlab:

```bash
srb ephemeris import --source lunarlab-zarr \
  --input /data/lunarlab/<product>/epochs.zarr \
  --baked-dir <terrain-cache>/moon/<site>/<patch-key>
```

PGDA-69 LPSR:

```bash
srb ephemeris import --source pgda-lpsr \
  --input /data/pgda/lpsr_shackleton.tif \
  --crs "IAU_2015:30135" --center-xy 0.0,10309.969398639505 \
  --size-m 4000 --gsd-m 5.0 --lat -89.66 --lon 0.0 \
  --out /tmp/shackleton_illumination.npz
```

## Disabling Auto-Fetch

Queries fetch missing kernels by default. Disable fetch to require pre-fetch:

```bash
export SRB_EPHEMERIS_AUTO_FETCH=0
srb ephemeris download   # run once, ahead of time, with network access
```

Missing kernels then raise `KernelsMissingError` naming `srb ephemeris download`.

## See Also

- [Ephemeris Configuration](../config/ephemeris.md): kernel bundle, environment variables, and the `query()` API.
