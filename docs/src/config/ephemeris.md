# Environment Configuration — Ephemeris

Use `srb.core.ephemeris` to query Sun/planet azimuth, elevation, distance, angular diameter, and irradiance scale for a surface site and UTC epoch. Live queries use NASA/JPL [NAIF SPICE](https://naif.jpl.nasa.gov/naif/) through [`spiceypy`](https://github.com/AndrewAnnex/SpiceyPy).

1. Install the optional dependency for live queries: `pip install 'srb[spice]'` (`srb[all]` also includes it). Core `srb` omits it (wheel ~30 MB); missing `spiceypy` raises `srb.utils.extras.MissingExtraError` with this install command.
1. Fetch kernels with `srb ephemeris download` or let first use fetch them. Imported table queries need neither kernels nor `spiceypy`; see [Illumination Products](../reference/ephemeris_products.md).

## Kernel Bundle

SPICE needs position (`.bsp`), orientation (`.bpc`/`.tf`), leap-second (`.tls`), and constants (`.tpc`) kernels; SRB pins and furnishes this ~43 MB bundle in table order. Bundle coverage is 1849-12-26–2150-01-22; `de440s.bsp` queries cover 1850-01-01–2149-12-31.

| File | Purpose | SHA-256 |
| --- | --- | --- |
| `naif0012.tls` | Leap seconds; current through 2016-12-31 | `678e32bdb5a744117a467cd9601cd6b373f0e9bc9bbde1371d5eee39600a039b` |
| `pck00011.tpc` | Radii, orientation, `IAU_MARS`/`IAU_EARTH` frames | `3dff7b1dbeceaa01f25467767d3fa25816051c85d162d1edf04acb310ee28bb1` |
| `de440s.bsp` | Sun, Earth, Moon, Mars barycenter positions; 1849–2150 | `c1c7feeab882263fc493a9d5a5b2ddd71b54826cdf65d8d17a76126b260a49f2` |
| `moon_pa_de440_200625.bpc` | Lunar principal-axis orientation, `MOON_ME` frame | `60cd55aa401ea2ea97360636f567554bfe4e37bb829f901b4460a455dfaf783f` |
| `moon_de440_250416.tf` | Lunar `MOON_ME` frame aliases | `a47c71e9c9f33796bdafb2c9d69a7ee447b6016ecad80f71cd6f3e479f9cf768` |

`KERNEL_BUNDLE` in `srb/core/ephemeris/kernels.py` authoritatively lists files, URLs, and hashes.

### Fetching kernels

`resolve_kernels()` fetches on demand by default, verifies SHA-256, and publishes atomically. Pre-fetch with:

```bash
srb ephemeris download
```

### Attribution and redistribution

Kernels come from NAIF's public archive (`https://naif.jpl.nasa.gov/pub/naif/generic_kernels`); unmodified files may be redistributed. SRB tries the Hugging Face dataset mirror (`SRB_EPHEMERIS_HF_REPO_ID`) before NAIF. Acknowledge SPICE/NAIF in published derived data: Acton (1996), “Ancillary data services of NASA's Navigation and Ancillary Information Facility”; Acton et al. (2018).

## Environment Variables

| Variable | Default | Effect |
| --- | --- | --- |
| `SRB_EPHEMERIS_CACHE_ROOT` | `<SRB cache>/ephemeris` (checkout: `<SRB repo>/.cache`; wheel: `$XDG_CACHE_HOME/srb` or `~/.cache/srb`; `SRB_CACHE_ROOT` overrides) | Kernel cache directory; override to share it or use a pre-provisioned read-only mount. |
| `SRB_EPHEMERIS_AUTO_FETCH` | `1` (enabled) | `0` or `false` disables on-demand downloads. Missing kernels then raise `KernelsMissingError` with `srb ephemeris download` as remedy; no silent network access. |
| `SRB_EPHEMERIS_HF_REPO_ID` | unset | Hugging Face dataset repo id (`org/name`) tried before NAIF. Unset uses NAIF directly. |
| `SRB_EPHEMERIS_TABLE_PRODUCT` | unset | Product file for `provider="table"` when `set_table_product()` was not called. |

## CLI Usage

See [CLI reference](../reference/cli_ephemeris.md). Examples:

```bash
# Fetch the pinned bundle into the cache
srb ephemeris download

# Query Sun/Earth geometry at a Shackleton-rim-like site
srb ephemeris info --body moon --lat -89.66 --lon 0.0 \
  --utc 2026-11-01T12:00:00 --bodies sun,earth
```

## Programmatic Usage

```python
from srb.core.ephemeris import SiteSpec, query

site = SiteSpec(body="moon", lat_deg=-89.66, lon_deg=0.0, alt_m=0.0)
state = query(site, "2026-11-01T12:00:00", bodies=("sun", "earth"))

sun = state.bodies["sun"]
print(f"az={sun.azimuth_deg:.2f} el={sun.elevation_deg:.2f}")
print(f"irradiance_scale={state.irradiance_scale:.4f}")  # (1 AU / distance)^2
```

`query()` dispatches through `EphemerisProvider`: `spice` queries kernels; `table` reads `srb_illum/2` without `spiceypy` at query time.

## Terrain lighting

Bakes query live SPICE; imported `table` products cannot drive terrain bakes.

1. In a [real-terrain site](../development/real_terrain_assets.md) manifest (`srb/terrain/manifest.yaml`), set `lighting: ephemeris`, plus surface `coords` and a curated UTC `epoch`:

   ```yaml
   my_site:
     body: moon
     coords: { lat: 20.1908, lon: 30.7717 }
     lighting: ephemeris
     epoch: "1972-12-11T19:54:57"
   ```

1. Bake the terrain. The bridge queries site/epoch once and builds a `LightingPreset`: solar irradiance and the night-visibility ramp set Sun intensity; body defaults set color temperature and base intensity. The output is `.../srb_assets/lights/ephemeris/<body>_<site>_<epoch>_<identity>/generations/<generation id>/lights.usd` under `SRB_ASSETS_DIR_SRB_LIGHTS`. The 16-hex SHA-256 `<identity>` covers writer version, base/computed presets, coordinates, epoch, and grid-north correction. Immutable generations contain `lights.usd` and `identity.json`; identical requests reuse a slot, while changed inputs (including `LIGHTS_WRITER_VERSION`) get a new one. Publication is locked against concurrent first use and interrupted writers; no manual cache deletion is needed.

**Grid north.** Convert true-north query azimuth with `az_grid = az_true + γ`, using persisted `north_convergence_deg` (`γ`) in sibling `meta.json`'s `georef` block; do not recompute it. `γ=0` for equirectangular/lon-0 sites and is nonzero for polar-stereographic patches. `srb asset list terrain` reports `sun=EL°@AZ° (ephemeris @ <utc>, true-north)`; `lights.usd` adds `+γ`, so bearings agree only when `γ=0`.

**Missing kernels.** The bridge never queries or downloads. Default `on_missing="fallback"` warns once per `(site, body, epoch)` (`warn_ephemeris_lighting_fallback_once`) and uses the body-default `lunar_default`/`mars_default` preset for offline bakes. `on_missing="error"` raises `KernelsMissingError`. Fetch with `srb ephemeris download`; asset listing reports `(ephemeris @ <utc>, kernels missing)` without querying.

Returning to a named static `lighting` preset restores byte-identical pre-ephemeris output; unused ephemeris lighting is inert. Static presets use the cache path `.../srb_assets/lights/<preset>/<content hash>/generations/<generation id>/lights.usd`. Its 16-character hash covers writer version, preset name, and every field, so catalog edits (for example `sun_elevation_deg`) get a fresh slot.

## Environment Lighting

`BaseEnvCfg.ephemeris` is always `EphemerisCfg()` (default `enabled=False`). When enabled, it drives the single global `/World/sunlight` without a terrain bake. Disabled mode never calls `query()` or touches the scene graph, so output is unchanged.

1. Set a fixed epoch to query once at config/scene-build time:

   ```bash
   srb agent teleop -e _ground \
     env.ephemeris.enabled=true \
     env.ephemeris.epoch.utc=2026-11-01T12:00:00
   ```

1. Or sample an epoch uniformly on each reset; draws use `env.seed` for reproducibility:

   ```bash
   srb agent zero -e waypoint_navigation \
     env.ephemeris.enabled=true \
     env.ephemeris.epoch.utc_range='[2026-01-01T00:00:00,2026-12-31T00:00:00]'
   ```

`EpochSpec` requires exactly one of `utc`, `utc_range`, or `utc_windows` (duration-weighted; see [`sunlit_epoch_windows`](../reference/ephemeris_products.md#consumers)). Fixed mode queries during config/scene construction; features needing live state query the fixed UTC again on reset, and positive `time_scale` causes interval queries.

A Sun at/below the horizon is valid night and warns once. Sampled mode uses a static config-time placeholder, then `randomize_sun_ephemeris` re-queries at reset with dedicated `np.random.default_rng(env_cfg.seed)` on `env.unwrapped`; equal seeds yield equal epoch sequences. The Sun light is scene-wide, so in sampled mode any reset, including one environment's partial reset, changes lighting and ephemeris observations for all environments. Visibility may set intensity exactly zero below `-angular_radius`. The year-long lunar sample spans many ~29.5-Earth-day synodic months and can produce roughly half fully dark Moon episodes: narrow the range or account for darkness in task/reward design. First dark reset per site warns once.

| `EphemerisCfg` option | Default | Behavior |
| --- | --- | --- |
| `enabled` | `false` | Opt-in gate; disabled path does not query or touch the scene. |
| `provider` | `"spice"` | Live SPICE or imported `"table"` data. |
| `epoch` | `EpochSpec(utc="2026-11-01T12:00:00")` | `time_scale=0.0`, `update_interval_s=600.0`. |
| `site` | `None` | Observer site; precedence below. |
| `orbit` | `None` | Required on `Domain.ORBIT`; invalid on surface domains. |
| `bodies` | `("sun",)` | Requested bodies; must include Sun. |
| `drive_sunlight` | `true` | When ephemeris is enabled and `drive_sunlight=true` (the default), drives `/World/sunlight`; four static randomizers (`randomize_sunlight_orientation`, `randomize_sunlight_intensity`, `randomize_sunlight_angular_diameter`, `randomize_sunlight_color_temperature`) are disabled so they cannot overwrite the ephemeris result. |
| `drive_skydome` | `false` | See [Skydome co-drive](#skydome-co-drive); requires `drive_sunlight=true`. |
| `expose_observations` | `false` | Adds the Direct observation vector; see [Observation vector](#observation-vector). |
| `eclipse` | `false` | Scales Sun intensity by the modeled occluder; see [Eclipse](#eclipse). |
| `on_missing` | `"fallback"` | `"error"` fails early; fallback warns and leaves existing lighting unchanged. |

**Site resolution** (once per environment build):

1. Explicit `env.ephemeris.site` wins. Its body must match surface `env.domain` (else build raises); in orbit mode it selects the central body.
1. Otherwise, matching-body `RealTerrain` with a curated manifest `site` supplies `coords` and persisted convergence as `SiteSpec.north_yaw_deg = −γ` (active/passive equivalent of terrain's `az_grid = az_true + γ`). A body mismatch ignores terrain with a warning.
1. Otherwise `DEFAULT_SITES`: equatorial lon-0 for Moon/Earth; Jezero crater (18.44°N, 77.45°E) for Mars.

Nested `SiteSpec` defaults to `None`, so `env.ephemeris.site.*` has no Hydra CLI overrides or completion keys; children appear only for populated nested defaults. Set it in Python or use real-terrain scenery:

```python
from srb.core.ephemeris import EphemerisCfg, SiteSpec

cfg.ephemeris = EphemerisCfg(
    enabled=True,
    site=SiteSpec(body="moon", lat_deg=-89.66, lon_deg=0.0),
)
```

**Radiometry.** Surface intensity is:

```text
intensity = domain.light_intensity
          × irradiance_multiplier(body, state.irradiance_scale)
          × sun_visibility_factor(elevation_deg, 0.5 * angular_diameter_deg)
```

`state.irradiance_scale = (1 AU / r)²` is absolute; `Domain.light_intensity` already uses each body's mean distance (Mars: 590 W/m² at ≈1.524 AU). Direct multiplication double-counts distance (Mars ≈254 W/m², 43% of correct). `irradiance_multiplier` normalizes by the mean-distance value, retaining only variation (≈±3.4% Earth/Moon; ≈±19% Mars); a query near mean Mars distance gives ≈591 vs `Domain.MARS` 590 W/m² (0.18% off). Orbit intensity references 1 AU and applies the scale directly, including Mars-centered orbits.

**Missing kernels.** Environment lighting only probes pinned kernels on disk. `on_missing="error"` raises `KernelsMissingError` before Kit boot; sampled mode rechecks each reset. Default `"fallback"` warns once per domain (fixed) or site body (sampled), retaining the built-in static or previous-reset light. It does not query, write, or create/advance the epoch RNG.

Enabled ephemeris conflicts with `env.sunlight_rpy`, since both drive orientation. Supported domains: `MOON`, `MARS`, `EARTH`, configured `ORBIT`; `ASTEROID` is rejected. Real terrain runs `attach_companions()` automatically. `drive_sunlight` suppresses companion lighting with a one-time warning naming both settings; otherwise companion lighting owns the scene sun (`scene.sunlight=None`, static sun randomizers skipped), enabling manifest `lighting: ephemeris` in the running env. See [Terrain Stacks: companion and ephemeris sun precedence](terrain_stacks.md#companions-and-the-ephemeris-sun). Per-env suns are unsupported.

With `drive_sunlight=false`, do not enable `drive_skydome`, `expose_observations`, `eclipse`, positive `epoch.time_scale`, or orbit mode; each requires Sun application and raises config `ValueError`. A surface `site.body` mismatch with `env.domain` also raises `ValueError`.

## Orbit and Dynamics

`Domain.ORBIT` adds a circular-orbit observer, analytic eclipse, moving Sun, optional observations, and skydome co-drive; it has no visual Earth/Moon prim. Orbit numerics use pure NumPy.

### Orbit mode (`Domain.ORBIT`)

The central body defaults to Earth; choose another in Python with `site=SiteSpec(body=..., lat_deg=None, lon_deg=None)`. Orbit sites have no lat/lon; surface sites are rejected.

| `OrbitSpec` field | Default | Meaning and limit |
| --- | --- | --- |
| `altitude_m` | required | Above mean body radius; must be `> 0`. |
| `inclination_deg` | `0.0` | Must be in `[0, 180]`. |
| `raan_deg` | `0.0` | Right ascension of ascending node. |
| `arg_lat_deg` | `0.0` | Angle from ascending node at reference epoch (`dt_s=0`). |

Circular Kepler motion uses central-body J2000 axes: `n = sqrt(GM/a³)`, `a = body_radius + altitude_m`, `u = arg_lat_deg + n·dt_s`, rotated by `R3(-raan) · R1(-inc)`. `GM_M3S2`/`BODY_RADIUS_M` cover Earth, Moon, Mars; other bodies raise. Sun direction uses the body-center SPICE query without observer offset (≈5×10⁻⁵ rad LEO error): suitable for lighting/eclipses, not navigation truth. No J2, drag, or eccentricity.

`env.ephemeris.orbit.*` has no Hydra CLI overrides or tab-completion keys because the nested default is `None`; configure it in Python:

```python
from srb.core.ephemeris import EphemerisCfg
from srb.core.ephemeris.orbit import OrbitSpec

cfg.ephemeris = EphemerisCfg(
    enabled=True,
    orbit=OrbitSpec(altitude_m=400_000.0, inclination_deg=51.6),
    eclipse=True,
)
```

### Eclipse

Disk overlap scales Sun intensity: `1.0` no overlap, `0.0` total, `1 − (a_occ/a_sun)²` annular, otherwise circular-lens partial-overlap area. No atmosphere, limb darkening, or extra penumbra gradient.

| Domain | Occluder / behavior |
| --- | --- |
| `ORBIT` | Central body synthesized from propagated observer position; no extra SPICE query. |
| `MOON` surface | Earth. Earth geometry is optional unless explicitly listed in `env.ephemeris.bodies`; absent optional geometry disables eclipse with a warning. |
| `MARS`/`EARTH` surface | No occluder; factor `1.0`, warning once per site body. Mars moons and Earth-site lunar/solar eclipse geometry are unsupported. |

`bodies` must include Sun. Accepted names: `sun`, `earth`, `moon`, `mars`, `venus`; duplicates/unknown names are config errors. SPICE supports all; table products always contain Sun and may contain Earth. A requested but unavailable product body raises typed `UnsupportedCapabilityError` (fallback allowed); an invalid name raises `InvalidInputError`. List non-Sun bodies when geometry is needed beyond eclipse's optional auto-add.

### Moving sun

`epoch.time_scale` must be finite and `>=0`; positive values update via `progress_sun_ephemeris` every `epoch.update_interval_s` simulation seconds (finite and `>0`) at `epoch0 + time_scale × elapsed_sim_time`, using fixed/reset lighting logic. `epoch0` is fixed UTC or the latest `utc_range`/`utc_windows` reset sample. Each reset resamples; shifts outside `de440s.bsp` query coverage (`1850-01-01`–`2149-12-31`) raise `_parse_epoch` `ValueError`.

With orbit eclipse and `time_scale=0`, reset uses `dt_s=0`, so eclipse remains fixed at `arg_lat_deg` for the episode. Set `time_scale>0` for eclipse cadence.

### Phase-dependent earthshine

Moon terrain bakes scale 200-lux earthshine by `f=(1+cos φ)/2`, where `φ` (`earthshine_fraction`) is the Earth phase between Earth→site and Earth→Sun. Full Earth gives `f≈1`; new Earth gives `f≈0`. Negative Sun elevation is retained; visibility sets intensity to zero below horizon. Environment mode has no earthshine prim.

### Observation vector

When enabled, Direct observations add `obs["ephemeris"]` with shape `(num_envs, 7)`:

| Slots | Values |
| --- | --- |
| `0:3` | Sun unit direction in robot base frame (inverse xyzw quaternion rotation). |
| `3:6` | Occluder unit direction in base frame; zero if no active occluder (eclipse off or none modeled). |
| `6:7` | Eclipse factor; `1.0` when eclipse is off. |

`ephemeris_observation` reads state cached at config/reset/interval, without SPICE or USD/Isaac work. It merges after the observation-delay wrapper (thus is not delayed); non-finite values become zero.

### Skydome co-drive

`drive_skydome` is inert in `ORBIT`. On surfaces with `drive_sunlight=true`, it sets skydome world yaw to `-azimuth_sun` on config/reset/interval and disables `randomize_skydome_orientation`. It requires `drive_sunlight`; yaw is deterministic and follows the Sun, but texture/cardinal alignment is not guaranteed because textures lack directional metadata.

## Memory and Threading Contract

`spice_provider.py` alone imports `spiceypy`; CLI help, completion, and parser construction remain Isaac-free. CSPICE's global kernel pool is not thread-safe across furnish/query cycles; `SpiceProvider` serializes access and furnishes once per process. Repeated queries reuse it. After fork (including `pytest-xdist` and multiprocessing data loaders), inherited SPICE state is uncertain; the next query calls `kclear()` before furnishing.
