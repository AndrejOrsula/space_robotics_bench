# Modelling Envelope

## 1. Dynamics

### Orbital tasks are microgravity, not orbital mechanics

`ORBIT` sets gravity magnitude and variation to 0.0. It supports zero-gravity contact, manipulation, and free-flyer tasks.

It omits Clohessy-Wiltshire/Hill relative motion and orbital rate. It cannot support formation flight, rendezvous geometry, or station-keeping fuel budgets.

### Manipulator arms do not feel domain gravity

Twenty robot/object files contain 24 `disable_gravity=True` declarations across manipulators, grippers, tools, floating, and aerial robots.

PhysX gravity is correct to five decimals in all five domains, but affects only free bodies and mobile bases. `MOON`, `MARS`, and `EARTH` change lighting and unattached objects, not arm loads. The excavation shovel is weightless.

Raph's hardware shovel has one lift joint and no curl joint; it only raises or lowers the blade. These assumptions suit orbital arms, not surface arms. Do not claim cross-domain gravity transfer.

### Aerial tasks are kinematic, not aerodynamic

Aerial tasks support navigation and guidance above flight control.

The multicopter term writes root velocity and matching attitude. Nominal-RPM rotors are visual and produce no thrust. There is no aerodynamics, rotor inflow, blade-element model, or airframe gravity. It follows body-frame velocity commands with bounded tilt.

Do not claim flight dynamics, control authority, disturbance or wind response, power, or endurance.

### Large spacecraft inertia comes from a uniform bulk density

Large spacecraft without authored mass use uniform 1500 kg/m³ density, several times higher than mostly hollow spacecraft. Scaling corrects mass, not inertia: uniform solids differ from wall-loaded structures, and inertia ratios govern attitude dynamics.

Landers have explicit mass, enabled gravity, and per-thruster body forces via `ThrustAction`. Do not use large uniform-density spacecraft for attitude-control results.

| Lander | Mass |
| --- | ---: |
| `Starship` | 100,000 kg |
| `SuperHeavy` | 200,000 kg |
| `ApolloLander` | 4,280 kg dry |
| `PeregrineLander` | 825 kg |
| `VikramLander` | 626 kg |
| `ResilienceLander` | 340 kg |

`mobile/landing` supports powered descent.

## 2. Terrain and contact

### There is no terramechanics, and friction is profile-specific

Use wheel-ground contact for navigation, path following, and obstacle negotiation. Do not use it for slip-ratio/sinkage predictions, traction budgets, or regolith trafficability.

`BaseEnvCfg` defaults to 0.50/0.50 Coulomb friction (static/dynamic), 0.0 restitution, and average combine mode. Wheels use actuator-driven velocity/position targets on rigid surfaces. There are no sinkage, pressure-sinkage, slip-sinkage coupling, bulldozing resistance, drawbar pull, or Bekker/Wong models.

The baked terrain path can apply these unvalidated engineering estimates. The registry has no asteroid entry.

| Body | Static/dynamic friction | Restitution |
| --- | ---: | ---: |
| Moon | 0.85/0.70 | 0.0 |
| Mars | 0.75/0.60 | 0.0 |
| Earth | 0.80/0.60 | 0.1 |

Without baked material, the global default is a parameter, not calibration. Pre-materials-v3 bakes lack bindings.

To restore bindings:

- Re-bake with `srb asset download terrain <site>`.
- For a region, add `--body`, `--lat`, `--lon`, and `--size-m` to that command.
- Alternatively, enable auto-fetch.

With `SRB_TERRAIN_AUTO_FETCH=0`, direct terrain construction raises `TerrainNotBakedError` for unbaked terrain; there is no old-bake fallback. Runtime surface selection instead warns and falls back to procedural terrain. Isaac Sim contact effects are unvalidated; asset materials contribute through average combine.

### Self-collision is disabled on most articulations

Self-collision is off on roughly 70% of articulations, so robots may pass through themselves in hardware-unreachable configurations. Require an independent collision check before claiming whole-body transfer.

### Actuators are implicit PD

Many actuators use implicit PD; some configurations include joint friction. Most omit backlash, gear compliance, current, and thermal limits. Do not claim torque-tracking fidelity or results dependent on actuator dynamics rather than kinematics and contact.

## 3. Granular material

### Cohesion and adhesion settings are ineffective for the current non-fluid particle setup

Excavation particles set `cohesion` and `adhesion` to 0.0. Landscaping sets cohesion to 0.0 and inherits adhesion 0.05; other scenes inherit base values 0.01/0.05.

PhysX applies cohesion and adhesion only to fluids, so they do not affect non-fluid PBD particles; friction does. The angle of repose is uncalibrated against lunar regolith and simulants. Without cohesion, particles cannot form bootprints or trench walls or hold scoop loads; another representation is required.

Do not claim excavation forces, scoop fill, trench-wall stability, bootprints/tracks, or simulant comparisons.

### Particles are two to three orders of magnitude too large

Excavation particles are about 0.7 cm; landscaping particles are 5 cm. Lunar regolith's median grain size is about 70 µm (Carrier et al., *Lunar Sourcebook*, 1991), requiring about 10¹² particles per cubic metre.

Coarsening is unavoidable. The model lacks grain interlocking, fines-scale cohesion, and sub-particle features. Do not claim results below particle diameter, including tool-edge detail or fine texture.

## 4. Real terrain

### Measured terrain resolves at source-specific native GSD; finer detail is synthetic

Use measured terrain for site-specific slope, illumination, and route-scale shape at source GSD. Do not use it for wheel-scale traversability. The real-data path is not fully validated.

Its pipeline records curated, SHA-256-pinned LROC NAC DTMs, `IAU_Moon_2015` CRS, per-site GSD, licence, attribution, and SPICE sun-position epochs. Native GSD varies by source:

| Source | GSD |
| --- | ---: |
| Mars HiRISE | `1.016553302... m` |
| Earth 3DEP | `1.0 m` |
| Earth Copernicus | `30.0 m` |
| Moon | `1.0–1000.0 m` |

Resampling and fusion add no detail below source GSD. Procedural rocks, craters, and noise supply synthetic rover-scale roughness, even at real sites. See [Terrain Stacks](../config/terrain_stacks.md) for site and multi-source stacks.

## 5. Sensing

### Noise and delay lack target-hardware calibration

Default task configs leave Isaac Lab's `observation_noise_model` and `action_noise_model` unset. Waypoint navigation retains relative observation perturbations. Opt-in `waypoint_navigation_synthetic_v1` adds Gaussian action noise and `(1, 2)`-step action/observation delays.

See [randomization profiles](../config/randomization_profiles.md) for parameters and reset behavior. Noise lacks hardware calibration and realistic IMU/image models. Do not claim target-device robustness without measurements.

### Depth noise is modelled on consumer hardware

Heightmap and depth models include quadratic stereo and time-of-flight noise, quantisation, incidence-angle and random dropout. Parameters come from consumer Intel RealSense devices; space-qualified sensors may differ.

Calibrate and re-parameterise for the target device before claiming sensor-specific transfer.

## 6. Illumination and appearance

### Light intensity is documented in W/m² but consumed as a renderer scale

Use configured sun elevation, azimuth, and shadow direction; SPICE supplies these when `env.ephemeris` is enabled. Uncalibrated rendered radiance gives no absolute photometry or radiometry.

`Domain.light_intensity` is documented in W/m², but USD `DistantLightCfg` consumes renderer scale, not irradiance. Nominal intensities are 1361 W/m² for Moon/orbit and 775 W/m² for Earth; Earth's range reaches 1000 W/m² (ASTM G173 AM1.5 clear-sky surface value). Mars's 590 W/m² is top-of-atmosphere, despite the docstring; SRB omits typical surface irradiance (~60–70%, dust-dependent).

### Ambient sky fill is one undifferentiated constant

All domains, including airless ones, apply `DomeLight` at 25% of direct intensity, without a derivation for `0.25`. It barely affects Moon shadows because `stars.exr` is near-black. Changing the constant or texture could sharply alter lighting.

### Mars sunlight is not reddened

Iron-oxide dust reddens Martian sunlight, producing a butterscotch appearance. SRB uses the Sun's 5778 K colour temperature and has no calibrated reddening value. Do not claim colour-accurate Mars imagery.

### The default lunar surface material is the wrong colour

Upstream `simforge_foundry` owns the installed `MoonSurface` material. It renders red-brown to magenta under solar illumination, unlike lunar regolith's grey with a slight brown cast. Geometry, illumination direction, and shadows are unaffected. Do not use default-path datasets for colour-dependent perception on procedural lunar terrain.

### The Earth domain renders a starfield sky

`Domain.EARTH` uses `stars.exr` as its skydome, giving daytime scenes a night sky; daylight textures exist in the asset pack. Do not transfer Earth-domain background vision models to real laboratories.

## 7. Time discretisation

### Aerial tasks run at most two physics steps per control step

Do not draw aerial-dynamics conclusions near control-rate frequencies.

| Task config | Physics interval | Control interval | Decimation |
| --- | ---: | ---: | ---: |
| Aerial family default, including `aerial_navigation` | 1/25 s | 1/25 s | 1 (40 ms) |
| `flight` | 1/50 s | 1/25 s | 2 (two 20 ms physics steps per 40 ms control update) |

### GPU contact-buffer sizing is on Isaac Lab's defaults

SRB applies no per-`num_envs` GPU buffer sizing; PhysX buffers keep Isaac Lab's defaults. CPU defaults are unaffected; check PhysX overflow warnings on CUDA runs with large `num_envs`.

## 8. Domain constants

### Asteroid gravity randomisation reaches zero

`ASTEROID` gravity variation is twice its magnitude, giving a sampled lower bound of 0.0 m/s². Some episodes run in free fall.

### No task defaults to the Asteroid domain

`srb/core/domain.py` declares five domains. Task configs explicitly default to `MOON` (6), `MARS` (4), and `ORBIT` (2), with none for `ASTEROID`; others inherit their family default. `modular_assembly` keeps the default domain but sets `gravity` to `EARTH` (`srb/tasks/manipulation/modular_assembly/task.py:170`). Do not treat an `ASTEROID` switch as validated.

## 9. Training configuration

### Two of the 42 tasks have tuned hyperparameters

`hyperparams/` has 28 reinforcement-learning and 11 imitation-learning configs; `hyperparams/task/` overrides two of 42 tasks. The other 40 use defaults, not best-achievable settings. Untuned scores are unfair across algorithms; published scores are not tuned or configuration-matched.

## Known limitations

| Area | Limitation |
| --- | --- |
| PDS3 conversion | `srb.terrain.sources.pds_img.img_to_geotiff` writes raw samples, ignoring `SCALING_FACTOR`/`OFFSET`; optional `pdr` fallback omits CRS, identity transform, and nodata. |
| Ortho fusion | `ortho_path` resizes images to DEM grid by array shape; images must be pixel-aligned. Fused colour is aligned. |
| Resolution | Reads finer than native resolution may not average; elevation can alias. |
| SpaceMouse | Use an independent deadman with a real robot: reads may succeed while reports are lost, keeping the last motion. |
| Dreamer | Real-transfer checkpoints save periodically, not on normal stop. |
| Installers | Isaac Lab, Dreamer, RSL-RL, and Robomimic installers in `.docker/install/` retain checkouts and leave pip packages in the Isaac Sim interpreter after failure; recreate that environment to remove them. |
