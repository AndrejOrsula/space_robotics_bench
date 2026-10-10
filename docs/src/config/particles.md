# Environment Configuration — Particles

Set `env.particles=true` to use granular materials such as regolith, sand, and dust. Requires Isaac Sim 6.1.0, Isaac Lab 3.0.0, PhysX, and CUDA GPU dynamics.

```bash
srb agent teleop -e _manipulation env.particles=true \
  env.particles_size=0.01 env.particles_ratio=0.1
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/XIJhFsaT1Ls?si=xJTKjGKLHRyMz4rO&mute=1&autoplay=1&loop=1&playlist=XIJhFsaT1Ls" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Enabling Particles

1. Set `env.particles=true`.
1. Choose parameters from the table.

Defaults are in `BaseEnvCfg` (`srb/core/env/common/base/env_cfg.py`). Scalars apply to all systems. Two-value ranges interpolate across `env.particles_systems`. Set `particles_init_pos` to `(x, y, z)`.

## Configuration Parameters

| Parameter | Meaning | Default |
| --- | --- | --- |
| `env.particles` | Enable | `false` |
| `env.particles_systems` | Independent systems per environment | `1` |
| `env.particles_spawner` | Pile kernel: `grid`, `pyramid`, `crater_pile`, or `mound_pile` | `grid` |
| `env.particles_size` | Diameter (m) | `(0.01, 0.005)` |
| `env.particles_area` | Pile width relative to environment spacing | `(0.75, 0.5)` |
| `env.particles_ratio` | Volume-fill ratio | `(0.025, 0.1)` |
| `env.particles_height` | Integer stack height per system | `(1, 10)` |
| `env.particles_density` | PBD density (kg/m³) | `1600.0` |
| `env.particles_friction` | Particle friction coefficient | `1.0` |
| `env.particles_damping` | Velocity damping | `0.2` |
| `env.particles_cohesion` | Particle stickiness | `0.01` |
| `env.particles_adhesion` | Surface stickiness | `0.05` |
| `env.particles_material` | Advanced, non-scalar visual override (`VisualMaterialCfg`) | `None` |
| `env.particles_dim_xy` | Grid count per system `(dim_x = dim_y)` | Derived (`None`) |
| `env.particles_init_pos` | First-system position `(x, y, z)` | Derived (`None`) |
| `env.particles_freeze` | Freeze after settling but keep rendering | `false` |
| `env.particles_conform_to_terrain` | Snap to terrain without PBD settling | `false` |
| `env.particles_conform_jitter` | Conforming XY scatter (m); `0` keeps a regular grid | `0.0` |
| `env.particles_pile_center_jitter` | Settled-pile XY offset per reset (m); `0` disables | `0.0` |

## Behavior

`grid` fills `particles_area`. `pyramid` makes centre-dense piles. Excavation derives `particles_dim_xy`, `particles_height`, and `particles_init_pos` from `particles_size` (a 0.35 m wide, 0.4 m tall pile at the scoop target) and ignores overrides of those three.

Multiple systems (`particles_systems>1`) stack vertically and interpolate settings across systems. Particles settle under gravity, collide with rigid bodies and articulations, and can be manipulated. They disable Fabric (`env.sim.use_fabric=false`) for consistent PhysX state.

Excavation and terrain landscaping track reshaping in runtime heightmaps because PhysX sets cannot be raycast. Landscaping's versioned [external-heightmap interface](../envs/mobile_robotics.md#terrain-landscaping-terrain_landscaping) uses a fixed footprint-aware surface rule, not ad-hoc binning.

## Where Particles Are Simulated vs. Where They Are Read

PhysX PBD requires GPU dynamics and a CUDA-capable NVIDIA GPU. Otherwise each set fails at parse time with `Particles feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!`; particles stay bit-identically inert while rigid bodies move.

The GPU solver exposes CPU-facing USD points (`UsdGeom.Points`) through `/physics/updateParticlesToUsd`. Isaac Sim 6's CUDA direct-data pipeline suppresses readback, so SRB uses CPU simulation with GPU scene dynamics. Landscaping sets `sim.device = "cpu"` so its heightmap, reward, and renderer read USD points.

App-global `/persistent/physics/overrideGPUSettings` overrides authored `physxScene:enableGPUDynamics=true`. “Force CPU” persists `0` in `kit/data/Kit/IsaacLab/*/user.config.json`, disabling particles; SRB clears it at spawn with a warning. Values are `-1` per scene, `0` for CPU, and `1` or `2` for GPU.

Each set's `simulationOwner` matches `env.sim.physics_prim_path`. Missing owners or non-`UsdPhysics.Scene` targets fail fast without stage traversal. Ownership makes scene assignment deterministic; unowned sets move, so missing ownership alone does not explain inert particles.

Systems are siblings (`<env>/particles0System`). Isaac Lab clones env_0's set and target for per-environment variants such as procedural terrain; SRB retargets clones after scene cloning and before physics (`DirectEnv._setup_scene`). Initialization rejects foreign or shared systems, which solve in the wrong collision group and fall through terrain.

Settling raises instead of caching if any environment's median height drops over `settle_max_median_drop` (10 m). Plateaus count as settled only below `settle_plateau_max_vel` (0.1 m/s); terminal-speed free fall also plateaus.

SRB converts cloned positions and velocities between local and world-frame `ParticleSystem` buffers, binding each clone to its own system. `ParticleSystem.initialize()` validates particle-system binding; runtime simulation behavior still needs testing; multiple environments force `env.stack=false` (see [Parallelism](parallelism.md)). Below-terrain grains may evade the median guard, remain cached, and fall every reset.

## Domain Randomization

| Randomization | Behavior |
| --- | --- |
| `env.particles_pile_center_jitter` | Each reset restores and rigidly translates the settled pile in XY by independent per-environment samples in `[-jitter, +jitter]`, without re-settling. `0.05` shifts ±5 cm without changing its profile; default `0` keeps it fixed. Ignored with `env.particles_freeze=true`. |
| Excavation and terrain-landscaping material | Each reset samples PBD friction `[0.6, 1.0]` and density `[2700, 3100]` kg/m³, varying repose angle and bulk behaviour. Task-wired, without an `env.*` knob; friction and density are spawn baselines, not episode constants. |

## Static Renders

For frozen renders, `env.particles_freeze=true` disables systems after settling by clearing PhysX `particleSystemEnabled` and zeroing velocities; points remain visible. Pair with [`--snapshot`](../reference/cli.md). `env.particles_conform_to_terrain=true` snaps the grid to terrain without PBD settling; `env.particles_conform_jitter` adds XY scatter. Use when settling is unavailable and regolith needs terrain relief.
