# Environment Configuration — Parallelism

Set `env.num_envs` to choose the number of parallel simulation instances.

## Number of Environments

`env.scene.num_envs` sets the count; `env.num_envs` overrides it when set:

```bash
srb agent rand -e _manipulation env.scene.num_envs=16
srb agent zero -e _manipulation env.num_envs=128
```

Instances share one batched `DirectEnv`; cross-environment collisions are
filtered. Visual sensors are not guaranteed isolation: spacing or sensor range
alone may not prevent cross-environment visibility.

## Environment Spacing

`env.scene.env_spacing` sets environment distance. `env.spacing` overrides it
when set; otherwise the scene value is inherited:

```bash
srb agent rand -e _manipulation env.num_envs=64 env.scene.env_spacing=20.0
srb agent rand -e _manipulation env.num_envs=32 env.spacing=10.0
```

> **Note:** `env.stack=true` forces `env.scene.env_spacing=0.0`; otherwise the
> framework applies `env.spacing` when set or inherits `env.scene.env_spacing`.

## Environment Stacking

`env.stack` controls asset sharing:

### Independent Environments (`env.stack=false`)

- ⚠️ Each environment has its own scene and position
- ✅ Greater visual and physical diversity
- ✅ Supports tasks with visual sensors
- ❗ Slower initialization
- ❗ Higher memory use

```bash
srb agent rand -e _manipulation env.num_envs=16 env.stack=false
```

### Stacked Environments (`env.stack=true`)

- ⚠️ Environments share scenery and position
- ✅ Faster initialization
- ✅ Lower memory use
- ❗ Less environmental diversity
- ❗ Visual sensor compatibility and camera isolation are not guaranteed; verify
  the task before relying on them
- ❗ Stacked multi-environment particle simulation is unsupported: with
  `env.particles=true` and `env.num_envs != 1`, the framework forces
  `env.stack=false` and logs a warning

```bash
srb agent rand -e _manipulation env.num_envs=16 env.stack=true
```

> **Note:** Set `env.stack=false` for visual observations or independent
> scenery. Defaults are task-specific. These task IDs use
> `env.stack=true`: `excavation_demo`, `lunalab`, `velocity_tracking`,
> `locomotion_velocity_tracking`, `waypoint_navigation`,
> `locomotion_waypoint_navigation`, `orbital_waypoint_navigation`,
> `formation_following`, `formation_following_decentralized`,
> `formation_following_orbital`, `moonfall`, `traversal`, and `landing`.
> Stacking shares scenery and forces spacing to zero:
>
> ```bash
> srb agent train --algo dreamer --env waypoint_navigation \
>   env.num_envs=64 env.stack=false
> ```
