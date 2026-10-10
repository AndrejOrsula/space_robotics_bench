# Environment Configuration

Configure environments with [Hydra](https://hydra.cc).

## Command-Line Overrides

```bash
srb agent <WORKFLOW> --env <ENV> \
    env.domain=moon \
    env.robot=ur10 \
    env.num_envs=4 \
    env.stack=false \
    ...
```

## Configuration Files

Each environment's `@configclass` extends [`BaseEnvCfg`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/core/env/common/base/env_cfg.py). Hydra loads task Python defaults, merges CLI overrides, and writes the resolved config to `<logdir>/.hydra/config.yaml` unless the command disables its output directory.

Save overrides in YAML and pass them with `--cfg`:

```bash
srb agent zero --env peg_in_hole --cfg ./my_overrides.yaml
```

For example, `my_overrides.yaml` can contain:

```yaml
env:
  domain: mars
  num_envs: 8
  robot: ur10
```

> **Note:** `--cfg default` uses the environment default; `--cfg ignore`, `none`, or `null` disables the config file.

> **Tip:** Dict-form asset shortcuts expand automatically: `env.robot=franka` becomes `env.robot.name=franka`.

## Key Parameters

### Scenario/Environment

- `env.domain` - [Domain](domain.md)
- `env.ephemeris` - [Ephemeris](ephemeris.md)
- `env.robot` - [Robot](robot.md)
- `env.scenery` - Set a registry asset name, such as `env.scenery=moon_surface`.
  - For asset-instance scenery, set fields such as `env.scenery.flat_area_size=3.0` or `env.scenery.boulder_count=0`.
  - If the task accepts a scenery variant (`Scenery | AssetVariant`), you can also set `body`, `lat_deg`, `lon_deg`, and `size_m`. Set `lat_deg`, `lon_deg`, and `size_m` to construct `RealTerrain`. The default `body` is `env.domain` (`moon`, `mars`, or `earth`).
  - Fixed-type scenery, such as `terrain_landscaping`'s `Plane`, accepts only that asset's fields.
- `env.robot_spawn_height_range` - Ground tasks only: root height offset range in metres above the robot's default spawn height, applied on reset, e.g. `env.robot_spawn_height_range=[0.3,0.5]`. If unset, the task's `randomize_robot_state` range applies.
- `env.num_envs`, `env.stack` - [Parallelism](parallelism.md)
- `env.particles` - [Particles](particles.md)
- `env.reward` - [Reward & Normalization](reward_cfg.md)

### Simulation

- `env.sim` - Low-level simulation parameters (physics, rendering, etc.)
- `env.visuals` - Visual appearance settings

### Debugging

- `env.debug_vis` - Enables debug visualization features
