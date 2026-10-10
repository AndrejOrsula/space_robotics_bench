# Randomization, noise and delay profiles

SRB configures three things separately: event randomization, action and
observation delay, and Isaac Lab action-noise models.

| Task/config | Randomization | Delay | Noise | Provenance |
| --- | --- | --- | --- | --- |
| `waypoint_navigation` | `target_pose_evolution`, a tagged task event at a 0.05 s interval | Action `(0, 3)` and observation `(0, 1)` agent steps; per-reset draws use `env.seed` | Waypoint-relative observation noise: 0.01 m / 2.5° episode bias plus 0.0025 m / 0.5° per-step noise | Task defaults; uncalibrated |
| `locomotion_waypoint_navigation` | Inherits `target_pose_evolution` | Inherits action `(0, 3)` and observation `(0, 1)` agent steps | Inherits waypoint-relative observation noise: 0.01 m / 2.5° episode bias plus 0.0025 m / 0.5° per-step noise | Inherited waypoint task config; uncalibrated |
| `waypoint_navigation` with `env.sim_to_real_profile=waypoint_navigation_synthetic_v1` | Same target event | Action and observation `(1, 2)` agent steps, fixed for each episode; 25 Hz means 40–80 ms | Gaussian action noise, standard deviation 0.02 in normalized policy-action units; the waypoint observation noise above remains in metres and radians | `synthetic_unmeasured` |
| Other shipped task configs | Many define tagged or untagged event terms | No other task config enables nonzero action or observation delay | No default task config enables Isaac Lab `action_noise_model` or `observation_noise_model` | Per-task terms |

Opt in to the synthetic profile:

```bash
srb agent train --algo sbx_ppo --env waypoint_navigation \
  env.sim_to_real_profile=waypoint_navigation_synthetic_v1 \
  env.seed=20260908
```

The base config sets `env.seed=0`. `default_seed` applies only when the task config has
`env.seed=None`; any explicit seed, including `0`, wins.

Delay ranges count integer agent steps. At 25 Hz, one step is 0.04 s. On reset,
each environment samples its delay. Observation-delay history starts with the
fresh observation; action-delay history starts with zeros.

Override delay and noise fields only with values matching the task default or
the profile; other values fail. Zero change probability disables boundary
redraws, so subset resets cannot reuse another environment's history.
The profile requires `agent_rate=0.04` seconds and rejects custom
`action_noise_model` to keep delay and noise units fixed.

The `synthetic_unmeasured` profile has synthetic delays and Gaussian action
noise, not hardware calibration, sensor statistics, or latency measurements.

`RandomizationView` exposes active event terms and reports untagged
`EventTermCfg` fields as `UNTAGGED`. The profile does not retag other task
configs or enable noise globally.
