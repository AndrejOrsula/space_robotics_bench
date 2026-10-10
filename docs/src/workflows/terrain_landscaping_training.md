# Terrain Landscaping Training

> No trained policy ships.

## Tasks

See [Mobile Robotics](../envs/mobile_robotics.md#terrain-landscaping-terrain_landscaping)
for the observation contract.

Targets: checked-in `crater_target_manifest.json`, `mound_target_manifest.json`;
general missions are hashed and mass-balanced.

| Task | Episode | Reward |
| :--- | :--- | :--- |
| [Crater](../envs/mobile_robotics.md#terrain-landscaping-crater-terrain_landscaping_crater) | 300 steps (30 s) | grading MAE |
| [Mound](../envs/mobile_robotics.md#terrain-landscaping-mound-terrain_landscaping_mound) | 300 steps (30 s) | excess volume above target |
| [General](../envs/mobile_robotics.md#terrain-landscaping-terrain_landscaping) | 600 steps (60 s) | grading MAE |

Documented success target (all tasks): at least 90% of observable grading cells
within 0.02 m of the target for 20 consecutive accepted frames, reported at
truncation. This target has not been validated against a physical system.

Choose `env.external_heightmap.profile`: `ideal` (default, no error),
`randomized` (up to 0.1 s delay, 5% dropped frames, 2 mm elevation noise, small
registration and pose errors), or `failure_stale` (stale-frame abort). Seeded
defaults do not represent measured lab errors.

## Training

Crater and mound auto-load task-local DreamerV3 and skrl PPO profiles, shadowing
global profiles; general uses global profiles. See
`hyperparams/task/terrain_landscaping_{crater,mound}/` for these files.

```bash
srb agent train --headless --algo dreamer -e terrain_landscaping_mound env.num_envs=1
srb agent train --headless --algo skrl_ppo -e terrain_landscaping_crater env.num_envs=1
```

- **Dreamer:** strict merge rejects unknown or dropped keys. Profiles use
  10,000 steps and `train_ratio: 512.0`; gradient updates equal
  `steps × train_ratio ÷ (batch_size × batch_length)`.

- **One environment:** Dreamer landscaping rejects `env.num_envs > 1`; mixed batches cannot
  selectively reset. Stacked multi-environment particle scenes are unsupported;
  multi-environment particle training has not been validated.

- **TD-MPC2 disabled:** all three tasks fail before creating logs, config,
  replay, or model:

  ```text
  TD-MPC2 landscaping is disabled: velocity-controlled shovel extension is unobserved and TD-MPC2 keeps no recurrent or history state to infer it.
  ```

- **GPU required:** PhysX particles need a GPU; tasks read them through
  CPU-facing USD (`sim.device = "cpu"`). "Force CPU" disables particles; see
  [Particles](../config/particles.md#where-particles-are-simulated-vs-where-they-are-read).

### Dreamer checkpoints

| Operation | Restores | Does not restore |
| :--- | :--- | :--- |
| `--continue` (same log directory) | step, agent parameters, optimizer, replay | simulator state, partial episodes, recurrent carries, RNG |
| Agent import into new run | agent state; starts at step 0 | everything else |
| Evaluation or policy load | agent state for inference | everything else |

Resume is not bitwise; it fails when `ckpt_generation.json` lacks replay chunks.
Loads verify task ID, IO-schema fingerprint, observation layout, target
manifest, measurement profile, and normalization against the live environment.
`SRB_ALLOW_UNVERIFIED_LANDSCAPING_CHECKPOINT` allows missing metadata, not mismatches.

## Results and limitations

- **Crater:** no shipped controller meets the success criterion; a pushing blade disturbs the
  graded surface about as fast as it fills the crater.
- **Mound:** the scripted reference clears 1.75 L.
- **General task:** the scripted reference does not improve on zero action.
- Leaving the mission-pose (±2.0 m) or body-velocity envelope terminates the
  episode. Out-of-envelope elevations raise `ExternalHeightmapFrameError` (see
  [Modelling Envelope](../misc/modelling_envelope.md)).
- Particle behavior is simulation-only and not calibrated against regolith.

## Deployment

See [Sim-to-Real](sim_to_real.md#52-terrain-landscaping)
for bridge status, capabilities, and crater-spec validation. [Validation
Workflow](sim_to_real.md#6-validation-workflow) covers dry runs.

Next: [Environments](../envs/index.md)
