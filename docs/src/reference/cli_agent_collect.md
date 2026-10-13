# `srb agent collect` — Collect Demonstrations

## Usage

```bash
srb agent collect --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

| Option | Description | Default |
| --- | --- | --- |
| `--collector` SOURCE | `human`, `policy`, `random`, or `zero`; defaults to `policy` with `--algo`/`--model`, else `human`. | inferred |
| `--num_episodes` N | Episodes to collect. | `100` |
| `--horizon` N | Max steps/episode. Horizon cuts and mid-episode simulator closes record the last row as `truncated=True`, `done=True`, not terminal success/failure. | `400` |
| `--success_only` | Save successful demos only; see [Tips](#tips). | `False` |
| `--teleop_device` [DEV ...] | `keyboard`, `spacemouse`, `ros`, `gamepad`, `haptic`. | env default, else `[keyboard]` |
| `--pos_sensitivity` VAL | Translation sensitivity. | `1.0` |
| `--rot_sensitivity` VAL | Rotation sensitivity. | `π` (≈3.14) |
| `--invert_controls`/`--invert` | In manipulation, invert only the first two translational controls; `--no-invert_controls`/`--no-invert` disable. | `True` |
| `--algo` ALGO | RL or imitation-learning policy algorithm. | — |
| `--model` MODEL | Raw checkpoint or SRB model artifact path. | — |
| `--obs` {state,visual} | Select policy config variant and latest-checkpoint log tree. | `state` |

## Collection Modes

### 1. Teleoperation (default)

Without `--algo`/`--model`, collect human actions and observations:

```bash
srb agent collect --env peg_in_hole --num_episodes 50
```

Keyboard teleop needs GUI; with `--headless` it fails before collection. Choose another `--teleop_device` or remove `--headless`. XR uses `--xr`; see [Extended Reality](../integrations/extended_reality.md).

For real teleoperation, `srb real_agent collect --collector human` requires generated `COMMAND_MAPPING` metadata; see [native direct-input contract](cli_real_agent_gen.md#native-direct-input) for devices and episode controls. Scalar RealEnv HDF5 steps have single-environment batch axes (`split=real`, `controller=human`).

### 2. Policy Rollout

Set `--algo` (optionally `--model`) to roll out a trained policy:

```bash
srb agent collect --env peg_in_hole --algo sbx_ppo --num_episodes 200
```

SRB supports `dreamer`, `tdmpc2`, `rsl_rl_ppo`, `sb3_*`, `sbx_*`, `skrl_*`, and `robomimic_*`. Without `--model`, it loads the latest standard training-logdir checkpoint and reuses that algorithm's latest run. Native RSL-RL `model_*.pt` and TD-MPC2 `models/*.pt` resolve as in evaluation. `--model` accepts raw checkpoints or SRB model artifact directories. An adjacent `config.yaml` supports stateful/config-heavy Dreamer, TD-MPC2, and RSL-RL adapters outside the original run. Metadata stores checkpoint/source as `model`/`model_artifact`.

### 3. Built-in Sources

Use `--collector random`/`zero` for baseline data:

```bash
srb agent collect --env peg_in_hole --collector random --num_episodes 20
srb agent collect --env excavation --collector zero --num_episodes 5 --horizon 200
```

`random` samples the action space using `env.seed` (same seed repeats actions). Split is independent: real collection may use `split=real` with `controller=random`, `controller=zero`, `controller=human`, or `controller=policy`.

### Multi-Modal Recording

Add `visual` to `env.active_sensors` for RGB-D and `events` for the [event camera](../sensors/event_camera.md), e.g. a floating platform in the Zero-G lab:

```bash
srb agent collect --headless --env zerog_lab --collector random \
  --num_episodes 10 env.num_envs=1 \
  'env.active_sensors=[proprioception,contact,transform,visual,events]' \
  --record-events
```

Each row stores RGB-D `image_<camera>` (`uint8`, depth in the last channel), the `event_<sensor>` tensor, the state `vector`, action, and reward. `--record-events` adds raw event HDF5, RGB/depth MP4s, and `events_manifest.jsonl` under `<run logdir>/camera_videos/<timestamp>/` (the input of `srb dataset` event exports). Set `env.camera_resolution=[W,H]` and `env.event_camera_resolution=[W,H]` to size the images.

## Output

Robomimic-compatible HDF5 output:

| Collection | Dataset directory |
| --- | --- |
| Human, random, zero | `logs/<env_id>/collect/<timestamp>/datasets/` |
| Policy | Resolved run directory, e.g. `logs/<env_id>/<algo>/<timestamp>/datasets/` |

Train from the dataset with [`srb agent learn`](cli_agent_learn.md); see [Imitation Learning Workflow](../workflows/imitation_learning.md).

Logs default to `<checkout>/logs` in a source checkout or `<cwd>/logs` when installed. Override the location with `--logdir`/`--logs`.

Policy reuse stores `metadata.json`, `run.json`, and `.hydra/` under `collect/<timestamp>-<suffix>/`, preserving training metadata/config. SRB artifacts are read in place. Metadata records the source, episode controls, teleop devices, and reproduction command.

If collection saves no episodes, it reports `no episodes were saved after N attempt(s)` and writes no dataset. If it saves fewer than requested, it keeps the dataset and warns with saved/requested counts.

Autonomous collection makes up to three attempts per requested episode. All modes stop when the simulator closes. A finite positive unwrapped `step_dt` sets autonomous/human sidecar `fps=1/step_dt`; otherwise timing is unknown. LeRobot v3 requires known timing; see [dataset CLI](cli_dataset.md).

### Observation format

Per-sample low-dimensional observations have shape `[D]`; cameras have shape `[H, W, C]` (Robomimic accepts cameras as images).

On errors or Ctrl-C, collection publishes saved episodes with masks and an SRB sidecar. Demos store `success`; LeRobot exports it only on the final row as Boolean `success`.

Simulated `uint8` depth is normalized over near/far and scaled by 255; it saturates beyond far or on empty rays. `env.camera_clipping_range=[near,far]` fills visual cameras retaining the render-default far plane (1000 m or more), preserving explicit ranges. Orbital cameras use `(0.01, 100)` m (about 39 cm/level), `rendezvous` `(0.01, 50)` m, `orbital_evasion` `(0.01, 150)` m, `landing`/`moonfall` `(0.01, 400)` m, and other default-far cameras 20 m (about 8 cm/level). Real-camera depth is metric `float32`, unlike simulated depth.

With `visual` or `events` in `env.active_sensors`, SRB raises `env.num_rerenders_on_reset` to at least 1 so the first episode image and event baseline show the reset scene. This adds one render per step where an environment resets. Set `env.camera_rerender_on_reset=false` to opt out.

Datasets omit policy determinism. SRB reports only plain `bc` (MLP, RNN, transformer) as deterministic when the saved config disables GMM/Gaussian/VAE heads. Other algorithms/heads (e.g. GMM, VAE, diffusion, BCQ), or omitted head settings, are reported as non-deterministic or unknown; bounded evaluation (`SRB_EVAL_EPISODES`) refuses them.

## Tips

`--success_only` checks the success term or, if absent, ending-step episode `success` info at the final row (termination, horizon, simulator shutdown, or teleop save). Earlier success does not count.

After reset, use episode info because the term sees the next episode. If an environment-ended episode publishes no `success` info, collection warns that success is unavailable and records it as false. Orbital tasks and `spacewalk` (never terminates; truncates at the time limit) are judged at the horizon.

Tasks with `success_signal = "episode_info"` in `docs/data/tasks.toml` (e.g. `excavation`, `formation_following`, `beam_transport`) publish the flag.

Without either signal, autonomous collection stops with `--success_only needs a success signal`. Keyboard saves count only with the task flag; teleop-device saves count as success. Real collection follows these rules; terminal `success` requires an evaluator, and `srb real_agent` attaches none.

Every mode requires `env.num_envs=1`. Otherwise, collection raises `robomimic collection requires num_envs == 1` before creating dataset files. Set the Hydra override explicitly.
