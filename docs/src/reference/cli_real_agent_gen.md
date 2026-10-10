# `srb real_agent` — Sim-to-Real Bridge

## Subcommands

```bash
srb real_agent <action> [options]
```

| Action | Purpose |
| --- | --- |
| `gen`, `sim2real_gen` | Generate Gymnasium counterpart (`sim2real_gen` alias). |
| `zero`, `rand` | Zero/random actions. |
| `teleop`, `ros` | Direct input; ROS 2 / Space ROS topics. |
| `train`, `eval` | Train/evaluate hardware policy. |
| `collect` | Human, policy, random, zero datasets. |
| `validate` | Run hardware telemetry checks and report pass/fail. |
| `rebuild-summary` | Rebuild `summary.json` / `report.md`. |
| `replay-twin` | Compare session with baseline `.rrd` in Rerun. |

For simulation-side counterparts, see [`srb agent`](cli.md#agent-subcommands).

## `srb real_agent gen`

1. Generate a hardware counterpart with Isaac Sim. Its generated runtime targets hardware:

   ```bash
   srb real_agent gen --env ENV_ID [options]
   srb real_agent gen --env waypoint_navigation env.robot=leo_rover --hardware ros_cmd_vel ros_tf ros_mw
   ```

   | Option | Default / limit |
   | --- | --- |
   | `-e`, `--env`, `--task` ENV_ID | Required; `ALL` selects cached non-template environments. |
   | `--cfg` PATH | `DEFAULT`; Hydra values `DEFAULT`, `IGNORE`, `NONE`, `NULL`. |
   | `--hardware`, `--hw` [HW ...] | `[]` keeps an existing module's `HARDWARE`; e.g. `ros_cmd_vel`, `ros_tf`, `ros_mw`, `ros_depth_heightmap`, `ros_imu`. |
   | `--check` | `False`; deterministic freshness gate, described below. |

   `RealEnvGenerator` converts the registered Gymnasium spec to `srb/interfaces/sim_to_real/env/<env_id>.py`.

   Before use, install the required modules and selected backends on the deployment target. If a runtime-dispatched command needs a generated environment and `srb_real/` registration is missing, it generates the environment in a subprocess. The command then exits with instructions to rerun it.

1. Review modules against the robot deployment config. Native teleop and human collection need declared live actions:

   - Six device values: translation and rotation on XYZ, after device gains.
   - An event mapped to named action slices.

   The generator and `RealEnv` check the exact action order. The adapter scales actions. Without a declaration, `COMMAND_MAPPING = None`. Policy and raw ROS ignore this mapping. Custom simulator mappers require it. Native teleop and human collection fail before hardware starts. Shipped modules declare the mapping.

### `--check` — the freshness gate

`--check` renders a temporary file beside the target. It formats the file with Ruff, or with fallback Black, then compares it byte-for-byte with the checked-in module.

If the files differ, the command prints a unified diff and writes nothing. Missing formatters raise `FormatterUnavailableError`. Formatter failures raise `FormatterFailedError`. With `--env ALL --check`, it checks cached modules and skips missing modules. It fails if there are orphans or the cache is empty.

### Landscaping task IDs

`terrain_landscaping_crater` uses a fixed crater manifest. `terrain_landscaping` uses a per-episode mission manifest. Neither task accepts `env.particles_height`. Missing capability tags cause `RealEnv` construction to fail. See [Terrain Landscaping](../workflows/sim_to_real.md#52-terrain-landscaping).

## Runtime commands

Shared runtime options:

| Option | Default / limit |
| --- | --- |
| `-e`, `--env`, `--task` ENV | Required; environment under `srb_real/`. |
| `--cfg` PATH | `DEFAULT`; Hydra YAML override. |
| `--hardware`, `--hw` [HW ...] | `[]`; runtime hardware interfaces. |
| `--logdir`, `--logs` PATH | `logs/` in a source checkout or current working directory when installed. |

| Command | Options and behavior |
| --- | --- |
| `teleop`, human `collect` | `--teleop_device` (`ros` default; `spacemouse`, `haptic`); `--pos_sensitivity` (`1.0`); `--rot_sensitivity` (`3.1415927` rad/s); `--invert_controls` (`True`). Simulator-only `keyboard`/`gamepad` fail before hardware or input acquisition. |
| `collect` | `--collector` (`human`, `policy`, `random`, `zero`; infers policy with `--algo`/`--model`, else human); `--num_episodes` (`100`); `--horizon` (`400`); `--success_only` (`False`); policy accepts `--algo`, `--model`. |
| `ros` | `--ros_sync`/`--ros-sync` (`False`); `--ros_action_timeout`/`--ros-action-timeout` (unset: 5 s startup wait, 0.5 s command lifetime); `--ros_unthrottled`/`--ros-unthrottled` (`False`). See [`real_agent ros`](cli_agent_ros.md#realenv-raw-policy-input). |
| `train` | RL `--algo` required (e.g. `dreamer`, `tdmpc2`, `sbx_ppo`); `--continue_training` (`False`) / `--model` (`None`). |
| `eval` | Requires `--algo` and/or `--model` (each defaults `None`); choices include `dreamer`, `tdmpc2`, `sbx_ppo`, `robomimic_bc`, `robomimic_cql`. |

Native `teleop` has no policy-command interface. It rejects `--algo` and `--model`; use `eval` or policy `collect` instead. `srb agent teleop` retains policy mode.

In `eval` and policy `collect`, SRB `--model` artifacts supply `--algo` when it is omitted. They also supply the bundled config. Real datasets set `split=real` and `controller=human|policy|random|zero`. Artifacts record the resolved checkpoint and source paths.

### Native direct input

Before a session starts, native teleop and human collection validate mappings and devices. The mapper returns one flat action for the adapter to scale. `--invert_controls` inverts XY for manipulation only.

ROS teleop uses global `/cmd_vel` (`Twist`) and `/gripper` (`Bool`) topics with sensitivity gains. Its inputs differ from the flat policy actions in [`real_agent ros`](cli_agent_ros.md#realenv-raw-policy-input). ROS and haptic timeouts are separate from the raw-policy sample deadline. Native haptic has no simulator-contact force feedback.

Termination or `--horizon` ends an episode. ROS and haptic input have no manual save, discard, or reset controls (see [collection](cli_agent_collect.md)). SpaceMouse keeps its buttons, including left-button reset; the CLI lists them.

Datasets preserve observation leaf contract shapes and terminal rows at environment or horizon end. The command reports open and cleanup failures. It cleans up input before releasing the environment ROS node/context. See [teleop](cli_agent_teleop.md) for device details.

## Validate hardware

1. Run validation with its YAML spec:

   ```bash
   srb real_agent validate --spec SPEC_PATH [options]
   ```

   | Option | Default / limit |
   | --- | --- |
   | `--spec` PATH | Required; e.g. `hyperparams/validation/excavation.yaml`. |
   | `--checkpoint` PATH | `None`; overrides `checkpoint`; raw path or SRB artifact. |
   | `--episodes` N | `None`; overrides `n_episodes`. |
   | `--wandb` | `False`; enable optional Weights & Biases summary sink. |
   | `--no-tensorboard` | `False`; disable TensorBoard scalar sink. |
   | `--no-rerun-web` | `False`; disable viewer; local `.rrd` remains if `rerun.record_to_rrd=true` (default), Rerun is installed and starts. |
   | `--post-status` | `False`; post pass/fail as a GitHub commit status; requires the `gh` CLI. |
   | `--storage-root` PATH | `logs/real_validation` in shipped specs. |
   | `--dry-run` | `False`; validates spec and session setup, skips deployment checks and environment/policy factories; on success `exit_code: 4` (`preflight_fail`), `pass_overall: false`. |

   Validation runs the generated environment and available policy reset hooks for each episode. The artifact manifest supplies `algo` if the spec omits it or sets `REQUIRED_AT_RUNTIME`. Conflicts fail.

   Artifacts bundle Dreamer, TD-MPC2, RSL-RL, or SKRL config. Raw SKRL/RSL-RL checkpoints need the training `.hydra/config.yaml`. Validation fails if that file is missing or the metadata policy config is sanitized. See [validation workflow and exit codes](../workflows/sim_to_real.md#63-run-validation-on-the-real-robot).

## Rebuild a validation summary

1. Rebuild `summary.json` and `report.md` after spec, metric, or post-processing changes without rerunning hardware. `SESSION_DIR` must contain `spec.yaml` and logged `.jsonl` / `.rrd` telemetry:

   ```bash
   srb real_agent rebuild-summary SESSION_DIR
   ```

## Replay against a baseline

1. Compare time-aligned hardware (`real solid`) with simulation (`sim ghost`) in local Rerun:

   ```bash
   srb real_agent replay-twin --session SESSION_DIR --baseline BASELINE_RRD_PATH
   ```

   | Option | Meaning |
   | --- | --- |
   | `--session` SESSION_DIR | Validation session containing `rollout.rrd`. |
   | `--baseline` BASELINE_RRD_PATH | Baseline `.rrd` captured from simulation. |
