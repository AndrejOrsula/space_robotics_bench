# Command Line Interface (CLI)

Run `srb <subcommand> [options]`; use `-h`/`--help` for help or `TAB` to browse:

```bash
srb --help
srb <subcommand> -h
srb <TAB> <TAB>
```

## Subcommands

| Subcommand | Description |
| ----------------------------------------------------------------- | ------------------------------------------------------ |
| [`agent`](#agent-subcommands) | Agents |
| [`real_agent`](cli_real_agent_gen.md) | Generate/run sim-to-real bridge |
| [`dataset`](cli_dataset.md) | Inspect/validate/convert/replay/publish datasets |
| [`model`](cli_model.md) | Package/validate/publish policies |
| [`list`/`ls`](cli_ls.md) | List assets, groups, environments |
| [`asset`/`assets`](cli_asset.md) | Manage surface assets, rocks, sky-domes, lighting presets |
| [`event-camera`](#event-camera-subcommands) | Weights/backend metadata |
| [`ephemeris`](cli_ephemeris.md) | Query Sun/planet geometry; fetch SPICE kernels; import illumination |
| [`cache`](#cache-subcommands) | Regenerate caches |
| [`gui`](gui.md#srb-gui--launch-gui) | Launch the GUI |
| [`isaac`/`isaacsim`](cli_repl.md) | Isaac Sim utilities (`sim`, `python`) |
| [`doc`/`docs`](../development/documentation.md#srb-doc-alias-srb-docs--local-preview) | Build docs; render thumbnails |
| [`test`](../development/testing.md#srb-test--run-tests) | Run Python/Rust tests |
| [`clean`](../development/clean_cache.md) | Remove SRB / Isaac Sim / SimForge caches |
| [`completions`](#completions-subcommands) | Manage completions |

`gui`, `isaacsim`, and `doc build`/`serve` return child status. `srb test` exits
1 on any Python tier failure; a Rust failure exits at once with Cargo's status. Signal `N` logs `terminated by signal <NAME>` and exits
`128+N` (`139` for `SIGSEGV`); timeout exits `124`.

## Agent Subcommands

Run `srb agent <action> [options]`:

| Subcommand | Purpose |
| :--- | :--- |
| [`zero`](cli_agent_zero.md) | Zero actions |
| [`rand`](cli_agent_rand.md) | Random actions |
| [`teleop`](cli_agent_teleop.md) | Direct/trained-policy teleoperation |
| [`ros`](cli_agent_ros.md) | ROS 2 / Space ROS action topics |
| [`train`](cli_agent_train.md) | Train RL policy |
| [`eval`](cli_agent_eval.md) | Evaluate policy |
| [`collect`](cli_agent_collect.md) | Record demos from human/policy/random/zero controllers |
| [`learn`](cli_agent_learn.md) | Offline imitation learning from demos |
| [`tune`](cli_agent_tune.md) | Optuna hyperparameter optimization; spawn training per trial |

### Shared Agent Options

Simulation-based commands share these; offline `learn`/`tune` use separate
options and do not launch Isaac Sim. `tune` spawns `srb agent train`.

| Argument | Description | Default / limit |
| --------------------------- | --------------------------------------------------------------------------------------- | :------------: |
| `-e`/`--env`/`--task` ENV | Environment ID | Required |
| `--cfg` PATH | Hydra YAML; `DEFAULT`/`IGNORE`/`NONE`/`NULL` recognized | `DEFAULT` |
| `--headless` | No display | `False` |
| `--hide_ui` | Hide most Isaac Sim UI; fullscreen | `False` |
| `--livestream` {-1,0,1,2} | Force: 0 off, 1 native, 2 WebRTC; `-1` defers to `LIVESTREAM` env var | `-1` |
| `--rendering_mode` {performance,balanced,quality,xr} | Isaac RTX preset after Kit start; `xr` applies none; defaults to `quality` for `srb/excavation_demo` | unset (auto: demo uses `quality`) |
| `--xr` | XR for VR/AR | `False` |
| `--enable_cameras` | Render cameras, not video; auto for `env.active_sensors=[…,visual]`/`event_camera`/`events`, `srb/excavation_demo` (unless `env.demo_cameras=false`), or `--video`/`--record-events`/`--snapshot`/`--xr` | `False` |
| `--kit_args=VALUE` | Forward args to Omniverse Kit | `""` |
| `--interface` [IFACE ...] | Enable interfaces `gui`/`ros` | `[]` |
| `--logdir`/`--logs` PATH | Root logs | Checkout: `logs/`; installed: working-dir `logs/` |
| `--video` | Record video | `False` |
| `--record-events`/`--record_events` | Record event-camera rollouts with provenance; requires `event_camera` (alias `events`) in `env.active_sensors`; container includes default `v2e`; see [Event Camera](../sensors/event_camera.md) | `False` |
| `--snapshot` PATH | One rendered PNG after initial reset, then exit; implies `--enable_cameras` | unset |
| `--snapshot-warmup` N | Render passes for path-traced/anti-aliased convergence; needs `--snapshot` | `16` |
| `--snapshot-settle` N | Robot-settling physics steps; `0` disables; needs `--snapshot` | `0` |
| `--perf` | Performance test instead of workflow | `False` |
| `--perf_output` PATH | Report path (`STDOUT`); needs `--perf` | `STDOUT` |
| `--perf_duration` SEC | Maximum duration (seconds); `0` = unlimited; needs `--perf` | `150.0` |
| `--repl` [BACKEND] | Python REPL with simulation (`ptpython`/`bpython`) | unset |

Hydra-composing `agent` commands forward unregistered tokens (`env.scene.num_envs=4`, `agent.*` keys of the selected algorithm, …); repeated keys apply in order, last wins (`env.seed=1 env.seed=2` sets `2`). Child wrappers may pass tokens after `--` (e.g. `srb test` to pytest/cargo). `srb agent tune` rejects bare overrides/`--`; use `--pin KEY=VALUE` per trial.

Attach values beginning with `-` to `--kit_args` using `=`, e.g. `--kit_args=--ext-folder=/path/to/ext1`.

## Event Camera Subcommands

Event-camera weights and backends:

| Sub-subcommand | Description |
| -------------------------------- | ---------------------------------------------------------------------------------- |
| `srb event-camera fetch-weights` | Download E2VID/V2CE weights; verify SHA256 |
| `srb event-camera list-mirrors` | List registered E2VID + V2CE mirrors |
| `srb event-camera fetch-fixture` | Download Gen4 HDF5 fixture; verify SHA256 |
| `srb event-camera list-fixtures` | List registered Gen4 HDF5 mirrors |
| `srb event-camera backends` | List registered generators; `--detailed` shows metadata |

## Cache Subcommands

Offline completion/GUI caches:

| Sub-subcommand | Description |
| ------------------ | --------------------------------------------------------- |
| `srb cache update` | Regenerate task/asset/config/enum/GUI caches |
| `srb cache claim-clean-root [--target srb\|isaacsim\|all]` | Add path/inode-bound `srb clean` markers; fixed Isaac and selected NGC roots may adopt existing content |
| `srb cache status [--json] [--strict]` | Inspect caches; no Isaac Sim |
| `srb cache doctor [--json] [--strict]` | Validate payloads; no Isaac Sim; reports by default; `--strict` exits 1 for incomplete/invalid cache |

## Completions Subcommands

Shell completions:

| Sub-subcommand | Description |
| ----------------------------- | --------------------------------------------------- |
| `srb completions generate` | Regenerate completion cache via `srb cache update` |
| `srb completions install` | Install Fish/Bash/Zsh completion scripts |

Restart with `exec "$SHELL" -l` to load Bash/Zsh registrations. Install fails
when no requested shell could be configured.

Besides subcommands and flags, completion covers task ids in `srb/x` and short
`x` forms (`srb_real/x` for `srb real_agent` runtime actions), `--algo` labels,
`--cfg` sentinels and YAML paths, asset target names, and event-camera mirrors.
After `srb agent`/`srb real_agent` actions it adds Hydra override keys and values
for the selected task (`-e`/`--env`/`--task`) and `--algo`: nested `env.*` and
`agent.*` keys, `+`/`++`/`~` prefixes, enum and boolean values, and bracketed
container elements such as `'env.active_sensors=[visual,events]'`. For
`srb agent tune`, overrides complete only after `--pin`. Fish falls back to path
completion when nothing else matches. Hydra candidates need `jq` and successful
`srb completions generate`; scripts follow the published cache pointer, so
regeneration needs no reinstall.
