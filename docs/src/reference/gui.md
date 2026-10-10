# Graphical User Interface (GUI)

Launch the Rust GUI with ROS 2 Jazzy sourced:

```bash
source /opt/ros/jazzy/setup.bash
srb gui
```

The GUI uses [egui](https://github.com/emilk/egui) and
[r2r](https://github.com/sequenceplanner/r2r). It does not support the optional
NVIDIA NGC image.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/zijvZRoUGQ8?si=5jAhBuNbRowkcsEN&mute=1&autoplay=1&loop=1&playlist=zijvZRoUGQ8" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Pages

- **Tasks:** Browse, filter, and select catalog tasks.
- **Session:** Run, restart, or end the selected task. Tune gravity and latency while it runs.
- **Metrics:** Plot environment 0 reward (`/gui/reward`) while a task runs.
- **Configure** (window): Edit task options and Hydra overrides. Then preview, copy, or run the equivalent command.
  - Displayed text redacts credentials. Processes receive the exact arguments.
  - Put argparse options before Hydra `key=value` overrides. Do not use `--`.

The footer toggles the theme and records episode configurations. Recorded `DATA` rows contain the launched configuration and published
`runtime_params`, such as gravity. The values stay `null` until the first
successful publish. Each launch republishes all parameters. Volatile ROS QoS
means new processes can miss values.

Demo defaults:

| Option | Default |
| --- | --- |
| Rendering mode | `quality` |
| Teleop devices | `keyboard spacemouse` |
| Interface | `gui` |
| Task-specific `env.stack` | Catalog default; `true` for `excavation_demo` ([parallelism](../config/parallelism.md)) |

Cached haptic devices have generic advanced-controls checkboxes. They have no
dedicated GUI controls.

## `srb gui` — Launch GUI

> **Hint:** Arguments after `--` go to the GUI application. The exceptions are
> `-h` and `--help`.

SRB checks for `cargo` and `rustc`. If either is missing, set an executable
prebuilt GUI:

```bash
SRB_GUI_BINARY=/path/to/gui srb gui
```

Outside a source checkout, run `python -m srb`. Use the Python interpreter
where SRB is installed: `ISAACSIM_PYTHON`, Isaac Sim's Python, or `python3`.

In a development tree, SRB uses `target/release/gui` or `target/debug/gui` only
when `cargo` or `rustc` is missing. It prints a warning in that case. Otherwise,
SRB builds the GUI from source. `SRB_GUI_BINARY` takes precedence. Missing or
invalid inputs cause a typed CLI error before startup.

### Example

```bash
srb gui -- --help
```

## Refresh GUI Catalog

After changing task metadata, cache generation, assets, thumbnails, or
interfaces, run `srb cache update`:

```bash
srb cache update
```

Discovery launches Isaac Sim 6.1. `srb cache status` and `srb cache doctor`
inspect catalogs without it but cannot regenerate metadata. Repair missing,
incomplete, or tampered pointers and generations with `srb cache update`.

Task cards and options use immutable `gui_catalog.json` generations.
[`srb cache update`](cli.md) hashes and validates twelve members. It then
atomically switches `current.json`.

Generations live at `<cache root>/namespaces/<namespace>/offline/generations/<generation-id>/`.

Set `SRB_CACHE_ROOT` to choose the cache root. Defaults:

- Source checkout: `<checkout>/.cache`
- Other installs: `~/.cache/srb`, or `$XDG_CACHE_HOME/srb` when set

Before reading the catalog, the GUI validates the pointer, manifest, members,
semantics, and catalog.

`srb gui` sets `SRB_GUI_OFFLINE_CACHE_ROOT` to Python's resolved root.

Direct launches query `srb cache status --json` with a 30-second timeout. A
failure or disagreement opens a recovery page.

## Task Thumbnails

Thumbnails load from:

```text
docs/src/_graphics/tasks/<task>.webp
```

Generate or refresh a thumbnail with:

```bash
srb doc thumbnail --task excavation_demo --force --width 320 --height 240
```

After rendering, `srb doc thumbnail` republishes the current generation. It
rebuilds the GUI catalog and copies the other members.

If the generation is absent or outdated, the command warns without changing
the cache. Missing thumbnails are nonfatal. The fallbacks are robot, scenery,
domain, and the SRB placeholder.

## Command Shape

```bash
srb agent teleop --env excavation_demo --hide_ui --rendering_mode quality --interface gui --teleop_device keyboard spacemouse env.domain=MOON env.seed=0 env.num_envs=1 env.stack=true env.robot=kinova_gen3n7+scoop_custom3
```

The customization panel accepts Hydra overrides, such as `env.demo_regolith_quality=visual`.
