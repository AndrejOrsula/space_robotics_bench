# `srb isaac` — Isaac Sim Utilities & Python REPL

`srb isaac` (`srb isaacsim`) wraps the Isaac Sim launcher and Python interpreter.

## Usage

```bash
srb isaac sim         # Launch the Isaac Sim GUI application
srb isaac python      # Run the Isaac Sim Python interpreter
```

Use `srb isaac python` without the app or a scene for scripting and
exploration. To attach a REPL to a running simulation, pass `--repl` to an
`agent` command: `zero`, `rand`, `teleop`, `ros`, `train`, `eval`, or `collect`.

Put `--` after the sub-subcommand to forward remaining arguments verbatim:

```bash
srb isaac python -- -m pip list
srb isaac sim -- --no-window
srb isaac sim -- --/app/window/title="My SRB Session"
```

Before launch, `srb isaac sim` checks that the resolved launcher is an
executable file. If Isaac Sim is unavailable, set `ISAACSIM_PATH` to the
installation containing `isaac-sim.sh`, or put `isaacsim` on `PATH`. Missing,
non-file, and non-executable paths produce typed CLI errors.

### Aliases

With no further argument, `srb isaacsim` aliases `srb isaac sim` and opens the
GUI.

## Python Example

```python
from srb.core.app import AppLauncher

app = AppLauncher(headless=True).app

import torch
from srb.tasks.manipulation.sample_collection import Task, TaskCfg

env_cfg = TaskCfg(num_envs=4)
env = Task(env_cfg)
env.reset()

for _ in range(50):
    env.step(action=torch.tensor(env.action_space.sample(), device=env.device))

env.close()
app.close()
```

## In-Flight REPL with `--repl`

Attach a Python REPL to a running simulation with `--repl`:

```bash
srb agent zero --env sample_collection --repl ptpython
srb agent train --algo sbx_ppo --env landing --headless --repl
```

The background thread preloads `env`, `sim_app`, and `mute()`/`unmute()` helpers
to silence simulator stdout. Choose `ptpython` (default) or `bpython`.

`SRB_REPL` can also enable the REPL:

| `SRB_REPL` value | Backend |
| --- | --- |
| `ptpython`, `1`, or `true` (case-insensitive) | `ptpython` |
| `bpython` | `bpython` |
| Any other value | REPL stays off |

Explicit `--repl` takes precedence.

> **Note:** Offline `srb agent learn` does not launch Isaac Sim, so it has no
> `--repl` option.

## See Also

- [`srb agent`](cli.md#agent-subcommands) — agent subcommands that accept `--repl`
- [`srb gui`](gui.md) — native Rust GUI for SRB
