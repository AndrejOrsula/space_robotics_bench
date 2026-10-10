# Dev Container

Use SRB's [Dev Container](https://containers.dev), configured in
[`devcontainer.json`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.devcontainer/devcontainer.json).

## Prerequisites

- Working Docker Engine and NVIDIA Container Toolkit. The Dev Container requests
  all GPUs.
- `xauth` and an X11 session for GUI forwarding.
- [Visual Studio Code (VS Code)](https://code.visualstudio.com) and the
  [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
- Host `code` CLI to use the optional helper script below.

The container builds from the repository `Dockerfile`, mounts the checkout at
`/root/ws`, forwards X11, and reuses host Omniverse and shader caches. It uses
the Isaac Sim and Isaac Lab versions pinned in [system requirements](../getting_started/requirements.md);
Isaac Sim lives at `/root/isaac-sim`.

Host caches: `~/.cache/isaac-sim`, `~/.cache/nvidia/GLCache`, `~/.cache/ov`,
`~/.nv/ComputeCache`, `~/.cache/simforge`. Data: `~/.nvidia-omniverse/data/ov`,
`~/.nvidia-omniverse/data/isaac-sim`. Logs: `~/.nvidia-omniverse/logs`,
`~/.nvidia-omniverse/logs/isaac-sim`. Use repository ownership markers and
`srb clean`/`srb cache claim-clean-root` for supported roots; SimForge cleanup
uses `simforge clean`.

## Open in VS Code

1. Open the repository folder:

   ```bash
   code /path/to/space_robotics_bench
   ```

1. Run **Dev Containers: Reopen in Container** from the Command Palette
   (`Ctrl+Shift+P`).

VS Code builds the image if needed and attaches. The first build includes the
full SRB image.

## Verify Container Setup

After attaching, verify the setup:

```bash
nvidia-smi
srb --help
srb cache update
echo "$DISPLAY"
```

## Optional: Open with the Helper Script

If the VS Code CLI is installed, run:

```bash
./.devcontainer/open.bash
```

## Other Container Access

The Dev Container requests all GPUs and `/dev/dri`. `.docker/run.bash` profiles:
`safe` has no GPU; `simulator` enables GPU and GUI; `trusted-simulator` also
grants `--privileged` and all of `/dev`; `ngc` runs the NGC Isaac Sim image as UID 1234.
For `.docker/run.bash`, `WITH_GPU=false` disables GPU passthrough.

For terminal access to a running SRB container, use `.docker/join.bash` instead
of opening another VS Code window.
