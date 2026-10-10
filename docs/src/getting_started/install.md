# Installation

Clone the repository with submodules, then follow one installation guide:

```bash
git clone --recurse-submodules https://github.com/AndrejOrsula/space_robotics_bench.git
cd space_robotics_bench
```

| Method | Use when | Limits |
| --- | --- | --- |
| [Docker (Recommended)](install_docker.md) | Simple, reproducible installation and updates; helper scripts for GUI, headless, and ephemeral sessions; least-privilege profile for lightweight commands. | Moderate development experience (via Dev Containers). Isaac Sim's trusted profile grants host-adjacent access (not suitable for HPC). |
| [Native](install_native.md) | Full system integration (documented path) and a smooth development experience. | Complex setup; potential dependency conflicts; untested. |
| [Apptainer/Singularity](install_apptainer.md) | HPC deployment. | Requires local Docker to build the Apptainer image (`.sif`); limited interactive development support; untested. |

Use Docker with a locally built image or a release image pinned by its full `sha256` digest. Do not pipe a mutable branch's setup script into a shell: clone or authenticate and inspect the runner first. See [system requirements](requirements.md) for platform support and simulator pins.

## Installed wheel support

Wheels support data and CLI commands that need no simulator assets or repository files. CLI dependencies need `pip install "srb[cli]"`; the base wheel omits them but includes the `srb` entry point and hyperparameter resources. `srb test`, `srb doc`, and simulator-launching commands need a source checkout; simulator commands also work in an SRB container with assets. Clone tagged releases with submodules, e.g. `git clone --recurse-submodules --branch 0.1.0 https://github.com/AndrejOrsula/space_robotics_bench.git`. GitHub source archives omit submodules and simulator assets.

Wheel runs log to `logs/` in the current directory. Cache defaults to `$XDG_CACHE_HOME/srb` (`~/.cache/srb` if `XDG_CACHE_HOME` is unset, empty, or relative); set `SRB_CACHE_ROOT` to change it.

## Temporary Docker sessions

The runner uses `--rm`, so the container filesystem is discarded on exit. See the [Docker installation guide](install_docker.md) for runner profiles and image requirements.

Next: [Docker installation](install_docker.md)
