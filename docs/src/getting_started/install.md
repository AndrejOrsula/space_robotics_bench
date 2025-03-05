# Installation

TODO: Say that there are three supported options
- Native
- Docker
- Apptainer (Singularity) for HPC cluster workflows


## Quickstart for temporary test

Say that by far, the simplest way is to use Docker via these scripts (does not require cloning the repo)

```bash
# [ALTERNATIVE] Raw content via wget
WITH_DEV_VOLUME=false bash -c "$(wget -qO - https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)" -- $TAG $CMD
```

```bash
# [ALTERNATIVE] Raw content via curl
WITH_DEV_VOLUME=false bash -c "$(curl -fsSL https://raw.githubusercontent.com/AndrejOrsula/space_robotics_bench/refs/heads/main/.docker/run.bash)" -- $TAG $CMD
```