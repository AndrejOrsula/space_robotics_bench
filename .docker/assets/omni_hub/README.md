# Hub Workstation Cache

Download the matching amd64 (`x86_64`) or arm64 (`aarch64`) archive from
[NVIDIA NGC](https://registry.ngc.nvidia.com/orgs/nvidia/teams/omniverse/resources/hub_workstation_cache/files)
(NVIDIA account required), named `omni_hub.linux-<arch>@...zip`.

```bash
architecture=x86_64 # use aarch64 for arm64
mv ~/Downloads/omni_hub.linux-${architecture}@*.zip .docker/assets/omni_hub/
```

Build the Docker image:

```bash
.docker/build.bash
```
