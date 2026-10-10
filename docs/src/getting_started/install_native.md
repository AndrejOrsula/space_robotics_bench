# Installation — Native

Native installs are untested and need manual work. Builds use Isaac Sim 6.1.0 and Isaac Lab 3.0.0 ([system requirements](requirements.md)). On Ubuntu 22.04, use `uv` Python 3.12; system Python is older. Use Isaac Sim's `python.sh` for SRB and Isaac Lab.

## 1. Prepare the host

```bash
sudo apt-get update
sudo apt-get install -y git git-lfs gcc-11 g++-11 cmake build-essential curl unzip xz-utils python3 jq
curl -LsSf https://astral.sh/uv/install.sh | sh
uv python install 3.12
uv python find 3.12
```

Without `sudo`, install `git-lfs` locally and add `~/.local/bin` to `PATH`:

```bash
mkdir -p ~/.local/bin /tmp/git-lfs-install
curl -sSL https://github.com/git-lfs/git-lfs/releases/download/v3.6.1/git-lfs-linux-amd64-v3.6.1.tar.gz -o /tmp/git-lfs.tar.gz
tar xzf /tmp/git-lfs.tar.gz -C /tmp/git-lfs-install
cp /tmp/git-lfs-install/git-lfs-*/git-lfs ~/.local/bin/
rm -rf /tmp/git-lfs.tar.gz /tmp/git-lfs-install
```

## 2. Initialize Git LFS

Run this before cloning, including after the local fallback:

```bash
git lfs install
```

## 3. Clone SRB

Initialize submodules, then run commands from the repository root:

```bash
git clone https://github.com/AndrejOrsula/space_robotics_bench.git
cd space_robotics_bench
git submodule update --init --recursive
```

## 4. Build Isaac Sim

Accept NVIDIA's license before setting `ACCEPT_EULA=Y`. The build requires `sudo`, substantial time, and disk space.

```bash
ACCEPT_EULA=Y \
SRC_BRANCH=release/6.1 \
SRC_COMMIT_SHA=7c206f75bdadd9e05fc457f19863ca4c3f0cb693 \
  ./.docker/install/install_isaacsim.bash "$HOME/isaac-sim"
```

## 5. Configure the Isaac Sim environment

Copy the installer's `export VK_DRIVER_FILES=...` line. It detects or creates the Vulkan ICD path. Add all exports to shell config, source it, and keep `~/.local/bin` on `PATH` for CLI wrappers.

### Environment Variables

| Shell | Configuration file | `PATH` command | Other exports |
| --- | --- | --- | --- |
| `bash` | `~/.bashrc` | `export PATH="$HOME/.local/bin:$PATH"` | `export ISAACSIM_PYTHON="$HOME/isaac-sim/python.sh"`; `export ISAACSIM_PATH="$HOME/isaac-sim"`; `export ISAAC_PATH="$HOME/isaac-sim"`; `export CARB_APP_PATH="$HOME/isaac-sim/kit"`; `export EXP_PATH="$HOME/isaac-sim/apps"`; `export LD_PRELOAD="$HOME/isaac-sim/kit/libcarb.so"`; `export ISAACSIM_ASSET_ROOT="$HOME/isaac-sim/assets"`; `export ISAACSIM_ASSET_ROOT_PROFILE="isaac-sim-6.1"`; `export OMNI_KIT_ALLOW_ROOT="1"` |
| `zsh` | `~/.zshrc` | `export PATH="$HOME/.local/bin:$PATH"` | Same `export` lines as `bash` |
| `fish` | universal variables | `fish_add_path "$HOME/.local/bin"` | `set -Ux ISAACSIM_PYTHON "$HOME/isaac-sim/python.sh"`; `set -Ux ISAACSIM_PATH "$HOME/isaac-sim"`; `set -Ux ISAAC_PATH "$HOME/isaac-sim"`; `set -Ux CARB_APP_PATH "$HOME/isaac-sim/kit"`; `set -Ux EXP_PATH "$HOME/isaac-sim/apps"`; `set -Ux LD_PRELOAD "$HOME/isaac-sim/kit/libcarb.so"`; `set -Ux ISAACSIM_ASSET_ROOT "$HOME/isaac-sim/assets"`; `set -Ux ISAACSIM_ASSET_ROOT_PROFILE "isaac-sim-6.1"`; `set -Ux VK_DRIVER_FILES "/etc/vulkan/icd.d/nvidia_icd.json"`; `set -Ux OMNI_KIT_ALLOW_ROOT "1"` |

For bash/zsh, append with `cat >> ~/.bashrc << 'EOF'` or `cat >> ~/.zshrc << 'EOF'`, close `EOF`, and source the file. In fish, set `VK_DRIVER_FILES` to the ICD path printed by the installer; the example above uses a common path. `ISAACSIM_ASSET_ROOT` defaults to `$HOME/isaac-sim/assets`; use the printed root and profile.

| Vulkan ICD location | Use |
| --- | --- |
| `/usr/share/vulkan/icd.d/nvidia_icd.json` | Common with Ubuntu driver packages |
| `/etc/vulkan/icd.d/nvidia_icd.json` | Created by the script if neither existing path is found |

## 6. Install Isaac Lab

Install the pinned Isaac Lab source. Its script resolves NumPy conflicts and installs `array_api_compat`.

```bash
SRC_BRANCH=release/3.0.0 \
SRC_COMMIT_SHA=fdc650c88dfd86aa158b9a1760b6537fef360555 \
  ./.docker/install/install_isaaclab.bash "$HOME/isaaclab"
```

## 7. Install Blender and SimForge

Follow [Blender's official Linux instructions](https://docs.blender.org/manual/en/4.5/getting_started/installing/linux.html#install-from-blender-org), then use SRB's [Blender installer](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/install/install_blender.bash) for Blender 4.5 and SimForge:

```bash
./.docker/install/install_blender.bash "$HOME/blender"
```

Installer downloads Blender, links `~/.local/bin/blender`, installs Docker-pinned `simforge[dev]` 0.4.0 and `simforge-foundry` 0.3.1 in Blender Python, checks exact pins before download, then applies SRB's Isaac Lab compatibility patch (`.docker/install/patch_simforge_isaaclab.bash`). Avoid Snap: it blocks Python dependency integration.

## 8. Install optional RL frameworks

Source your shell configuration first; most scripts require `ISAACSIM_PYTHON`.

| Framework | Command | Behavior / limit |
| --- | --- | --- |
| DreamerV3 | `./.docker/install/install_dreamerv3.bash "$HOME/dreamerv3"` | Editable source install |
| TD-MPC2 | `./.docker/install/install_tdmpc2.bash "$HOME/tdmpc2"` | Clones source only; install dependencies such as `tensordict` and `torchrl` later via `pyproject.toml` |
| robomimic | `./.docker/install/install_robomimic.bash "$HOME/robomimic"` | Pins upstream; applies Docker-matching lazy-language patch (`srb/integrations/robomimic/patches/`); verifies revision before install and prevents offline language-model downloads |
| rsl_rl | `./.docker/install/install_rsl_rl.bash "$HOME/rsl_rl"` | Source install |

The TD-MPC2 script pins upstream and applies `srb/integrations/tdmpc2/patches/`, including the Q-ensemble initialization fix. It skips upstream's dependency-pin commit to avoid downgrading Isaac Sim Torch/TensorDict/TorchRL. SRB rejects checkouts without the backport; use this script or the error's pin, never plain `git clone`.

Set `TDMPC2_PATH`:

```bash
# Bash
echo "export TDMPC2_PATH='$HOME/tdmpc2/tdmpc2'" >> ~/.bashrc && source ~/.bashrc
# Zsh
echo "export TDMPC2_PATH='$HOME/tdmpc2/tdmpc2'" >> ~/.zshrc && source ~/.zshrc
# Fish
set -Ux TDMPC2_PATH "$HOME/tdmpc2/tdmpc2"
```

The robomimic checkout remains in place; no SRB extra includes it.

## 9. Install SRB

Install SRB with optional dependencies for all workflows:

```bash
"$ISAACSIM_PYTHON" -m pip install --editable ".[all]"
```

Adjust the `all` extra in [`pyproject.toml`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/pyproject.toml) as needed. Published metadata rejects direct Git references, so extras omit Git-pinned upstreams; install robomimic in step 8. `all` omits the v2e event-camera backend and E2VID reconstruction operator. Install these pinned groups with bundled pip 25.1 or newer:

```bash
"$ISAACSIM_PYTHON" -m pip install --upgrade 'pip>=25.1'
"$ISAACSIM_PYTHON" -m pip install --editable ".[event-v2e,event-reconstruction]" --group source-event-v2e --group source-event-reconstruction
```

## 10. Set up the CLI

Put executable `ISAACSIM_PYTHON` and its destination (default `~/.local/bin`) on `PATH`. Setup uses `ISAACSIM_PYTHON` or fallback `python3` for SRB's Hydra completion cache. `register-python-argcomplete` lookup checks Isaac Sim, system `PATH`, then Isaac Sim's Python package.

```bash
./.docker/helpers/setup_cli.bash
# Regenerate completions only for these shells
./.docker/helpers/setup_cli.bash --completions-only bash zsh fish
srb completions generate
srb completions install
exec "$SHELL" -l
```

The setup script copies `srb`, `simforge`, and `space_robotics_bench` shims to `~/.local/bin` and configures detected shells; it exits with an error when no requested shell gets completions. Hydra override completions need `jq`. If setup fails or shims are skipped, run `"$ISAACSIM_PYTHON" -m srb`.

## 11. Verify the installation

The first Isaac Sim launch takes longer while shaders compile. See [Troubleshooting](#troubleshooting) for failures.

| Component | Check |
| --- | --- |
| Isaac Sim | `"$ISAACSIM_PYTHON" -c "`<br>`from isaacsim import SimulationApp`<br>`sim = SimulationApp({'headless': True})`<br>`print('Isaac Sim OK — is_running:', sim.is_running())`<br>`sim.close()`<br>`"` |
| Isaac Lab | `"$ISAACSIM_PYTHON" -m pip show isaaclab` |
| Blender | `blender --version` |
| SRB and CLI | `"$ISAACSIM_PYTHON" -m srb --help`; `command -v srb`; `srb --help`; `"$ISAACSIM_PYTHON" -m srb cache update` |
| Quick simulation | `"$ISAACSIM_PYTHON" -m srb agent zero --headless -e sample_collection --cfg ignore --perf --perf_duration 10`; confirm the final performance report includes steps per second |

______________________________________________________________________

## Troubleshooting

### "Failed to create any GPU devices"

**Symptom:** Isaac Sim cannot create a GPU device. **Cause:** `VK_DRIVER_FILES` does not point to a valid Vulkan ICD file. **Fix:** Check the configured path, then set it to the existing ICD file.

```bash
ls -la "$VK_DRIVER_FILES"
find /etc/vulkan /usr/share/vulkan -name "nvidia_icd.json" 2>/dev/null
```

Update `VK_DRIVER_FILES` in your shell configuration.

### "Application failed to start"

**Symptom:** Isaac Sim fails to start. **Cause:** `DISPLAY` is unset. **Fix:** On a headless server, set it for the virtual display:

```bash
# Bash/Zsh
echo 'export DISPLAY=:99' >> ~/.bashrc  # or ~/.zshrc
# Fish
set -Ux DISPLAY :99
```

### numpy version conflicts

**Symptom:** NumPy version errors return. **Cause:** Manual package changes reintroduced conflicts after the installer resolved them. **Fix:** Install compatible packages:

```bash
"$ISAACSIM_PYTHON" -m pip install "numpy>=2.1" array_api_compat
```

______________________________________________________________________

## Extras

### Development

See [IDE configuration](../development/ide_cfg.md).

Next: [Basic Usage](basic_usage.md)
