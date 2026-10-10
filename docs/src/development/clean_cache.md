# Development Utilities — Clean Cache

Use `srb clean` after framework upgrades or asset changes to remove SRB, Isaac
Sim, and [SimForge](https://github.com/AndrejOrsula/simforge) caches. SRB caches
are namespaced by Isaac Sim/Lab versions, provider, architecture, and
source/asset digests. Isaac Sim vendor caches are shared across versions, so
clean them after changing the Isaac Sim pin.

## Procedure

1. Claim the fixed SRB and Isaac cache roots before cleaning. Their versioned
   ownership markers bind to canonical path, device, and inode. Existing
   content is adopted only at known roots.

   ```bash
   srb cache claim-clean-root --target srb
   srb cache claim-clean-root --target isaacsim
   ```

1. To include the NGC cleanup root, set an absolute `SRB_NGC_CACHE_DIR`; a
   relative value selects none.

1. Run `srb clean`:

   ```bash
   srb clean
   ```

   SRB prompts before removing marker-owned directories unless `-y/--yes` is
   set. SimForge handles its own confirmation.

Adoption supports the fixed repository `.cache`, five Isaac vendor roots, and
selected NGC profile root. NGC adoption allows only Docker profile mount
entries (`cache`, `compute`, `kit`, `logs`, `config`, `data`, `pkg`, `hub`);
entries need not all exist. After the NGC profile chowns the root to UID 1234,
claim it as a user with write access. Populated custom `SRB_CACHE_ROOT` roots
are never adopted; create and claim a new, empty root before writing cache data.

## Targets and Options

```bash
srb clean [TARGET ...] [-y/--yes]
```

| Argument | Description | Default |
| ---------------- | -------------------------------------------------------------------------- | ----------- |
| `TARGET ...` | One or more of `all`, `srb`, `isaacsim`, `simforge` (multiple allowed) | `[all]` |
| `-y`/`--yes` | Skip the interactive confirmation prompt | `False` |

`all` (default) selects `srb`, `isaacsim`, and `simforge`. `simforge` rejects
tokens after `--` with exit 2. A missing `simforge` CLI, SimForge clean failure,
timeout, or unexpected state makes `srb clean` exit 1.

`--yes` skips the prompt and passes `-y` to SimForge. Ownership, root-depth,
repository/home, mount, symlink, device, and entry-identity checks still apply.
Removal uses anchored directory descriptors and preserves the cache root and
marker.

## What Gets Removed

| Target | Affected paths |
| ----------- | -------------- |
| `srb` | The selected SRB cache root (registry caches, completion caches, …); by default this is `<repository root>/.cache` in a source checkout and `$XDG_CACHE_HOME/srb` (when absolute) or `~/.cache/srb` when installed. `SRB_CACHE_ROOT` overrides it. |
| `isaacsim` | Marker-owned roots `~/.cache/isaac-sim`, `~/.cache/ov`, `~/.cache/nvidia/GLCache`, `~/.nv/ComputeCache`, `~/.cache/packman`, and `${SRB_NGC_CACHE_DIR:-~/.cache/space_robotics_bench/ngc/isaac-sim-6.1/<arch>}` (where `<arch>` is `amd64` or `arm64`, as selected by `.docker/run.bash`) |
| `simforge` | Delegated to `simforge clean` (procedurally generated SimForge assets) |

SimForge cleanup follows its own ownership policy; SRB does not recursively
remove that directory.

The `.docker/run.bash` profile writes to these vendor roots, except
`~/.cache/packman`. `srb clean isaacsim` warns about present, unclaimed roots;
claim them first. The marker remains. `~/.cache/nvidia/GLCache` and
`~/.nv/ComputeCache` are shared with CUDA/OpenGL applications;
`~/.cache/ov` is shared with Omniverse applications. Marker-owned cleanup is
safe, but those applications rebuild their caches on next use, which can take
time.

## Examples

Remove only SRB caches without prompting:

```bash
srb clean srb -y
```

Remove only the Isaac Sim Omniverse caches:

```bash
srb clean isaacsim -y
```

Clean only SimForge's generated assets:

```bash
srb clean simforge -y
```

> **Reference:** [SimForge CLI — `simforge clean`](https://andrejorsula.github.io/simforge/instructions/cli.html#simforge-clean)
