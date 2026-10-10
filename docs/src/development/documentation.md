# Documentation

Build SRB's Markdown documentation with [mdBook](https://rust-lang.github.io/mdBook).

## `srb doc` (alias `srb docs`) — Local Preview

`srb doc` (alias `srb docs`) starts the live-reload preview by default and opens it in a browser.

| Sub-subcommand | Description |
| --- | --- |
| `srb doc serve` *(default)* | Run an mdBook live-reload server and open it in a browser |
| `srb doc build` | Build the static site for local preview |
| `srb doc docgen` | Regenerate auto-introspected asset, task, and catalog pages; launches Isaac Sim headless |
| `srb doc thumbnail` | Render asset or task thumbnails into `docs/src/_graphics/assets/` or `docs/src/_graphics/tasks/` |

```bash
srb doc docgen --static-only
srb doc thumbnail --asset-type robot
srb doc thumbnail --task excavation_demo
srb doc thumbnail --task ALL
```

`docgen` uses headless Isaac Sim to regenerate asset, task, compatibility,
action-space, sensor, stats, and attribution pages. `--assets-only` and
`--tasks-only` limit writes. `--static-only` checks schema versions and known
sidecar field types in `docs/data/assets.toml` and `docs/data/tasks.toml`
(unknown keys ignored), plus task GUI themes and ranks. It reports errors or
asset/task counts without launching Isaac Sim or writing pages, but cannot
verify sidecars name registered assets or tasks.

Preserve hand-written sections with `<!-- PRESERVE:<key> --> … <!-- /PRESERVE:<key> -->`.

`thumbnail` accepts `--width`, `--height`, `--output-dir`, `--asset`,
`--asset-type`, `--task`, `--force` (re-render existing images), and
`--allow-failures` (exit successfully when some tasks in a batch fail; failures
are still reported). Output defaults to `docs/src/_graphics/assets/` or
`docs/src/_graphics/tasks/`. After rendering task thumbnails, it republishes the
offline cache with a rebuilt GUI catalog; refresh exceptions are warnings.

> **Hint:** `srb doc serve` and `srb doc build` forward tokens after `--`
> directly to `mdbook`.

## Automated Deployment

The docs workflow runs on `dev` or `main` pushes that change `.github/workflows/docs.yml`, `docs/**`, or a root `*.md` file; same-repository pull requests; published releases (prereleases skip the build); and manual dispatch. Runs in `AndrejOrsula/space_robotics_bench` deploy to GitHub Pages on `main` pushes, published non-prereleases, or manual dispatch; pull requests and `dev` pushes do not deploy.

## Contributing

Report errors or suggest improvements through an issue or pull request.
