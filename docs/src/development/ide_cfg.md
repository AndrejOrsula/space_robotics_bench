# IDE Configuration

## VSCode Setup

[Visual Studio Code (VS Code)](https://code.visualstudio.com) is recommended. Other IDEs work too.

### Extensions

Install these recommended extensions. SRB uses Python annotations, and SRB configures strict mypy for selected modules; CI runs pytest and pre-commit checks, not mypy:

- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
- [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance)
- [mypy](https://marketplace.visualstudio.com/items?itemName=ms-python.mypy-type-checker)
- [Ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff)
- [Even Better TOML](https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml)

```bash
code --reuse-window \
     --install-extension ms-python.python \
     --install-extension ms-python.vscode-pylance \
     --install-extension ms-python.mypy-type-checker \
     --install-extension charliermarsh.ruff \
     --install-extension tamasfe.even-better-toml
```

For Rust:

- [Rust Analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer)
- [Even Better TOML](https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml)

```bash
code --reuse-window \
     --install-extension rust-lang.rust-analyzer \
     --install-extension tamasfe.even-better-toml
```

### Workspace Settings

The [workspace settings](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.vscode/settings.json) configure Python paths with these assumptions:

- Isaac Sim is installed at `../isaac-sim`
- Isaac Lab is installed at `../isaaclab`
- SimForge sources, if used, are at `../simforge` and `../simforge_foundry`

## Using the Dev Container

For a preconfigured environment, use the [Dev Container](devcontainer.md).
