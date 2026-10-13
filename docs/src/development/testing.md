# Testing

SRB tests cover host imports and Isaac Sim CLI workflows.

## Scope

The public suite includes host-runnable `tests/import_test.py` and Isaac Sim-backed `tests/cli_ls_test.py` and `tests/cli_agent_test.py`. `.github/workflows/python.yml` runs import and packaged-CLI checks for same-repository pull requests and pushes to `dev` or `main`.

> **Hint:** The `unit` tier (`tests/import_test.py`) does **not** require Isaac Sim or a GPU — run it anywhere with `pytest -q tests/import_test.py`.

## `srb test` — Run Tests

`srb test` runs Python (`pytest`) and/or Rust (`cargo test`).

```bash
srb test [options]
```

Python runs two `pytest` processes: `unit` (`tests/import_test.py`, run in CI),
then `workflows` (`tests/cli_*_test.py`, Isaac Sim). A failing tier does not stop
the next (an interrupt or incomplete process cleanup does), each run tier gets a
report line, and any failure sets a nonzero exit. `long_training`
(end-to-end training matrices) is deselected by default.

### Options

| Argument | Description | Default |
| -------------------------- | ----------------------------------------------------------------------- | :--------: |
| `-l`/`--language`/`--lang` | Language suites to test | `[python]` |
| `-t`/`--tier` | Python tiers to run (`all` = every tier) | `[all]` |

> Arguments after `--` are forwarded to every `pytest` tier and to `cargo test`.

### Examples

Forward `-vx` to pytest:

```bash
srb test -- -vx
```

Run the host unit or Isaac Sim workflow tier:

```bash
srb test --tier unit
srb test --tier workflows -- -x
```

Run the opt-in training matrix (minutes per environment):

```bash
srb test --tier workflows -- -m long_training
```

Run tests for both Python and Rust:

```bash
srb test --lang python rust
```
