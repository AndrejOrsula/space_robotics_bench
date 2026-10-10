# `srb model` — Manage Model Artifacts

Package checkpoints as self-contained SRB model artifacts with SRB identity, metrics, framework restore metadata, and optional dataset provenance, including from consumed-demo runs.

## Usage

```bash
srb model <action> [options]
```

## Actions

| Action | Purpose |
| ----------------- | -------------------------------------------------------------- |
| `create CHECKPOINT --output OUT` | Package one checkpoint. Requires `--algo`, plus `--task`, `--embodiment`, and `--action-mode` unless inferred from `--dataset`. |
| `from-run RUN --output OUT` | Package a completed SRB run's latest checkpoint; infer algorithm and task from `metadata.json`. Without `--dataset` or consumed-demo provenance, also pass `--embodiment` and `--action-mode`. |
| `info PATH` | Print manifest JSON. |
| `validate PATH` | Validate manifest, checkpoint, and bundled config paths; `--strict` makes warnings errors. |
| `card PATH` | Generate a Hugging Face `README.md` model card; schema 3 artifacts bundle the canonical card without rewriting it. |
| `push PATH` | Upload or dry-run Hugging Face publication. |

Both package actions accept repeated `--metric key=value` and `--metadata key=value`. Metrics must be finite numbers; text, `nan`, and `inf` are rejected. Metadata values are strings.

## Portable Restore

`from-run` preserves available restore context:

- Dreamer and TD-MPC2 can include `config.yaml`.
- RSL-RL checks run-root `config.yaml`, then `.hydra/config.yaml`'s `agent` section, then `metadata.json`'s `run.agent_cfg`.
- SKRL checks `.hydra/config.yaml`'s `agent` section, then `metadata.json`'s `run.agent_cfg`.

Reject the last source if it contains redacted or non-finite placeholders. The error names the key path. Redaction covers secret-looking key names and credential strings (URL userinfo, credential headers, and `Bearer` values). Warnings show key paths only.

For SB3/SBX, packaging requires inventoried `vecnormalize.pkl` statistics when the source RL-Zoo config declares normalization. Missing or ambiguous source evidence can remain unknown. Verify normalization state before packaging.

These commands accept artifact directories wherever they accept checkpoint paths: `srb agent eval`, `srb agent collect`, `srb agent teleop`, `srb real_agent eval`, and `srb real_agent validate`.

Loading an artifact can execute code (pickled checkpoints and statistics, SKRL model expressions). Load only artifacts from trusted sources.

SB3/SBX preserves RL-Zoo normalization state:

- Packaging:
  - Normalized runs require inventoried, hashed `vecnormalize.pkl`.
  - Non-normalized runs record that state.
  - Missing or contradictory context fails packaging.
  - Re-package artifacts without a normalization declaration from the original run before inference.
- Inference:
  - Restores frozen statistics without normalizing evaluation rewards.
  - Dict observations and recurrent episode state retain the policy layout.

Publish with dataset provenance from `--dataset`, a recorded consumed-demo run,
or `--dataset-repo-id` plus optional `--dataset-revision`.

Preparation, validation, and restore stream hashes under per-file and aggregate
limits, rejecting oversized or changing files. Schema 1/2 restore copies
checkpoints and configs to temporary storage, verifies hashes, and removes
partial copies on failure.

## Examples

```bash
# Package the latest checkpoint from a run and infer identity from the dataset
srb model from-run logs/excavation/rsl_rl_ppo/<run> \
  --dataset /data/srb_excavation_policy \
  --output /models/srb_excavation_rsl_rl_ppo

# Validate and inspect the artifact
srb model validate /models/srb_excavation_rsl_rl_ppo --strict
srb model info /models/srb_excavation_rsl_rl_ppo

# Dry-run the Hugging Face upload, including the generated README.md
srb model push /models/srb_excavation_rsl_rl_ppo --dry-run

# Reuse the packaged artifact for policy rollout
srb agent collect --env excavation --model /models/srb_excavation_rsl_rl_ppo
```
