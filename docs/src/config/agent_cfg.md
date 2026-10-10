# Agent Configuration

Configure agents with [Hydra](https://hydra.cc).

```bash
srb agent train --algo sb3_ppo --env peg_in_hole \
    agent.learning_rate=0.0003 \
    agent.batch_size=128
```

## Default Hyperparameters

Use [`hyperparams/`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/hyperparams) for defaults. Add optional task overrides in `hyperparams/task/`.

## Modifying Configurations

Edit framework defaults or pass Hydra overrides.

### Configuration Files

Defaults are grouped by framework (YAML except Robomimic's JSON):

```
hyperparams/
├── sb3/           # Stable Baselines 3 (ppo, sac, td3, ...)
├── sbx/           # SB3 eXtensions
├── skrl/          # SKRL (ppo, a2c, ppo_rnn, ...)
├── rsl_rl/        # RSL RL
├── robomimic/     # Robomimic
├── dreamerv3.yaml # DreamerV3 (single-algorithm framework)
└── tdmpc2.yaml    # TD-MPC2 (single-algorithm framework)
```

Use framework-prefixed algorithm names (`sb3_ppo`, `sbx_sac`, `skrl_ppo`, `rsl_rl_ppo`, `dreamer`, `tdmpc2`) and pass Hydra overrides on the command line.

> **Note:** For algorithms with a registered agent dictionary, `--cfg` replaces the whole `agent` mapping. Include every required setting; omitted registered values are dropped.

### SKRL SAC

`hyperparams/skrl/sac.yaml` pins SRB's bounded/squashed Gaussian policy, two
critics and target critics, plus explicit `terminated`/`truncated` replay flags.
`skrl_sac` supports train, eval, and policy collection.

## Tuned configurations

[`srb agent tune`](../reference/cli_agent_tune.md) writes the requested modality's config, resolving **task-specific → global** layers:

- `hyperparams/<framework>/<algo>.yaml` — global per-algorithm set (state observations); `<algo>_visual.yaml` for visual observations.
- `hyperparams/task/<task>/<framework>/<algo>.yaml` — optional per-task override that takes precedence for that task only.

The `# Tuned by srb agent tune …` provenance header records study, trial, objective, seeds, date, git SHA, and Optuna version. See [Hyperparameter Tuning](../workflows/hyperparameter_tuning.md).

Visual profiles (`hyperparams/visual_profiles.yaml`) do not ship. Optuna tuning spaces are included under `srb/tuning/spaces`. Without them, `train` and `eval` reject `--obs visual`; use `--obs state` or supply a valid catalog. `tune` supports only `--obs state`.
