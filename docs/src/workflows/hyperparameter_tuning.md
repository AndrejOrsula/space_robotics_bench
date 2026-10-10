# Hyperparameter Tuning

Tune an algorithm with [Optuna](https://optuna.org) through `srb agent tune`. Install the tuning extra first:

```bash
uv sync --extra tune
```

## Study modes

1. **Preflight a study.** Run the check before a long search. It assembles a trial without Isaac. Omit `--gpus`; see [CLI reference](../reference/cli_agent_tune.md#check-options).

   ```bash
   srb agent tune check -e <env> --algo <algo> --trial-steps <n>
   ```

1. **Choose a study mode and run it.**

   **Single task:** Tune one task and export its task config.

   ```bash
   srb agent tune run --env sample_collection --algo skrl_ppo \
     --trial-steps 20_000_000 --n-trials 40 --n-workers 2 --gpus 0,1
   ```

   **Portfolio:** Tune tasks sequentially, one child per segment. Divergence and running aggregates prune weak trials. Export globally. See [portfolio details](../reference/cli_agent_tune.md#portfolio-multi-task-studies).

   ```bash
   srb agent tune run --tasks sample_collection peg_in_hole landing --algo skrl_ppo \
     --trial-steps 10_000_000 --n-trials 30 --normalize baseline --aggregate median
   ```

## Inspecting and exporting

```bash
srb agent tune status --study srb-skrl_ppo-state-sample_collection-v1
srb agent tune export --study srb-skrl_ppo-state-sample_collection-v1 --to task --dry-run
srb agent tune export --study srb-skrl_ppo-state-sample_collection-v1 --to task
```

TPE's one-seed default can make `best_trial` a lucky outlier. Use `validate` to rerun top trials with fresh seeds and rank by mean; see [CLI reference](../reference/cli_agent_tune.md#validate-options-noise-arbiter).

Use `--seeds-per-trial K` to average distinct seeds during search. See its cost and pruning behavior in the [CLI reference](../reference/cli_agent_tune.md#averaging-seeds-during-a-study---seeds-per-trial).

`export` records provenance and restores production budget and W&B flag. `--to` defaults to task for single-task, **global** for portfolios, where `--to task` is rejected. Review and commit the diff; see [CLI reference](../reference/cli_agent_tune.md#status--export--repair-options) for overwrite and shadow warnings.

## Picking a trial budget (wall-clock worksheet)

Set `--trial-steps` where reference runs reach 50–70% of converged return. Production YAMLs may train until stopped (for example, `2e8`). Estimate:

```
hours_per_trial ≈ trial_steps / (env_steps_per_second) / 3600 × n_tasks
study_hours     ≈ n_trials × hours_per_trial × seeds_per_trial / n_workers
```

Runtime depends on throughput, pruning, and worker overhead. At 40,000 env-steps/s, `20_000_000` steps take ~8.3 min/trial; 40 trials with 2 workers take ~2.8 h. At 5,000 env-steps/s, expect ~67 min/trial or ~22 h. Project with `--fps`, cap with `--max-wall-hours`, pilot 10–20 trials.

## Config layering

Under `hyperparams/`, configs resolve task-specific to global by modality:

| Slot | Scope | State | Visual |
| --- | --- | --- | --- |
| L0 / L0v | Global | `<framework>/<algo>.yaml` | `<framework>/<algo>_visual.yaml` |
| L1 / L1v | Task-specific | `task/<task>/<framework>/<algo>.yaml` | `task/<task>/<framework>/<algo>_visual.yaml` |

Pass `--obs {state,visual}` to `train`, `eval`, and `tune` for modality config and log directory (`…/<algo>/` or `…/<algo>-visual/`). It prevents cross-loading and preserves the algorithm label. Visual tuning is unsupported; see [Agent Configuration](../config/agent_cfg.md#tuned-configurations).

## How it works

1. The **Isaac-free** driver samples with `study.ask()` and launches `train` children in process groups: one per seed for single-task trials, and one per task segment per seed for portfolio trials. Each child boots Isaac.
1. It writes full `agent:` config and `env:` seed and `num_envs` to `trial_cfg.yaml`. Children report episode statistics as JSONL; the driver counts **total environment steps**.
1. Single-task studies report smoothed return at 20 rungs; portfolios report running aggregates at task-segment rungs. The terminal budget is reserved for the final objective and is not reported as a pruning rung. `MedianPruner` stops weak trials with `killpg`; crashes and timeouts FAIL, a non-finite reported return PRUNEs, and a missing or NaN final objective FAILs. The study continues; objective is the **tail mean** over the final training quarter.

Isolation contains VRAM failures and NaN physics. Worker processes across nodes share a journal.

## Cross-task normalization

Trial `user_attrs` store each task's raw and normalized score (`seg/<task>/raw`, `seg/<task>/norm`).

## Parallelism and resuming

1. Set `--n-workers N` slots per driver; `--gpus 0,1` round-robins GPUs. Starts stagger to avoid Isaac shader-cache races until metrics arrive.
1. Share `logs/tune/<study>/journal.log` across nodes; each SLURM array element is one worker, and Optuna assigns trials without scheduler integration.
1. Rerun to **resume**; terminal and `RUNNING` trials count toward `--n-trials`. Changes to search space, budget, objective, obs mode, or task set need a new `--version`.
1. Abrupt driver termination may leave trials `RUNNING`; handled cancellation marks active trials `FAIL`; repair with `srb agent tune repair --study S --stale-hours H`.
1. Pruned child logdirs are deleted, including checkpoints and TensorBoard events. `--keep-artifacts` retains them; `trial_cfg.yaml` and `metrics.jsonl` remain.

## Reproducibility

Each `logs/tune/<study>/trials/<n>/` preserves config, seed (`seed-base + trial × seeds-per-trial + replicate`), and metadata. Replicates use `rep-<k>/trial_cfg.yaml`. `metadata.json` records study/trial; `logs/tune/<study>/driver.json` records host, pid, and start time. Records aid reconstruction, not identical results. See [Agent Configuration](../config/agent_cfg.md).

Next: [Imitation Learning](imitation_learning.md)
