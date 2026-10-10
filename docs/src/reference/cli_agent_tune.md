# `srb agent tune` — Hyperparameter Optimization

## Verbs

`check` preflights; `run` starts or resumes; `status` summarizes; `validate` reruns top trials on fresh seeds; `export` writes a tuned config; `repair` fails stale trials; `baseline` captures portfolio normalization values. The Isaac-free study runs trials in child processes; one failure does not stop it. Install Optuna with `uv sync --extra tune`.

```bash
srb agent tune check    [options]
srb agent tune run      [options]
srb agent tune status   --study S
srb agent tune validate --study S
srb agent tune export   --study S
srb agent tune repair   --study S
srb agent tune baseline [options]
```

All actions reject bare `key=value` overrides and tokens after `--` with `srb agent tune <action>: error: unsupported arguments ...; fix a setting for every trial with --pin KEY=VALUE`. Use `--pin` to fix values across trials.

## `check` Options

`check` runs `run`'s pre-trial checks, assembles one trial config in memory, validates the adapter, search space, base config, pins and collisions, sampler, merge, and budget conversion, and launches no study or training child. Checks print `[OK]`, `[WARN]`, or `[FAIL]`; hard errors return non-zero. Accepted `run` options:

| Options | Default or limit |
| --- | --- |
| `-e`/`--env`/`--task`, `--tasks` | Choose one task or portfolio; pass either `--env`/`--task` or `--tasks`, not both. |
| `--algo` | Required. |
| `--obs`, `--trial-steps` | `--trial-steps` required. |
| `--pin`, `--trial-num-envs`, `--seed-base`, `--seeds-per-trial`, `--normalize`, `--aggregate`, `--objective`, `--fps`, `--n-trials`, `--n-workers`, `--version`, `--logdir` | See `run` defaults below. |

For `--normalize baseline` portfolios, it verifies fresh floor/ref baselines for every task. `--fps` adds the `run` wall-clock projection.

```bash
srb agent tune check --env sample_collection --algo skrl_ppo --trial-steps 10_000_000 --fps 50000
srb agent tune check --tasks sample_collection peg_in_hole --algo skrl_ppo \
  --trial-steps 10_000_000 --normalize baseline   # checks baselines.yaml too
```

## `run` Options

| Option | Default or limit |
| --- | --- |
| `-e`/`--env`/`--task` ENV | Single task; mutually exclusive with `--tasks` |
| `--tasks` T [T ...] | Portfolio; mutually exclusive with `-e`/`--env`/`--task` |
| `--algo` ALGO | **REQUIRED**; e.g. `skrl_ppo` |
| `--obs` {state,visual} | `state`; `visual` is unsupported |
| `--trial-steps` N | **REQUIRED**; env-step budget per trial and task segment |
| `--n-trials` N | `20`; terminal and `RUNNING` trials count on resume |
| `--n-workers` N | `1` concurrent trial slots |
| `--gpus` `0,1` | Unset inherits environment/framework default; otherwise round-robin |
| `--trial-num-envs` N | `1024`; pinned parallel envs, drives budget conversion |
| `--seed-base` N | `42` |
| `--seeds-per-trial` K | `1` |
| `--pin` `agent.X=Y` … | —; fixed values across trials |
| `--objective` {ep_return_mean,success_rate} | `ep_return_mean` (episode-weighted mean over the last 25% of steps) |
| `--normalize` {none,baseline,rank} | `none` |
| `--aggregate` {median,mean,min} | `median` |
| `--max-wall-hours` H | —; stop scheduling at this wall limit |
| `--trial-timeout` S | —; kill trials exceeding this duration |
| `--fps` F | —; measured env-steps/s for wall-clock projection |
| `--n-startup` N | `10`; `15` for portfolio |
| `--boot-stagger` S | `30` if `--n-workers` > 1, else `0`; skipped after first child emits metrics |
| `--keep-artifacts` | `False`; PRUNED child logdirs removed; `trial_cfg.yaml` and `metrics.jsonl` survive |
| `--retry-failed` N | `0`; retry up to N FAILed sets except `reason=timeout`; raise `--n-trials` to run them |
| `--version` N | `1`; bump for an incompatible rerun |
| `--logdir`/`--logs` | `logs/tune` |
| `--storage` URL | Journal default; PostgreSQL requires `tune-rdb` (`postgresql://` or `postgresql+psycopg2://`) |

Journal storage is default; `--storage` selects PostgreSQL without fallback. SRB validates and redacts credentials, bounds the pool, and records frozen study identity before creation. Schema checks fail closed; SRB does not migrate. Back up before upgrades. Install both extras with `uv sync --extra tune --extra tune-rdb`.

The task must emit the selected objective. If it never reports success, `--objective success_rate` fails each trial early with `success_rate_unavailable` instead of running its full budget.

### Averaging seeds during a study (`--seeds-per-trial`)

`--seeds-per-trial K` runs K distinct seeds (`seed-base + trial*K + replicate`) and gives TPE their mean. Pruning uses replicate 0: weak trials stop after one child; survivors cost about K times more. Crashed replicates are excluded; `n_seeds` and `n_seeds_ok` are recorded. Replicate 0 uses `trials/NNN/`; others use `trials/NNN/rep-K/` and discard checkpoints. Use K for cleaner search objectives; [`validate`](#validate-options-noise-arbiter) can rerank top trials on fresh seeds at lower cost than a multi-seed study.

Resume validates the frozen manifest and refuses changes to search space, `--trial-steps`, `--objective`, `--obs`, `--seeds-per-trial`, or tasks; bump `--version` for incompatible studies. Seed-base and Optuna-version drift are allowed. Keep K fixed so trials use the same seed schedule (`seed-base + trial*K + replicate`).

## `baseline` Options

Capture per-task floor/ref values for `--normalize baseline`; output: `hyperparams/tune/baselines.yaml`.

| Option | Default or limit |
| --- | --- |
| `-e`/`--env`/`--task`, `--tasks` | —; tasks to baseline |
| `--algo` ALGO | **REQUIRED** |
| `--obs` {state,visual} | `state` |
| `--budget`/`--ref-steps` N | **REQUIRED**; reference env-steps equal trial budget |
| `--floor-steps` N | `200000` random-rollout env-steps |
| `--trial-num-envs` N | `1024`; pinned parallel envs, drives budget conversion |
| `--seed-base` N | `42`; task *k* uses `seed-base + k` |
| `--gpus` `0,1` | Unset inherits environment/framework default; otherwise round-robin |
| `--objective` {ep_return_mean,success_rate} | `ep_return_mean` |
| `--trial-timeout` S | — |
| `--force`, `--dry-run` | `False`; overwrite entries / compute without writing `baselines.yaml` |
| `--logdir`/`--logs` PATH | `logs/tune` |

Entries store random-policy mean `floor`, reference-run tail-mean `ref`, source config path, and its git blob SHA. `tune run` and `tune check` warn on SHA drift; recapture before trusting normalization.

## Portfolio (Multi-Task) Studies

`--tasks A B C …` tunes one parameter set across tasks in fixed order, with one `srb agent train` child per segment under `trials/NNN/seg-<task>/`. Normalized scores form one objective; diverging segments prune the trial. MedianPruner compares each segment’s running aggregate, skipping the first `max(1, K // 3)` of `K` segments and the last, so two-task studies prune only on divergence.

Raw returns (`--normalize none`) require shared task reward scales; otherwise the driver warns. `baseline` uses `(score − floor) / (ref − floor)` from `baselines.yaml`; missing entries fail, stale SHAs warn. `rank` uses fractional rank per task across completed trials; it needs no baseline but early ranks are non-stationary. Aggregation defaults to `median` (resists outliers); `min` lets one weak task sink a trial; `mean` averages scores. Portfolio exports default to global L0.

1. Capture floor/ref baselines for the task set.
1. Run the portfolio study normalized against those baselines.
1. Export the winner to the global per-algorithm config.

```bash
srb agent tune baseline --tasks sample_collection peg_in_hole landing \
  --algo skrl_ppo --budget 10_000_000
srb agent tune run --tasks sample_collection peg_in_hole landing --algo skrl_ppo \
  --trial-steps 10_000_000 --n-trials 30 --normalize baseline --aggregate median
srb agent tune export --study srb-skrl_ppo-state-multi-<hash8>-v1   # --to global by default
```

## Registered search spaces

State-observation spaces are registered for these online RL labels:

| Framework | Labels |
| --- | --- |
| skrl | `skrl_ppo`; `skrl_sac`; `skrl_ppo_rnn` |
| stable-baselines3 | `sb3_ppo`, `sb3_sac`, `sb3_tqc`, `sb3_td3`, `sb3_crossq` |
| sbx (JAX SB3) | `sbx_ppo`, `sbx_sac`, `sbx_tqc`, `sbx_td3`, `sbx_crossq` |
| rsl_rl | `rsl_rl_ppo` |
| DreamerV3 | `dreamer` |
| TD-MPC2 | `tdmpc2` |

Unknown `--algo` labels list available spaces and exit. Registered but unavailable labels fail capability preflight.

### DreamerV3 / TD-MPC2 caveats

Dreamer uses `run.steps`; TD-MPC2 uses `steps` as its native total-step budget. Export resets these driver-owned keys to production values; `--pin` rejects them. With `num_envs > 1`, framework and reporter step counts may differ by a constant factor; calibrate a short run before trusting projections.

Dreamer tunes `run.train_ratio` and a `model_size` capacity preset (12M to 100M). TD-MPC2 tunes optimizer, MPPI planner, and loss; `model_size` sets architecture dimensions at load.

## `validate` Options (noise arbiter)

`validate` reruns the top K completed trials on fresh seeds, ranks their mean objectives, and writes `<study>/validation.json` without changing study records. Candidates use recorded `trials/NNN/trial_cfg.yaml` files, including pins; only the seed and `--validate-steps` budget change. Portfolio candidates rerun every segment with study normalization. Winners need `--min-ok` COMPLETEd seeds (default: strict majority); others are marked `(below min-ok)`.

| Option | Default or limit |
| --- | --- |
| `--study` NAME | **REQUIRED** |
| `--storage` URL | journal default; PostgreSQL Optuna URL |
| `--top-k` K | `3` trials by training value |
| `--seeds` S | `3` fresh seeds per candidate, averaged |
| `--seed-base` B | `10000`; seed = `B + i`; keep `B` through `B+S-1` clear of training seeds |
| `--min-ok` N | Majority of `--seeds` COMPLETEd seeds |
| `--validate-steps` N | Study budget (env-steps); raise for higher fidelity |
| `--gpus` `0,1` | Unset inherits environment/framework default; otherwise round-robin |
| `--n-workers` N | `1` concurrent rerun slots |
| `--trial-timeout` S | —; kill reruns exceeding this many seconds |
| `--keep-artifacts` | `False` |
| `--logdir`/`--logs` | `logs/tune` |

```bash
srb agent tune validate --study srb-skrl_ppo-state-sample_collection-v1 \
  --top-k 5 --seeds 4 --gpus 0,1
```

The printout names the robust winner and prints its `export` command; if it differs from `best_trial`, it notes the change.

## `status` / `export` / `repair` Options

| Option | Applies to | Default or limit |
| --- | --- | --- |
| `--study` NAME | All | **REQUIRED**; e.g. `srb-skrl_ppo-state-pick-v1` |
| `--storage` URL | status, export, repair | journal default; PostgreSQL Optuna URL |
| `--logdir`/`--logs` PATH | status, export, repair | `logs/tune` |
| `--trial` (trial number or `best`) | export | `best` |
| `--to` {task,global} | export | Study-dependent: task for single-task, global for portfolio; portfolio `--to task` is ambiguous and errors |
| `--dry-run` | export | `False`; print diff without writing |
| `--force` | export | `False`; overwrite existing file |
| `--stale-hours` H | repair | `24`; fail RUNNING trials older than this |

## Examples

```bash
srb agent tune run --env sample_collection --algo skrl_ppo \
  --trial-steps 20_000_000 --n-trials 40 --n-workers 2 --gpus 0,1
```

## Output

Study files live under `logs/tune/<study-name>/`:

```text
logs/tune/srb-skrl_ppo-state-sample_collection-v1/
  journal.log        # Optuna JournalStorage (NFS-safe; shared by parallel workers)
  study.yaml         # frozen manifest (search-space hash, budget, objective, seed base)
  trials/000/        # per-trial: trial_cfg.yaml, metrics.jsonl, logs/
```

Exports include a provenance header in `hyperparams/<framework>/<algo>.yaml` (global) or `hyperparams/task/<task>/<framework>/<algo>.yaml` (task-specific); `<algo>` is the framework-local name (e.g. `ppo`). Dreamer and TD-MPC2 use single files: `dreamerv3.yaml` and `tdmpc2.yaml` in the same roots.

## See Also

- [Hyperparameter Tuning Workflow](../workflows/hyperparameter_tuning.md) — concepts and wall-clock estimates
- [`srb agent train`](cli_agent_train.md) — the per-trial training subprocess
- [Agent Configuration](../config/agent_cfg.md) — config layering and tuned-file locations
