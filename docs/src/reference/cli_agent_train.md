# `srb agent train` — Train Agent

## Start or Resume a Run

1. Choose an algorithm and environment.
1. Add environment overrides after options.
1. For replay prefill, see [SKRL](#sac-demonstration-prefill) or [Dreamer](#dreamer-demonstration-prefill).
1. Optionally package a successful run (see [Model Artifacts](#model-artifacts)).

```bash
srb agent train --algo sbx_ppo --env landing env.num_envs=512 --hide_ui

srb agent train --algo sbx_ppo --env landing --continue_training
srb agent train --algo sbx_ppo --env landing \
  --model logs/landing/sbx_ppo/<run>/ckpt/<ckpt>
```

Explicit `--model` checkpoint paths resolve relative to the current working directory. Logs default to `<repo>/logs` in a checkout and `<cwd>/logs` when installed; use `--logdir` to change the root.

### Run records

Each run has `metadata.json` and `run.json`; the record lists source and asset commits, simulator pair, image, host, GPU, driver, seed, algorithm, implementation, task, step budget, and hyperparameter overrides. Online training writes TensorBoard scalars to `metrics.jsonl` (`{"tag", "step", "wall_time", "value"}` per line); re-export interrupted runs with `python -m srb.utils.metrics_jsonl <logdir>`.

Source checkouts record Git commits; `site-packages` installs report `null`. Asset metadata records its submodule commit, pinned gitlink, and dirty flag (`null` if unknown). Metadata files and config snapshots redact secret-named values and flags, forwarded `kit_args`, URL credentials and query values, plus `Authorization` and `Bearer` values as `<redacted>`. SRB atomically reserves directories and suffixes same-second runs, such as `-001`.

## Options | [Shared Agent Options](cli.md#shared-agent-options)

| Option | Default | Limits / behavior |
| --- | --- | --- |
| `--algo` ALGO | **REQUIRED** | RL algorithms only |
| `--obs {state,visual}` | `state` | Selects the config and log tree (`<algo>` or `<algo>-visual`) and prevents cross-modality loading. `visual` is unsupported; see [Agent Configuration](../config/agent_cfg.md#tuned-configurations). |
| `--continue_training`, `--continue` | `False` | Resumes the latest checkpoint of the latest run, or trains from scratch in that run when it has none; mutually exclusive with `--model`. Support depends on the framework; TD-MPC2 rejects it. |
| `--model` PATH | unset | Resumes from the specified checkpoint. Support depends on the framework; TD-MPC2 imports weights only. |
| `--demo-dataset`, `--demo_dataset` PATH | unset | Canonical SRB JSONL for `skrl_sac`, `skrl_ddpg`, `skrl_td3`, or `dreamer`; requires `--demo-max-transitions`. |
| `--demo-max-transitions`, `--demo_max_transitions` N | unset | Positive cap on the complete source; requires `--demo-dataset`. |

### Resume behavior by framework

| Framework | Behavior and limits |
| --- | --- |
| TD-MPC2 | `--model` imports weights only; the optimizer, replay, schedules, and step counter restart at logged step 0. `--continue_training` is unsupported. |
| `sb3_*`, `sbx_*` | Both options restore the step and learning-rate schedule, then train only the remaining portion of the original `n_timesteps`; increase that value to extend training. Checkpoints are saved every `+agent.save_freq=N` environment steps (default `50000`). Continuation after a finished run warns and does nothing; `--model` at the budget fails. With `+agent.save_replay_buffer=true`, off-policy runs save `ckpt/rl_model_replay_buffer_<N>_steps.pkl`; only the newest periodic buffer is kept, plus `ckpt/replay_buffer.pkl`. Each buffer can take gigabytes. If the matching buffer is missing, `learning_starts` restarts at the resumed step. With normalization, `ckpt/rl_model_<N>_steps.zip` pairs with `ckpt/rl_model_vecnormalize_<N>_steps.pkl` for resume and evaluation. A missing matching per-checkpoint stats file fails if the run uses per-checkpoint statistics; legacy checkpoints may fall back to run-level stats with a warning. If no stats exist, resume and evaluation fail instead of using fresh stats. |
| `rsl_rl_ppo` | Adds `max_iterations` training iterations. |
| `skrl_*` | Resumes from the checkpoint timestep through `agent.trainer.timesteps` and retains `agent_<N>.pt`. Continuation after a finished run warns and does nothing; `--model` at the budget fails. If the checkpoint lacks a timestep or uses `experiment.store_separately`, resume fails unless `+agent.resume_timestep=N` supplies one. A saved timestep takes precedence, and evaluation ignores the override. Replay is not saved for ordinary SKRL runs; demo-prefilled replay is included in combined agent checkpoints. If off-policy `skrl_sac`, `skrl_td3`, or `skrl_ddpg` replay is empty, `learning_starts` restarts at the resumed timestep; `random_timesteps` does not repeat. Demo-prefilled replay keeps the configured warm-up. The actor cannot receive `state_privileged__*`; the critic can. |

## Supported Algorithm Families

Registered labels include `dreamer`, `tdmpc2`, `sbx_ppo`, `sb3_ppo`, `rsl_rl_ppo`, `skrl_ppo`, `skrl_sac`, and `skrl_ppo_rnn`. SRB provides SKRL models and profiles for `skrl_sac` and `skrl_ppo_rnn`. See the [Reinforcement Learning Workflow](../workflows/reinforcement_learning.md) for Hydra overrides and Weights & Biases logging.

<a id="sac-demonstration-prefill"></a>

## SKRL Demonstration Prefill

`skrl_sac`, `skrl_ddpg`, and `skrl_td3` prefill replay from canonical SRB JSONL exports with [`metadata.demo_io_contract`](cli_dataset.md#demonstration-io-metadata). The trainer validates observation projections and action coordinates, including runtime scales, against the live wrapper. It preserves explicit next observations and separate termination and truncation flags.

The cap applies to the whole source; exceeding it causes an error. The transition count must divide evenly by `env.num_envs` and fit the replay capacity; rows are neither padded nor dropped. The trainer checks admission before native allocation. Limits are 192 MiB for replay payloads and 256 MiB for serialized checkpoints. Oversized configurations report the maximum supported `agent.memory.memory_size` for the environment count and I/O dimensions. These limits do not bound peak process memory.

```bash
srb agent train --algo skrl_sac --env peg_in_hole \
  --demo-dataset /data/peg_in_hole_canonical \
  --demo-max-transitions 1000 \
  env.num_envs=1 agent.memory.memory_size=8192 --hide_ui
```

Resume requires both options and identical source bytes, I/O layout, and replay dimensions. Combined checkpoints restore replay, optimizer, and random state at the saved next timestep; the simulation resets. Resume rejects `agent.agent.experiment.store_separately=true`. Evaluation uses weights only.

DDPG and TD3 restore exploration-noise state and learning-rate schedules; TD3 also restores its delayed actor-update counter. Resume requires complete native modules and unchanged update, scheduler, sampling, and exploration settings, including native Gaussian or Ornstein-Uhlenbeck noise parameters and actor cadence. Custom noise implementations are rejected. Demo and online replay use the checkpoint-bound `agent.agent.rewards_shaper_scale`; arbitrary reward-shaping callbacks are unsupported.

Prefill is unsupported for other SKRL labels, SB3 (`sb3_*`), SBX (`sbx_*`), and RSL-RL (`rsl_rl_ppo`). `srb dataset compatibility DATASET --purpose train` reports the training mode and compatibility for each algorithm. For offline demonstrations, use Robomimic with [`srb agent learn`](cli_agent_learn.md).

## Dreamer Checkpoints and Continuation

Dreamer resumable state requires the original `agent.replay.save=true` and `agent.replay.save_wait=true` settings; persistence is disabled by default. Weights-only checkpoints lack replay and cannot use `--continue`; import them into a new run. Dreamer training runs in worker processes and rejects `--perf`, `--hide_ui`, and `--repl`.

The authoritative manifest selects the committed agent, step, and replay generation together. In-run `--model` must identify that checkpoint; older generations are rejected. Use a new log directory to import weights from another generation.

Cleanup retains the current generation and its parent, preserves files with uncertain ownership, and waits for same-run readers. Snapshots pin saved chunks and exclude later in-memory updates. Failed cleanup may leave extra files without invalidating the committed checkpoint.

## Dreamer Demonstration Prefill

`dreamer` uses paired options and canonical SRB JSONL. The task, observation terms, action layout, and runtime scales must match; actions must be continuous and within `[-1, 1]`. Each simulator worker needs matching projection metadata; missing or mismatched metadata causes rejection. The local route uses one environment; the process route uses one environment per worker.

Prefilled runs also require `replicas: 1`, positive `replay_context`, and the save settings above. Set `replay.size` and `replay.chunksize` for the source and observation/context dimensions; defaults of several million items may exceed admission limits. Replay arrays and staged rows, including recurrent context, are each capped at 256 MiB. Total process memory and model allocations are not bounded.

Dreamer adds a successor row per episode, retains terminal observations, and separates termination from timeout. Two 12-transition episodes yield 26 native rows and 18 windows at sequence length 9. Capacity counts windows and must fit the batch window; excess source transitions are rejected. Within each episode, projected `next_obs` must equal the next observation. Recurrent context uses the demonstrated previous actions.

Same-run resume requires an identical source path and bytes, projection, and bound learner/replay configuration. A mismatch is rejected before restore or config/metadata overwrite. Resume restores the agent and replay without reinserting demonstrations. Native replay omits the sampler cursor and random state, so batches are not bit-exact. Evaluation uses weights only.

Dreamer demonstration prefill has not been validated with GPU simulator training or on hardware. TD-MPC2 does not support it.

## Model Artifacts

Package successful runs as portable SRB artifacts with `--model-artifact-output`. Record provenance with `--model-artifact-dataset`, or package the run afterward with `srb model from-run`.

| Option | Default | Limits / behavior |
| --- | --- | --- |
| `--model-artifact-output` PATH | unset | Output artifact directory, after training succeeds |
| `--model-artifact-checkpoint` PATH | unset | Checkpoint to package; absolute or run-directory-relative |
| `--model-artifact-task` TASK | unset | Override task identity |
| `--model-artifact-embodiment` NAME | unset | Override embodiment identity |
| `--model-artifact-action-mode` MODE | unset | Override action-mode identity |
| `--model-artifact-framework` NAME | unset | Override framework metadata |
| `--model-artifact-dataset` PATH | unset | Dataset manifest, artifact directory, or file for provenance and identity inference |
| `--model-artifact-dataset-repo-id` ID | unset | Training dataset repository id |
| `--model-artifact-dataset-revision` REVISION | unset | Training dataset revision |
| `--model-artifact-repo-id` ID | unset | Override Hugging Face model repository id |

For demo-prefilled runs, packaging retains the training-time manifest and member digests if the source moves or is removed. Explicit `--model-artifact-dataset` controls the declared dataset link; consumed demo binding stays in artifact metadata.
