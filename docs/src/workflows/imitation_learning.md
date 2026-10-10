# Imitation Learning Workflow

Collect expert demonstrations, train offline, evaluate, then deploy a compatible policy. SRB supports compatible tasks, datasets, and algorithms. Robomimic requires one environment and compatible observations and actions.

## 1. Collect demonstrations

Teleoperate the task and keep successful demonstrations. Follow the [success rules](../reference/cli_agent_collect.md#tips).

```bash
srb agent collect --env peg_in_hole --num_episodes 50
```

The keyboard is the default. A 6-DoF SpaceMouse gives higher fidelity. An XR headset adds an immersive view, not input ([Extended Reality](../integrations/extended_reality.md)).

```bash
srb agent collect --env peg_in_hole --teleop_device spacemouse
```

For collection defaults, success-only behavior, and dataset paths, see the [`srb agent collect` reference](../reference/cli_agent_collect.md).

## 2. Train an IL policy

Train offline; Isaac Sim is not needed. Pass `--dataset`. A fresh `learn` run directory does not find earlier `collect/` data and errors if no dataset is supplied. Verified private HDF5 copies stay open until workers, handles, and logging close. Configured allocation limits are checked before provider reads. See [limits, provenance, and `--normalize_actions`](../reference/cli_agent_learn.md#dataset-admission-and-normalization).

```bash
srb agent learn --algo robomimic_bc --env peg_in_hole \
  --dataset logs/peg_in_hole/collect/<timestamp>/datasets/demos_<timestamp>.hdf5
```

For the RNN variant, use `robomimic_bc_rnn`:

```bash
srb agent learn --algo robomimic_bc_rnn --env peg_in_hole \
  --dataset logs/peg_in_hole/collect/<timestamp>/datasets/demos_<timestamp>.hdf5
```

HDF5 stores per-step `dones`, `terminated`, and `truncated`. `dones = terminated OR truncated` marks episode boundaries. Value learners bootstrap through time-limit and collector-horizon truncations; they stop only on `terminated`. Robomimic `bcq`, `cql`, `iql`, `td3_bc`, and `iris` (BCQ value component) read `dones` and treat truncations as terminal.

## 3. Evaluate the policy

Evaluate a [learn checkpoint](../reference/cli_agent_eval.md):

```bash
srb agent eval --algo robomimic_bc --env peg_in_hole env.num_envs=1
```

Robomimic uses one environment:

- Collection raises `robomimic collection requires num_envs == 1` if configured otherwise.
- Evaluation rejects `num_envs != 1` with `integration.robomimic.vector_env.unsupported`.
- Offline `learn` creates no environment.

Evaluation checks task `success` at rollout end. If the task has no success signal, it warns and reports `success_rate: null`, never 0%.

For normalized actions, checkpoint `norm_factor_min` and `norm_factor_max` restore the environment range. The caller cannot override this range. If the checkpoint lacks a range but the run directory has `logs/normalization_params.txt`, evaluation errors and requires retraining. Otherwise, it loads unscaled actions.

Nested observations use slash-joined training keys.

## 4. Deploy to hardware

Use IL for sim-to-real with a compatible task, checkpoint, and hardware interface. Hardware operation is unvalidated. See [Sim-to-Real Transfer](sim_to_real.md) for `real_agent gen` and `real_agent eval`.

## 5. Optional: iterate

### Augment with policy rollouts

Collect more demonstrations with a partially trained policy (DAgger-style). Apply the same `--success_only` rules as in step 1.

```bash
srb agent collect --algo robomimic_bc --env peg_in_hole \
  --num_episodes 200
```

Rerun `srb agent learn` on merged HDF5 files. It accepts one `--dataset` for union training.

### Combine with RL

IL may not generalize to unseen conditions. Incompatible checkpoint formats prevent direct RL warm-starting. See the [reinforcement learning workflow](reinforcement_learning.md):

- Train RL from scratch on the same task while IL collects demos.
- Resume interrupted RL with `srb agent train --continue_training` or a same-framework `--model`.
- Prefill `skrl_sac`, `skrl_ddpg`, or `skrl_td3` replay from canonical export with `--demo-dataset`, `--demo-max-transitions` ([metadata, limits, resume](../reference/cli_agent_train.md#skrl-demonstration-prefill)).
- Prefill Dreamer sequence replay from the canonical format. See [continuous-action requirements, persistence, capacity, and source-bound resume](../reference/cli_agent_train.md#dreamer-demonstration-prefill).

Next: [Sim-to-Real Transfer](sim_to_real.md)
