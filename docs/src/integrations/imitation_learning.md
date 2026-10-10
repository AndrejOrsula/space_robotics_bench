# Integration — Imitation Learning

Train an imitation learning (IL) policy from demonstrations with
`robomimic_*` algorithms:

1. Collect demonstrations with [`srb agent collect`](../reference/cli_agent_collect.md),
   or use an existing dataset in the robomimic-compatible HDF5 format.
1. Train with `srb agent learn --dataset` and a `robomimic_*` algorithm. See
   the [learning reference](../reference/cli_agent_learn.md).
1. To prefill an integrated reinforcement learning (RL) framework, export
   canonical SRB JSONL with `srb dataset export`. `skrl_sac`, `skrl_ddpg`,
   `skrl_td3`, and Dreamer can prefill replay from this format; integrated RL
   frameworks cannot load Robomimic HDF5 directly.

## When to Use Imitation Learning

Use IL alongside RL when reward engineering is difficult, rewards are sparse
(for example, assembly, sample collection, or docking), or expert trajectories
are available from teleoperation, planners, or prior policies. Demonstrations
can reduce exploration, but they do not establish a safe operating envelope;
hardware deployment needs separate validation. IL rollouts can collect
demonstrations or shape RL curricula, but RL checkpoints cannot warm-start from
IL checkpoints because their formats differ.

## Available Integrations

[robomimic](https://robomimic.github.io/) (PyTorch) provides BC
(MLP/RNN/Transformer), BCQ, CQL, IQL, TD3-BC, GL, HBC, IRIS, and Diffusion
Policy, using a standard HDF5 dataset format.

## Data Format

Demonstrations use the robomimic-compatible HDF5 layout:

- Each episode group contains `obs/`, `next_obs/`, `actions`, `rewards`,
  `terminated`, `truncated`, and `dones` datasets, plus a per-episode `success`
  attribute.
- Observation keys use `<group>/<leaf>` paths for SRB observation groups, such
  as `state/<leaf>`, `proprio/<leaf>`, `command/<leaf>`, and
  `state_privileged/<leaf>` when those groups are present.
- Per-episode success flags support collection-time `--success_only` filtering.

Datasets from `srb agent collect` load directly in `srb agent learn`.

## See Also

- [`srb agent learn`](../reference/cli_agent_learn.md) — train an IL policy
- [Reinforcement Learning Integration](reinforcement_learning.md) — complementary paradigm

Next: [Imitation Learning Workflow](../workflows/imitation_learning.md)
