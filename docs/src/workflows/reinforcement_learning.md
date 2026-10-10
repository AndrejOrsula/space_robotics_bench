# Reinforcement Learning Workflow

## 1. Train your 1<sup>st</sup> RL Agent

> Reference: [`srb agent train` — Train Agent](../reference/cli_agent_train.md)

Train online with `srb agent train`; offline algorithms use separate workflows.

Watch [SBX](https://github.com/araffin/sbx) [PPO](https://arxiv.org/abs/1707.06347) on `landing`; omit `--headless`:

```bash
srb agent train --algo sbx_ppo --env landing env.num_envs=512 --hide_ui
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/hx4NqG6NCGQ?si=uBZ8kceMUxkprtwJ&mute=1&autoplay=1&loop=1&playlist=hx4NqG6NCGQ" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Check terminal progress; checkpoints save regularly. Usually stop with Ctrl+C.

## 2. Evaluate your Agent

> Reference: [`srb agent eval` — Evaluate Agent](../reference/cli_agent_eval.md)

```bash
srb agent eval --algo sbx_ppo --env landing env.num_envs=16
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/mi247B_OcZU?si=xDBg_ZFAwHYd7wTR&mute=1&autoplay=1&loop=1&playlist=mi247B_OcZU" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

```bash
srb agent eval --algo sbx_ppo --env landing env.num_envs=16 --model logs/landing/sbx_ppo/${TIMESTAMP}/ckpt/${CHECKPOINT}
```

> **Note:** SB3/SBX `.zip` checkpoints live in `ckpt/`.

## 3. Try a Different Algorithm

| Type | Framework algorithms |
| --- | --- |
| **Model-based** | DreamerV3: `dreamer`; TD-MPC2: `tdmpc2` |
| **On-policy** | Stable-Baselines3 (SB3): `sb3_a2c`, `sb3_ppo`, `sb3_ppo_lstm`, `sb3_trpo`; SBX: `sbx_ppo`; RSL-RL: `rsl_rl_ppo`; skrl: `skrl_a2c`, `skrl_ppo`, `skrl_ppo_rnn`, `skrl_rpo`, `skrl_trpo` |
| **Off-policy** | SB3: `sb3_ddpg`, `sb3_td3`, `sb3_sac`, `sb3_crossq`, `sb3_tqc`; SBX: `sbx_ddpg`, `sbx_td3`, `sbx_sac`, `sbx_crossq`, `sbx_tqc`; skrl: `skrl_ddpg`, `skrl_td3`, `skrl_sac` |
| **Evolutionary** | SB3: `sb3_ars` |

> **skrl:** Hyperparameter profiles are untuned starting points.

```bash
srb agent train --headless --algo <ALGO> --env landing env.num_envs=1024
```

> **Hint:** `--headless` and more parallel environments can speed training.

## 4. Monitor Training Progress

Compare runs with TensorBoard; local artifacts and TensorBoard are on by default. W&B is opt-in:

```bash
tensorboard --logdir ./logs --bind_all
```

- **DreamerV3:** `srb agent train ... 'agent.logger.outputs=[tensorboard,wandb]'`
- **SB3 & SBX:** `srb agent train ... +agent.wandb=true`
- **skrl:** `srb agent train ... agent.agent.experiment.wandb=true`

> **Note:** W&B logging requires an account and API key.

Each run records its provenance and metrics; see [Run records](../reference/cli_agent_train.md#run-records).

## 5. Configure Hyperparameters

> Reference: [Agent Configuration](../config/agent_cfg.md)

```bash
srb agent train --algo dreamer      agent.run.train_ratio=128  ...
srb agent train --algo sb3_ppo      agent.gamma=0.99           ...
srb agent train --algo sbx_sac      agent.learning_rate=0.0002 ...
srb agent train --algo skrl_ppo     agent.models.separate=True ...
```

Next: [Hyperparameter Tuning](hyperparameter_tuning.md)
