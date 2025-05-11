<div class="warning">

### This page is under construction. Please check back later for updates.

</div>

# Reinforcement Learning Workflow

Reinforcement Learning (RL) is one of the primary focus areas of the Space Robotics Bench. While there are several RL frameworks with their unique peculiarities, SRB offers a unified interface for training and evaluating policies across a diverse set of space robotics tasks.

## 1. Train your 1<sup>st</sup> RL Agent

> Reference: [`srb agent train` — Train Agent](../reference/cli_agent_train.md)

The fastest way to get started with training an RL agent is by using the `srb agent train` command, which provides a streamlined interface for all integrated RL frameworks. In general, you want to specify the RL algorithm to use, the environment to train on, and the number of parallel environment instances used for rollout collection.

Let's start with a simple `landing` environment using the `sbx_ppo` algorithm ([PPO](https://arxiv.org/abs/1707.06347) implementation of [SBX](https://github.com/araffin/sbx)). For now, omit the `--headless` flag so that you can observe the convergence in real time:

```bash
srb agent train --algo sbx_ppo --env landing env.num_envs=512
```

<!-- TODO[docs]: Video of training -->

As you begin to observe the training process, you can also monitor the progress in your terminal. After about 25M timesteps, you will see that the agent found a stable policy that successfully solves the task. Checkpoints are saved regularly, so you are free to stop the training process at any point by sending an interrupt signal (Ctrl+C in most terminals).

## 2. Evaluate your Agent

> Reference: [`srb agent eval` — Evaluate Agent](../reference/cli_agent_eval.md)

Once training is complete, you can evaluate your agent with the `srb agent eval` command:

```bash
srb agent eval --algo sbx_ppo --env landing env.num_envs=16
```

<!-- TODO[docs]: Video of evaluation -->

By default, the latest checkpoint from the training run is loaded for evaluation. However, you might want to run the evaluation for a checkpoint specified via `--model`:

```bash
srb agent eval --algo sbx_ppo --env landing env.num_envs=16 --model space_robotics_bench/logs/landing/sbx_ppo/ckpt/${CHECKPOINT}
```

## 3. Try a Different Algorithm

SRB directly supports several popular RL algorithms from different frameworks:

| Algorithm Type      | DreamerV3 | Stable-Baselines3 | SBX        | skrl         |
| ------------------- | --------- | ----------------- | ---------- | ------------ |
| **Model-based**     | dreamer   |                   |            |              |
|                     |           |                   |            |              |
| **On-Policy**       |           | sb3_a2c           |            | skrl_a2c     |
|                     |           | sb3_ppo           | sbx_ppo    | skrl_ppo     |
|                     |           | sb3_ppo_lstm      |            | skrl_ppo_rnn |
|                     |           |                   |            | skrl_rpo     |
|                     |           | sb3_trpo          |            | skrl_trpo    |
|                     |           |                   |            |              |
| **Off-Policy**      |           | sb3_ddpg          | sbx_ddpg   | skrl_ddpg    |
|                     |           | sb3_td3           | sbx_td3    | skrl_td3     |
|                     |           | sb3_sac           | sbx_sac    | skrl_sac     |
|                     |           | sb3_crossq        | sbx_crossq |              |
|                     |           | sb3_tqc           | sbx_tqc    |              |
|                     |           |                   |            |              |
| **Evolutionary**    |           | sb3_ars           |            |              |
|                     |           |                   |            | skrl_cem     |
|                     |           |                   |            |              |
| **Imitation-based** |           |                   |            | skrl_amp     |

This time, you can train another agent using an algorithm of your choice:

```bash
srb agent train --headless --algo <ALGO> --env landing env.num_envs=1024
```

> **Hint:** Use `--headless` mode with more parallel environments for faster convergence.

## 4. Monitor Training Progress

While training, you might be interested in monitoring the progress and comparing different runs through a visual interface. By default, TensorBoard logs are saved for all algorithms and environments in the `space_robotics_bench/logs` directory. You can start TensorBoard to visualize the training progress:

```bash
tensorboard --logdir ./space_robotics_bench/logs --bind_all
```

Furthermore, you can enable Weights & Biases (`wandb`) logging by passing framework-specific flags **\[subject to future standardization\]**:

- **DreamerV3:** `srb agent train ... +agent.logger.outputs=wandb`
- **SB3 & SBX:** `srb agent train ... agent.track=true`
- **skrl:** `srb agent train ... +agent.experiment.wandb=true`

> **Note:** Logging to Weights & Biases requires an account and API key.

## 5. Configure Hyperparameters

> Reference: [Agent Configuration](../config/agent_cfg.md)

The default hyperparameters for all algorithms and environments are available under the [space_robotics_bench/hyperparams](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/hyperparams) directory. Similar to the environment configuration, you can adjust the hyperparameters of the selected RL algorithm through [Hydra](https://hydra.cc). However, the available hyperparameters and their structure is specific to each framework and algorithm.

Here are some examples (consult hyperparameter configs for more details):

```bash
srb agent train --algo dreamer      agent.run.train_raio=128   ...
sed agent train --algo sb3_ppo      agent.gamma=0.99           ...
srb agent train --algo sbx_sac      agent.learning_rate=0.0002 ...
srb agent train --algo skrl_ppo_rnn agent.models.separate=True ...
```
