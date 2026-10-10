# `srb agent eval` — Evaluate Agent

Run a trained policy in a registered environment.

## Usage

```bash
srb agent eval --env ENV_ID (--algo ALGO | --model MODEL) [options]
```

Supported algorithm labels: `sb3_*`, `sbx_*`, `skrl_*`, `rsl_rl_ppo`, `dreamer`, `tdmpc2`, and `robomimic_*`.

## Options | [Shared Agent Options](cli.md#shared-agent-options)

| Option | Description | Default |
| --- | --- | --- |
| `--algo` ALGO | Algorithm. Loads latest standard-log checkpoint unless `--model` is set. | unset |
| `--model` PATH | Checkpoint or SRB artifact. Infers algorithm from path if `--algo` is omitted. | unset |
| `--obs` {state,visual} | Selects observation, agent-config variant, and checkpoint tree (`<algo>` or `<algo>-visual`). `visual` is unsupported; see [Agent Configuration](../config/agent_cfg.md#tuned-configurations). | `state` |

The command errors unless `--algo` or `--model` is set.

## Bounded quantitative evaluation

Set positive `SRB_EVAL_EPISODES=N` to request bounded evaluation. SB3, SBX, and RSL-RL act deterministically. SRB artifacts are bounded only when the loader reports deterministic inference.

The quota is split across environments: each gets `N // num_envs` episodes, and the lowest `N % num_envs` environment IDs get one extra. Count only episodes started during evaluation. Environments keep stepping after their quota. Extra episodes do not count, which avoids short-episode bias.

Mean return is undiscounted. Rewards are summed without discount or extra framework normalization. Task shaping and configured normalization still apply, so returns are not unshaped physical metrics.

Task `success` info supplies success metrics. Without it, `success_rate`, `successes`, and `failures` are `null`. Termination alone does not count as success.

### Results and output

Bounded results contain:

- Episode counts: `episodes_requested`, `episodes_completed`, and `n_episodes`.
- Metrics: `mean_return`, `success_rate`, `successes`, and `failures`.
- Run state: `complete`, `incomplete`, `steps`, `max_steps`, and `stop_reason`.
- Environment details: `num_envs` and `episode_sampling` (`per_env_quota`).

SB3 and SBX also return `requested_checkpoint` (pre-fallback selection or `null`), `loaded_checkpoint` (file or `null`), and `checkpoint_fallback` (whether an incomplete selection was replaced). The returned result and its JSON contain these fields. Ignore unused fields.

Bounded paths log one `SRB_EVAL_RESULT` JSON record. They write formatted results to `SRB_EVAL_OUT` or, if unset, the log-directory `eval.json`.

The finite guard allows twice the requested episodes multiplied by the maximum episode length. If the maximum is unavailable, it allows 1000 steps per episode. An early stop sets `complete: false` and `incomplete: true`.

Without `SRB_EVAL_EPISODES`, providers run their visualization or diagnostic evaluation.

### Provider-specific behavior

Robomimic (`robomimic_*`) requires checkpoint-reported deterministic inference. It rejects stochastic or unknown behavior before acting. It runs N episodes in one environment; `horizon` does not truncate them. Set and record `env.seed` (default 42). Unbounded mode uses its rollout count and writes to `SRB_EVAL_OUT` or the log-directory `eval.json`.

SKRL is not quantitative. It only lowers a higher trainer budget to about twice the requested episodes times the time limit. It ignores non-integer or nonpositive values. It does not count episodes, enforce quotas, report shortfalls, or emit the record. Do not compare its output quantitatively.

TD-MPC2 MPC stays stochastic even with exploration noise disabled. Bounded evaluation rejects it; unbounded MPC is diagnostic only. With `agent.mpc=false`, it uses the policy mean and the quotas and results described above.

Pinned Dreamer samples its latent posterior and policy, so it is not deterministic.

Set `env.seed` explicitly; the episode budget does not change it:

```bash
SRB_EVAL_EPISODES=256 SRB_EVAL_OUT=/tmp/landing-eval.json \
  srb agent eval --env landing --algo sb3_ppo env.seed=1000
```

## Examples

```bash
# Latest SBX PPO checkpoint on landing
srb agent eval --env landing --algo sbx_ppo env.num_envs=16

# Infer algorithm from checkpoint path
srb agent eval --env landing \
  --model logs/landing/sbx_ppo/<run>/ckpt/<ckpt>

# SRB artifact, bundled restore config if available
srb agent eval --env excavation \
  --model /models/srb_excavation_kinova_gen3_smooth_osc_rsl_rl_ppo
```

SRB artifacts use shared loading for [collection](cli_agent_collect.md) and real-world validation, preserving bundled Dreamer, TD-MPC2, or RSL-RL configs without the training log directory ([model docs](cli_model.md)).
