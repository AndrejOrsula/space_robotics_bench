# Reward & Normalization Configuration

______________________________________________________________________

Define task reward configs by inheriting from `BaseRewardCfg` or a domain subclass.

> **To retune a term, use a CLI override** such as `env.reward.penalty_action_rate.weight=-0.2`. You usually do not need to edit Python. See the task's `RewardCfg` for defaults.

## 1. Defining a Reward Configuration

Example:

```python
from srb.core.reward import BaseRewardCfg, LinearRewardTerm, GaussianRewardTerm

class RewardCfg(BaseRewardCfg):
    # Smoothness & efficiency penalties
    penalty_action_rate: LinearRewardTerm = LinearRewardTerm(weight=-0.5, limit=4.0)

    # Task specific target rewards
    reward_target_approach: GaussianRewardTerm = GaussianRewardTerm(
        weight=4.0, scale=0.5
    )
```

On each environment step, the runtime:

1. Calls `advance_step_state()` to update per-step state.
1. Calls `extract_step_return()` to return current-state metrics in the task dictionary.
1. Sums the reward terms the task returns and optionally normalizes the total.

______________________________________________________________________

## 2. Types of Reward Terms

Terms are in `srb.core.reward.terms`, except `TukeyRewardTerm` in `srb.core.reward.normalize`. All are exported from `srb.core.reward` and inherit from `RewardTerm`.

### 📐 Linear and Clamped Terms

- **`LinearRewardTerm`**: Scales `|metric|` linearly; use `ScaledRewardTerm` to preserve sign.
  $$\\text{Reward} = \\text{clamp}(\\text{weight} \\times |\\text{metric}|, -\\text{limit}, \\text{limit})$$
- **`QuadraticRewardTerm`**: Scales squared metrics by weight.
- **`OffsetLinearRewardTerm` / `OffsetQuadraticRewardTerm`**: Activate beyond a deadband threshold.

### 🎯 Kernel and Tracking Terms

- **`GaussianRewardTerm`**: Returns a positive bell-shaped value multiplied by `weight`; use a negative weight for a penalty. It depends on squared error, so the error sign does not change the result.
  $$\\text{Reward} = \\text{weight} \\times \\exp\\left(-\\frac{\\text{metric}^2}{2 \\, \\text{scale}^2}\\right)$$
- **`GaussianTrackingRewardTerm`**: Gaussian-shaped around its configured target, scaled by `weight` (positive for reward, negative for penalty).
- **`GaussianZoneTrackingRewardTerm`**: Rewards values within a flat band around its target.

### 🛡️ Barrier Terms

- **`HyperbolicBarrierRewardTerm` / `GaussianBarrierRewardTerm`**: Shape rewards or penalties around safety thresholds; contributions approach zero farther beyond them.

______________________________________________________________________

## 3. Reward Normalization

To estimate best and worst step returns, `BaseRewardCfg` sums signed term `reward_bounds` and `normalize_offset_per_step`. One-sided terms use the weight sign. Sign-changing terms such as `ScaledRewardTerm` and `SmoothMaxRewardTerm` span both bounds:

$$R\_{\\text{worst}} = \\sum\_{T} \\text{worst}(T) + o\_{\\text{worst}} \\qquad R\_{\\text{best}} = \\sum\_{T} \\text{best}(T) + o\_{\\text{best}}$$

### Normalization Modes

Set the reward config's `normalize` parameter:

| Normalization Mode | Value | Behavior & Mathematics | Recommended For |
| :--- | :--- | :--- | :--- |
| **`AUTO`** *(Default)* | `"auto"` | Uses `EPISODE` for finite-horizon tasks, otherwise `PER_STEP`. | Default; `excavation` and `terrain_landscaping` override it with `NONE`. |
| **`NONE`** | `"none"` | Returns the sum unchanged. | Debugging or classic benchmarks. |
| **`PER_STEP`** | `"per_step"` | Divides by $S = \\max(R\_{\\text{best}}, |R\_{\\text{worst}}|)$; steps scale about $[-1, 1]$. | Infinite-horizon tracking, such as `locomotion_velocity_tracking`. |
| **`EPISODE`** | `"episode"` | Divides by $S = \\max(R\_{\\text{best}}, |R\_{\\text{worst}}|) \\times N\_{\\text{max_steps}}$; episodes scale about $[-1, 1]$. | Finite-horizon tasks with terminal states, such as `landing` and `debris_capture`. |

______________________________________________________________________

## 4. Normalization Troubleshooting & Best Practices

> [!TIP]
> Balance reward terms. An extreme limit such as `limit=1000.0` can dominate the scale and make smaller terms negligible.

Print reward bounds to find dominant terms and assess the normalization range:

```python
reward_cfg = YourTaskRewardCfg()
worst, best = reward_cfg.get_bounds()
print(f"Worst per-step: {worst}, Best per-step: {best}")
```
