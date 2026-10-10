# Development Utilities

Use these utilities to [update assets](update_assets.md) or [clean the cache](clean_cache.md).

## Reward-function reference plots

Generate PDF references for implemented reward terms with CPU Torch:

1. From a source checkout, install the `plots` extra (Matplotlib, NumPy, Torch, Seaborn).

1. Run this command to write the PDFs to `outputs/reward_terms/`:

   ```bash
   MPLBACKEND=Agg uv run --extra plots python - <<'PY'
   from pathlib import Path

   import matplotlib.pyplot as plt

   from srb.core.reward.scripts.plot_reward_terms import plot_library_final_order

   output = Path("outputs/reward_terms")
   output.mkdir(parents=True, exist_ok=True)
   plot_library_final_order(weight=1.0, limit=4.0, output_path=output / "reward_terms.pdf")
   plot_library_final_order(weight=-2.0, limit=64.0, output_path=output / "penalty_terms.pdf")
   plt.close("all")
   PY
   ```

`Agg` needs no display. PDFs show reward functions, not task dynamics or learned
behavior. Running `srb/core/reward/scripts/plot_reward_terms.py` directly writes PDFs
beside the script; the function above writes to `outputs/`.

For the manual TF trajectory visualizer, see the
[ROS integration guide](../integrations/ros2.md#manual-tf-trajectory-visualizer).
