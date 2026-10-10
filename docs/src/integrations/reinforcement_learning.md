# Integration — Reinforcement Learning

Start training with these steps:

1. [Install SRB](../getting_started/install.md).
1. Complete [Basic Usage](../getting_started/basic_usage.md).
1. Follow the [RL Workflow](../workflows/reinforcement_learning.md) to train and evaluate an agent.
1. Optionally, configure [Parallelism](../config/parallelism.md) for environment count, layout, and stacking constraints.

SRB's Gymnasium environments support Reinforcement Learning (RL) policy
prototyping, training, evaluation, and generalization studies across space
robotics tasks with modern frameworks. SRB targets Isaac Sim 6.1.0 and Isaac
Lab 3.0.0; see [Requirements](../getting_started/requirements.md).

<iframe style="width:100%;aspect-ratio:5/4" src="https://www.youtube.com/embed/ulxwdki9vzs?si=_ftFARI9fDwxio1Q&mute=1&autoplay=1&loop=1&playlist=ulxwdki9vzs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

RL can develop autonomous behavior under communication delays or environmental
uncertainty. SRB provides tasks from navigation to mobile manipulation,
procedural generation and domain randomization, and consistent evaluation
metrics. Batched data collection depends on task/provider support. The
[compatibility matrix](../reference/compatibility.md) covers supported robot
categories; see [Parallelism](../config/parallelism.md) for environment-count
and stacking constraints.

## Available Integrations

Official framework integrations:

- **[Dreamer](https://danijar.com/project/dreamerv3) [JAX]** — Model-based RL.
- **[Stable-Baselines3 (SB3)](https://stable-baselines3.readthedocs.io) [PyTorch]** — Popular RL algorithms.
- **[SBX](https://github.com/araffin/sbx) [JAX]** — JAX implementations of popular RL algorithms.
- **[skrl](https://skrl.readthedocs.io) [PyTorch]** - SRB's adapter supports single-agent RL algorithms through the PyTorch API. It targets the skrl 2.1 API (`skrl~=2.1`, the Isaac Lab 3.0 profile); other major versions fail early with `optional_dependency_unavailable`, and checkpoints from skrl 1.x are not supported.
- **[TD-MPC2](https://www.tdmpc2.com) [PyTorch]** — Scalable, sample-efficient model-based RL.
- **[RSL-RL](https://github.com/leggedrobotics/rsl_rl) [PyTorch]** — Fast on-policy PPO, widely used in legged locomotion research.

### Unsupported Frameworks

ACME is unsupported: adapters share Isaac Sim's Python process, and ACME's
dependencies (`dm-reverb`, `dm-launchpad`, and `dm-acme[jax]`'s
`jaxlib==0.4.3` pin) do not resolve with the SRB Docker runtime's Python 3.12.

RL Games and RLinf are not SRB integrations.

Next: [Reinforcement Learning Workflow](../workflows/reinforcement_learning.md)
