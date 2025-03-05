# Random and Zero Agents

Instead of manually controlling each environment via `teleop.py`, you can use random and zero agents to test and debug certain functionalities.

## Random Agent

The [`random_agent.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/random_agent.py) script allows environments to act based on random actions sampled from the action space. This is particularly useful for verifying if environments are running as intended without manual control:

```bash
.docker/run.bash scripts/random_agent.py --task sample_collection --num_envs 4
```

## Zero Agent

Alternatively, [`zero_agent.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/zero_agent.py) executes environments where all actions are zero-valued, mimicking a steady-state system. This can be useful for analyzing the idle behaviour of environments:

```bash
.docker/run.bash scripts/zero_agent.py --task sample_collection --num_envs 4
```
