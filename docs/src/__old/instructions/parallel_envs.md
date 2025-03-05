# Simulating Parallel Environments

## The `--num_envs` Argument

All Python entrypoint scripts, e.g. [`teleop.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/teleop.py), [`random_agent.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/random_agent.py) and [`ros2.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/ros2.py), accept an optional `--num_envs` argument. By default, this is set to `1`, but you can specify more environments for parallel execution. For example, to run four environments, use the following command:

```bash
.docker/run.bash scripts/teleop.py --task sample_collection --num_envs 4
```

Each environment will generate its own procedural assets, providing unique experiences across different simulations. However, note that the time taken to generate these assets scales linearly with the number of environments. These assets will be cached for future runs unless the cache is cleared (explained later in this document).

After the environments are initialized, they can be controlled in sync using the same keyboard scheme displayed in the terminal.
