# Benchmark Configuration

## Environment Configuration

The environments can be configured in two ways:

1. **Modifying the [`env.yaml`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/config/env.yaml) file**.
1. **Using environment variables**.

The default configuration file contains various settings that control the seed, scenario, level of detail, and options for assets (robot, object, terrain, vehicle).

```yaml
seed: 42 # SRB_SEED [int]
scenario: mars # SRB_SCENARIO [mars, moon, orbit]
detail: 0.5 # SRB_DETAIL [float]
assets:
  robot:
    variant: dataset # SRB_ASSETS_ROBOT_VARIANT [dataset]
  object:
    variant: procedural # SRB_ASSETS_OBJECT_VARIANT [primitive, dataset, procedural]
  terrain:
    variant: procedural # SRB_ASSETS_TERRAIN_VARIANT [none, primitive, dataset, procedural]
  vehicle:
    variant: dataset # SRB_ASSETS_VEHICLE_VARIANT [none, dataset]
```

### Setting Configuration via Environment Variables

Values from the configuration file can be overridden using environment variables. Furthermore, you can directly pass them into the [`.docker/run.bash`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash) script. For instance:

```bash
.docker/run.bash -e SRB_DETAIL=1.0 -e SRB_SCENARIO=moon ...
```

## CLI Arguments

The following arguments are common across all entrypoint scripts, e.g. [`teleop.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/teleop.py), [`random_agent.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/random_agent.py) and [`ros2.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/ros2.py):

- `-h`, `--help`: Display the help message and exit.
- `-t TASK`, `--task TASK`, `-e ENV`, `--env ENV`, `--demo DEMO`: Specify the name of the environment (task/demo). You can list available tasks using `list_envs.py`.
- `--num_envs NUM_ENVS`: Number of parallel environments to simulate.
- `--hide_ui`: Disable the majority of the Isaac Sim UI.
- `--headless`: Force the display to remain off, making the simulation headless.
- `--device DEVICE`: Set the device for simulation (e.g., `"cpu"`, `"cuda"`, or `"cuda:N"` where `N` is the device ID).

## Additional Environment Variables

- `SRB_WITH_TRACEBACK` (default: `false`): When set to `"true"`|`1`, rich traceback information is displayed for exceptions. This can be useful for debugging.
  - `SRB_WITH_TRACEBACK_LOCALS` (default: `false`): When set to `"true"`|`1` and `SRB_WITH_TRACEBACK` is enabled, local variables are included in the traceback information. This can be useful for debugging, but it can also be overwhelming in some cases.
