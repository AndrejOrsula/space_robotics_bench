# Contributing — New Tasks

Copy a task template, adapt its environment, and test it.

## 1. Duplicate a Template

Copy a package under [`srb/tasks`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/srb/tasks). Update its registration and config, give it a unique package name, and keep it under `srb.tasks`: recursive imports register tasks, and family placement sets their documentation category. Choose a template:

| Template | Description |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| [`_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/manipulation/_manipulation/task.py) | Fixed-base manipulation with robotic arms |
| [`_ground`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile/_ground/task.py) | Ground traversal on planetary surfaces |
| [`_aerial`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile/_aerial/task.py) | Aerial navigation above planetary surfaces |
| [`_orbital`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile/_orbital/task.py) | Spaceflight maneuvers |
| [`_ground_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile_manipulation/_ground_manipulation/task.py) | Mobile manipulation with ground-based robots |
| [`_aerial_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile_manipulation/_aerial_manipulation/task.py) | Mobile manipulation with flying robots |
| [`_orbital_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile_manipulation/_orbital_manipulation/task.py) | Mobile manipulation with spacecraft |

## 2. Modify the Environment

Adapt the scene and assets, choose a robot category, then implement observations, rewards, and termination conditions. Ask the community for task-specific guidance if needed.

## 3. Debug the Task

Test the task with [`teleop`](../reference/cli_agent_teleop.md), [`zero`](../reference/cli_agent_zero.md), or [`rand`](../reference/cli_agent_rand.md), as its action space permits. Enable debug visualization and check parallel instances. First, update `docs/data/tasks.toml`. Then regenerate the offline task cache (`srb cache update`) and task docs (`srb doc docgen`) to sync the generated catalog and task pages. Run a compatible command:

```bash
srb agent teleop -e <your_task_name> env.debug_vis=true env.num_envs=4
```

```bash
srb agent zero -e <your_task_name> env.debug_vis=true env.num_envs=4
```

```bash
srb agent rand -e <your_task_name> env.debug_vis=true env.num_envs=4
```

## 4. Use the Task in a Workflow

> Reference: [Workflows](../workflows/index.md)

Use workflows compatible with the task's config, action space, and runtime interfaces. Check compatibility before training, deployment, or validation.

Share your task with the community.

Next: [Adding Terrain Sites](adding_terrain_sites.md)
