# Environment Configuration

The Space Robotics Bench provides a flexible configuration system for environments through [Hydra](https://hydra.cc). This document explains how to customize environment parameters across different domains and tasks.

## How Does It Work?

Each SRB environment is registered alongside its Python configuration class, which defines the default parameters for that specific environment. All environment configuration classes are organized in a hierarchical structure via inheritance, where [`BaseEnvCfg`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/core/env/common/base/env_cfg.py) does most of the heavy lifting. This design supports a modular and extensible configuration system that allows for easy customization of environment parameters.

## Modifying Configurations

You can modify environment configurations in several ways:

### 1. Command-Line Overrides

The most direct way to modify environment parameters is through command-line overrides:

```bash
srb agent <WORKFLOW> --env <ENV> \
    env.domain=moon \
    env.robot=ur10 \
    env.num_envs=4 \
    env.stack=false \
    ...
```

### 2. Configuration Files

<!-- TODO[docs]: Add instructions for environment configuration files -->

<div class="warning">

#### This section is under construction. Please check back later for updates.

</div>

## Key Parameters

Below are the most important configuration parameters organized by category (several parameters have their own detailed documentation pages):

### Scenario/Environment

- `env.domain` - [Domain](domain.md)
- `env.robot` - [Robot](robot.md)
- `env.num_envs`, `env.stack` - [Parallelism](parallelism.md)
- `env.particles` - [Particles](particles.md)

### Simulation

- `env.sim` - Low-level simulation parameters (physics, rendering, etc.)
- `env.visuals` - Visual appearance settings

### Debugging

- `env.debug_vis` - Enables debug visualization features
