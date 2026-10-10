[Introduction](README.md)

______________________________________________________________________

# Getting Started

- [System Requirements](getting_started/requirements.md)
- [Installation](getting_started/install.md)
  - [Native](getting_started/install_native.md)
  - [Docker (Recommended)](getting_started/install_docker.md)
  - [Apptainer/Singularity](getting_started/install_apptainer.md)
- [Basic Usage](getting_started/basic_usage.md)
- [Workflows](workflows/index.md)
  - [ROS 2](workflows/ros2.md)
  - [Reinforcement Learning](workflows/reinforcement_learning.md)
  - [Hyperparameter Tuning](workflows/hyperparameter_tuning.md)
  - [Imitation Learning](workflows/imitation_learning.md)
  - [Sim-to-Real Transfer](workflows/sim_to_real.md)
  - [Terrain Landscaping Training](workflows/terrain_landscaping_training.md)

______________________________________________________________________

# Overview

- [Environments](envs/index.md)
  - [Manipulation](envs/manipulation.md)
  - [Mobile Robotics](envs/mobile_robotics.md)
  - [Mobile Manipulation](envs/mobile_manipulation.md)
  - [Digital Twins](envs/digital_twin.md)
- [Assets](assets/index.md)
  - [Manipulators](assets/robots/manipulators.md)
  - [Tools](assets/robots/tools.md)
  - [Mobile Robots](assets/robots/mobile_robots.md)
  - [Mobile Manipulators](assets/robots/mobile_manipulators.md)
  - [Objects](assets/objects/objects.md)
  - [Scenery](assets/scenery/scenery.md)
- [Integrations & Interfaces](integrations/index.md)
  - [Reinforcement Learning](integrations/reinforcement_learning.md)
  - [Imitation Learning](integrations/imitation_learning.md)
  - [Extended Reality](integrations/extended_reality.md)
  - [ROS 2](integrations/ros2.md)

______________________________________________________________________

# Configuration

- [Environment Configuration](config/env_cfg.md)
  - [Parallelism](config/parallelism.md)
  - [Domain](config/domain.md)
  - [Robot](config/robot.md)
  - [Particles](config/particles.md)
  - [Reward & Normalization](config/reward_cfg.md)
  - [Randomization, Noise & Delay Profiles](config/randomization_profiles.md)
  - [Ephemeris](config/ephemeris.md)
  - [Terrain Stacks](config/terrain_stacks.md)
- [Agent Configuration](config/agent_cfg.md)

______________________________________________________________________

# Sensors

- [Event Camera (IMX636)](sensors/event_camera.md)
  - [Weight Mirrors](sensors/event_camera_weight_mirrors.md)
  - [Fixture Mirrors](sensors/event_camera_fixture_mirrors.md)

______________________________________________________________________

# Reference

- [Command Line Interface](reference/cli.md)
  - [List Assets and Tasks](reference/cli_ls.md)
  - [Zero Agent](reference/cli_agent_zero.md)
  - [Random Agent](reference/cli_agent_rand.md)
  - [Teleoperate Agent](reference/cli_agent_teleop.md)
  - [ROS 2 Agent](reference/cli_agent_ros.md)
  - [Train Agent](reference/cli_agent_train.md)
  - [Evaluate Agent](reference/cli_agent_eval.md)
  - [Collect Demonstrations](reference/cli_agent_collect.md)
  - [Learn from Demonstrations](reference/cli_agent_learn.md)
  - [Tune Hyperparameters](reference/cli_agent_tune.md)
  - [Sim-to-Real Bridge](reference/cli_real_agent_gen.md)
  - [Manage Assets](reference/cli_asset.md)
  - [Dataset Artifacts](reference/cli_dataset.md)
  - [Model Artifacts](reference/cli_model.md)
  - [Isaac Sim Utilities & REPL](reference/cli_repl.md)
  - [Ephemeris](reference/cli_ephemeris.md)
- [Graphical User Interface](reference/gui.md)
- [Compatibility Matrix](reference/compatibility.md)
- [Action Space Catalog](reference/action_spaces.md)
- [Sensor Catalog](reference/sensors.md)
- [Illumination Products](reference/ephemeris_products.md)
- [Stats](reference/stats.md)

______________________________________________________________________

# Development

- [IDE Configuration](development/ide_cfg.md)
- [Dev Container](development/devcontainer.md)
- [Testing](development/testing.md)
- [Documentation](development/documentation.md)
- [Utilities](development/utilities.md)
  - [Update Assets](development/update_assets.md)
  - [Clean Cache](development/clean_cache.md)
- [Real Terrain Assets](development/real_terrain_assets.md)
- [Units & Conventions](development/units_conventions.md)

______________________________________________________________________

# Contributing

- [New Assets](contributing/new_assets.md)
- [New Tasks](contributing/new_tasks.md)
- [Adding Terrain Sites](contributing/adding_terrain_sites.md)

______________________________________________________________________

# Resources

- [Modelling Envelope](misc/modelling_envelope.md)
- [Attributions](misc/attributions.md)
- [Contributors](misc/contributors.md)
- [Citation](misc/citation.md)
- [Community](misc/community.md)
- [Troubleshooting](misc/troubleshooting.md)
