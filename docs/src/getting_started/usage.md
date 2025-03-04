# Basic Usage

After successful [installation](./installation/index.html), you are ready to use the Space Robotics Bench. This page will guide you through controlling robots in various scenarios using a simple teleoperation.

<div class="warning">
When using the Docker setup, it is strongly recommended that you always use the provided <a href="https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/run.bash"><code>.docker/run.bash</code></a> script. It configures the environment automatically and mounts caching volumes. You can optionally provide a command that will be executed immediately inside the container. If no command is specified, you will be dropped into an interactive shell. Throughout this documentation, if you omit the <code>.docker/run.bash</code> prefix, it assumes that you are already inside the Docker container, or you are using a local installation.

```bash
# cd space_robotics_bench
.docker/run.bash ${OPTIONAL_CMD}
```

</div>

## Verify the Functionality of Isaac Sim

Let's start by verifying that Isaac Sim is functioning correctly:

<div class="warning">
The first time Isaac Sim starts, it may take a few minutes to compile shaders. However, subsequent runs will use cached artefacts, which significantly speed up the startup.
</div>

```bash
# Single quotes are required for the tilde (~) to expand correctly inside the container.
.docker/run.bash '~/isaac-sim/isaac-sim.sh'
```

If any issues arise, consult the [Troubleshooting](../misc/troubleshooting.md#runtime-errors) section or the [official Isaac Sim documentation](https://docs.omniverse.nvidia.com/isaacsim), as this issue is likely unrelated to this project.

## Journey into the Unknown

Once Isaac Sim is confirmed to be working, you can begin exploring the demos and tasks included with the environments. Let's start with a simple teleoperation example with the [`teleop.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/teleop.py) script:

```bash
# Option 1: Using the script directly
.docker/run.bash scripts/teleop.py --env perseverance
# Option 2: Using ROS 2 package installation
.docker/run.bash ros2 run srb teleop.py --env perseverance
```

After a few moments, Isaac Sim should appear. The window will briefly remain inactive as the assets are procedurally generated in the background. The generation time depends on the complexity of the assets and your hardware, particularly the GPU, which will be used to bake PBR textures. However, future runs will use cached assets, as long as the configuration remains unchanged and the cache is not cleared, see [Clean the Assets Cache](../instructions/utils/clean_cache.md).

Eventually, you will be greeted by the Mars Perseverance Rover on a procedurally generated Martian landscape.

![](../_images/perseverance_ui.jpg)

At the same time, the terminal will display the following keyboard scheme:

```
+------------------------------------------------+
|  Keyboard Scheme (focus the Isaac Sim window)  |
+------------------------------------------------+
+------------------------------------------------+
| Reset: [ L ]                                   |
+------------------------------------------------+
| Planar Motion                                  |
|                     [ W ] (+X)                 |
|                       ↑                        |
|                       |                        |
|          (-Y) [ A ] ← + → [ D ] (+Y)           |
|                       |                        |
|                       ↓                        |
|                     [ S ] (-X)                 |
+------------------------------------------------+
```

While the Isaac Sim window is in focus, you can control the rover using the `W`, `A`, `S`, and `D` keys for motion. Use your mouse to navigate the camera. If the rover gets stuck, pressing `L` will reset its position.

To close the demo, press `Ctrl+C` in the terminal. This will gracefully shut down the demo, close Isaac Sim, and return you to your host environment.

### Blurry Textures?

By default, the textures in the environment might appear blurry due to the configuration setting the baked texture resolution to 50.0% (`default=0.5`). This setting allows procedural generation to be faster on low-end hardware. If your hardware is capable, you can increase the resolution by adjusting the `detail` parameter, see [Benchmark Configuration](../instructions/benchmark/cfg.md):

```bash
.docker/run.bash -e SRB_DETAIL=1.0 scripts/teleop.py --env perseverance
```

## Explore Unknown Domains

You can explore other environments by using the `--env`, `--task`, or `--demo` arguments interchangeably. A full list of available environments is documented in the [Environment Overview](../overview/envs/index.html), or you can conveniently list them using this command:

```bash
.docker/run.bash scripts/list_envs.py
```

Use this example as a **gateway** into exploring further on your own:

```bash
.docker/run.bash scripts/teleop.py --env perseverance
```
