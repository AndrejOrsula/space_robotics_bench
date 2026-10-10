# Troubleshooting

## Runtime Errors

### Driver or CUDA Compiler Messages

**Symptom:** Either error appears during startup or compilation:

```log
[Error] [carb.graphics-vulkan.plugin] VkResult: ERROR_INCOMPATIBLE_DRIVER
```

```log
XlaRuntimeError: UNIMPLEMENTED: /usr/local/lib/python3.12/site-packages/jax_plugins/xla_cuda12/../../nvidia/cuda_nvcc/bin/ptxas ptxas too old. Falling back to the driver to compile.
```

**Cause:** `ERROR_INCOMPATIBLE_DRIVER` indicates Vulkan driver incompatibility.
`ptxas too old` means XLA's CUDA assembler is old and compilation falls back to
the driver; that warning alone does not indicate driver incompatibility.

**Fix:** Check the [Isaac Sim driver requirements](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)
and framework requirements. If compilation still fails, check CUDA/`ptxas`, then
driver and framework compatibility.

### GLXBadFBConfig Error

**Symptom:** An `srb agent` workflow crashes with this message:

```log
X Error of failed request:  GLXBadFBConfig
  Major opcode of failed request:  150 (GLX)
  Minor opcode of failed request:  0 ()
  Serial number of failed request:  133
  Current serial number in output stream:  133
There was an error running python
```

**Cause:** The reported OpenGL version may be incompatible.

**Fix:** On Mesa, `MESA_GL_VERSION_OVERRIDE` changes the OpenGL version reported
to applications. It may help if an application rejects that version, but it does not change the driver or guarantee a
fix:

```bash
MESA_GL_VERSION_OVERRIDE=4.6 srb agent teleop -e _ground
```

For simulated workflows in the provided Docker setup, select the simulator
profile and pass the variable into the container:

```bash
SRB_DOCKER_PROFILE=simulator ./.docker/run.bash -e MESA_GL_VERSION_OVERRIDE=4.6
```

### Slow Startup

**Symptom:** Isaac Sim's first launch may take several minutes, or later launches
remain slow.

**Cause:** The first launch compiles shaders and caches assets; later launches
are faster.

**Fix:** Let the first launch finish. For persistent slow startup, try a
workaround from [this issue](https://github.com/isaac-sim/IsaacSim/issues/506#issue-3995098852).

### Slow Startup with Real Terrain

**Symptom:** Startup is slow with real terrain selected and a cold cache.

**Cause:** SRB may fetch and bake terrain before launch.

**Fix:** Disable auto-fetch for interactive runs. SRB warns once and falls back
to the procedural surface without contacting remote providers:

```bash
SRB_TERRAIN_AUTO_FETCH=0 srb agent teleop -e _ground env.domain=mars env.scenery=mars_surface env.scenery.source=real
```

With the provided Docker development wrapper and a virtual display:

```bash
DISPLAY=:99 DOCKER_TTY=false .docker/dev.bash -e SRB_TERRAIN_AUTO_FETCH=0 srb agent teleop -e _ground env.domain=mars env.scenery=mars_surface env.scenery.source=real
```

The Docker development wrapper defaults to `LOG_LEVEL=info`, keeping expected
Isaac and SimForge debug logs from hiding actionable warnings. For one verbose
run, prefix the command with `LOG_LEVEL=debug`.

### Hidden Isaac Startup Logs

**Symptom:** Known Isaac Sim and PhysX startup logs are missing during an
`srb agent` workflow.

**Cause:** SRB filters known noisy startup logs.

**Fix:** Disable the filters to see raw vendor logs when diagnosing an upstream
Isaac or Kit startup issue:

```bash
SRB_DISABLE_ISAAC_LOG_FILTERS=1 srb agent teleop -e _ground
```

With the provided Docker development wrapper and a virtual display, pass the
same variable into the command environment:

```bash
DISPLAY=:99 DOCKER_TTY=false .docker/dev.bash -e SRB_DISABLE_ISAAC_LOG_FILTERS=1 srb agent teleop -e _ground
```

## Unexpected Behavior

### Teleoperation Stuck

**Symptom:** During keyboard teleoperation, the robot keeps moving in one
direction after you change window focus.

**Cause:** Omniverse may miss a button release when window focus changes.

**Fix:** Press `L` to reset the environment.

______________________________________________________________________

Need more help? Ask in the [Discord community](https://discord.gg/p9gZAPWa65)
or search [GitHub Issues](https://github.com/AndrejOrsula/space_robotics_bench/issues).

[![Discord](https://img.shields.io/badge/Discord-invite-5865F2?logo=discord)](https://discord.gg/p9gZAPWa65)
[![GitHub Issues](https://img.shields.io/badge/GitHub%20Issues-new-blue?logo=github)](https://github.com/AndrejOrsula/space_robotics_bench/issues/new)
