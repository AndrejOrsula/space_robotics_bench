# Troubleshooting

## Runtime Errors

### Driver Incompatibility

If you encounter the following error message:

```log
[Error] [carb.graphics-vulkan.plugin] VkResult: ERROR_INCOMPATIBLE_DRIVER
```

This indicates that your NVIDIA driver is incompatible with Omniverse. To resolve the issue, update your NVIDIA driver according to the [Isaac Sim driver requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#isaac-sim-short-driver-requirements).

### GLXBadFBConfig Error

If your `srb agent` simulated workflow crashes and you encounter the following error message, this might indicate that your system's OpenGL version is not compatible with the requirements.

```log
X Error of failed request:  GLXBadFBConfig
  Major opcode of failed request:  150 (GLX)
  Minor opcode of failed request:  0 ()
  Serial number of failed request:  133
  Current serial number in output stream:  133
There was an error running python
```

To resolve this, you can set the `MESA_GL_VERSION_OVERRIDE` environment variable when running the `srb agent` command as shown below:

```bash
MESA_GL_VERSION_OVERRIDE=4.6 srb agent ...
```

Alternatively for users of the provided Docker setup, you can set the environment variable for the entire Docker container by running:

```bash
./space_robotics_bench/.docker/run.bash -e MESA_GL_VERSION_OVERRIDE=4.6
```

## Unexpected Behavior

### Teleoperation Stuck

During teleoperation with the keyboard, if you change your window focus, Omniverse may fail to register a button release, causing the robot to move continuously in one direction. To fix this, press the `L` key to reset the environment.

______________________________________________________________________

Haven't found a solution to your problem? You can search for help or ask any questions by joining our [Discord community](https://discord.gg/p9gZAPWa65) or by seeking assistance through [GitHub Issues](https://github.com/AndrejOrsula/space_robotics_bench/issues).

[![Discord](https://img.shields.io/badge/Discord-invite-5865F2?logo=discord)](https://discord.gg/p9gZAPWa65)
[![GitHub Issues](https://img.shields.io/badge/GitHub%20Issues-new-blue?logo=github)](https://github.com/AndrejOrsula/space_robotics_bench/issues/new)
