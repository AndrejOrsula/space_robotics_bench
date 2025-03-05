# Troubleshooting

## Runtime Errors

### Driver Incompatibility

If you encounter the following error message:

```log
[Error] [carb.graphics-vulkan.plugin] VkResult: ERROR_INCOMPATIBLE_DRIVER
```

This indicates that your NVIDIA driver is incompatible with Omniverse. To resolve the issue, update your NVIDIA driver according to the [Isaac Sim driver requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#isaac-sim-short-driver-requirements).

## Unexpected Behavior

### Teleoperation Stuck

During teleoperation, if you change your window focus, Omniverse may fail to register a button release, causing the robot to continuously move in one direction. To fix this, press the `L` key to reset the environment.
