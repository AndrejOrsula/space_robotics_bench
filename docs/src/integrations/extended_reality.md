# Integration — Extended Reality (XR)

Add `--xr` to a simulation-based agent command for XR teleoperation,
demonstration collection, or scene inspection through Isaac Sim's OpenXR
pipeline. Target versions: Isaac Sim 6.1.0 and Isaac Lab 3.0.0. SRB XR is
untested with headsets.

XR supports immersive teleoperation and demonstration collection, plus inspection
of scale, lighting, and hardware placement in Martian, Lunar, and orbital scenes.
Trajectory quality and Imitation Learning performance depend on task and device.

## Hardware & Software Requirements

XR uses Isaac Sim's OpenXR runtime. Supported Isaac Sim headsets may work with
SRB: Meta Quest (Link/Air Link), Valve Index, HTC Vive, and Windows Mixed
Reality. See the [Isaac Sim 6.1 documentation](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/index.html)
for the support matrix and runtime setup.

## Enabling XR

Use these flags:

- `--xr` — enables the VR/AR XR session.
- `--rendering_mode xr` — accepted for XR launches; applies no RTX preset because the XR Kit experience selected by `--xr` carries XR rendering settings. `performance`, `balanced`, and `quality` select SRB RTX presets.

These flags work with simulation-based `agent` subcommands: `zero`, `rand`, `teleop`, `ros`,
`train`, `eval`, and `collect`. Flags affect rendering and headset display;
actions remain subcommand-specific (`teleop` uses `--teleop_device`; `zero`,
`rand`, `eval`, and `ros` use their respective sources).

### Visualization of Trained Policies in XR

Inspect a trained policy in-scene, for example when behavior seems odd:

```bash
srb agent eval --env peg_in_hole --algo sbx_ppo --xr --rendering_mode xr
```

### Immersive Teleoperation

Pair XR with a teleop device. The headset provides depth perception and head
motion; the device supplies actions.

```bash
srb agent teleop --env peg_in_hole --xr --rendering_mode xr \
  --teleop_device spacemouse
```

### Demonstration Collection in XR

During collection, observations come from the environment-configured sensors; XR changes the display, not the observations.

```bash
srb agent collect --env peg_in_hole --xr --rendering_mode xr \
  --teleop_device spacemouse --num_episodes 50
```

## Tips

- Prefer `env.num_envs=1`; parallel environments increase rendering cost and may lower comfort-critical frame rates.
- For stereo-induced motion sickness, lower `env.num_envs` or set `--rendering_mode performance` and use XR for inspection.
- When collecting, cap long XR episodes with `srb agent collect --horizon` to limit dataset length.
- XR controllers are unsupported as teleoperation input.

## See Also

- [`srb agent teleop`](../reference/cli_agent_teleop.md) — core teleoperation command
- [`srb agent collect`](../reference/cli_agent_collect.md) — record demonstrations
- [Imitation Learning Workflow](../workflows/imitation_learning.md)

Next: [ROS 2](ros2.md)
