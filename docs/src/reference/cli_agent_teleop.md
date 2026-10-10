# `srb agent teleop` — Teleoperate Agent

Test environments and agent behavior by controlling robots with supported devices.

## Usage

```bash
srb agent teleop --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

| Option | Description | Default |
| ----------------------------- | ----------------------------------------- | ------------ |
| `--teleop_device` [DEV ...] | Interface (`keyboard`, `spacemouse`, `ros`, `gamepad`, `haptic`) | env default, else `[keyboard]` |
| `--pos_sensitivity` VAL | Translation sensitivity | `1.0` |
| `--rot_sensitivity` VAL | Rotation sensitivity (rad/s) | `π` (≈3.14) |
| `--invert_controls`/`--invert` | In manipulation mode, flip the first two translational controls. Boolean optional forms `--no-invert_controls` and `--no-invert` disable inversion. | `True` |
| **Teleoperation via Policy** | | |
| `--algo ALGO` | Algorithm of the policy (RL or IL) | |
| `--model MODEL` | Path to the policy checkpoint or an SRB model artifact directory; this exact policy is loaded | |
| `--obs` {state,visual} | Observation modality of the policy: selects the agent-config variant and the log tree searched for its latest checkpoint | `state` |

## Teleoperation Modes

Action space selects direct or policy control:

- [**Direct Teleoperation**](#direct-teleoperation)
- [**Teleoperation via Policy**](#teleoperation-via-policy)

### Direct Teleoperation

Use direct teleoperation when actions map to high-level commands such as
desired end-effector pose or mobile-robot acceleration.

#### Examples

In `_manipulation`, use `ur10`:

```bash
srb agent teleop -e _manipulation env.robot=ur10
```

In `_orbital`, use `spacemouse`:

```bash
srb agent teleop -e _orbital --teleop_device spacemouse
```

### Teleoperation via Policy

Use a trained RL or imitation-learning policy for complex or high-dimensional
actions, such as humanoid joint control.

Any algorithm supported by [`srb agent eval`](cli_agent_eval.md) works. The
device supplies command-like observations. The policy generates actions.

#### Examples

> Reference: [Reinforcement Learning Workflow](../workflows/reinforcement_learning.md)\
> Reference: [`srb agent train` — Train Agent](../reference/cli_agent_train.md)

1. Train Dreamer on `locomotion_velocity_tracking`:

   ```bash
   srb agent train --headless --algo dreamer -e locomotion_velocity_tracking env.num_envs=256
   ```

1. Teleoperate with it:

   ```bash
   srb agent teleop --algo dreamer -e locomotion_velocity_tracking
   ```

1. To load another checkpoint instead of the latest, set `--model`:

   ```bash
   srb agent teleop --algo dreamer -e locomotion_velocity_tracking --model logs/locomotion_velocity_tracking/dreamer/...
   ```

## SpaceMouse input

Failed or `None` reads discard buffered six-axis motion. No successful read for
0.5 s also discards it. Valid reads restore input, while lost reports during
continuing reads retain the last motion.

Reset discards in-flight reads. Failed reads preserve the gripper toggle. This
is not a hardware stop or inactivity timeout.

## Native teleoperation

Native `srb real_agent teleop` requires generated `COMMAND_MAPPING` metadata and
direct ROS, SpaceMouse, or haptic input. ROS is the default. Keyboard, gamepad,
and policy-command arguments are unavailable. See the
[native direct-input contract](cli_real_agent_gen.md#native-direct-input).

## Public Demo Launch

For public events, `excavation_demo` defaults to one environment, manual reset,
long horizon, all HUD lines visible, and the high-quality RTX preset.

```bash
srb agent teleop --env excavation_demo
```

Add `--hide_ui` for fullscreen without the Isaac Sim UI:

```bash
srb agent teleop --env excavation_demo --hide_ui
```

Task config defaults to `spacemouse`, `haptic`, and `keyboard`; thus
`--teleop_device` is optional. Set venue or visual tier with Hydra overrides
such as `env.scenery=lunalab` or `env.demo_regolith_quality=visual`.
