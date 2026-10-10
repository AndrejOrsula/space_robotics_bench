# Environment Configuration — Robot

> **Tip:** List registered robots and categories with `srb ls --live robot`; list cached names with `srb ls robot`. Use supported `_` categories in `-e`.

## Robot Selection

```bash
srb agent rand -e _manipulation env.robot=ur10
```

```bash
srb agent rand -e _ground env.robot=unitree_go1
```

```bash
srb agent rand -e _aerial env.robot=crazyflie
```

Select a robot with `env.robot=<name>`. This replaces task values and resets
the robot's owned fields, end effector, and payload. Kinova on Franka gets its
gripper; Husky gets its default payload.

- Attach a task tool with `env.robot=ur10+<tool>`; selecting a robot drops task
  tools.
- Set `env.robot.end_effector=...` or `env.robot.payload=...` in any CLI order.
  Set a single field with an override such as
  `env.robot.end_effector.<field>=...`.
- Set or clear asset fields explicitly when changing selectors. `srb agent eval`
  and `--continue` replay embodiment by reapplying `.hydra/overrides.yaml`, but
  values supplied in `--cfg` YAML can remain through a selector change.
- Selecting a named top-level asset, such as `env.sample=moon_rock`, resets it
  too.
  Selecting `env.robot.mobile_base=...` or `.manipulator=...` preserves that
  component's sub-assets.

### End Effector for Manipulators

Select an end effector with `base+end_effector`:

```bash
srb agent rand -e _manipulation env.robot=ur10+shadow_hand
```

### Payload for Mobile Robots

Select a payload with `base+payload`:

```bash
srb agent rand -e _ground env.robot=spot+cargo_bay
```

Payloads can be static rigid geometry or `ActivePayload` articulations. Active
payloads mount as separate scene articulations and add actions after the base.
For landscaping, the action order is two filtered RaphRover drive channels,
then one continuous RaphShovel velocity channel.

```bash
srb agent rand -e terrain_landscaping env.robot=raph_rover+raph_shovel
```

Landscaping maps normalized `[-1, 1]` actions to `robot/cmd_vel` slice `[0:2]`
(linear scale and saturation `0.4 m/s`; angular scale and saturation
`1.0 rad/s`) and `payload/joint_vel` slice `[2:3]` (scale and saturation
`0.04 m/s`). These are **simulator** limits, not hardware-calibrated. See
[Terrain Landscaping Training](../workflows/terrain_landscaping_training.md).

> **Asset-instance CLI customizations.** Each `srb agent` or `srb real_agent`
> invocation rebuilds task config through Hydra.
>
> - `extract_defaults_from_class` serializes assets as `{"name": ...}`, with
>   `action_mode`, nested assets, and all serializable plain fields (numbers,
>   strings, booleans, or lists, tuples, and dicts thereof), including class
>   defaults. Serializable config fields pass through; nonserializable
>   manager-term configs are skipped.
> - `AssetResolver` calls asset constructors with declared values for
>   `model_post_init` and the spawner. It also sends overrides such as
>   `env.scenery.flat_area_size=3.0` to simulation.

## Combine Mobile Manipulators

In [mobile-manipulation environments](../envs/mobile_manipulation.md), set
`env.robot.mobile_base` and `env.robot.manipulator`:

```bash
srb agent rand -e _ground_manipulation env.robot.mobile_base=unitree_a1 env.robot.manipulator=unitree_z1
```

```bash
srb agent rand -e _ground_manipulation env.robot.mobile_base=anymal_d+cargo_bay env.robot.manipulator=ur10+shadow_hand
```

Compose a mobile manipulator in one `env.robot` token. Use a registered mobile
robot, manipulator, and optional end effector. Tasks whose default robot is not
a composite mobile manipulator accept only ground mobile bases here:

```bash
srb agent rand -e _ground_manipulation env.robot=unitree_a1+unitree_z1
srb agent rand -e _ground_manipulation env.robot=husky+ur10+robotiq_hand_e
```

## Change Action Modes

Set `action_mode` with a Hydra override on the robot, mobile base, manipulator,
or tool. Each robot, active payload, and active tool adds its `action_mode` to
the action space. This works for registered tasks whose composed robot exposes
the field:

```bash
srb agent rand -e _manipulation env.robot.action_mode=joint_pos_rel
```

```bash
srb agent rand -e _manipulation env.robot=ur10 env.robot.action_mode=joint_pos_rel
```
