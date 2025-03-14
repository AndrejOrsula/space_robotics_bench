# Basic Usage

After successful [installation](./install.md), you are now ready to explore the Space Robotics Bench. This guide covers the essentials for getting started with the environments.

<div class="warning">

**Native Installation** — If the `srb` command is not available, you can use this syntax:

```bash
"$ISAAC_SIM_PYTHON" -m srb
```

</div>

## 1. List Registered Assets & Environments

> Reference: [`srb ls` — List Assets and Environments](../reference/cli_ls.md)

As a first step, it is recommended that you list all registered assets, action groups, and tasks to get an overview of what SRB has to offer:

```bash
srb ls
```

After a while, you should see 3 tables printed in the terminal:

<details>
  <summary><strong>1. Assets:</strong> Simulation assets categorized under <strong>sceneries</strong>, <strong>objects</strong>, and <strong>robots</strong> <i>(click to expand)</i></summary>

- **Sceneries** - Terrains, space stations, ...
- **Objects** - Interactive objects, tools, ...
- **Robots** - Manipulators, mobile robots, ...

```
┏━━━━━━━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┓
┃ Name         ┃ Type    ┃ Subtype            ┃ Parent Class      ┃ Asset Config    ┃
┡━━━━━━━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━┩
│ mars_surface │ scenery │ terrain            │ Terrain           │ AssetBaseCfg    │
│ sample_tube  │ object  │ common             │ Object            │ RigidObjectCfg  │
│ scoop        │ object  │ tool               │ Tool              │ RigidObjectCfg  │
│ shadow_hand  │ object  │ tool               │ ActiveTool        │ ArticulationCfg │
│ ur10         │ robot   │ manipulator        │ SerialManipulator │ ArticulationCfg │
│ cubesat      │ robot   │ mobile_robot       │ OrbitalRobot      │ RigidObjectCfg  │
│ ingenuity    │ robot   │ mobile_robot       │ Multicopter       │ ArticulationCfg │
│ perseverance │ robot   │ mobile_robot       │ WheeledRobot      │ ArticulationCfg │
│ unitree_g1   │ robot   │ mobile_manipulator │ Humanoid          │ ArticulationCfg │
│ ...          │ ...     │ ...                │ ...               │ ...             │
└──────────────┴─────────┴────────────────────┴───────────────────┴─────────────────┘
```

#### Scenery asset (automatically registered as "mars_surface" scenery/terrain)
```py
{{#include ../../../srb/assets/scenery/planetary_surface.py:example}}
```

#### Object asset (automatically registered as "sample_tube" object/common)
```py
{{#include ../../../srb/assets/object/sample.py:example}}
```

#### Robot asset (automatically registered as "franka" robot/manipulator)
```py
{{#include ../../../srb/assets/robot/manipulation/franka.py:example_p1}}
            ...
{{#include ../../../srb/assets/robot/manipulation/franka.py:example_p2}}
        ...
{{#include ../../../srb/assets/robot/manipulation/franka.py:example_p3}}
```
</details>

<details>
  <summary><strong>2. Action groups:</strong> Pre-configured action spaces for <strong>robots</strong> and <strong>active tools</strong></summary>

- **Actions for robots** - Each robot (mobile or manipulator) has an action group
- **Actions for active tools** - Each active tool (e.g. gripper) has an action group

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Name                            ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ body_velocity_action_group      │
│ joint_position_action_group     │
│ joint_velocity_action_group     │
│ joint_effort_action_group       │
│ inverse_kinematics_action_group │
│ ...                             │
└─────────────────────────────────┘
```

</details>

<details>
  <summary><strong>3. Environments:</strong> Gymnasium environments for <strong>templates</strong> and <strong>tasks</strong></summary>

- **Templates** - Barebones environments that can be used as a starting point
- **Tasks** - Goal-oriented environments that provide a specific scenario

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ ID                               ┃ Entrypoint                   ┃ Config                             ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ _manipulation <template>         │ Task(ManipulationEnv)        │ TaskCfg(ManipulationEnvCfg)        │
│ sample_collection                │ Task(ManipulationEnv)        │ TaskCfg(ManipulationEnvCfg)        │
│ _aerial <template>               │ Task(AerialEnv)              │ TaskCfg(AerialEnvCfg)              │
│ _ground <template>               │ Task(GroundEnv)              │ TaskCfg(GroundEnvCfg)              │
│ _orbital <template>              │ Task(OrbitalEnv)             │ TaskCfg(OrbitalEnvCfg)             │
│ locomotion_velocity_tracking     │ Task(GroundEnv)              │ TaskCfg(GroundEnvCfg)              │
│ _aerial_manipulation <template>  │ Task(AerialManipulationEnv)  │ TaskCfg(AerialManipulationEnvCfg)  │
│ _ground_manipulation <template>  │ Task(GroundManipulationEnv)  │ TaskCfg(GroundManipulationEnvCfg)  │
│ _orbital_manipulation <template> │ Task(OrbitalManipulationEnv) │ TaskCfg(OrbitalManipulationEnvCfg) │
│ ...                              │ ...                          │ ...                                │
└──────────────────────────────────┴──────────────────────────────┴────────────────────────────────────┘
```

</details>

## 2. Teleoperate your 1<sup>st</sup> Robot across Diverse Domains

> Reference: [`srb agent teleop` — Teleoperate Agent](../reference/cli_agent_teleop.md)

Let's start with the `sample_collection` environment where you can manually control the **Franka** manipulator through your keyboard to collect samples on the Moon:

```bash
srb agent teleop --env sample_collection
```

Eventually, Isaac Sim will open with the selected environment and you will be greeted in your terminal with a schematic of the teleoperation interface.

<details>
  <summary><strong>Teleoperation Interface</strong> (click to expand)</summary>

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃  Keyboard Scheme (focus the Isaac Sim window)  ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ Reset: [ L ]                                   │
│ Decrease Gain [ O ]   │ Increase Gain: [ P ]   │
│ Event: [ R / K ]                               │
├────────────────────────────────────────────────┤
│ Translation                                    │
│             [ W ] (+X)            [ Q ] (+Z)   │
│               ↑                     ↑          │
│               │                     │          │
│  (-Y) [ A ] ←─┼─→ [ D ] (+Y)        ┼          │
│               │                     │          │
│               ↓                     ↓          │
│             [ S ] (-X)            [ E ] (-Z)   │
├────────────────────────────────────────────────┤
│ Rotation                                       │
│       [ Z ] ←————————(±X)————————→ [ X ]       │
│                                                │
│       [ T ] ↻————————(±Y)————————↺ [ G ]       │
│                                                │
│       [ C ] ↺————————(±Z)————————↻ [ V ]       │
└────────────────────────────────────────────────┘
```
</details>

<br>

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/d6KhKuB-XAs?si=DM0I2IFinw-rqgdD&mute=1&autoplay=1&loop=1&playlist=d6KhKuB-XAs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Note:** Most tasks employ action spaces that support direct teleoperation (e.g. via Inverse Kinematics). However, some tasks such as `locomotion_velocity_tracking` rely on low-level control of individual joints. In this case, direct teleoperation is not supported and you will need to provide a control policy that maps your teleoperation commands into low-level control signals. Further instructions are provided in the section for [Teleoperation via Policy](../reference/cli_agent_teleop.md#teleoperation-via-policy).

### Move to a Different Domain

> Reference: [Environment Configuration](../config/env_cfg.md)\
> Reference: [Environment Configuration — Domain](../config/domain.md)

What if we want to collect samples on Mars instead? Luckily, you can easily configure many aspects of the environment through [Hydra](https://hydra.cc). For instance, you can adjust the **domain** and **sample** asset in order to simulate the Mars Sample Return mission:

```bash
srb agent teleop --env sample_collection env.domain=mars env.sample=sample_tube
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/dlzwlct1BLA?si=d_oEZzmvS7SQviO1&mute=1&autoplay=1&loop=1&playlist=dlzwlct1BLA" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 3. Observe Random Agents in Action

> Reference: [`srb agent rand` — Random Agent](../reference/cli_agent_rand.md)

Now, let's observe an environment where agents act based on random actions sampled uniformly from their action space, which is particularly useful for verifying that environments function as intended. To demonstrate a random agent, we will use the `locomotion_velocity_tracking` task that uses the **Spot** quadruped by default:

```bash
srb agent rand --env locomotion_velocity_tracking
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/3BTaKjtAbQs?si=UOlJxa6GlE05UcWQ4on&mute=1&autoplay=1&loop=1&playlist=3BTaKjtAbQs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Hint:** Use `--hide_ui` option to disable most of the Isaac Sim UI, as shown in the video above.

### Change the Robot

> Reference: [Environment Configuration — Robot](../config/robot.md)

Selecting a different robot for any environment is as simple as adjusting the `env.robot` parameter. This particular environment supports all legged robots, so let's try the **Unitree G1** humanoid:

```bash
srb agent rand --env locomotion_velocity_tracking env.robot=unitree_g1
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/viFvuz0Uq-g?si=SqoaHgz073j5V4on&mute=1&autoplay=1&loop=1&playlist=viFvuz0Uq-g" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Simulate Multiple Robots

> Reference: [Environment Configuration — Parallel Simulation](../config/parallel_sim.md)

Many workflows benefit from running multiple parallel simulation instances. This can be achieved with the `env.num_envs` parameter. For instance, let's try **16 instances** of the Cassie biped:

```bash
srb agent rand -e locomotion_velocity_tracking env.robot=cassie env.num_envs=16
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/eZt123VywGQ?si=7toO7FbDO3OxMbq1&mute=1&autoplay=1&loop=1&playlist=eZt123VywGQ" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Hint:** As you can see in the video above, all 16 robots share the same terrain. This is the default behaviour of `locomotion_velocity_tracking` (the default behaviour is task-specific). However, you can easily create a unique terrain for each robot by setting `env.stack=false`. This will automatically trigger the generation of 16 unique assets with different geometry and materials. Here, we speed up the procedural generation by disabling texture baking with `SF_BAKER=0`:
>
> ```bash
> SF_BAKER=0 srb agent rand -e locomotion_velocity_tracking env.stack=false env.num_envs=16
> ```
>
> ![Image](https://github.com/user-attachments/assets/2711d026-e0b5-4839-af2b-ff3a0423b683)

## 4. Explore & Experiment with Environment Templates

> Reference: [Robots](../robots/index.md)\
> Reference: [Contributing — New Tasks](../contributing/new_tasks.md)

Both `sample_collection` and `locomotion_velocity_tracking` are examples of tasks that implement specific goal-oriented scenarios. However, SRB also provides a set of environment templates that can serve as a foundation for exploring and experimenting with custom scenarios.

In general, each robot category has its own template:

| Template                | Description                                  |
| ----------------------- | -------------------------------------------- |
| `_manipulation`         | Fixed-base manipulation with robotic arms    |
| `_ground`               | Ground traversal on planetary surfaces       |
| `_aerial`               | Aerial navigation above planetary surfaces   |
| `_orbital`              | Spaceflight maneuvers                        |
| `_ground_manipulation`  | Mobile manipulation with ground-based robots |
| `_aerial_manipulation`  | Mobile manipulation with flying robots       |
| `_orbital_manipulation` | Mobile manipulation with spacecraft          |

With this in mind, let's explore the `_ground_manipulation` template that combines the mobile **Spot** quadruped with **Franka** manipulator into an integrated mobile manipulation system:

```bash
srb agent rand -e _ground_manipulation
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/iFRMExXJEfI?si=sCPI-UuyVgJF7ucL&mute=1&autoplay=1&loop=1&playlist=iFRMExXJEfI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Diversify Robot Configurations

Robot configurations can also be randomized across multiple parallel instances. For example, you can combine a random **Unitree** quadruped (`random_unitree_quadruped`) with a random **Universal Robots** manipulator (`random_ur_manipulator`),  ranging all the way from **UR3** to **UR30**! For mobile manipulators, changing the mobile base and manipulator is separated into two parameters for more flexibility, namely `env.robot.mobile_base` and `env.robot.manipulator`:

```bash
srb agent rand -e _ground_manipulation env.robot.mobile_base=random_unitree_quadruped env.robot.manipulator=random_ur_manipulator env.num_envs=6 env.stack=true
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/ac5p5kPCHyE?si=pujsR8vHy9pn-ZRG&mute=1&autoplay=1&loop=1&playlist=ac5p5kPCHyE" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Customize Payloads & End Effectors

Modifying only the robot might not be enough for your envisioned scenario. You might also want to customize the **payload of mobile robots** or the **end effector of manipulators**. Similar to previous examples, this can also be configured via `env.robot` for mobile robots and manipulators; or `env.robot.mobile_base` and `env.robot.manipulator` for mobile manipulators. The configuration is context-aware and you can specify payloads and end effectors by separating them with a `+` sign, i.e. `mobile_base+payload` or `manipulator+end_effector`. For example, let's combine the **Anymal D** quadruped with the **Cargo Bay** payload and the **Unitree Z1** manipulator with the **Shadow Hand** end effector:

```bash
srb agent rand -e _ground_manipulation env.robot.mobile_base=anymal_d+cargo_bay env.robot.manipulator=unitree_z1+shadow_hand
```

> **Hint:** If you wish to only change the payload or end effector but keep the rest of the robot configuration the same, 



## What's Next?

Everything you learned so far is just the tip of the iceberg, but it is applicable to all environments and workflows within the Space Robotics Bench. Diving deeper into the codebase will allow you to customize and extend the environments to suit your specific needs.


The next step is to dive deeper into the [reference section](../reference/index.md) to explore more advanced features and functionalities. If you are interested in contributing to the project, you can also check out the [contributing section](../contributing/index.md) to learn how to create new tasks, assets, and robots.