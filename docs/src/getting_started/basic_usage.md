# Basic Usage

After successful [installation](./install.md), you are now ready to explore the Space Robotics Bench. This guide covers the essentials for getting started.

<div class="warning">

**Native Installation** — If the `srb` command is not available, you can use this syntax:

```bash
"$ISAAC_SIM_PYTHON" -m srb
```

</div>

## 1. List Registered Assets & Environments

> Reference: [`srb ls` — List Assets and Environments](../instructions/cli_ls.md).

As a first step, it is recommended that you list all registered assets and tasks to get an overview of what SRB has to offer:

```bash
srb ls
```

After a while, you should see 3 tables printed in the terminal:

1. **Assets** - Simulation assets categorized under **sceneries**, **objects**, and **robots**
   - **Sceneries** - Terrains, space stations, ...
   - **Objects** - Interactive objects, tools, ...
   - **Robots** - Manipulators, mobile robots, ...
1. **Action groups** - Pre-configured action spaces for **robots** and **active tools**
1. **Environments** - Gymnasium environments for **templates** and **tasks**
   - **Templates** - Barebones environments
   - **Tasks** - Goal-oriented environments

#### Assets

```
┏━━━━━━━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┓
┃ Name         ┃ Type    ┃ Subtype            ┃ Parent Class      ┃ Asset Config    ┃
┡━━━━━━━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━┩
│ mars_surface │ scenery │ terrain            │ Terrain           │ AssetBaseCfg    │
│ scoop        │ object  │ tool               │ Tool              │ RigidObjectCfg  │
│ shadow_hand  │ object  │ tool               │ ActiveTool        │ ArticulationCfg │
│ sample_tube  │ object  │ common             │ Object            │ RigidObjectCfg  │
│ ur10         │ robot   │ manipulator        │ SerialManipulator │ ArticulationCfg │
│ cubesat      │ robot   │ mobile_robot       │ OrbitalRobot      │ RigidObjectCfg  │
│ ingenuity    │ robot   │ mobile_robot       │ Multicopter       │ ArticulationCfg │
│ perseverance │ robot   │ mobile_robot       │ WheeledRobot      │ ArticulationCfg │
│ unitree_g1   │ robot   │ mobile_manipulator │ Humanoid          │ ArticulationCfg │
│ ...          │ ...     │ ...                │ ...               │ ...             │
└──────────────┴─────────┴────────────────────┴───────────────────┴─────────────────┘
```

#### Environments

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ ID                           ┃ Entrypoint           ┃ Config                      ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ _aerial <template>           │ Task(AerialEnv)      │ TaskCfg(AerialEnvCfg)       │
│ _ground <template>           │ Task(GroundEnv)      │ TaskCfg(GroundEnvCfg)       │
│ _manipulation <template>     │ Task(ManipulatorEnv) │ TaskCfg(ManipulatorEnvCfg)  │
│ _orbital <template>          │ Task(OrbitalEnv)     │ TaskCfg(OrbitalEnvCfg)      │
│ locomotion_velocity_tracking │ Task(GroundEnv)      │ TaskCfg(GroundEnvCfg)       │
│ sample_collection            │ Task(ManipulatorEnv) │ TaskCfg(ManipulatorEnvCfg)  │
│ orbital_evasion              │ Task(OrbitalEnv)     │ TaskCfg(OrbitalEnvCfg)      │
│ ...                          │ ...                  │ ...                         │
└──────────────────────────────┴──────────────────────┴─────────────────────────────┘
```

## 2. Teleoperate your 1<sup>st</sup> Robot

> Reference: [`srb agent teleop` — Teleoperate Agent](../instructions/cli_agent_teleop.md).

Let's start with your first environment where you can manually control a robot through your keyboard. This can be accomplished through the `agent teleop` subcommand.

> **Note:** Most tasks employ action spaces that support direct teleoperation (e.g. via Inverse Kinematics). However, some tasks such as `locomotion_velocity_tracking` rely on low-level control of individual joints. In this case, direct teleoperation is not supported and you will need to provide a control policy that maps your teleoperation commands into low-level control signals. Further instructions are provided in the section for [Teleoperation via Policy](../instructions/cli_agent_teleop.md#teleoperation-via-policy).

### Sample Collection — Default (Moon)

To demonstrate direct teleoperation, we will use the `sample_collection` environment as it provides high-level of configurability:

```bash
srb agent teleop --env sample_collection
```

Eventually, Isaac Sim will open with the selected environment and you will be greeted in your terminal with the teleoperation interface:

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

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/d6KhKuB-XAs?si=DM0I2IFinw-rqgdD&mute=1&autoplay=1&loop=1&playlist=d6KhKuB-XAs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Sample Collection — Mars

Now, we learn how to configure some aspects of the environment using [Hydra](https://hydra.cc). For instance, we can adjust the **domain** and **sample asset** in order to simulate a scenario on Mars:

```bash
srb agent teleop --env sample_collection env.domain=mars env.sample=sample_tube
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/dlzwlct1BLA?si=d_oEZzmvS7SQviO1&mute=1&autoplay=1&loop=1&playlist=dlzwlct1BLA" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 3. Observe a Random Agent

> Reference: [`srb agent rand` — Random Agent](../instructions/cli_agent_rand.md).

Now, let's observe an environment where an agent acts based on random actions sampled from the action space. This is particularly useful for verifying if environments are running as intended without manual control.

### Locomotion — Default (Spot)

To demonstrate a random agent, we will use the `locomotion_velocity_tracking` environment that uses **Boston Dynamics' Spot** quadruped by default:

```bash
srb agent rand --env locomotion_velocity_tracking
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/3BTaKjtAbQs?si=UOlJxa6GlE05UcWQ4on&mute=1&autoplay=1&loop=1&playlist=3BTaKjtAbQs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Hint:** Use `--hide_ui` option to disable most of the Isaac Sim UI, as shown in the video above.

### Locomotion — Unitree G1

Selecting a different robot for any environment is as simple as changing the `env.robot` parameter. This particular environment supports all legged robots, including humanoids. For instance, let's try the **Unitree G1** humanoid:

```bash
srb agent rand --env locomotion_velocity_tracking env.robot=unitree_g1
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/viFvuz0Uq-g?si=SqoaHgz073j5V4on&mute=1&autoplay=1&loop=1&playlist=viFvuz0Uq-g" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Locomotion — Cassie | 16 Environment Instances

Many workflows benefit from running multiple parallel instances of the same environment. This can be achieved with the `env.num_envs` parameter. For instance, let's observe **16 instances** of the **Cassie** bipedal robot:

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
