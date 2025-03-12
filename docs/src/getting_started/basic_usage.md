# Basic Usage

After successful [installation](./install.md), you are now ready to explore the Space Robotics Bench. This guide covers the essentials for getting started.

<div class="warning">

**Native Installation** — If the `srb` command is not available, you can use this syntax:

```bash
"$ISAAC_SIM_PYTHON" -m srb
```

</div>

## 1. List Registered Assets & Environments

> Reference: [`srb ls` — List Assets and Environments](../instructions/cli_ls.md)

As a first step, it is recommended that you list all registered assets and tasks to get an overview of what SRB has to offer:

```bash
srb ls
```

After a while, you should see 3 tables printed in the terminal (output is truncated here for clarity):

1. **Assets** - Simulation assets categorized under **sceneries**, **objects**, and **robots**
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

2. **Action groups** - Pre-configured action spaces for **robots** and **active tools**

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

3. **Environments** - Gymnasium environments for **templates** and **tasks**
   - **Templates** - Barebones environments
   - **Tasks** - Goal-oriented environments

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

## 2. Teleoperate your 1<sup>st</sup> Robot across Diverse Domains

> Reference: [`srb agent teleop` — Teleoperate Agent](../instructions/cli_agent_teleop.md)

Let's start with your first environment where you can manually control a robot through your keyboard. This can be accomplished through the `agent teleop` subcommand.

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

> Reference: [Environment Configuration](../config/env_cfg.md)\
> Reference: [Environment Configuration — Domain](../config/domain.md)

Now, we learn how to configure some aspects of the environment using [Hydra](https://hydra.cc). For instance, we can adjust the **domain** and **sample asset** in order to simulate a scenario on Mars:

```bash
srb agent teleop --env sample_collection env.domain=mars env.sample=sample_tube
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/dlzwlct1BLA?si=d_oEZzmvS7SQviO1&mute=1&autoplay=1&loop=1&playlist=dlzwlct1BLA" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Note:** Most tasks employ action spaces that support direct teleoperation (e.g. via Inverse Kinematics). However, some tasks such as `locomotion_velocity_tracking` rely on low-level control of individual joints. In this case, direct teleoperation is not supported and you will need to provide a control policy that maps your teleoperation commands into low-level control signals. Further instructions are provided in the section for [Teleoperation via Policy](../instructions/cli_agent_teleop.md#teleoperation-via-policy).

## 3. Observe Random Agents with Different Robots

> Reference: [`srb agent rand` — Random Agent](../instructions/cli_agent_rand.md)

Now, let's observe an environment where an agent acts based on random actions sampled from the action space. This is particularly useful for verifying if environments are running as intended without manual control.

### Locomotion — Default (Spot)

To demonstrate a random agent, we will use the `locomotion_velocity_tracking` environment that uses **Boston Dynamics' Spot** quadruped by default:

```bash
srb agent rand --env locomotion_velocity_tracking
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/3BTaKjtAbQs?si=UOlJxa6GlE05UcWQ4on&mute=1&autoplay=1&loop=1&playlist=3BTaKjtAbQs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Hint:** Use `--hide_ui` option to disable most of the Isaac Sim UI, as shown in the video above.

### Locomotion — Unitree G1

> Reference: [Environment Configuration — Robot](../config/robot.md)

Selecting a different robot for any environment is as simple as changing the `env.robot` parameter. This particular environment supports all legged robots, including humanoids. For instance, let's try the **Unitree G1** humanoid:

```bash
srb agent rand --env locomotion_velocity_tracking env.robot=unitree_g1
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube-nocookie.com/embed/viFvuz0Uq-g?si=SqoaHgz073j5V4on&mute=1&autoplay=1&loop=1&playlist=viFvuz0Uq-g" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Locomotion — 16 Parallel Environments

> Reference: [Environment Configuration — Parallel Simulation](../config/parallel_sim.md)

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

## 4. Assemble your Mobile Manipulator

> Reference: [Mobile Manipulators — Combined](../robots/combined_manipulators.md)

So far, you explored how to use pre-configured robots in diverse parallel environments. However, it is often useful to combine multiple robots and tools into an integrated mobile manipulation system.

### Ground Mobile Manipulation Template

First, let's start with the ground mobile manipulation template environment:

```bash
srb agent rand --env _ground_manipulation
```

The default template uses the **Spot** mobile base with a **UR10** arm, which is already a capable mobile manipulation platform. However, we can customize this combination as needed.

### Spot with Franka Panda

Let's modify our mobile manipulator to use a **Franka Panda** arm instead:

```bash
srb agent rand --env _ground_manipulation env.robot.manipulator=franka_panda
```

### Husky with Shadow Hand

You can also combine a **Clearpath Husky** wheeled base with a dexterous **Shadow Hand**:

```bash
srb agent rand --env _ground_manipulation env.robot.mobile_base=husky env.robot.manipulator=shadow_hand
```

## 5. Create your Custom Simulation

> Reference: [Environment Configuration](../config/env_cfg.md)

One of the advantages of SRB is the ability to create highly customized simulation scenarios. Let's explore how to configure advanced simulations.

### Custom Rendering Settings

For better visual quality, you can configure the rendering parameters:

```bash
srb agent rand --env sample_collection \
  rendering.ray_tracing=True \
  rendering.ray_tracing_settings.max_bounces=8 \
  rendering.ray_tracing_settings.samples_per_pixel=4
```

### Multi-Robot Simulation

You can also set up environments with multiple robots. For instance, let's simulate two Spot robots with different arm configurations:

```bash
srb agent rand --env _ground_manipulation \
  env.stack=true \
  env.num_envs=2 \
  env.robots=[spot+ur10,spot+franka_panda]
```

## 6. Use the Python API

> Reference: [Python API Examples](../examples/python_api.md)

While the CLI is convenient for quick experiments, SRB is primarily designed to be used programmatically through its Python API for reinforcement learning, motion planning, or other robotic applications.

### Basic Control Loop

Here's a simple example showing how to interact with an environment:

```python
import gymnasium as gym
import space_robotics_bench as srb

# Create environment
env = gym.make("sample_collection", headless=False)

# Get initial observation
obs, info = env.reset()

# Run simulation loop
for _ in range(1000):
    # Sample random action
    action = env.action_space.sample()

    # Apply action and get next observation, reward, etc.
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        obs, info = env.reset()

# Clean up
env.close()
```

### Custom Environment Configuration

You can configure environments with the same flexibility as the CLI:

```python
import gymnasium as gym
import space_robotics_bench as srb

# Configure environment with hydra
env_cfg = {
    "robot": "unitree_a1",
    "domain": "mars",
    "num_envs": 4,
    "stack": False,
    "camera": {
        "eye": [2.5, 0.0, 1.5],
        "target": [0.0, 0.0, 0.5],
    }
}

# Create environment with configuration
env = gym.make("locomotion_velocity_tracking", cfg=env_cfg, headless=False)
```

## Next Steps

Now that you're familiar with the basic usage of Space Robotics Bench, you might want to explore:

1. [Detailed Configuration Options](../config/env_cfg.md) — Learn about all the ways you can customize your simulations
1. [Running Reinforcement Learning](../examples/reinforcement_learning.md) — Train your first RL policy
1. [Creating Custom Tasks](../examples/custom_tasks.md) — Design your own robotic tasks
1. [Simulation-to-Real](../examples/sim2real.md) — Transfer learned policies to real robots

Continue exploring with the tutorials in the sidebar to get the most out of Space Robotics Bench!
