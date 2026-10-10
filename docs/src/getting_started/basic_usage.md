# Basic Usage

After [installation](install.md), list registries and run an environment:

<div class="warning">

**Native installation:** If `srb` is unavailable, run:

```bash
"$ISAACSIM_PYTHON" -m srb --help
```

</div>

## 1. List Assets, Actions & Environments

> Reference: [`srb ls` — List Assets and Environments](../reference/cli_ls.md)

```bash
srb ls --live
```

Use `--live` for headless Isaac Sim discovery and cache refresh; plain `srb ls` prints cached names. See the [CLI reference](../reference/cli_ls.md) for categories and options.

Live tables list assets (scenery, objects, robots), action groups, hardware interfaces, and environments. Hardware interface examples are not validated on hardware.

<details>
  <summary><strong>1. Assets</strong> <i>(click to expand)</i></summary>

#### Scenery asset (automatically registered as "mars_surface" scenery/terrain)

```py
{{#include ../../../srb/assets/scenery/terrain.py:example}}
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
    ...
{{#include ../../../srb/assets/robot/manipulation/franka.py:example_p4}}
```

</details>

## 2. Teleoperate a Robot

> Reference: [`srb agent teleop` — Teleoperate Agent](../reference/cli_agent_teleop.md)

Collect Mars Sample Return tubes in `sample_collection` with the **Franka** manipulator:

```bash
srb agent teleop --env sample_collection
```

Isaac Sim opens; focus its window to use the terminal controls.

| Control | Keys |
| --- | --- |
| Reset | `L` |
| Gain | `O` decrease; `P` increase |
| Event | `R` / `K` |
| Translate | `W` / `S`: +X / −X; `A` / `D`: −Y / +Y; `Q` / `E`: +Z / −Z |
| Rotate | `Z` / `X`: ±X; `T` / `G`: ↻ / ↺ about ±Y; `C` / `V`: ↺ / ↻ about ±Z |

<br>

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/dlzwlct1BLA?si=d_oEZzmvS7SQviO1&mute=1&autoplay=1&loop=1&playlist=dlzwlct1BLA" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Note:** Direct teleoperation uses high-level actions. Low-level joint tasks (e.g., `locomotion_velocity_tracking`) need a policy to map commands to actions; see [Teleoperation via Policy](../reference/cli_agent_teleop.md#teleoperation-via-policy).

### Move to a Different Domain

> Reference: [Environment Configuration](../config/env_cfg.md)\
> Reference: [Environment Configuration — Domain](../config/domain.md)

Collect procedural `moon_rock` samples on the Moon with Hydra overrides:

```bash
srb agent teleop --env sample_collection env.domain=moon env.sample=moon_rock
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/d6KhKuB-XAs?si=DM0I2IFinw-rqgdD&mute=1&autoplay=1&loop=1&playlist=d6KhKuB-XAs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 3. Run Random Actions

> Reference: [`srb agent rand` — Random Agent](../reference/cli_agent_rand.md)

Run `locomotion_velocity_tracking` with random actions (default **Spot**):

```bash
srb agent rand --env locomotion_velocity_tracking
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/3BTaKjtAbQs?si=UOlJxa6GlE05UcWQ4on&mute=1&autoplay=1&loop=1&playlist=3BTaKjtAbQs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Hint:** `--hide_ui` hides most of the Isaac Sim UI.

### Change the Robot

> Reference: [Environment Configuration — Robot](../config/robot.md)

Set `env.robot` to a legged robot or humanoid such as **Unitree G1**:

```bash
srb agent rand --env locomotion_velocity_tracking env.robot=unitree_g1
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/viFvuz0Uq-g?si=SqoaHgz073j5V4on&mute=1&autoplay=1&loop=1&playlist=viFvuz0Uq-g" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Simulate Multiple Robots

> Reference: [Environment Configuration — Parallelism](../config/parallelism.md)

Run **16 Cassie** instances in parallel:

```bash
srb agent rand -e locomotion_velocity_tracking env.robot=cassie env.num_envs=16
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/eZt123VywGQ?si=7toO7FbDO3OxMbq1&mute=1&autoplay=1&loop=1&playlist=eZt123VywGQ" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

> **Hint:** `locomotion_velocity_tracking` shares terrain by default (other tasks may differ). Set `env.stack=false` for unique terrain geometry and materials per robot. Set `SF_BAKER=0` to skip texture baking and speed generation; unbaked surfaces render with a magenta placeholder material:
>
> ```bash
> SF_BAKER=0 srb agent rand -e locomotion_velocity_tracking env.stack=false env.num_envs=16
> ```
>
> ![](https://github.com/user-attachments/assets/2711d026-e0b5-4839-af2b-ff3a0423b683)

## 4. Customize Environment Templates

> Reference: [Assets](../assets/index.md)\
> Reference: [Environment Configuration — Robot](../config/robot.md)

Start custom scenarios from a robot-category [template](../contributing/new_tasks.md#1-duplicate-a-template); `sample_collection` and `locomotion_velocity_tracking` are goal-oriented tasks.

Try `_ground_manipulation` with the **Spot** quadruped and **Franka** manipulator:

```bash
srb agent rand -e _ground_manipulation
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/iFRMExXJEfI?si=sCPI-UuyVgJF7ucL&mute=1&autoplay=1&loop=1&playlist=iFRMExXJEfI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Diversify Robot Configurations

Randomize **Unitree** quadrupeds (`random_unitree_quadruped`) and **Universal Robots** manipulators (`random_ur_manipulator`, **UR3e**–**UR30**); select mobile-manipulator parts via `env.robot.mobile_base` and `env.robot.manipulator`.

```bash
srb agent rand -e _ground_manipulation env.robot.mobile_base=random_unitree_quadruped env.robot.manipulator=random_ur_manipulator env.num_envs=6 env.stack=true
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/ac5p5kPCHyE?si=pujsR8vHy9pn-ZRG&mute=1&autoplay=1&loop=1&playlist=ac5p5kPCHyE" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Customize Payloads & End Effectors

> Reference: [`srb agent zero` — Zero Agent](../reference/cli_agent_zero.md)

Use `+` in `env.robot` or its mobile-base/manipulator fields to attach **Cargo Bay** to **Anymal D** and pair **Unitree Z1** with **Shadow Hand**:

```bash
srb agent zero -e _ground_manipulation env.robot.mobile_base=anymal_d+cargo_bay env.robot.manipulator=unitree_z1+shadow_hand
```

![](https://github.com/user-attachments/assets/edce5af8-9fa5-4686-9829-550c07efba76)

> **Hint:** Both sides of `+` are required (for example, `anymal_d+cargo_bay`); omitting either side (`+payload`, `+end_effector`, `mobile_robot+`, or `manipulator+`) raises an error.

Task and domain determine supported combinations; sensor poses and action-space dimensions adjust automatically.

```bash
srb agent zero -e _aerial_manipulation env.robot.mobile_base=ingenuity env.robot.manipulator=franka+scoop
```

```bash
srb agent zero -e _orbital_manipulation env.robot.mobile_base=gateway env.robot.manipulator=canadarm3
```

<img src="https://github.com/user-attachments/assets/e12669d1-8a30-4220-91f5-b86df2b0f718" style="width:50%" /><img src="https://github.com/user-attachments/assets/f61a71b6-0cdb-4dcf-9b1b-fe4a24d2b5bc" style="width:50%" />

## 5. Enable Cameras

Most tasks disable visual sensors for performance; some enable them by default. Where supported, add `visual` to `env.active_sensors`.

View the **Ingenuity** helicopter on Mars:

```bash
srb agent teleop -e _aerial 'env.active_sensors=[proprioception,contact,transform,visual]'
```

| Override | Default | Limit or meaning |
| --- | --- | --- |
| `env.camera_resolution` | `(64, 64)` | Width and height; tasks may override. |
| `env.camera_data_types` | `("rgb", "depth")` | Task configuration determines available types and may override RGB and depth defaults. |
| `env.camera_update_period` | `-1.0` | For cameras without an authored period, negative selects the agent rate. |
| `env.camera_clipping_range` | `None` | Near/far planes in metres; near must be positive, far greater than near and below 1000 m. Fills missing ranges or ranges with render-default far planes; preserves camera-authored ranges with far planes below 1000 m. |

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/NHh0EiwaD2A?si=dE5Z_7zV6JT5hLU3&mute=1&autoplay=1&loop=1&playlist=NHh0EiwaD2A" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Next: [Workflows](../workflows/index.md)
