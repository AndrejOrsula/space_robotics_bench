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

In this guide, we will focus on the registered **assets** and **environments**.

#### Assets

```
┏━━━┳━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃  Type   ┃      Subtype       ┃ Parent Class      ┃ Asset Cfg       ┃ Name         ┃ Path                             ┃
┡━━━╇━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ scenery │      terrain       │ Terrain           │ AssetBaseCfg    │ mars_surface │ planetary_surface.py             │
│ 2 │ object  │        tool        │ Tool              │ RigidObjectCfg  │ scoop        │ tool/scoop.py                    │
│ 3 │ object  │        tool        │ ActiveTool        │ ArticulationCfg │ shadow_hand  │ tool/shadow_hand.py              │
│ 4 │ object  │       common       │ Object            │ RigidObjectCfg  │ sample_tube  │ sample.py                        │
│ 5 │  robot  │    manipulator     │ SerialManipulator │ ArticulationCfg │ ur5          │ manipulation/universal_robots.py │
│ 6 │  robot  │    mobile_robot    │ Multicopter       │ ArticulationCfg │ ingenuity    │ mobile/ingenuity.py              │
│ 7 │  robot  │    mobile_robot    │ WheeledRobot      │ ArticulationCfg │ perseverance │ mobile/perseverance.py           │
│ 8 │  robot  │ mobile_manipulator │ Humanoid          │ ArticulationCfg │ unitree_g1   │ mobile_manipulation/unitree.py   │
│ . │ ....... │ .................. │ ................. │ ............... │ ............ │ ................................ │
└───┴─────────┴────────────────────┴───────────────────┴─────────────────┴──────────────┴──────────────────────────────────┘
```

#### Environments

```
┏━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃ ID                           ┃ Entrypoint           ┃ Config                     ┃ Path                                ┃
┡━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ _aerial <template>           │ Task(AerialEnv)      │ TaskCfg(AerialEnvCfg)      │ mobile/_aerial                      │
│ 2 │ _ground <template>           │ Task(GroundEnv)      │ TaskCfg(GroundEnvCfg)      │ mobile/_ground                      │
│ 3 │ _manipulation <template>     │ Task(ManipulatorEnv) │ TaskCfg(ManipulatorEnvCfg) │ manipulation/_manipulation          │
│ 4 │ _orbital <template>          │ Task(OrbitalEnv)     │ TaskCfg(OrbitalEnvCfg)     │ mobile/_orbital                     │
│ 5 │ locomotion_velocity_tracking │ Task(GroundEnv)      │ TaskCfg(GroundEnvCfg)      │ mobile/locomotion_velocity_tracking │
│ 6 │ sample_collection            │ Task(ManipulatorEnv) │ TaskCfg(ManipulatorEnvCfg) │ manipulation/sample_collection      │
│ . │ ............................ │ .................... │ .......................... │ ................................... │
└───┴──────────────────────────────┴──────────────────────┴────────────────────────────┴─────────────────────────────────────┘
```

## 2. Teleoperate your First Robot

> Reference: [`srb agent teleop` — Teleoperate Agent](../instructions/cli_agent_teleop.md).

```bash
srb agent teleop
```
