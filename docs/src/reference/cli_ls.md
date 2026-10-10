# `srb list` (alias `srb ls`) — List Assets and Environments

`srb ls` lists cached asset, hardware-interface, and environment names. Stale or incomplete snapshots trigger a warning.

## Usage

```bash
srb list [options]
srb ls   [options]   # alias
srb ls --live        # discover with Isaac Sim and refresh the cache
```

### Options

| Argument | Description | Default |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------ | :-----: |
| \*categories | Categories: `all`, `action`, `asset`, `env`, `scenery`, `object`, `robot`. `asset` selects `scenery`, `object`, and `robot`. | `[all]` |
| `--live` | Discover registries and refresh the cache. | `False` |

With `--live`, discover current registries, including action groups, in headless Isaac Sim and refresh the [completion and GUI cache](cli.md#cache-subcommands).

1. After registration changes, run `srb ls --live` to discover them and refresh the cache.

> **Note:** Underscore-prefixed task names (such as `_aerial`) are templates listed with concrete tasks. Customize them with Hydra overrides such as `env.robot=<name>`.

## Output

`--live` prints the requested tables. By default, `all` includes action groups and **Hardware Interfaces**. The **Assets** table adds an `Action Modes` column; `*` marks the default.

Offline output lists cached names only. In offline output, `action` lists hardware interfaces, not action groups.

> **Hint:** In VSCode, `--live` tables link to source code.

### Example `--live` Output (Truncated)

```
                                                              Assets
┏━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃ Name                        ┃ Type    ┃ Subtype            ┃ Parent Class          ┃ Asset Config    ┃ Path                             ┃ Action Modes                         ┃
┡━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ mars_surface                │ scenery │ terrain            │ Terrain               │ AssetBaseCfg    │ terrain.py                       │                                      │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┼──────────────────────────────────────┤
│ 2 │ cargo_bay                   │ object  │ payload            │ Payload               │ AssetBaseCfg    │ payload.py                       │                                      │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┼──────────────────────────────────────┤
│ 3 │ scoop                       │ object  │ tool               │ Tool                  │ RigidObjectCfg  │ tool/scoop.py                    │                                      │
│ 4 │ shadow_hand                 │ object  │ tool               │ ActiveTool            │ ArticulationCfg │ tool/gripper/shadow_hand.py      │                                      │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┼──────────────────────────────────────┤
│ 5 │ sample_tube                 │ object  │ common             │ Object                │ RigidObjectCfg  │ sample.py                        │                                      │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┼──────────────────────────────────────┤
│ 6 │ franka                      │ robot   │ manipulator        │ SerialManipulator     │ ArticulationCfg │ manipulation/franka.py           │ *diff_ik, osc_fixed, ...             │
│ 7 │ random_ur_manipulator       │ robot   │ manipulator        │ SerialManipulator     │ ArticulationCfg │ manipulation/universal_robots.py │ ...                                  │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┼──────────────────────────────────────┤
│ 8 │ random_unitree_quadruped    │ robot   │ mobile_robot       │ LeggedRobot           │ ArticulationCfg │ mobile/ground/unitree.py         │ ...                                  │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┼──────────────────────────────────────┤
│ 9 │ unitree_g1                  │ robot   │ mobile_manipulator │ Humanoid              │ ArticulationCfg │ mobile_manipulation/unitree.py   │ *joint_pos                           │
│ . │ ...                         │ ...     │ ...                │ ...                   │ ...             │ ...                              │ ...                                  │
└───┴─────────────────────────────┴─────────┴────────────────────┴───────────────────────┴─────────────────┴──────────────────────────────────┴──────────────────────────────────────┘

                           Action Groups
┏━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃ Name                          ┃ Path                       ┃
┡━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ body_acceleration             │ common/body.py             │
│ 2 │ body_acceleration_relative    │ common/body.py             │
│ 3 │ joint_position                │ common/joint.py            │
│ 4 │ joint_position_relative       │ common/joint.py            │
│ 5 │ joint_position_binary         │ common/joint.py            │
│ 6 │ joint_velocity                │ common/joint.py            │
│ 7 │ joint_velocity_binary         │ common/joint.py            │
│ 8 │ joint_effort                  │ common/joint.py            │
│ 9 │ inverse_kinematics            │ manipulation/task_space.py │
│ . │ ...                           │ ...                        │
└───┴───────────────────────────────┴────────────────────────────┘

                                                          Environments
┏━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃ ID                               ┃ Entrypoint                   ┃ Config                             ┃ Path                                      ┃
┡━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ _manipulation <template>         │ Task(ManipulationEnv)        │ TaskCfg(ManipulationEnvCfg)        │ manipulation/_manipulation                │
│ 2 │ sample_collection                │ Task(ManipulationEnv)        │ TaskCfg(ManipulationEnvCfg)        │ manipulation/sample_collection            │
│ 3 │ _aerial <template>               │ Task(AerialEnv)              │ TaskCfg(AerialEnvCfg)              │ mobile/_aerial                            │
│ 4 │ _ground <template>               │ Task(GroundEnv)              │ TaskCfg(GroundEnvCfg)              │ mobile/_ground                            │
│ 5 │ _orbital <template>              │ Task(OrbitalEnv)             │ TaskCfg(OrbitalEnvCfg)             │ mobile/_orbital                           │
│ 6 │ locomotion_velocity_tracking     │ LocomotionTask(Task)         │ LocomotionTaskCfg(TaskCfg)         │ mobile/velocity_tracking                  │
│ 7 │ _aerial_manipulation <template>  │ Task(AerialManipulationEnv)  │ TaskCfg(AerialManipulationEnvCfg)  │ mobile_manipulation/_aerial_manipulation  │
│ 8 │ _ground_manipulation <template>  │ Task(GroundManipulationEnv)  │ TaskCfg(GroundManipulationEnvCfg)  │ mobile_manipulation/_ground_manipulation  │
│ 9 │ _orbital_manipulation <template> │ Task(OrbitalManipulationEnv) │ TaskCfg(OrbitalManipulationEnvCfg) │ mobile_manipulation/_orbital_manipulation │
│ . │ ...                              │ ...                          │ ...                                │ ...                                       │
└───┴──────────────────────────────────┴──────────────────────────────┴────────────────────────────────────┴───────────────────────────────────────────┘
```
