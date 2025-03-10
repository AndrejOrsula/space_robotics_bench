# `srb ls` — List Assets and Environments

```
srb ls
```

#### Assets of the Space Robotics Bench

```
┏━━━━┳━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃  # ┃  Type   ┃      Subtype       ┃ Parent Class          ┃ Asset Cfg       ┃ Name                        ┃ Path                                    ┃
┡━━━━╇━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│  1 │ scenery │      terrain       │ Terrain               │ AssetBaseCfg    │ moon_surface                │ planetary_surface.py                    │
│  2 │ object  │        tool        │ Tool                  │ RigidObjectCfg  │ scoop                       │ tool/scoop.py                           │
│  3 │ object  │        tool        │ ActiveTool            │ ArticulationCfg │ shadow_hand                 │ tool/shadow_hand.py                     │
│  4 │ object  │       common       │ Object                │ RigidObjectCfg  │ asteroid                    │ asteroid.py                             │
│  5 │  robot  │    manipulator     │ SerialManipulator     │ ArticulationCfg │ ur5                         │ manipulation/universal_robots.py        │
│  6 │  robot  │    mobile_robot    │ Multicopter           │ ArticulationCfg │ ingenuity                   │ mobile/ingenuity.py                     │
│  7 │  robot  │    mobile_robot    │ WheeledRobot          │ ArticulationCfg │ perseverance                │ mobile/perseverance.py                  │
│  8 │  robot  │ mobile_manipulator │ Humanoid              │ ArticulationCfg │ unitree_g1                  │ mobile_manipulation/unitree.py          │
│  . │  ...    │ ...                │ ...                   │ ...             │ ...                         │ ...                                     │
└────┴─────────┴────────────────────┴───────────────────────┴─────────────────┴─────────────────────────────┴─────────────────────────────────────────┘
```

#### Action Groups of the Space Robotics Bench

```
┏━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃  # ┃ Name                                    ┃ Path                       ┃
┡━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│  1 │ body_velocity_action_group              │ common/body.py             │
│  2 │ joint_position_action_group             │ common/joint.py            │
│  3 │ inverse_kinematics_action_group         │ manipulation/task_space.py │
│  . │ ...                                     │ ...                        │
└────┴─────────────────────────────────────────┴────────────────────────────┘
```

#### Environments of the Space Robotics Bench

```
┏━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃  # ┃ ID                           ┃ Entrypoint                   ┃ Config                             ┃ Path                                      ┃
┡━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│  1 │ _aerial <template>           │ Task(AerialEnv)              │ TaskCfg(AerialEnvCfg)              │ mobile/_aerial                            │
│  2 │ _ground <template>           │ Task(GroundEnv)              │ TaskCfg(GroundEnvCfg)              │ mobile/_ground                            │
│  3 │ _manipulation <template>     │ Task(ManipulatorEnv)         │ TaskCfg(ManipulatorEnvCfg)         │ manipulation/_manipulation                │
│  4 │ _orbital <template>          │ Task(OrbitalEnv)             │ TaskCfg(OrbitalEnvCfg)             │ mobile/_orbital                           │
│  5 │ locomotion_velocity_tracking │ Task(GroundEnv)              │ TaskCfg(GroundEnvCfg)              │ mobile/locomotion_velocity_tracking       │
│  6 │ sample_collection            │ Task(ManipulatorEnv)         │ TaskCfg(ManipulatorEnvCfg)         │ manipulation/sample_collection            │
│  . │ ...                          │ ...                          │ ...                                │ ...                                       │
└────┴──────────────────────────────┴──────────────────────────────┴────────────────────────────────────┴───────────────────────────────────────────┘
```

VSCode
