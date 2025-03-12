# `srb ls` — List Assets and Environments

```
srb ls
```

#### Assets

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

#### Action Groups

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

#### Environments

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

mention VSCode links
