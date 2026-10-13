# Environments -- Digital Twins

Facility-inspired simulator environments, not validated against their physical counterparts.

### Lunalab (`lunalab`)

> Digital twin of the LunaLab planetary analogue testbed

![lunalab](../_graphics/tasks/lunalab.webp)

| | |
|---|---|
| **Episode Length** | 100000.0s (2500000 steps; truncation disabled) |
| **Reward Terms** | 2 |
| **Default Robot** | [Generic Ground Manipulator](../assets/robots/mobile_manipulators.md#generic_ground_manipulator) |

#### Robots

[Generic Ground Manipulator](../assets/robots/mobile_manipulators.md#generic_ground_manipulator)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |

</details>

### Zerog Lab (`zerog_lab`)

> Digital twin of a zero-gravity laboratory testbed

![zerog_lab](../_graphics/tasks/zerog_lab.webp)

| | |
|---|---|
| **Episode Length** | 100000.0s (1000000 steps; truncation disabled) |
| **Reward Terms** | 2 |
| **Default Robot** | [Generic Orbital Manipulator](../assets/robots/mobile_manipulators.md#generic_orbital_manipulator) |

#### Robots

[Generic Orbital Manipulator](../assets/robots/mobile_manipulators.md#generic_orbital_manipulator)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_fuel_consumption | Linear | -1 | – | |

</details>
