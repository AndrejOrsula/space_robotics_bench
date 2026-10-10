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

### Orbital Lunalab (`orbital_lunalab`)

> Floating platform hovering over the LunaLab regolith

![orbital_lunalab](../_graphics/tasks/orbital_lunalab.webp)

| | |
|---|---|
| **Episode Length** | 100000.0s (2500000 steps; truncation disabled) |
| **Reward Terms** | 2 |
| **Default Robot** | [Floating Platform](../assets/robots/mobile_robots.md#floating_platform) |

#### Robots

[Floating Platform](../assets/robots/mobile_robots.md#floating_platform) | [Int Ball1](../assets/robots/mobile_robots.md#int_ball1) | [Int Ball2](../assets/robots/mobile_robots.md#int_ball2) | [Apollo Lander](../assets/robots/mobile_robots.md#apollo_lander) | [Peregrine Lander](../assets/robots/mobile_robots.md#peregrine_lander) | [Vikram Lander](../assets/robots/mobile_robots.md#vikram_lander) | [Resilience Lander](../assets/robots/mobile_robots.md#resilience_lander) | [Random Lander](../assets/robots/mobile_robots.md#random_lander) | [Iss](../assets/robots/mobile_robots.md#iss) | [Gateway](../assets/robots/mobile_robots.md#gateway) | [Cubesat](../assets/robots/mobile_robots.md#cubesat) | [Satellite Mockup](../assets/robots/mobile_robots.md#satellite_mockup) | [Venus Express](../assets/robots/mobile_robots.md#venus_express) | [Starship](../assets/robots/mobile_robots.md#starship) | [Super Heavy](../assets/robots/mobile_robots.md#super_heavy)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_fuel_consumption | Linear | -1 | – | |

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
