# Environments -- Manipulation (Static)

Fixed-base robot arm manipulation environments.

### Manipulation (`_manipulation`)

> Template environment for fixed-base manipulation tasks. Configure via `env.robot=<manipulator>`.

![_manipulation](../_graphics/tasks/_manipulation.webp)

| | |
|---|---|
| **Episode Length** | 10.0s (500 steps) |
| **Reward Terms** | 4 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |

</details>

### Debris Capture (`debris_capture`)

> Capture and securement of floating debris in microgravity with a fixed-base manipulator

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/3zog4qGHg6A?si=docgen&mute=1&autoplay=1&loop=1&playlist=3zog4qGHg6A" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 10.0s (500 steps) |
| **Reward Terms** | 11 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |
| reward_distance_ee_to_obj | Gaussian | 16 | 0.08 | |
| reward_grasp | SoftThreshold | 16 | – | |
| penalty_debris_velocity_lin | Linear | -3 | – | |
| penalty_debris_velocity_ang | Linear | -1 | – | |
| reward_approach_velocity_alignment | Hyperbolic | 4 | 0.1 | |
| penalty_approach_speed | Linear | -2 | – | |
| penalty_debris_too_far | Linear | -8 | – | |

</details>

<!-- PRESERVE:notes_debris_capture -->

- `distance_debris_too_far` applies `penalty_debris_too_far` only; episodes end by horizon (`truncated`).
- Episode info omits `success`; bounded evaluation reports return only.

<!-- /PRESERVE:notes_debris_capture -->

### Excavation (`excavation`)

> Extraction of granular material (regolith)

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/XIJhFsaT1Ls?si=docgen&mute=1&autoplay=1&loop=1&playlist=XIJhFsaT1Ls" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 8.0s (400 steps) |
| **Reward Terms** | 13 |
| **Default Robot** | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) |

#### Robots

[Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.05 | – | |
| penalty_joint_torque | Linear | -0.00001 | – | |
| penalty_joint_acceleration | Linear | 0 | – | |
| penalty_undesired_robot_contacts | Linear | 0 | – | |
| reward_approach_pile | Gaussian | 1.5 | 0.15 | |
| reward_scoop_depth | Tukey | 2 | – | |
| reward_scoop_orientation | GaussianTracking | 1 | 0.26 | |
| reward_lift | Hyperbolic | 2 | 0.2 | |
| penalty_jerk | Linear | -0.02 | – | |
| penalty_joint_acc | Linear | -0.000001 | – | |
| penalty_undesired_contacts | Linear | -0.5 | – | |
| terminal_particle_count_scaled | Scaled | 10 | – | |
| terminal_success_bonus | Scaled | 50 | – | |

</details>

<!-- PRESERVE:notes_excavation -->

- Heightmap bins live regolith positions onto the observation grid at runtime; PhysX particle sets cannot be raycast.
- `imu_pedestal` is active only at `env.num_envs=1`; otherwise it is disabled with a warning and returns zero observations.
- Episode-end info includes `success`; bounded evaluation reports success rate and return.
- Wrist event camera: add `events` to `env.active_sensors`.

<!-- /PRESERVE:notes_excavation -->

### Excavation Demo (`excavation_demo`)

> Manual excavation demo with presentation-oriented lunar regolith defaults

![excavation_demo](../_graphics/tasks/excavation_demo.webp)

| | |
|---|---|
| **Episode Length** | 1800.0s (90000 steps; truncation disabled) |
| **Reward Terms** | 13 |
| **Default Robot** | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) |

#### Robots

[Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.05 | – | |
| penalty_joint_torque | Linear | -0.00001 | – | |
| penalty_joint_acceleration | Linear | 0 | – | |
| penalty_undesired_robot_contacts | Linear | 0 | – | |
| reward_approach_pile | Gaussian | 1.5 | 0.15 | |
| reward_scoop_depth | Tukey | 2 | – | |
| reward_scoop_orientation | GaussianTracking | 1 | 0.26 | |
| reward_lift | Hyperbolic | 2 | 0.2 | |
| penalty_jerk | Linear | -0.02 | – | |
| penalty_joint_acc | Linear | -0.000001 | – | |
| penalty_undesired_contacts | Linear | -0.5 | – | |
| terminal_particle_count_scaled | Scaled | 10 | – | |
| terminal_success_bonus | Scaled | 50 | – | |

</details>

### Modular Assembly (`modular_assembly`)

> Pick-and-place modular elements into an assembly configuration

![modular_assembly](../_graphics/tasks/modular_assembly.webp)

| | |
|---|---|
| **Episode Length** | 20.0s (1000 steps) |
| **Reward Terms** | 10 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | 0 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -32 | – | |
| reward_top_down_orientation | GaussianTracking | 16 | 0.25 | |
| reward_pair_position | Linear | 8 | – | |
| reward_pair_orientation | Linear | 2 | – | |
| reward_peg_hole_entrance_alignment | Linear | 8 | – | |
| reward_peg_hole_alignment | Linear | 64 | – | |
| reward_success | Linear | 512 | – | |

</details>

### Multi Peg In Hole (`multi_peg_in_hole`)

> A variation of peg-in-hole assembly with multiple pegs and holes

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/50nKU-myKbM?si=docgen&mute=1&autoplay=1&loop=1&playlist=50nKU-myKbM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 40.0s (2000 steps) |
| **Reward Terms** | 19 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |
| reward_top_down_orientation | GaussianTracking | 1 | 0.25 | |
| reward_distance_ee_to_obj | Hyperbolic | 0 | 0.25 | |
| reward_grasp | Linear | 8 | – | |
| reward_lift | Linear | 4 | – | |
| reward_align_peg_to_hole_primary | GaussianTracking | 0 | 0.4 | |
| reward_align_peg_to_hole_secondary | Gaussian | 0 | 0.2 | |
| reward_distance_peg_to_hole_entrance_gradual | Hyperbolic | 0 | 0.16 | |
| reward_distance_peg_to_hole_entrance | Gaussian | 0 | 0.04 | |
| reward_distance_peg_to_hole_bottom | Gaussian | 0 | 0.003 | |
| reward_distance_ee_to_objs | Linear | 2.5 | – | |
| reward_align_pegs_to_holes_primary | Linear | 8 | – | |
| reward_align_pegs_to_holes_secondary | Linear | 4 | – | |
| reward_distance_pegs_to_holes_entrance_gradual | Linear | 8 | – | |
| reward_distance_pegs_to_holes_entrance | Linear | 32 | – | |
| reward_distance_pegs_to_holes_bottom | Linear | 256 | – | |

</details>

### Multi Sample Collection (`multi_sample_collection`)

> A variation of sample collection with multiple samples

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/CyPJr6LSUac?si=docgen&mute=1&autoplay=1&loop=1&playlist=CyPJr6LSUac" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 22.5s (1125 steps) |
| **Reward Terms** | 12 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |
| reward_top_down_orientation | GaussianTracking | 1 | 0.25 | |
| reward_distance_ee_to_obj | Gaussian | 0 | 0.15 | |
| reward_grasp | Linear | 4 | – | |
| reward_lift | Linear | 8 | – | |
| reward_distance_obj_to_target | Gaussian | 0 | 0.15 | |
| penalty_obj_fallen | Linear | 0 | – | |
| reward_distance_ee_to_objs | Linear | 2.5 | – | |
| reward_distance_objs_to_target | Linear | 32 | – | |

</details>

### Peg In Hole (`peg_in_hole`)

> Precision assembly task requiring the robot to insert a peg into a hole

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/C8xhE6hHG9I?si=docgen&mute=1&autoplay=1&loop=1&playlist=C8xhE6hHG9I" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 10.0s (500 steps) |
| **Reward Terms** | 13 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |
| **Multi Variant** | `multi_peg_in_hole` |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |
| reward_top_down_orientation | GaussianTracking | 1 | 0.25 | |
| reward_distance_ee_to_obj | Hyperbolic | 4 | 0.25 | |
| reward_grasp | SoftThreshold | 8 | – | |
| reward_lift | GaussianZoneTracking | 4 | 0.08 | |
| reward_align_peg_to_hole_primary | GaussianTracking | 8 | 0.4 | |
| reward_align_peg_to_hole_secondary | Gaussian | 4 | 0.2 | |
| reward_distance_peg_to_hole_entrance_gradual | Hyperbolic | 8 | 0.16 | |
| reward_distance_peg_to_hole_entrance | Gaussian | 32 | 0.04 | |
| reward_distance_peg_to_hole_bottom | Gaussian | 256 | 0.003 | |

</details>

<!-- PRESERVE:notes_peg_in_hole -->

**Evaluation**

- Episode info has no `success`; bounded evaluation reports return only.

<!-- /PRESERVE:notes_peg_in_hole -->

### Sample Collection (`sample_collection`)

> Grasping and collection of samples on planetary surfaces

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/1NYquT2EZdo?si=docgen&mute=1&autoplay=1&loop=1&playlist=1NYquT2EZdo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 7.5s (375 steps) |
| **Reward Terms** | 10 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |
| **Multi Variant** | `multi_sample_collection` |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |
| reward_top_down_orientation | GaussianTracking | 1 | 0.25 | |
| reward_distance_ee_to_obj | Gaussian | 4 | 0.15 | |
| reward_grasp | SoftThreshold | 6 | – | |
| reward_lift | GaussianZoneTracking | 12 | 0.15 | |
| reward_distance_obj_to_target | Gaussian | 32 | 0.15 | |
| penalty_obj_fallen | Linear | -16 | – | |

</details>

<!-- PRESERVE:notes_sample_collection -->

- `obj_floor_z` drives `penalty_obj_fallen`; falls do not terminate. Horizon ends as `truncated`.
- Wrist event camera: add `events` to `env.active_sensors`.
- Episode info omits `success`; bounded evaluation reports return only.

<!-- /PRESERVE:notes_sample_collection -->

### Screwdriving (`screwdriving`)

> Fastening of a pre-inserted bolt

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/w25DwrgeG2M?si=docgen&mute=1&autoplay=1&loop=1&playlist=w25DwrgeG2M" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 15.0s (750 steps) |
| **Reward Terms** | 13 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |
| reward_top_down_orientation | GaussianTracking | 8 | 0.04 | |
| penalty_object_top_down_orientation | Linear | -64 | – | |
| reward_distance_ee_to_bolt_driver_slot | Hyperbolic | 12 | 0.4 | |
| reward_distance_ee_to_bolt_driver_slot_close | Hyperbolic | 32 | 0.02 | |
| reward_distance_ee_to_bolt_driver_slot_precision | Hyperbolic | 512 | 0.005 | |
| reward_contact | SoftThreshold | 4 | – | |
| reward_screwing | Scaled | 128 | – | |
| penalty_distance_obj_to_target_too_far | Threshold | -256 | – | |
| reward_distance_obj_to_target | Linear | 2048 | – | |

</details>

<!-- PRESERVE:notes_screwdriving -->

**Evaluation**

- Episode info has no `success`; bounded evaluation reports return only.

<!-- /PRESERVE:notes_screwdriving -->

### Solar Panel Assembly (`solar_panel_assembly`)

> Complex assembly sequence involving solar panel components

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/tY5XDuhdzOc?si=docgen&mute=1&autoplay=1&loop=1&playlist=tY5XDuhdzOc" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

| | |
|---|---|
| **Episode Length** | 50.0s (2500 steps) |
| **Reward Terms** | 17 |
| **Default Robot** | [Franka](../assets/robots/manipulators.md#franka) |

#### Robots

[Canadarm3](../assets/robots/manipulators.md#canadarm3) | [Franka](../assets/robots/manipulators.md#franka) | [Kinova J2N6S](../assets/robots/manipulators.md#kinova_j2n6s) | [Kinova J2N7S](../assets/robots/manipulators.md#kinova_j2n7s) | [Kinova Gen3N7](../assets/robots/manipulators.md#kinova_gen3n7) | [Levion Arms](../assets/robots/manipulators.md#levion_arms) | [So Arm100 D5](../assets/robots/manipulators.md#so_arm100_d5) | [So Arm100 D7](../assets/robots/manipulators.md#so_arm100_d7) | [Unitree Z1](../assets/robots/manipulators.md#unitree_z1) | [Ur3](../assets/robots/manipulators.md#ur3) | [Ur3E](../assets/robots/manipulators.md#ur3e) | [Ur5](../assets/robots/manipulators.md#ur5) | [Ur5E](../assets/robots/manipulators.md#ur5e) | [Ur10](../assets/robots/manipulators.md#ur10) | [Ur10E](../assets/robots/manipulators.md#ur10e) | [Ur16E](../assets/robots/manipulators.md#ur16e) | [Ur20](../assets/robots/manipulators.md#ur20) | [Ur30](../assets/robots/manipulators.md#ur30) | [Random Ur Manipulator](../assets/robots/manipulators.md#random_ur_manipulator) | [Vispa](../assets/robots/manipulators.md#vispa) | [Zeroglab Arms](../assets/robots/manipulators.md#zeroglab_arms)

<details><summary>Rewards</summary>

| Term | Type | Weight | Scale | Notes |
|---|---|---|---|---|
| penalty_action_rate | Linear | -0.1 | – | |
| penalty_joint_torque | Linear | -0.000025 | – | |
| penalty_joint_acceleration | Linear | -0.0005 | – | |
| penalty_undesired_robot_contacts | Threshold | -0.5 | – | |
| reward_top_down_orientation | GaussianTracking | 1 | 0.25 | |
| reward_distance_ee_to_objs | Linear | 2.5 | – | |
| reward_grasp | SoftThreshold | 8 | – | |
| reward_lift | Linear | 4 | – | |
| reward_align_pegs_to_holes_primary | Linear | 8 | – | |
| reward_align_pegs_to_holes_secondary | Linear | 4 | – | |
| reward_distance_pegs_to_holes_entrance_gradual | Linear | 8 | – | |
| reward_distance_pegs_to_holes_entrance | Linear | 32 | – | |
| reward_distance_pegs_to_holes_bottom | Linear | 256 | – | |
| reward_distance_ee_to_panel | Hyperbolic | 10 | 0.25 | |
| reward_panel_lift | GaussianZoneTracking | 16 | 0.1 | |
| reward_distance_panel_to_target_gradual | Hyperbolic | 32 | 0.1 | |
| reward_distance_panel_to_target | Gaussian | 1024 | 0.015 | |

</details>

<!-- PRESERVE:notes_solar_panel_assembly -->

**Evaluation**

- Episode info has no `success`; bounded evaluation reports return only.

<!-- /PRESERVE:notes_solar_panel_assembly -->
