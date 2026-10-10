# Mobile Manipulators

Mobile manipulators registered in the Space Robotics Bench: generic platforms that mount a manipulator on an aerial, ground, or orbital base, and humanoid robots. Generic platforms assemble their parts at runtime, so their listed DoF, sensors, and action modes cover only the class defaults; the composed robot inherits those of its parts.

![Overview](../../_graphics/assets/_collage_mobile_manipulators.webp)

## Combined Platforms

<a id="generic_aerial_manipulator"></a>

#### Generic Aerial Manipulator (`generic_aerial_manipulator`)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Sensors** | 0 |
| **Action Modes** | – |

<a id="generic_ground_manipulator"></a>

#### Generic Ground Manipulator (`generic_ground_manipulator`)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Sensors** | 0 |
| **Action Modes** | – |

<a id="generic_orbital_manipulator"></a>

#### Generic Orbital Manipulator (`generic_orbital_manipulator`)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Sensors** | 0 |
| **Action Modes** | – |

## Humanoids

<a id="humanoid21"></a>

#### Humanoid21 (`humanoid21`)

![humanoid21](../../_graphics/assets/humanoid21.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `pelvis` |
| **Sensors** | 3 |
| **Action Modes** | default: `joint_pos` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| body | `.*` | – | – | 2–20 | 0.1–5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `pelvis` |
| contacts_feet | Contact Sensor | `(left\|right)_foot` |
| cam_front | Camera | `pelvis/camera_front` |

</details>

<a id="humanoid28"></a>

#### Humanoid28 (`humanoid28`)

![humanoid28](../../_graphics/assets/humanoid28.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `pelvis` |
| **Sensors** | 3 |
| **Action Modes** | default: `joint_pos` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| body | `.*` | – | – | – | – |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `pelvis` |
| contacts_feet | Contact Sensor | `(left\|right)_foot` |
| cam_front | Camera | `pelvis/camera_front` |

</details>

<a id="unitree_g1"></a>

#### Unitree G1 (`unitree_g1`)

![unitree_g1](../../_graphics/assets/unitree_g1.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `pelvis` |
| **Sensors** | 3 |
| **Action Modes** | default: `joint_pos` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| legs | `.*_hip_yaw_joint`, `.*_hip_roll_joint`, `.*_hip_pitch_joint`, `.*_knee_joint`, `torso_joint` | 300 N·m | 100 rad/s | 150–200 | 5 |
| feet | `.*_ankle_pitch_joint`, `.*_ankle_roll_joint` | 20 N·m | – | 20 | 2 |
| arms | `.*_shoulder_pitch_joint`, `.*_shoulder_roll_joint`, `.*_shoulder_yaw_joint`, `.*_elbow_pitch_joint`, `.*_elbow_roll_joint`, `.*_five_joint`, `.*_three_joint`, `.*_six_joint`, `.*_four_joint`, `.*_zero_joint`, `.*_one_joint`, `.*_two_joint` | 300 N·m | 100 rad/s | 40 | 10 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `pelvis` |
| contacts_feet | Contact Sensor | `(left\|right)_ankle_roll_link` |
| cam_front | Camera | `pelvis/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*_hip_pitch_joint` | -0.2 rad (-11.459156 deg) |
| `.*_knee_joint` | 0.42 rad (24.064227 deg) |
| `.*_ankle_pitch_joint` | -0.23 rad (-13.178029 deg) |
| `.*_elbow_pitch_joint` | 0.87 rad (49.847328 deg) |
| left_shoulder_roll_joint | 0.16 rad (9.167325 deg) |
| left_shoulder_pitch_joint | 0.35 rad (20.053523 deg) |
| right_shoulder_roll_joint | -0.16 rad (-9.167325 deg) |
| right_shoulder_pitch_joint | 0.35 rad (20.053523 deg) |
| left_one_joint | 1 rad (57.29578 deg) |
| right_one_joint | -1 rad (-57.29578 deg) |
| left_two_joint | 0.52 rad (29.793805 deg) |
| right_two_joint | -0.52 rad (-29.793805 deg) |

</details>

The DoF and `joint_pos` action dimensions are runtime-resolved from the Unitree G1 USD articulation. The actuator configuration includes shoulder, elbow, ankle, and hand-like `zero` through `six` joint patterns.

<a id="unitree_h1"></a>

#### Unitree H1 (`unitree_h1`)

![unitree_h1](../../_graphics/assets/unitree_h1.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `pelvis` |
| **Sensors** | 3 |
| **Action Modes** | default: `joint_pos` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| legs | `.*_hip_yaw`, `.*_hip_roll`, `.*_hip_pitch`, `.*_knee`, `torso` | 300 N·m | 100 rad/s | 150–200 | 5 |
| feet | `.*_ankle` | 100 N·m | 100 rad/s | 20 | 4 |
| arms | `.*_shoulder_pitch`, `.*_shoulder_roll`, `.*_shoulder_yaw`, `.*_elbow` | 300 N·m | 100 rad/s | 40 | 10 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `pelvis` |
| contacts_feet | Contact Sensor | `(left\|right)_ankle_link` |
| cam_front | Camera | `pelvis/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*_hip_yaw` | 0.0 rad |
| `.*_hip_roll` | 0.0 rad |
| `.*_hip_pitch` | -0.28 rad (-16.042818 deg) |
| `.*_knee` | 0.79 rad (45.263666 deg) |
| `.*_ankle` | -0.52 rad (-29.793805 deg) |
| torso | 0.0 rad |
| `.*_shoulder_pitch` | 0.28 rad (16.042818 deg) |
| `.*_shoulder_roll` | 0.0 rad |
| `.*_shoulder_yaw` | 0.0 rad |
| `.*_elbow` | 0.52 rad (29.793805 deg) |

</details>
