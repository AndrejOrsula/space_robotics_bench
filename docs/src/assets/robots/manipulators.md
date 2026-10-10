# Robot Arms & Manipulators

Serial and dual-arm manipulators registered in the Space Robotics Bench for dexterous manipulation, assembly, and maintenance across terrestrial and space environments.

![Overview](../../_graphics/assets/_collage_manipulators.webp)

## Franka

<a id="franka"></a>

#### Franka (`franka`)

> 7-DoF torque-controlled research arm from Franka Robotics

![franka](../../_graphics/assets/franka.webp)

| | |
|---|---|
| **DoF** | 7 |
| **End Effector** | FrankaHand |
| **Base Frame** | `panda_link0` |
| **Flange Frame** | `panda_link7` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |
| **Source** | SRB |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| panda_shoulder | `panda_joint[1-4]` | 87 N·m | 2.175 rad/s | 4000 | 800 |
| panda_forearm | `panda_joint[5-7]` | 12 N·m | 2.61 rad/s | 4000 | 800 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `panda_link0/camera_base` |
| cam_wrist | Camera | `panda_link7/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| panda_joint1 | 0.0 rad |
| panda_joint2 | 0.0 rad |
| panda_joint3 | 0.0 rad |
| panda_joint4 | -1.570796 rad (-90 deg) |
| panda_joint5 | 0.0 rad |
| panda_joint6 | 1.570796 rad (90 deg) |
| panda_joint7 | 0.785398 rad (45 deg) |

</details>

## Kinova

<a id="kinova_gen3n7"></a>

#### Kinova Gen3N7 (`kinova_gen3n7`)

> 7-DoF lightweight research arm from Kinova Robotics

![kinova_gen3n7](../../_graphics/assets/kinova_gen3n7.webp)

| | |
|---|---|
| **DoF** | 7 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `end_effector_link` |
| **Sensors** | 3 |
| **Action Modes** | 6 (default: `diff_ik`) |
| **Source** | Isaac Lab |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `smooth_osc` | Smooth Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `joint_[1-7]` | 540–2340 N·m | 10 rad/s | 500 | 200 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `end_effector_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| joint_1 | 0.0 rad |
| joint_2 | 0.785398 rad (45 deg) |
| joint_3 | 0.0 rad |
| joint_4 | 0.785398 rad (45 deg) |
| joint_5 | 0.0 rad |
| joint_6 | 1.570796 rad (90 deg) |
| joint_7 | 0.0 rad |

</details>

<a id="kinova_j2n6s"></a>

#### Kinova J2N6S (`kinova_j2n6s`)

![kinova_j2n6s](../../_graphics/assets/kinova_j2n6s.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | Kinova300 |
| **Base Frame** | `j2n6s_link_base` |
| **Flange Frame** | `j2n6s_link_6` |
| **Sensors** | 3 |
| **Action Modes** | 2 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `j2n6s_joint_[1-6]` | 20–80 N·m | 100 rad/s | 1500–4000 | 500–1000 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `j2n6s_link_base/camera_base` |
| cam_wrist | Camera | `j2n6s_link_6/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| j2n6s_joint_1 | 0.0 rad |
| j2n6s_joint_2 | 2.617994 rad (150 deg) |
| j2n6s_joint_3 | 5.759587 rad (330 deg) |
| j2n6s_joint_4 | -1.570796 rad (-90 deg) |
| j2n6s_joint_5 | 0.0 rad |
| j2n6s_joint_6 | 0.0 rad |

</details>

<a id="kinova_j2n7s"></a>

#### Kinova J2N7S (`kinova_j2n7s`)

![kinova_j2n7s](../../_graphics/assets/kinova_j2n7s.webp)

| | |
|---|---|
| **DoF** | 7 |
| **End Effector** | Kinova300 |
| **Base Frame** | `j2n7s_link_base` |
| **Flange Frame** | `j2n7s_link_7` |
| **Sensors** | 3 |
| **Action Modes** | 2 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `j2n7s_joint_[1-7]` | 20–80 N·m | 100 rad/s | 1000–4000 | 500–1000 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `j2n7s_link_base/camera_base` |
| cam_wrist | Camera | `j2n7s_link_7/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| j2n7s_joint_1 | 0.0 rad |
| j2n7s_joint_2 | 2.617994 rad (150 deg) |
| j2n7s_joint_3 | 0.0 rad |
| j2n7s_joint_4 | 0.523599 rad (30 deg) |
| j2n7s_joint_5 | 1.570796 rad (90 deg) |
| j2n7s_joint_6 | 0.0 rad |
| j2n7s_joint_7 | 0.0 rad |

</details>

## Multi-Arm

<a id="levion_arms"></a>

#### Levion Arms (`levion_arms`)

![levion_arms](../../_graphics/assets/levion_arms.webp)

| | |
|---|---|
| **DoF** | 4 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Sensors** | 0 |
| **Action Modes** | default: `dual_arm_joint_pos_abs` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `dual_arm_joint_pos_abs` * | Joint Position To Limits | 4 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| left_arm | `left_shoulder_joint`, `left_elbow_joint` | 15 N·m | – | 400 | 80 |
| right_arm | `right_shoulder_joint`, `right_elbow_joint` | 15 N·m | – | 400 | 80 |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*shoulder_joint` | 0.0 rad |
| right_elbow_joint | 0.4 rad (22.918312 deg) |
| left_elbow_joint | -0.4 rad (-22.918312 deg) |

</details>

<a id="zeroglab_arms"></a>

#### Zeroglab Arms (`zeroglab_arms`)

![zeroglab_arms](../../_graphics/assets/zeroglab_arms.webp)

| | |
|---|---|
| **DoF** | 7 |
| **End Effector** | None |
| **Base Frame** | `_virtual_world` |
| **Sensors** | 0 |
| **Action Modes** | default: `dual_arm_osc` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `dual_arm_osc` * | Operational Space Controller | 24 | `arm_ceiling`: pos=0.1, ori=0.1, stiffness=120.0; `arm_wall`: pos=0.1, ori=0.1, stiffness=120.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| rail | `rail_joint` | 400 N·m | 0.2 rad/s | 3000 | 2500 |
| arm | `.*shoulder.*`, `.*elbow.*`, `.*wrist.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| rail_joint | 2 rad (114.591559 deg) |
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

Two arms (ceiling and wall) share this configuration, each with a `rail_joint` and six arm joints. DoF, actuators, and initial joints are per arm; `dual_arm_osc` drives both arms with 12 dimensions each.

## SO-ARM100

<a id="so_arm100_d5"></a>

#### So Arm100 D5 (`so_arm100_d5`)

> 5-DoF low-cost open-source motor-driven arm

![so_arm100_d5](../../_graphics/assets/so_arm100_d5.webp)

| | |
|---|---|
| **DoF** | 5 |
| **End Effector** | SOArm100Gripper |
| **Base Frame** | `Base` |
| **Flange Frame** | `Fixed_Gripper` |
| **Sensors** | 3 |
| **Action Modes** | 3 (default: `diff_ik`) |
| **Source** | SRB |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.05, ori=0.05, stiffness=120.0 |
| `joint_pos_rel` | Relative Joint Position | 5 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 10 N·m | 10 rad/s | 100 | 20 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `Base/camera_base` |
| cam_wrist | Camera | `Fixed_Gripper/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| Shoulder_Rotation | 0.0 rad |
| Shoulder_Pitch | -0.261799 rad (-15 deg) |
| Elbow | 0.349066 rad (20 deg) |
| Wrist_Pitch | 0.087266 rad (5 deg) |
| Wrist_Roll | 0.0 rad |

</details>

<a id="so_arm100_d7"></a>

#### So Arm100 D7 (`so_arm100_d7`)

> 7-DoF low-cost open-source motor-driven arm

![so_arm100_d7](../../_graphics/assets/so_arm100_d7.webp)

| | |
|---|---|
| **DoF** | 7 |
| **End Effector** | None |
| **Base Frame** | `Base` |
| **Flange Frame** | `End_Servo` |
| **Sensors** | 3 |
| **Action Modes** | 2 (default: `diff_ik`) |
| **Source** | SRB |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 10 N·m | 10 rad/s | 100 | 20 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `Base/camera_base` |
| cam_wrist | Camera | `End_Servo/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| Shoulder_Pitch | 2.094395 rad (120 deg) |
| Shoulder_Yaw | 0.0 rad |
| Humeral_Rotation | 0.0 rad |
| Elbow | -1.570796 rad (-90 deg) |
| Wrist_Roll | 0.0 rad |
| Wrist_Yaw | 0.0 rad |
| Wrist_Pitch | -1.570796 rad (-90 deg) |

</details>

## Space

<a id="canadarm3"></a>

#### Canadarm3 (`canadarm3`)

> Large-scale space manipulator for the Lunar Gateway station

![canadarm3](../../_graphics/assets/canadarm3.webp)

| | |
|---|---|
| **DoF** | 7 |
| **End Effector** | None |
| **Base Frame** | `canadarm3_large_0` |
| **Flange Frame** | `canadarm3_large_7` |
| **Sensors** | 3 |
| **Action Modes** | 2 (default: `diff_ik`) |
| **Source** | SRB |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| joints | `canadarm3_large_joint_[1-7]` | 2500 N·m | 5 rad/s | 40000 | 25000 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `canadarm3_large_0/camera_base` |
| cam_wrist | Camera | `canadarm3_large_7/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| canadarm3_large_joint_1 | 0.872665 rad (50 deg) |
| canadarm3_large_joint_2 | 0.0 rad |
| canadarm3_large_joint_3 | 0.959931 rad (55 deg) |
| canadarm3_large_joint_4 | 1.308997 rad (75 deg) |
| canadarm3_large_joint_5 | -0.523599 rad (-30 deg) |
| canadarm3_large_joint_6 | 0.0 rad |
| canadarm3_large_joint_7 | 0.0 rad |

</details>

<a id="vispa"></a>

#### Vispa (`vispa`)

> Space manipulator designed for planetary surface operations

![vispa](../../_graphics/assets/vispa.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `link0` |
| **Flange Frame** | `link6` |
| **Sensors** | 3 |
| **Action Modes** | 3 (default: `diff_ik`) |
| **Source** | SRB |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| joints | `joint[1-6]` | 500 N·m | 5 rad/s | 2500 | 500 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `link0/camera_base` |
| cam_wrist | Camera | `link6/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| joint1 | 0.0 rad |
| joint2 | -0.523599 rad (-30 deg) |
| joint3 | 2.094395 rad (120 deg) |
| joint4 | 0.0 rad |
| joint5 | 1.570796 rad (90 deg) |
| joint6 | 0.0 rad |

</details>

## Unitree

<a id="unitree_z1"></a>

#### Unitree Z1 (`unitree_z1`)

> Compact lightweight arm from Unitree Robotics

![unitree_z1](../../_graphics/assets/unitree_z1.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `link00` |
| **Flange Frame** | `link06` |
| **Sensors** | 3 |
| **Action Modes** | 2 (default: `diff_ik`) |
| **Source** | Isaac Lab |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `link00/camera_base` |
| cam_wrist | Camera | `link06/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| joint1 | 0.0 rad |
| joint2 | 1.570796 rad (90 deg) |
| joint3 | -0.785398 rad (-45 deg) |
| joint4 | 0.785398 rad (45 deg) |
| joint5 | 0.0 rad |
| joint6 | 0.0 rad |

</details>

## Universal Robots

<a id="random_ur_manipulator"></a>

#### Random Ur Manipulator (`random_ur_manipulator`)

![random_ur_manipulator](../../_graphics/assets/random_ur_manipulator.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur10"></a>

#### Ur10 (`ur10`)

> Heavy-duty collaborative arm from Universal Robots

![ur10](../../_graphics/assets/ur10.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `ee_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |
| **Source** | Isaac Lab |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `ee_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur10e"></a>

#### Ur10E (`ur10e`)

![ur10e](../../_graphics/assets/ur10e.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur16e"></a>

#### Ur16E (`ur16e`)

![ur16e](../../_graphics/assets/ur16e.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur20"></a>

#### Ur20 (`ur20`)

![ur20](../../_graphics/assets/ur20.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur3"></a>

#### Ur3 (`ur3`)

> Compact collaborative robot arm from Universal Robots

![ur3](../../_graphics/assets/ur3.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |
| **Source** | Isaac Sim |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur30"></a>

#### Ur30 (`ur30`)

![ur30](../../_graphics/assets/ur30.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur3e"></a>

#### Ur3E (`ur3e`)

![ur3e](../../_graphics/assets/ur3e.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur5"></a>

#### Ur5 (`ur5`)

> Versatile mid-range collaborative arm from Universal Robots

![ur5](../../_graphics/assets/ur5.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |
| **Source** | Isaac Sim |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>

<a id="ur5e"></a>

#### Ur5E (`ur5e`)

![ur5e](../../_graphics/assets/ur5e.webp)

| | |
|---|---|
| **DoF** | 6 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Flange Frame** | `wrist_3_link` |
| **Sensors** | 3 |
| **Action Modes** | 5 (default: `diff_ik`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| arm | `.*` | 87 N·m | 100 rad/s | 800 | 40 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| contacts_robot | Contact Sensor | `.*` |
| cam_base | Camera | `base_link/camera_base` |
| cam_wrist | Camera | `wrist_3_link/camera_wrist` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| shoulder_pan_joint | 0.0 rad |
| shoulder_lift_joint | -1.570796 rad (-90 deg) |
| elbow_joint | 1.570796 rad (90 deg) |
| wrist_1_joint | -1.570796 rad (-90 deg) |
| wrist_2_joint | -1.570796 rad (-90 deg) |
| wrist_3_joint | -1.570796 rad (-90 deg) |

</details>
