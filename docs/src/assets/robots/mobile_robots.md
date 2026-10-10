# Mobile Robots

Mobile platforms registered in the Space Robotics Bench include wheeled rovers, legged robots, aerial vehicles, landers, spacecraft, and a surface vessel for navigation and locomotion tasks.

![Overview](../../_graphics/assets/_collage_mobile_robots.webp)

## Landers

<a id="apollo_lander"></a>

#### Apollo Lander (`apollo_lander`)

![apollo_lander](../../_graphics/assets/apollo_lander.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 4280 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 17 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

<a id="peregrine_lander"></a>

#### Peregrine Lander (`peregrine_lander`)

![peregrine_lander](../../_graphics/assets/peregrine_lander.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 825 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 17 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

<a id="random_lander"></a>

#### Random Lander (`random_lander`)

![random_lander](../../_graphics/assets/random_lander.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 3 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

<a id="resilience_lander"></a>

#### Resilience Lander (`resilience_lander`)

![resilience_lander](../../_graphics/assets/resilience_lander.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 340 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 9 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

<a id="vikram_lander"></a>

#### Vikram Lander (`vikram_lander`)

![vikram_lander](../../_graphics/assets/vikram_lander.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 626 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 13 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

## Legged Robots

<a id="anymal_c"></a>

#### Anymal C (`anymal_c`)

![anymal_c](../../_graphics/assets/anymal_c.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
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
| legs | `.*HAA`, `.*HFE`, `.*KFE` | 80 N·m | 7.5 rad/s | – | – |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| contacts_feet | Contact Sensor | `.*FOOT` |
| cam_front | Camera | `base/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*HAA` | 0.0 rad |
| `.*F_HFE` | 0.4 rad (22.918312 deg) |
| `.*H_HFE` | -0.4 rad (-22.918312 deg) |
| `.*F_KFE` | -0.8 rad (-45.836624 deg) |
| `.*H_KFE` | 0.8 rad (45.836624 deg) |

</details>

<a id="anymal_d"></a>

#### Anymal D (`anymal_d`)

![anymal_d](../../_graphics/assets/anymal_d.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
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
| legs | `.*HAA`, `.*HFE`, `.*KFE` | 80 N·m | 7.5 rad/s | – | – |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| contacts_feet | Contact Sensor | `.*FOOT` |
| cam_front | Camera | `base/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*HAA` | 0.0 rad |
| `.*F_HFE` | 0.4 rad (22.918312 deg) |
| `.*H_HFE` | -0.4 rad (-22.918312 deg) |
| `.*F_KFE` | -0.8 rad (-45.836624 deg) |
| `.*H_KFE` | 0.8 rad (45.836624 deg) |

</details>

<a id="cassie"></a>

#### Cassie (`cassie`)

![cassie](../../_graphics/assets/cassie.webp)

| | |
|---|---|
| **DoF** | 12 |
| **End Effector** | None |
| **Base Frame** | `pelvis` |
| **Sensors** | 3 |
| **Action Modes** | default: `joint_pos` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `joint_pos` * | Joint Position To Limits | 12 | 1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| legs | `hip_.*`, `thigh_.*`, `ankle_.*` | 200 N·m | 10 rad/s | 100–200 | 3–6 |
| toes | `toe_.*` | 20 N·m | 10 rad/s | 20 | 1 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `pelvis` |
| contacts_feet | Contact Sensor | `(left\|right)_toe` |
| cam_front | Camera | `pelvis/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| hip_abduction_left | 0.1 rad (5.729578 deg) |
| hip_rotation_left | 0.0 rad |
| hip_flexion_left | 1 rad (57.29578 deg) |
| thigh_joint_left | -1.8 rad (-103.132403 deg) |
| ankle_joint_left | 1.57 rad (89.954374 deg) |
| toe_joint_left | -1.57 rad (-89.954374 deg) |
| hip_abduction_right | -0.1 rad (-5.729578 deg) |
| hip_rotation_right | 0.0 rad |
| hip_flexion_right | 1 rad (57.29578 deg) |
| thigh_joint_right | -1.8 rad (-103.132403 deg) |
| ankle_joint_right | 1.57 rad (89.954374 deg) |
| toe_joint_right | -1.57 rad (-89.954374 deg) |

</details>

<a id="random_anymal_quadruped"></a>

#### Random Anymal Quadruped (`random_anymal_quadruped`)

![random_anymal_quadruped](../../_graphics/assets/random_anymal_quadruped.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
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
| legs | `.*HAA`, `.*HFE`, `.*KFE` | 80 N·m | 7.5 rad/s | – | – |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| contacts_feet | Contact Sensor | `.*FOOT` |
| cam_front | Camera | `base/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*HAA` | 0.0 rad |
| `.*F_HFE` | 0.4 rad (22.918312 deg) |
| `.*H_HFE` | -0.4 rad (-22.918312 deg) |
| `.*F_KFE` | -0.8 rad (-45.836624 deg) |
| `.*H_KFE` | 0.8 rad (45.836624 deg) |

</details>

<a id="random_unitree_quadruped"></a>

#### Random Unitree Quadruped (`random_unitree_quadruped`)

![random_unitree_quadruped](../../_graphics/assets/random_unitree_quadruped.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `trunk` |
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
| base_legs | `.*_hip_joint`, `.*_thigh_joint`, `.*_calf_joint` | 33.5 N·m | 21 rad/s | 25 | 0.5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `trunk` |
| contacts_feet | Contact Sensor | `.*foot` |
| cam_front | Camera | `trunk/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*L_hip_joint` | 0.1 rad (5.729578 deg) |
| `.*R_hip_joint` | -0.1 rad (-5.729578 deg) |
| `F[L,R]_thigh_joint` | 0.8 rad (45.836624 deg) |
| `R[L,R]_thigh_joint` | 1 rad (57.29578 deg) |
| `.*_calf_joint` | -1.5 rad (-85.943669 deg) |

</details>

<a id="spot"></a>

#### Spot (`spot`)

![spot](../../_graphics/assets/spot.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `body` |
| **Sensors** | 3 |
| **Action Modes** | default: `joint_pos` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `joint_pos` * | Joint Position | runtime-resolved | 0.2 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| spot_hip | `.*_h[xy]` | 45 N·m | – | 60 | 1.5 |
| spot_knee | `.*_kn` | – | – | 60 | 1.5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `body` |
| contacts_feet | Contact Sensor | `.*foot` |
| cam_front | Camera | `body/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `[fh]l_hx` | 0.1 rad (5.729578 deg) |
| `[fh]r_hx` | -0.1 rad (-5.729578 deg) |
| `f[rl]_hy` | 0.9 rad (51.566202 deg) |
| `h[rl]_hy` | 1.1 rad (63.025357 deg) |
| `.*_kn` | -1.5 rad (-85.943669 deg) |

</details>

<a id="unitree_a1"></a>

#### Unitree A1 (`unitree_a1`)

![unitree_a1](../../_graphics/assets/unitree_a1.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `trunk` |
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
| base_legs | `.*_hip_joint`, `.*_thigh_joint`, `.*_calf_joint` | 33.5 N·m | 21 rad/s | 25 | 0.5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `trunk` |
| contacts_feet | Contact Sensor | `.*foot` |
| cam_front | Camera | `trunk/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*L_hip_joint` | 0.1 rad (5.729578 deg) |
| `.*R_hip_joint` | -0.1 rad (-5.729578 deg) |
| `F[L,R]_thigh_joint` | 0.8 rad (45.836624 deg) |
| `R[L,R]_thigh_joint` | 1 rad (57.29578 deg) |
| `.*_calf_joint` | -1.5 rad (-85.943669 deg) |

</details>

<a id="unitree_go1"></a>

#### Unitree Go1 (`unitree_go1`)

![unitree_go1](../../_graphics/assets/unitree_go1.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `trunk` |
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
| base_legs | `.*_hip_joint`, `.*_thigh_joint`, `.*_calf_joint` | 23.7 N·m | 30 rad/s | – | – |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `trunk` |
| contacts_feet | Contact Sensor | `.*foot` |
| cam_front | Camera | `trunk/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*L_hip_joint` | 0.1 rad (5.729578 deg) |
| `.*R_hip_joint` | -0.1 rad (-5.729578 deg) |
| `F[L,R]_thigh_joint` | 0.8 rad (45.836624 deg) |
| `R[L,R]_thigh_joint` | 1 rad (57.29578 deg) |
| `.*_calf_joint` | -1.5 rad (-85.943669 deg) |

</details>

<a id="unitree_go2"></a>

#### Unitree Go2 (`unitree_go2`)

![unitree_go2](../../_graphics/assets/unitree_go2.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
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
| base_legs | `.*_hip_joint`, `.*_thigh_joint`, `.*_calf_joint` | 33.5 N·m | 21 rad/s | 25 | 0.5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| contacts_feet | Contact Sensor | `.*foot` |
| cam_front | Camera | `base/camera_front` |

</details>

<details><summary>Initial Joint Configuration</summary>

| Joint | Position |
|---|---|
| `.*L_hip_joint` | 0.1 rad (5.729578 deg) |
| `.*R_hip_joint` | -0.1 rad (-5.729578 deg) |
| `F[L,R]_thigh_joint` | 0.8 rad (45.836624 deg) |
| `R[L,R]_thigh_joint` | 1 rad (57.29578 deg) |
| `.*_calf_joint` | -1.5 rad (-85.943669 deg) |

</details>

## Multicopters

<a id="crazyflie"></a>

#### Crazyflie (`crazyflie`)

![crazyflie](../../_graphics/assets/crazyflie.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `body` |
| **Sensors** | 2 |
| **Action Modes** | default: `body_acc` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `body_acc` * | Multicopter Body Acceleration | 4 | 0.5 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| dummy | `.*` | – | – | 0 | 0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `body` |
| cam_onboard | Camera | `body/camera_onboard` |

</details>

<a id="ingenuity"></a>

#### Ingenuity (`ingenuity`)

![ingenuity](../../_graphics/assets/ingenuity.webp)

| | |
|---|---|
| **DoF** | 2 |
| **End Effector** | None |
| **Base Frame** | `body` |
| **Sensors** | 3 |
| **Action Modes** | default: `body_acc` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `body_acc` * | Multicopter Body Acceleration | 4 | 0.5 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| rotors | `rotor_joint_[1-2]` | 7.5 N·m | 261.799388 rad/s | 0 | 1000 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `body` |
| cam_onboard | Camera | `body/camera_onboard` |
| altimeter | Heightmap Sensor | `body` |

</details>

## Orbital Robots

<a id="cubesat"></a>

#### Cubesat (`cubesat`)

![cubesat](../../_graphics/assets/cubesat.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `cubesat` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 12 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `cubesat` |
| cam_onboard | Camera | `cubesat/camera_onboard` |

</details>

<a id="floating_platform"></a>

#### Floating Platform (`floating_platform`)

![floating_platform](../../_graphics/assets/floating_platform.webp)

| | |
|---|---|
| **DoF** | 1 |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Sensors** | 2 |
| **Action Modes** | 2 (default: `thrust_only`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust_only` * | Thrust | 8 | 1.0 |
| `thrust_and_reaction` | Thrust | 9 | `thrust`=1.0; `reaction_wheel_torque`=0.05 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| reaction_wheel | `rw_revolute_joint` | 1 N·m | 100 rad/s | 0 | 0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base_link` |
| cam_onboard | Camera | `base_link/camera_onboard` |

</details>

<a id="gateway"></a>

#### Gateway (`gateway`)

![gateway](../../_graphics/assets/gateway.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `gateway` |
| **Sensors** | 2 |
| **Action Modes** | default: `body_acc` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `body_acc` * | Body Acceleration | 6 | 2.5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `gateway` |
| cam_onboard | Camera | `gateway/camera_onboard` |

</details>

<a id="int_ball1"></a>

#### Int Ball1 (`int_ball1`)

![int_ball1](../../_graphics/assets/int_ball1.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 1 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 6 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base_link` |
| cam_onboard | Camera | `base_link/camera_onboard` |

</details>

<a id="int_ball2"></a>

#### Int Ball2 (`int_ball2`)

![int_ball2](../../_graphics/assets/int_ball2.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 1.5 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 6 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base_link` |
| cam_onboard | Camera | `base_link/camera_onboard` |

</details>

<a id="iss"></a>

#### Iss (`iss`)

![iss](../../_graphics/assets/iss.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `body_acc` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `body_acc` * | Body Acceleration | 6 | 2.5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

<a id="satellite_mockup"></a>

#### Satellite Mockup (`satellite_mockup`)

![satellite_mockup](../../_graphics/assets/satellite_mockup.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `satellite_mockup` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 12 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `satellite_mockup` |
| cam_onboard | Camera | `satellite_mockup/camera_onboard` |

</details>

<a id="starship"></a>

#### Starship (`starship`)

![starship](../../_graphics/assets/starship.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 100000 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 18 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

<a id="super_heavy"></a>

#### Super Heavy (`super_heavy`)

![super_heavy](../../_graphics/assets/super_heavy.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |
| **Mass** | 200000 kg |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 59 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

<a id="venus_express"></a>

#### Venus Express (`venus_express`)

![venus_express](../../_graphics/assets/venus_express.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 19 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base` |
| cam_onboard | Camera | `base/camera_onboard` |

</details>

## Surface Vehicles

<a id="heron"></a>

#### Heron (`heron`)

![heron](../../_graphics/assets/heron.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `base_link` |
| **Sensors** | 2 |
| **Action Modes** | default: `thrust` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `thrust` * | Thrust | 2 | 1.0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `base_link` |
| cam_onboard | Camera | `base_link/camera_onboard` |

</details>

## Wheeled Robots

<a id="cadre"></a>

#### Cadre (`cadre`)

![cadre](../../_graphics/assets/cadre.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `chassis` |
| **Sensors** | 3 |
| **Action Modes** | default: `drive` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.4, ang=0.7853981633974483 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| wheel_joints | `wheel_joint_.*` | 250 N·m | 6.28 rad/s | 0 | 2000 |
| suspension_joints | `suspension_joint.*` | 500 N·m | 0.25 rad/s | 400 | 5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `chassis` |
| contacts_wheels | Contact Sensor | `.*wheel.*` |
| cam_front | Camera | `chassis/camera_front` |

</details>

<a id="husky"></a>

#### Husky (`husky`)

![husky](../../_graphics/assets/husky.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `chassis` |
| **Sensors** | 3 |
| **Action Modes** | default: `drive` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.1, ang=0.7853981633974483 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| wheel_joints | `wheel_joint_.*` | 250 N·m | 6.28 rad/s | 0 | 2000 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `chassis` |
| contacts_wheels | Contact Sensor | `.*wheel.*` |
| cam_front | Camera | `chassis/camera_front` |

</details>

<a id="leo_rover"></a>

#### Leo Rover (`leo_rover`)

![leo_rover](../../_graphics/assets/leo_rover.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `chassis` |
| **Sensors** | 3 |
| **Action Modes** | default: `drive` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.4, ang=1.0471975511965976 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| drive | `wheel_joint_.*` | 80 N·m | 30 rad/s | 0 | 5000 |
| rocker | `rocker_joint_.*` | 500 N·m | 2 rad/s | 2 | 0.5 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `chassis` |
| contacts_wheels | Contact Sensor | `.*wheel.*` |
| cam_front | Camera | `chassis/camera_front` |

</details>

<a id="mars_rover"></a>

#### Mars Rover (`mars_rover`)

![mars_rover](../../_graphics/assets/mars_rover.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `Body` |
| **Sensors** | 3 |
| **Action Modes** | default: `drive` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.5, ang=0.7853981633974483 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| base_steering | `.*Steer.*` | 12 N·m | 6 rad/s | 8000 | 1000 |
| base_drive | `.*Drive.*` | 12 N·m | 6 rad/s | 100 | 4000 |
| passive_joints_boogie | `.*RevoluteJoint` | 0 N·m | 15 rad/s | 0 | 0 |
| passive_joints_rocker | `.*Rocker.*` | 0 N·m | 15 rad/s | 0 | 0 |
| passive_joints_differential | `.*Differential.*` | 0 N·m | 15 rad/s | 0 | 0 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `Body` |
| contacts_wheels | Contact Sensor | `.*[Dd]rive.*` |
| cam_front | Camera | `Body/camera_front` |

</details>

<a id="nifty"></a>

#### Nifty (`nifty`)

![nifty](../../_graphics/assets/nifty.webp)

| | |
|---|---|
| **DoF** | 12 |
| **End Effector** | None |
| **Base Frame** | `chassis` |
| **Sensors** | 3 |
| **Action Modes** | default: `nifty` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `nifty` * | Joint Velocity | 12 | `joint_vel_wheel_drive`=1.5707963267948966; `joint_pos_steer`=0.7853981633974483; `joint_pos_suspension_yaw`=0.5235987755982988; `joint_pos_suspension_roll`=0.2617993877991494 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| wheel_joints | `wheel_joint_fl`, `wheel_joint_fr`, `wheel_joint_rl`, `wheel_joint_rr` | 80 N·m | 1.570796 rad/s | 0 | 80 |
| steer_joints | `steer_joint_fl`, `steer_joint_fr`, `steer_joint_rl`, `steer_joint_rr` | 200 N·m | 0.523599 rad/s | 100 | 50 |
| suspension_joint_yaw | `suspension_joint_yaw_f`, `suspension_joint_yaw_r` | 200 N·m | 0.261799 rad/s | 1250 | 750 |
| suspension_joint_roll | `suspension_joint_roll_f`, `suspension_joint_roll_r` | 200 N·m | 0.261799 rad/s | 15000 | 5000 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `chassis` |
| contacts_wheels | Contact Sensor | `.*wheel.*` |
| cam_front | Camera | `chassis` |

</details>

<a id="open_source_rover"></a>

#### Open Source Rover (`open_source_rover`)

![open_source_rover](../../_graphics/assets/open_source_rover.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `chassis` |
| **Sensors** | 3 |
| **Action Modes** | default: `drive` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.2, ang=0.3490658503988659 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| joint_wheel | `joint_wheel_.*` | 50 N·m | 40 rad/s | 0 | 2000 |
| joints_steer | `joint_steer_.*` | 500 N·m | 0.5 rad/s | 100 | 80 |
| joints_rocker | `joint_rocker_.*` | 1500 N·m | 0.3 rad/s | 25 | 2 |
| joints_bogie | `joint_bogie_.*` | 750 N·m | 0.6 rad/s | 1 | 4 |
| joint_linkage | `joint_linkage` | 25 N·m | 0.2 rad/s | 0.05 | 0.1 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `chassis` |
| contacts_wheels | Contact Sensor | `.*wheel.*` |
| cam_front | Camera | `chassis/camera_front` |

</details>

<a id="perseverance"></a>

#### Perseverance (`perseverance`)

![perseverance](../../_graphics/assets/perseverance.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `body` |
| **Sensors** | 3 |
| **Action Modes** | default: `drive` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.7, ang=1.3089969389957472 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| drive_joints | `drive_joint.*` | 150 N·m | 40 rad/s | 0 | 25000 |
| steer_joints | `steer_joint.*` | 400 N·m | 2 rad/s | 500 | 200 |
| rocker_joints | `suspension_joint_rocker.*` | 2500 N·m | 5 rad/s | 4000 | 400 |
| bogie_joints | `suspension_joint_bogie.*` | 500 N·m | 4 rad/s | 200 | 25 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `body` |
| contacts_wheels | Contact Sensor | `.*wheel.*` |
| cam_front | Camera | `body/camera_front` |

</details>

<a id="pragyan"></a>

#### Pragyan (`pragyan`)

![pragyan](../../_graphics/assets/pragyan.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `body` |
| **Sensors** | 3 |
| **Action Modes** | default: `drive` |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.35, ang=1.3962634015954636 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| drive_joints | `wheel_drive_joint_.*` | 150 N·m | 40 rad/s | 0 | 5000 |
| rocker_joints | `rocker_joint_.*` | 2500 N·m | 5 rad/s | 1000 | 400 |
| bogie_joints | `boogie_joint_.*` | 500 N·m | 4 rad/s | 250 | 200 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `body` |
| contacts_wheels | Contact Sensor | `.*wheel.*` |
| cam_front | Camera | `body/camera_front` |

</details>

<a id="raph_rover"></a>

#### Raph Rover (`raph_rover`)

![raph_rover](../../_graphics/assets/raph_rover.webp)

| | |
|---|---|
| **DoF** | runtime-resolved |
| **End Effector** | None |
| **Base Frame** | `chassis` |
| **Sensors** | 3 |
| **Action Modes** | 2 (default: `drive`) |

<details><summary>Action Modes</summary>

| Mode | Controller | Dims | Scale |
|---|---|---|---|
| `drive` * | Wheeled Drive | 2 | lin=0.4, ang=1.0471975511965976 |
| `drive_filtered` | Filtered Wheeled Drive | 2 | lin=0.4, ang=1.0 |

</details>

<details><summary>Actuators</summary>

| Group | Joints | Effort | Velocity | Stiffness | Damping |
|---|---|---|---|---|---|
| drive | `drive_joint_.*` | 80 N·m | 30 rad/s | 0 | 500 |
| steer | `steer_joint_.*` | 500 N·m | 2 rad/s | 400 | 50 |

</details>

<details><summary>Sensors</summary>

| Name | Type | Location |
|---|---|---|
| imu_robot | Imu | `chassis` |
| contacts_wheels | Contact Sensor | `wheel.*` |
| cam_front | Camera | `chassis/camera_front` |

</details>
