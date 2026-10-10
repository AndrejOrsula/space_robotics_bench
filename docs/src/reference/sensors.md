# Sensor Catalog

Each robot ships with configured sensors. **Location** is the prim-path regex relative to the robot prim where a sensor is attached. Types: **Camera** (RGB / depth images), **Imu** (linear acceleration and angular velocity), **Contact Sensor** (per-link contact state), and **Heightmap Sensor** (ray-cast elevation grid).

| Robot | Sensor Name | Type | Location |
|---|---|---|---|
| Canadarm3 | contacts_robot | Contact Sensor | `.*` |
| Canadarm3 | cam_base | Camera | `canadarm3_large_0/camera_base` |
| Canadarm3 | cam_wrist | Camera | `canadarm3_large_7/camera_wrist` |
| Franka | contacts_robot | Contact Sensor | `.*` |
| Franka | cam_base | Camera | `panda_link0/camera_base` |
| Franka | cam_wrist | Camera | `panda_link7/camera_wrist` |
| Kinova J2N6S | contacts_robot | Contact Sensor | `.*` |
| Kinova J2N6S | cam_base | Camera | `j2n6s_link_base/camera_base` |
| Kinova J2N6S | cam_wrist | Camera | `j2n6s_link_6/camera_wrist` |
| Kinova J2N7S | contacts_robot | Contact Sensor | `.*` |
| Kinova J2N7S | cam_base | Camera | `j2n7s_link_base/camera_base` |
| Kinova J2N7S | cam_wrist | Camera | `j2n7s_link_7/camera_wrist` |
| Kinova Gen3N7 | contacts_robot | Contact Sensor | `.*` |
| Kinova Gen3N7 | cam_base | Camera | `base_link/camera_base` |
| Kinova Gen3N7 | cam_wrist | Camera | `end_effector_link/camera_wrist` |
| So Arm100 D5 | contacts_robot | Contact Sensor | `.*` |
| So Arm100 D5 | cam_base | Camera | `Base/camera_base` |
| So Arm100 D5 | cam_wrist | Camera | `Fixed_Gripper/camera_wrist` |
| So Arm100 D7 | contacts_robot | Contact Sensor | `.*` |
| So Arm100 D7 | cam_base | Camera | `Base/camera_base` |
| So Arm100 D7 | cam_wrist | Camera | `End_Servo/camera_wrist` |
| Unitree Z1 | contacts_robot | Contact Sensor | `.*` |
| Unitree Z1 | cam_base | Camera | `link00/camera_base` |
| Unitree Z1 | cam_wrist | Camera | `link06/camera_wrist` |
| Ur3 | contacts_robot | Contact Sensor | `.*` |
| Ur3 | cam_base | Camera | `base_link/camera_base` |
| Ur3 | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Ur3E | contacts_robot | Contact Sensor | `.*` |
| Ur3E | cam_base | Camera | `base_link/camera_base` |
| Ur3E | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Ur5 | contacts_robot | Contact Sensor | `.*` |
| Ur5 | cam_base | Camera | `base_link/camera_base` |
| Ur5 | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Ur5E | contacts_robot | Contact Sensor | `.*` |
| Ur5E | cam_base | Camera | `base_link/camera_base` |
| Ur5E | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Ur10 | contacts_robot | Contact Sensor | `.*` |
| Ur10 | cam_base | Camera | `base_link/camera_base` |
| Ur10 | cam_wrist | Camera | `ee_link/camera_wrist` |
| Ur10E | contacts_robot | Contact Sensor | `.*` |
| Ur10E | cam_base | Camera | `base_link/camera_base` |
| Ur10E | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Ur16E | contacts_robot | Contact Sensor | `.*` |
| Ur16E | cam_base | Camera | `base_link/camera_base` |
| Ur16E | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Ur20 | contacts_robot | Contact Sensor | `.*` |
| Ur20 | cam_base | Camera | `base_link/camera_base` |
| Ur20 | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Ur30 | contacts_robot | Contact Sensor | `.*` |
| Ur30 | cam_base | Camera | `base_link/camera_base` |
| Ur30 | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Random Ur Manipulator | contacts_robot | Contact Sensor | `.*` |
| Random Ur Manipulator | cam_base | Camera | `base_link/camera_base` |
| Random Ur Manipulator | cam_wrist | Camera | `wrist_3_link/camera_wrist` |
| Vispa | contacts_robot | Contact Sensor | `.*` |
| Vispa | cam_base | Camera | `link0/camera_base` |
| Vispa | cam_wrist | Camera | `link6/camera_wrist` |
| Humanoid21 | imu_robot | Imu | `pelvis` |
| Humanoid21 | contacts_feet | Contact Sensor | `(left\|right)_foot` |
| Humanoid21 | cam_front | Camera | `pelvis/camera_front` |
| Humanoid28 | imu_robot | Imu | `pelvis` |
| Humanoid28 | contacts_feet | Contact Sensor | `(left\|right)_foot` |
| Humanoid28 | cam_front | Camera | `pelvis/camera_front` |
| Unitree H1 | imu_robot | Imu | `pelvis` |
| Unitree H1 | contacts_feet | Contact Sensor | `(left\|right)_ankle_link` |
| Unitree H1 | cam_front | Camera | `pelvis/camera_front` |
| Unitree G1 | imu_robot | Imu | `pelvis` |
| Unitree G1 | contacts_feet | Contact Sensor | `(left\|right)_ankle_roll_link` |
| Unitree G1 | cam_front | Camera | `pelvis/camera_front` |
| Crazyflie | imu_robot | Imu | `body` |
| Crazyflie | cam_onboard | Camera | `body/camera_onboard` |
| Ingenuity | imu_robot | Imu | `body` |
| Ingenuity | cam_onboard | Camera | `body/camera_onboard` |
| Ingenuity | altimeter | Heightmap Sensor | `body` |
| Heron | imu_robot | Imu | `base_link` |
| Heron | cam_onboard | Camera | `base_link/camera_onboard` |
| Anymal C | imu_robot | Imu | `base` |
| Anymal C | contacts_feet | Contact Sensor | `.*FOOT` |
| Anymal C | cam_front | Camera | `base/camera_front` |
| Anymal D | imu_robot | Imu | `base` |
| Anymal D | contacts_feet | Contact Sensor | `.*FOOT` |
| Anymal D | cam_front | Camera | `base/camera_front` |
| Random Anymal Quadruped | imu_robot | Imu | `base` |
| Random Anymal Quadruped | contacts_feet | Contact Sensor | `.*FOOT` |
| Random Anymal Quadruped | cam_front | Camera | `base/camera_front` |
| Cadre | imu_robot | Imu | `chassis` |
| Cadre | contacts_wheels | Contact Sensor | `.*wheel.*` |
| Cadre | cam_front | Camera | `chassis/camera_front` |
| Cassie | imu_robot | Imu | `pelvis` |
| Cassie | contacts_feet | Contact Sensor | `(left\|right)_toe` |
| Cassie | cam_front | Camera | `pelvis/camera_front` |
| Husky | imu_robot | Imu | `chassis` |
| Husky | contacts_wheels | Contact Sensor | `.*wheel.*` |
| Husky | cam_front | Camera | `chassis/camera_front` |
| Leo Rover | imu_robot | Imu | `chassis` |
| Leo Rover | contacts_wheels | Contact Sensor | `.*wheel.*` |
| Leo Rover | cam_front | Camera | `chassis/camera_front` |
| Mars Rover | imu_robot | Imu | `Body` |
| Mars Rover | contacts_wheels | Contact Sensor | `.*[Dd]rive.*` |
| Mars Rover | cam_front | Camera | `Body/camera_front` |
| Nifty | imu_robot | Imu | `chassis` |
| Nifty | contacts_wheels | Contact Sensor | `.*wheel.*` |
| Nifty | cam_front | Camera | `chassis` |
| Open Source Rover | imu_robot | Imu | `chassis` |
| Open Source Rover | contacts_wheels | Contact Sensor | `.*wheel.*` |
| Open Source Rover | cam_front | Camera | `chassis/camera_front` |
| Perseverance | imu_robot | Imu | `body` |
| Perseverance | contacts_wheels | Contact Sensor | `.*wheel.*` |
| Perseverance | cam_front | Camera | `body/camera_front` |
| Pragyan | imu_robot | Imu | `body` |
| Pragyan | contacts_wheels | Contact Sensor | `.*wheel.*` |
| Pragyan | cam_front | Camera | `body/camera_front` |
| Raph Rover | imu_robot | Imu | `chassis` |
| Raph Rover | contacts_wheels | Contact Sensor | `wheel.*` |
| Raph Rover | cam_front | Camera | `chassis/camera_front` |
| Spot | imu_robot | Imu | `body` |
| Spot | contacts_feet | Contact Sensor | `.*foot` |
| Spot | cam_front | Camera | `body/camera_front` |
| Unitree A1 | imu_robot | Imu | `trunk` |
| Unitree A1 | contacts_feet | Contact Sensor | `.*foot` |
| Unitree A1 | cam_front | Camera | `trunk/camera_front` |
| Unitree Go1 | imu_robot | Imu | `trunk` |
| Unitree Go1 | contacts_feet | Contact Sensor | `.*foot` |
| Unitree Go1 | cam_front | Camera | `trunk/camera_front` |
| Unitree Go2 | imu_robot | Imu | `base` |
| Unitree Go2 | contacts_feet | Contact Sensor | `.*foot` |
| Unitree Go2 | cam_front | Camera | `base/camera_front` |
| Random Unitree Quadruped | imu_robot | Imu | `trunk` |
| Random Unitree Quadruped | contacts_feet | Contact Sensor | `.*foot` |
| Random Unitree Quadruped | cam_front | Camera | `trunk/camera_front` |
| Floating Platform | imu_robot | Imu | `base_link` |
| Floating Platform | cam_onboard | Camera | `base_link/camera_onboard` |
| Int Ball1 | imu_robot | Imu | `base_link` |
| Int Ball1 | cam_onboard | Camera | `base_link/camera_onboard` |
| Int Ball2 | imu_robot | Imu | `base_link` |
| Int Ball2 | cam_onboard | Camera | `base_link/camera_onboard` |
| Apollo Lander | imu_robot | Imu | `base` |
| Apollo Lander | cam_onboard | Camera | `base/camera_onboard` |
| Peregrine Lander | imu_robot | Imu | `base` |
| Peregrine Lander | cam_onboard | Camera | `base/camera_onboard` |
| Vikram Lander | imu_robot | Imu | `base` |
| Vikram Lander | cam_onboard | Camera | `base/camera_onboard` |
| Resilience Lander | imu_robot | Imu | `base` |
| Resilience Lander | cam_onboard | Camera | `base/camera_onboard` |
| Random Lander | imu_robot | Imu | `base` |
| Random Lander | cam_onboard | Camera | `base/camera_onboard` |
| Iss | imu_robot | Imu | `base` |
| Iss | cam_onboard | Camera | `base/camera_onboard` |
| Gateway | imu_robot | Imu | `gateway` |
| Gateway | cam_onboard | Camera | `gateway/camera_onboard` |
| Cubesat | imu_robot | Imu | `cubesat` |
| Cubesat | cam_onboard | Camera | `cubesat/camera_onboard` |
| Satellite Mockup | imu_robot | Imu | `satellite_mockup` |
| Satellite Mockup | cam_onboard | Camera | `satellite_mockup/camera_onboard` |
| Venus Express | imu_robot | Imu | `base` |
| Venus Express | cam_onboard | Camera | `base/camera_onboard` |
| Starship | imu_robot | Imu | `base` |
| Starship | cam_onboard | Camera | `base/camera_onboard` |
| Super Heavy | imu_robot | Imu | `base` |
| Super Heavy | cam_onboard | Camera | `base/camera_onboard` |
