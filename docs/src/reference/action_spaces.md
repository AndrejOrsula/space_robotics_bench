# Action Space Catalog

Each robot offers one or more action modes, chosen at launch via `env.robot.action_mode=<mode>` (the default mode is marked `*`). **Dims** gives action-vector length; **Scale** multiplies policy output before it reaches the controller. `runtime-resolved` length depends on the loaded asset (e.g. a joint-space mode with pattern `.*`) and is read from the action manager when the environment is built.

The machine-readable action spec (`env.srb_spec.action`) targets the action manager's runtime terms. Schema-version-2 descriptors name runtime terms directly, e.g. the multicopter `cmd_vel`, body-acceleration `body_vel` and task-space `delta_twist` terms. A schema-version-1 descriptor may split one runtime term into semantic fields, such as `lin` and `ang` for the wheeled-drive `cmd_vel` term, but its summed field dimensions must equal the summed runtime term dimensions. A mismatch of either kind fails the spec build instead of publishing a wrong layout. Groups without descriptors use the schema-version-0 fallback: no semantics, no dimension check. Metadata that carries an unsupported descriptor (the multicopter `thrust` / `multicopter.rotor_thrusts`, the task-space quaternion `ee_rot` / `end_effector.orientation`, or the velocity-labelled body `lin` / `body.linear_velocity`) is rejected on load.

Task-space modes publish one runtime-length `delta_twist` term: three position-delta channels (m), then three axis-angle rotation-delta channels (rad); position-only inverse kinematics uses the first three. Variable-stiffness operational-space modes append six stiffness channels, and full-impedance modes append six stiffness and then six damping-ratio channels.

Multicopter Body Acceleration (`body_acc` on the multicopters) is a velocity command despite its name: four channels (`linear_x`, `linear_y`, `linear_z`, `angular_z`) are clamped to [-1, 1] and multiplied by the scale into m/s and rad/s, and x and y are rotated by the current yaw. Body Acceleration (`body_acc` on the orbital bodies) is an acceleration command: six channels are clamped to [-1, 1] and multiplied by the scale into a linear (m/s²) and angular (rad/s²) acceleration in the world frame, which is integrated into the root velocity as `acceleration * physics_dt` on every physics step. The velocity change over one environment step is therefore `acceleration * step_dt` whatever the decimation. The relative controller variant, when configured, expresses the same command in the body frame.

Task-space modes (`diff_ik`, `osc_*`, `smooth_osc`) act on the tool centre point that tasks observe as `robot_ee` (the `tf_end_effector` target): the manipulator's flange frame composed with the mounted tool's TCP, or the bare flange when no tool is mounted. The environment config sets every such term's `body_offset` to that frame. Isaac Lab's task-space terms carry the body Jacobian to it in the articulation root frame, the frame the end-effector pose and velocity are measured in (a term's `base_name`, when set, must name the root body), and the operational-space terms damp the TCP velocity derived from the flange link frame rather than its centre of mass. A pure rotation command therefore turns the tool about its TCP.

| Robot | Mode | Controller | Dims | Scale |
|---|---|---|---|---|
| Canadarm3 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Canadarm3 | `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |
| Franka | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Franka | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Franka | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Franka | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Franka | `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |
| Kinova J2N6S | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Kinova J2N6S | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Kinova J2N7S | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Kinova J2N7S | `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |
| Kinova Gen3N7 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Kinova Gen3N7 | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Kinova Gen3N7 | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Kinova Gen3N7 | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Kinova Gen3N7 | `smooth_osc` | Smooth Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Kinova Gen3N7 | `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |
| Levion Arms | `dual_arm_joint_pos_abs` * | Joint Position To Limits | 4 | 1.0 |
| So Arm100 D5 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| So Arm100 D5 | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.05, ori=0.05, stiffness=120.0 |
| So Arm100 D5 | `joint_pos_rel` | Relative Joint Position | 5 | 1.0 |
| So Arm100 D7 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| So Arm100 D7 | `joint_pos_rel` | Relative Joint Position | 7 | 1.0 |
| Unitree Z1 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| Unitree Z1 | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur3 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur3 | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur3 | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur3 | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur3 | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur3E | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur3E | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur3E | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur3E | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur3E | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur5 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur5 | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur5 | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur5 | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur5 | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur5E | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur5E | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur5E | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur5E | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur5E | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur10 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur10 | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur10 | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur10 | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur10 | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur10E | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur10E | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur10E | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur10E | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur10E | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur16E | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur16E | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur16E | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur16E | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur16E | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur20 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur20 | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur20 | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur20 | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur20 | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Ur30 | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Ur30 | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Ur30 | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Ur30 | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Ur30 | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Random Ur Manipulator | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.1 |
| Random Ur Manipulator | `osc_fixed` | Operational Space Controller | 6 | pos=0.1, ori=0.1 |
| Random Ur Manipulator | `osc_var_stiffness` | Operational Space Controller | 12 | pos=0.1, ori=0.1, stiffness=120.0 |
| Random Ur Manipulator | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Random Ur Manipulator | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Vispa | `diff_ik` * | Differential Inverse Kinematics | 6 | 0.05 |
| Vispa | `osc_var_full` | Operational Space Controller | 18 | pos=0.1, ori=0.1, stiffness=120.0, damping=1.0 |
| Vispa | `joint_pos_rel` | Relative Joint Position | 6 | 1.0 |
| Zeroglab Arms | `dual_arm_osc` * | Operational Space Controller | 24 | `arm_ceiling`: pos=0.1, ori=0.1, stiffness=120.0; `arm_wall`: pos=0.1, ori=0.1, stiffness=120.0 |
| Humanoid21 | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Humanoid28 | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Unitree H1 | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Unitree G1 | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Crazyflie | `body_acc` * | Multicopter Body Acceleration | 4 | 0.5 |
| Ingenuity | `body_acc` * | Multicopter Body Acceleration | 4 | 0.5 |
| Heron | `thrust` * | Thrust | 2 | 1.0 |
| Anymal C | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Anymal D | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Random Anymal Quadruped | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Cadre | `drive` * | Wheeled Drive | 2 | lin=0.4, ang=0.7853981633974483 |
| Cassie | `joint_pos` * | Joint Position To Limits | 12 | 1.0 |
| Husky | `drive` * | Wheeled Drive | 2 | lin=0.1, ang=0.7853981633974483 |
| Leo Rover | `drive` * | Wheeled Drive | 2 | lin=0.4, ang=1.0471975511965976 |
| Mars Rover | `drive` * | Wheeled Drive | 2 | lin=0.5, ang=0.7853981633974483 |
| Nifty | `nifty` * | Joint Velocity | 12 | `joint_vel_wheel_drive`=1.5707963267948966; `joint_pos_steer`=0.7853981633974483; `joint_pos_suspension_yaw`=0.5235987755982988; `joint_pos_suspension_roll`=0.2617993877991494 |
| Open Source Rover | `drive` * | Wheeled Drive | 2 | lin=0.2, ang=0.3490658503988659 |
| Perseverance | `drive` * | Wheeled Drive | 2 | lin=0.7, ang=1.3089969389957472 |
| Pragyan | `drive` * | Wheeled Drive | 2 | lin=0.35, ang=1.3962634015954636 |
| Raph Rover | `drive` * | Wheeled Drive | 2 | lin=0.4, ang=1.0471975511965976 |
| Raph Rover | `drive_filtered` | Filtered Wheeled Drive | 2 | lin=0.4, ang=1.0 |
| Spot | `joint_pos` * | Joint Position | runtime-resolved | 0.2 |
| Unitree A1 | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Unitree Go1 | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Unitree Go2 | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Random Unitree Quadruped | `joint_pos` * | Joint Position To Limits | runtime-resolved | 1.0 |
| Floating Platform | `thrust_only` * | Thrust | 8 | 1.0 |
| Floating Platform | `thrust_and_reaction` | Thrust | 9 | `thrust`=1.0; `reaction_wheel_torque`=0.05 |
| Int Ball1 | `thrust` * | Thrust | 6 | 1.0 |
| Int Ball2 | `thrust` * | Thrust | 6 | 1.0 |
| Apollo Lander | `thrust` * | Thrust | 17 | 1.0 |
| Peregrine Lander | `thrust` * | Thrust | 17 | 1.0 |
| Vikram Lander | `thrust` * | Thrust | 13 | 1.0 |
| Resilience Lander | `thrust` * | Thrust | 9 | 1.0 |
| Random Lander | `thrust` * | Thrust | 3 | 1.0 |
| Iss | `body_acc` * | Body Acceleration | 6 | 2.5 |
| Gateway | `body_acc` * | Body Acceleration | 6 | 2.5 |
| Cubesat | `thrust` * | Thrust | 12 | 1.0 |
| Satellite Mockup | `thrust` * | Thrust | 12 | 1.0 |
| Venus Express | `thrust` * | Thrust | 19 | 1.0 |
| Starship | `thrust` * | Thrust | 18 | 1.0 |
| Super Heavy | `thrust` * | Thrust | 59 | 1.0 |
