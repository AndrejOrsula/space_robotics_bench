# ROS 2 Workflow

Connect SRB to ROS 2 or Space ROS for autonomy development and validation.

The active ROS 2 agent receives actions from ROS and steps the environment.
The passive interface lets a workflow control actions while ROS publishes
sensors and exposes services.

## 1. Start an Active ROS 2 Agent

Use `srb agent ros` with a batched Box action space:

```bash
srb agent ros -e peg_in_hole env.num_envs=2 'env.active_sensors=[proprioception,contact,transform,visual]'
```

For closed-loop control, `--ros_sync` awaits fresh actions after step one:

```bash
srb agent ros --ros_sync -e peg_in_hole env.num_envs=2 'env.active_sensors=[proprioception,contact,transform,visual]'
```

See the [ROS agent reference](../reference/cli_agent_ros.md) for option details.

Debug with action and observation delays at zero:

```bash
srb agent ros --ros_sync -e waypoint_navigation env.action_delay_steps=0 env.observation_delay_steps=0
```

### List Topics

Without local ROS 2, run `.docker/join.bash`, then list topics:

```bash
ros2 topic list
```

<details>
<summary>Topics (expand)</summary>

```
/clock
/parameter_events
/rosout
/srb/env0/action/cmd_vel
/srb/env0/action/event
/srb/env0/action/raw
/srb/env0/cam_base/camera_info
/srb/env0/cam_base/image_depth
/srb/env0/cam_base/image_rgb
/srb/env0/cam_base/pointcloud
/srb/env0/cam_scene/camera_info
/srb/env0/cam_scene/image_depth
/srb/env0/cam_scene/image_rgb
/srb/env0/cam_scene/pointcloud
/srb/env0/cam_wrist/camera_info
/srb/env0/cam_wrist/image_depth
/srb/env0/cam_wrist/image_rgb
/srb/env0/cam_wrist/pointcloud
/srb/env0/end_effector/joint_pos
/srb/env0/end_effector/joint_states
/srb/env0/reward
/srb/env0/reward/penalty_action_rate
/srb/env0/reward/penalty_undesired_robot_contacts
/srb/env0/reward/reward_align_peg_to_hole_primary
/srb/env0/reward/reward_align_peg_to_hole_secondary
/srb/env0/reward/reward_distance_ee_to_obj
/srb/env0/reward/reward_distance_peg_to_hole_bottom
/srb/env0/reward/reward_distance_peg_to_hole_entrance
/srb/env0/reward/reward_grasp
/srb/env0/reward/reward_lift
/srb/env0/robot/delta_twist
/srb/env0/robot/joint_states
/srb/env0/terminated
/srb/env0/truncated
/srb/env1/...          (mirrors env0)
/srb/envs/action/cmd_vel
/srb/envs/action/event
/srb/envs/action/raw
/srb/envs/end_effector/joint_pos
/srb/envs/robot/delta_twist
/tf
/tf_static
```

</details>

Namespaces:

- `/srb/env[0-N]/...` targets one environment; `/srb/envs/...` targets all.

With `env.num_envs=1`, prefixes vanish, e.g. `/srb/reward` and
`/srb/action/cmd_vel`.

| Topic | Message Type | Description |
| ------------------------------- | ------------------------ | ----------------------------------------- |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Scene transforms |
| **Control** | | |
| `**/{robot_name}/{action_name}` | Robot-specific | Robot control; normalized policy actions (message fields are containers, not SI units) |
| `**/action/raw` | `std_msgs/Float64MultiArray` | Flat action vector in action-space order |
| `**/action/cmd_vel` | `geometry_msgs/Twist` | Available when the task action config supports direct commands |
| `**/action/event` | `std_msgs/Bool` | Available when the task action config supports direct commands |
| **Perception** | | |
| `**/[{sensor_name}]/{data}` | Sensor-specific | Scene sensors |
| `**/{robot_name}/joint_states` | `sensor_msgs/JointState` | Articulation joints |
| **MDP** | | |
| `**/reward` | `std_msgs/Float32` | Total reward |
| `**/reward/{term_name}` | `std_msgs/Float32` | Reward terms |
| `**/terminated` | `std_msgs/Bool` | Termination flag |
| `**/truncated` | `std_msgs/Bool` | Truncation flag |

### Subscribe to Topics

Echo env0's reward:

```bash
ros2 topic echo /srb/env0/reward
```

```text
data: 2.36441707611084
---
data: 2.3652756214141846
---
data: 2.36613130569458
---
data: 2.3669850826263428
---
```

### Publish to Topics

Publish at 20 Hz:

```bash
ros2 topic pub --rate 20 /srb/envs/action/cmd_vel geometry_msgs/Twist "{linear: {z: -0.2}, angular: {z: 0.785}}"
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/NU8t8WfrMSc?si=eTHoGjNMIhWPlLIW&mute=1&autoplay=1&loop=1&playlist=NU8t8WfrMSc" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 2. Start the Passive ROS 2 Interface

Enable the passive interface with `--interface ros`. Run `rand` on eight
`multi_sample_collection` instances with visual sensors:

```bash
srb agent rand -e multi_sample_collection env.num_envs=8 'env.active_sensors=[proprioception,contact,transform,visual]' --interface ros
```

`--interface` accepts multiple values; following Hydra `key=value` overrides
forward automatically.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/_wPt32cVOY4?si=Rvlalg0z1SLFFDrQ&mute=1&autoplay=1&loop=1&playlist=_wPt32cVOY4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Call Services

Reset one via `/srb/env{i}/reset`, or all via `/srb/envs/reset`. Requests are queued and apply after the next step; with `--ros_sync`, that step waits for a fresh action.

```bash
ros2 service list
```

Reset one environment:

```bash
ros2 service call /srb/env0/reset std_srvs/srv/Empty
```

```bash
ros2 service call /srb/envs/reset std_srvs/srv/Empty
```

Next: [Reinforcement Learning Workflow](reinforcement_learning.md)
