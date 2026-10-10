# Interface — ROS 2 & Space ROS

Use ROS 2 to access supported sensor data and robot actions from simulation and
agent workflows. Availability depends on the action space and runtime.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/Eohg5A6JgmE?si=RxobxeqbXkz_TWNi&mute=1&autoplay=1&loop=1&playlist=Eohg5A6JgmE" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

[ROS 2](https://ros.org) exposes supported simulation data and actions through
standard interfaces and messages such as `geometry_msgs` and `sensor_msgs`.
Use RViz2, tf2, and rosbag2 to visualize, debug, and record data. Nodes can run
across processes or machines. Reuse algorithms and control systems as a starting
point for hardware workflows, then validate them separately.

### Space ROS

[Space ROS](https://space.ros.org) extends ROS beyond Earth. For SRB, use ROS 2
Jazzy, set `RMW_IMPLEMENTATION` to `rmw_cyclonedds_cpp` or
`rmw_fastrtps_cpp`, and set `ROS_DOMAIN_ID` to an integer from 0 through 232.
Space ROS compatibility is unvalidated. Space Robotics Bench continues
*Parallel ProcGen Environments*, developed during the [NASA Space ROS Sim
Summer Sprint Challenge](https://www.freelancer.com/contest/NASA-Space-ROS-Sim-Summer-Sprint-Challenge-2417552/updates).

Choose either integration:

- **Indirect (recommended)**: Run SRB's ROS 2 interface on standard ROS 2 middleware and communicate with Space ROS nodes through standard ROS 2 interfaces; no Space ROS-specific setup is required beyond the ROS settings above.
- **Direct**: Source Space ROS before SRB to run its ROS 2 interface with Space ROS middleware alongside other Space ROS nodes.

## Implementation

The bridge dynamically provides:

- **Dynamic interfaces** - Creates supported publishers, subscribers, and services for the selected environment; no manual endpoint configuration.
- **Parallel environments** - Uses separate ROS namespaces per simulation instance; global control is also available.
- **Message translation** - Translates environment states, sensor outputs, and control commands from simulation tensors to ROS interfaces.
- **Data and actions** - Interface modules expose supported sensors, transforms, batched Box actions, and step outputs, but not every simulation entity.

## Get Started

1. [Installation](../getting_started/install.md)
1. [Basic Usage](../getting_started/basic_usage.md)
1. [ROS 2 Workflow](../workflows/ros2.md)

## Manual TF trajectory visualizer

Use the standalone TF-to-Path node in the checkout to observe transforms. Run it
in the ROS domain containing those transforms. It requires sourced ROS 2 Jazzy and the `rclpy`, `tf2_ros`,
`geometry_msgs`, and `nav_msgs` ROS packages; the Python plotting extra does not
provide them.

1. Source ROS 2 Jazzy and run the node from the checkout, replacing the frame
   names with those from your system:

   ```bash
   source /opt/ros/jazzy/setup.bash
   python srb/interfaces/sim_to_real/utils/visual/ros_tf_traj_display.py \
     --ros-args -p source_frame:=world -p target_frame:=target -p publish_rate_hz:=10.0
   ```

1. Add `target_path` (`nav_msgs/msg/Path`, published in `source_frame`) as an
   RViz Path display.

1. Stop the node when the observation session ends.

The node preserves transform orientation and keeps at most 1000 poses by
default. It publishes visualization data, not robot commands, and has no
installed `ros2 run` executable or `srb` subcommand.

The waypoint evaluator at `srb/interfaces/sim_to_real/utils/eval/waypoint_navigation.py`
is a separate manual evaluation tool.

Next: [ROS 2 Workflow](../workflows/ros2.md)
