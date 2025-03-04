# ROS 2 Workflow

Environments of the Space Robotics Bench can be integrated with ROS 2 to enable control of the robots and data collection over the middleware communication.

> **Note:** If you wish to use Space ROS inside the container instead of regular ROS 2, you can build the image using the `--build-arg INSTALL_SPACEROS=true` build argument. This option is generally not recommended as there is little benefit to using Space ROS on the simulation side, but it is available for experimentation.

## Single Environment

The [`ros2.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/ros2.py) script is the primary entry point for interfacing with the environments through ROS 2. This script spawns a single ROS node that maps inputs and outputs for the environment and provides miscellaneous functionalities such as resetting the simulation. Here is an example using the Ingenuity demo:

```bash
.docker/run.bash ros2 run srb ros2.py --env ingenuity
```

Once the environment is initialized, open a new terminal to inspect the available ROS topics. You can either use your ROS setup or join the running Docker container with the [`.docker/join.bash`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.docker/join.bash) script:

```bash
.docker/join.bash
```

Now, list the available ROS topics:

```bash
ros2 topic list
# Expected output:
# /clock
# /env/info
# /env/reward
# /env/terminated
# /env/truncated
# /parameter_events
# /robot/cmd_vel
# /rosout
# /tf
```

To control the robot, publish a `Twist` message to the `/robot/cmd_vel` topic:

```bash
ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'
```

You can reset the simulation by calling the `/sim/reset` service:

```bash
ros2 service call /sim/reset std_srvs/srv/Empty
```

## Parallel Environments

You can run multiple environments in parallel by using the `--num_envs` argument. Each environment will map to its own ROS namespace. For example, try running the Ingenuity demo with 4 environments:

```bash
.docker/run.bash ros2 run srb ros2.py --env ingenuity --num_envs 4
```

List the available ROS topics again:

```bash
ros2 topic list
# Expected output:
# /clock
# /env0/reward
# /env0/robot/cmd_vel
# /env0/terminated
# /env0/truncated
# /env1/reward
# /env1/robot/cmd_vel
# /env1/terminated
# /env1/truncated
# /env2/reward
# /env2/robot/cmd_vel
# /env2/terminated
# /env2/truncated
# /env3/reward
# /env3/robot/cmd_vel
# /env3/terminated
# /env3/truncated
# /envs/info
# /envs/robot/cmd_vel
# /parameter_events
# /rosout
# /tf
```

Each environment has its own namespace, allowing individual control. For example:

```bash
ros2 topic pub --once /env0/robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -1.0}}'
ros2 topic pub --once /env1/robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'
ros2 topic pub --once /env2/robot/cmd_vel geometry_msgs/msg/Twist '{linear: {y: -1.0}}'
ros2 topic pub --once /env3/robot/cmd_vel geometry_msgs/msg/Twist '{linear: {y: 1.0}}'
```

## Launch with `rviz2` and `teleop_twist_keyboard`

For convenience, you can launch `rviz2` alongside `ros2.py` and `teleop_twist_keyboard` for visualization and control via keyboard:

```bash
.docker/run.bash ros2 launch srb demo.launch.py task:=ingenuity_visual num_envs:=4
```
