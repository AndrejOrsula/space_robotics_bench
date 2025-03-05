# Graphical User Interface (GUI)

The Space Robotics Bench comes with a simple GUI application that can serve as a more approachable demonstration of its capabilities than pure CLI. The GUI is built on top of [egui](https://github.com/emilk/egui) and leverages [r2r](https://github.com/sequenceplanner/r2r) ROS 2 Rust bindings to communicate with the rest of the benchmark. The initial screen of the GUI is shown below.

![](../_images/srb_gui.jpg)

## Usage

To run the GUI application, you can use the included [`gui.bash`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/gui.bash) script, which internally calls a variant of `cargo run -p srb_gui` command.

```
srb gui
```

The GUI runs the [`teleop.py`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/scripts/teleop.py) script for the selected environments, but the idea is to eventually support multiple workflows. Nine pre-configured tasks/demos are available in the Quick Start window, and a specific scenario can also be defined through the advanced configuration.
