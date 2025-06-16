import importlib
import inspect
from pathlib import Path

import jinja2

# This mapping is the CRITICAL link between the abstract tensor names
# in your _compute_step_return function and the concrete ROS world.
# You would expand this map for all the data types you need.
TENSOR_TO_ROS_MAP = {
    "tf_pos_robot": {
        "topic": "/odom",
        "type": "nav_msgs.msg.Odometry",
        "attribute": "pose.pose",
        "is_vector3": True,
        "shape": "(1,3)",
    },
    "vel_lin_robot": {
        "topic": "/odom",
        "type": "nav_msgs.msg.Odometry",
        "attribute": "twist.twist.linear",
        "is_vector3": True,
        "shape": "(1,3)",
    },
    "vel_ang_robot": {
        "topic": "/odom",
        "type": "nav_msgs.msg.Odometry",
        "attribute": "twist.twist.angular",
        "is_vector3": True,
        "shape": "(1,3)",
    },
    "imu_lin_acc": {
        "topic": "/imu/data",
        "type": "sensor_msgs.msg.Imu",
        "attribute": "linear_acceleration",
        "is_vector3": True,
        "shape": "(1,3)",
    },
    "imu_ang_vel": {
        "topic": "/imu/data",
        "type": "sensor_msgs.msg.Imu",
        "attribute": "angular_velocity",
        "is_vector3": True,
        "shape": "(1,3)",
    },
}


def generate_environment_file(task_module_str: str):
    """
    Generates a real-robot Gym environment file from an SRB task module.

    Args:
        task_module_str: The Python import string for the task (e.g., "srb.tasks.navigation.waypoint").
    """
    # 1. Dynamically import the task module and its contents
    task_module = importlib.import_module(task_module_str)
    TaskCfg = getattr(task_module, "TaskCfg")
    _compute_step_return = getattr(task_module, "_compute_step_return")

    # 2. Inspect the function signature to find required tensors
    sig = inspect.signature(_compute_step_return)
    required_tensors = list(sig.parameters.keys())

    # 3. Build the rendering "context" for Jinja2
    context = {}
    task_name = task_module_str.split(".")[-1]
    context["task_name"] = task_name
    context["class_name"] = f"{task_name.capitalize()}RealEnv"
    context["task_module_path"] = task_module_str
    context["task_cfg_class"] = "TaskCfg"

    # --- Configure ROS topics based on required tensors ---
    ros_imports = set()
    ros_subscribers = {}
    observation_mappings = []

    for tensor_name in required_tensors:
        if tensor_name in TENSOR_TO_ROS_MAP:
            mapping = TENSOR_TO_ROS_MAP[tensor_name]
            # Add to import list (e.g., "from nav_msgs.msg import Odometry")
            module, msg_type = mapping["type"].rsplit(".", 1)
            ros_imports.add(f"from {module} import {msg_type}")

            # Add to subscriber list if not already there
            if mapping["topic"] not in ros_subscribers:
                ros_subscribers[mapping["topic"]] = {
                    "topic": mapping["topic"],
                    "type": msg_type,
                }

            # Create the mapping for the get_observations method
            observation_mappings.append(
                {
                    "tensor_name": tensor_name,
                    "ros_topic_key": mapping["topic"],
                    **mapping,
                }
            )

    context["ros_imports"] = sorted(list(ros_imports))
    context["ros_subscribers"] = {
        k.replace("/", "_"): v for k, v in ros_subscribers.items()
    }  # Make valid var names
    context["observation_mappings"] = observation_mappings

    # --- Statically define action publisher for now ---
    context["ros_publishers"] = {"action": {"topic": "/cmd_vel", "type": "Twist"}}
    ros_imports.add("from geometry_msgs.msg import Twist")  # Ensure it's imported

    # Add other context variables (this would be parsed from TaskCfg)
    # This part can be made more sophisticated by inspecting the cfg instance
    cfg_instance = TaskCfg()
    context["action_space_low"] = [-1.0, -1.0]
    context["action_space_high"] = [1.0, 1.0]
    context["observation_space"] = {  # This is simplified
        "vel_lin_robot": {"shape": [3]},
        "vel_ang_robot": {"shape": [3]},
        "tf_pos_robot_to_target": {"shape": [2]},
    }
    context["compute_fn_args"] = required_tensors
    context["compute_fn_arg_shapes"] = {
        k: TENSOR_TO_ROS_MAP.get(k, {}).get("shape", "(1,1)") for k in required_tensors
    }

    # 4. Render the template
    template_loader = jinja2.FileSystemLoader(searchpath=".")
    template_env = jinja2.Environment(loader=template_loader)
    template = template_env.get_template("real_env_template.py.j2")
    output_text = template.render(context)

    # 5. Save the generated file
    output_path = Path(f"{task_name}_real.py")
    output_path.write_text(output_text)
    print(f"Successfully generated environment file at: {output_path}")


if __name__ == "__main__":
    # Example usage:
    generate_environment_file("srb.tasks.navigation.waypoint")
