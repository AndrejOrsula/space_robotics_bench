#!/usr/bin/env python3
"""
A comprehensive, single-file tool for the collection, processing, and
analysis of 3D robot trajectory data for academic publications.

This script provides a full workflow via four commands:
  1. collect:  (Requires ROS 2) Records raw trajectory data from TF2 frames.
  2. process:  Calculates performance metrics from raw data files.
  3. report:   Generates all publication-quality png plots and LaTeX tables.
  4. animate:  Creates a high-quality video of a single experimental run.
"""

import argparse
import json
import math
import threading
from itertools import cycle
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Duration, Time
    from tf2_ros import Buffer, TransformListener

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS 2 not available. 'collect' command will be disabled.")

# Configure plotting style for publication quality
plt.style.use("seaborn-v0_8-whitegrid")
PLOT_STYLE = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "Bitstream Vera Serif", "serif"],
    "axes.labelsize": 14,
    "xtick.labelsize": 18,
    "ytick.labelsize": 18,
    "legend.fontsize": 12,
    "figure.titlesize": 16,
    "figure.dpi": 300,
}
plt.rcParams.update(PLOT_STYLE)


class TrajectoryCollectorNode(Node):
    """ROS 2 Node for high-fidelity recording of 3D trajectory and orientation data."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("trajectory_collector_3d")
        self.output_file = args.output
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.fixed_frame = args.fixed_frame
        self.robot_frame = args.robot_frame
        self.target_frame = args.target_frame
        self.data_lock = threading.Lock()
        self.data: List[Dict[str, float]] = []
        self.start_time: Optional[Time] = None

        self.timer = self.create_timer(1.0 / args.rate, self.timer_callback)
        self.get_logger().info("3D Collector started.")
        self.get_logger().info(f"Saving raw data to: {self.output_file}")
        self.get_logger().info(
            f"Tracking '{self.robot_frame}' and '{self.target_frame}' relative to '{self.fixed_frame}'."
        )
        self.get_logger().info("Press Ctrl+C to stop collection and save.")

    @staticmethod
    def _quaternion_to_euler(q) -> Tuple[float, float, float]:
        """Converts a ROS Quaternion to Euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        try:
            now = self.get_clock().now()
            robot_tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            target_tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )

            if self.start_time is None:
                self.start_time = now

            time_elapsed = (now - self.start_time).nanoseconds / 1e9
            robot_roll, robot_pitch, robot_yaw = self._quaternion_to_euler(
                robot_tf.transform.rotation
            )
            target_roll, target_pitch, target_yaw = self._quaternion_to_euler(
                target_tf.transform.rotation
            )

            with self.data_lock:
                self.data.append(
                    {
                        "time": time_elapsed,
                        "robot_x": robot_tf.transform.translation.x,
                        "robot_y": robot_tf.transform.translation.y,
                        "robot_z": robot_tf.transform.translation.z,
                        "robot_roll": robot_roll,
                        "robot_pitch": robot_pitch,
                        "robot_yaw": robot_yaw,
                        "robot_quat_w": robot_tf.transform.rotation.w,
                        "robot_quat_x": robot_tf.transform.rotation.x,
                        "robot_quat_y": robot_tf.transform.rotation.y,
                        "robot_quat_z": robot_tf.transform.rotation.z,
                        "target_x": target_tf.transform.translation.x,
                        "target_y": target_tf.transform.translation.y,
                        "target_z": target_tf.transform.translation.z,
                        "target_roll": target_roll,
                        "target_pitch": target_pitch,
                        "target_yaw": target_yaw,
                        "target_quat_w": target_tf.transform.rotation.w,
                        "target_quat_x": target_tf.transform.rotation.x,
                        "target_quat_y": target_tf.transform.rotation.y,
                        "target_quat_z": target_tf.transform.rotation.z,
                    }
                )

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=5.0)

    def save_data(self):
        """Saves the collected data to a CSV file upon shutdown."""
        self.get_logger().info("Shutdown signal received. Saving data...")
        if not self.data:
            self.get_logger().warn("No data was collected. Exiting without saving.")
            return
        df = pd.DataFrame(self.data)
        df.to_csv(self.output_file, index=False)
        self.get_logger().info(f"Saved {len(df)} data points to {self.output_file}")


def _quaternion_angle_difference(q1, q2):
    """Calculate angular difference between two quaternions in radians."""
    # Compute the dot product
    dot_product = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]

    # Ensure the dot product is in valid range
    dot_product = np.clip(np.abs(dot_product), 0.0, 1.0)

    # Calculate the angle
    return 2 * np.arccos(dot_product)


def _process_file(input_path: Path):
    """Calculates performance metrics for a single raw 3D trajectory file."""
    print(f"Processing: {input_path.name}")
    try:
        df = pd.read_csv(input_path)
    except pd.errors.EmptyDataError:
        print("  -> Skipping, file is empty.")
        return

    if len(df) < 5:
        print(
            "  -> Skipping, requires at least 5 data points for stable derivative calculation."
        )
        return

    df = df.sort_values(by="time").reset_index(drop=True)
    dt = df["time"].diff()
    dt.iloc[0] = dt.median()
    dt_filled = dt.replace(0, 1e-9)

    # 3D Euclidean distance error
    df["euclidean_error"] = np.sqrt(
        (df["target_x"] - df["robot_x"]) ** 2
        + (df["target_y"] - df["robot_y"]) ** 2
        + (df["target_z"] - df["robot_z"]) ** 2
    )

    # Quaternion-based orientation error (more accurate for 3D)
    robot_quats = df[
        ["robot_quat_w", "robot_quat_x", "robot_quat_y", "robot_quat_z"]
    ].values
    target_quats = df[
        ["target_quat_w", "target_quat_x", "target_quat_y", "target_quat_z"]
    ].values

    orientation_errors = []
    for i in range(len(df)):
        error = _quaternion_angle_difference(robot_quats[i], target_quats[i])
        orientation_errors.append(error)
    df["orientation_error_rad"] = orientation_errors

    # Individual axis orientation errors for analysis
    roll_error_raw = df["target_roll"] - df["robot_roll"]
    df["roll_error_rad"] = np.arctan2(np.sin(roll_error_raw), np.cos(roll_error_raw))

    pitch_error_raw = df["target_pitch"] - df["robot_pitch"]
    df["pitch_error_rad"] = np.arctan2(np.sin(pitch_error_raw), np.cos(pitch_error_raw))

    yaw_error_raw = df["target_yaw"] - df["robot_yaw"]
    df["yaw_error_rad"] = np.arctan2(np.sin(yaw_error_raw), np.cos(yaw_error_raw))

    # 3D velocity, acceleration, and jerk
    df["robot_velocity"] = (
        np.sqrt(
            df["robot_x"].diff() ** 2
            + df["robot_y"].diff() ** 2
            + df["robot_z"].diff() ** 2
        )
        / dt_filled
    ).fillna(0)

    df["robot_accel"] = (df["robot_velocity"].diff() / dt_filled).fillna(0)
    df["robot_jerk"] = (df["robot_accel"].diff() / dt_filled).fillna(0)

    # Angular velocity and acceleration
    df["robot_angular_velocity"] = (
        np.sqrt(
            df["robot_roll"].diff() ** 2
            + df["robot_pitch"].diff() ** 2
            + df["robot_yaw"].diff() ** 2
        )
        / dt_filled
    ).fillna(0)

    df["robot_angular_accel"] = (
        df["robot_angular_velocity"].diff() / dt_filled
    ).fillna(0)

    # Summary metrics compatible with task.py variables
    summary = {
        "ate_m": df["euclidean_error"].mean(),  # Average Translation Error
        "ate_rad": df["orientation_error_rad"].mean(),  # Average Rotation Error
        "max_position_error_m": df["euclidean_error"].max(),
        "max_orientation_error_rad": df["orientation_error_rad"].max(),
        "jerk_avg_m_s3": df["robot_jerk"].abs().mean(),
        "angular_accel_avg_rad_s2": df["robot_angular_accel"].abs().mean(),
        "roll_error_rms_rad": np.sqrt(np.mean(df["roll_error_rad"] ** 2)),
        "pitch_error_rms_rad": np.sqrt(np.mean(df["pitch_error_rad"] ** 2)),
        "yaw_error_rms_rad": np.sqrt(np.mean(df["yaw_error_rad"] ** 2)),
        "final_position_error_m": df["euclidean_error"].iloc[-1],
        "final_orientation_error_rad": df["orientation_error_rad"].iloc[-1],
        "trajectory_length_m": df["robot_velocity"].sum() * dt_filled.mean(),
        "average_velocity_m_s": df["robot_velocity"].mean(),
        "max_velocity_m_s": df["robot_velocity"].max(),
        "average_angular_velocity_rad_s": df["robot_angular_velocity"].mean(),
        "max_angular_velocity_rad_s": df["robot_angular_velocity"].max(),
    }

    processed_csv_path = input_path.with_name(f"{input_path.stem}_processed.csv")
    summary_json_path = input_path.with_name(f"{input_path.stem}_summary.json")

    df.to_csv(processed_csv_path, index=False)
    with open(summary_json_path, "w") as f:
        json.dump(summary, f, indent=4)
    print(f"  -> Saved processed data and summary for {input_path.name}")


def _load_and_group_experiments(
    exp_definitions: List[Tuple[str, str]],
) -> Dict[str, Dict[str, List[Path]]]:
    """Groups experiment files by trajectory and method for comparative analysis."""
    grouped_results = {}
    print("--- Loading and Grouping Experiment Files ---")
    for method_name, glob_pattern in exp_definitions:
        paths = sorted(list(Path(".").glob(glob_pattern)))
        if not paths:
            print(
                f"Warning: No files found for method '{method_name}' with pattern '{glob_pattern}'"
            )
            continue
        print(f"Found {len(paths)} files for method '{method_name}'")
        for path in paths:
            traj_name = path.stem.split("_")[0]
            if traj_name not in grouped_results:
                grouped_results[traj_name] = {}
            if method_name not in grouped_results[traj_name]:
                grouped_results[traj_name][method_name] = []
            grouped_results[traj_name][method_name].append(path)
    return grouped_results


def _quaternion_to_direction_vector(quat_w, quat_x, quat_y, quat_z, length=0.3):
    """Convert quaternion to forward direction vector for visualization."""
    # Convert quaternion to rotation matrix and extract forward direction (X-axis)
    # Forward direction in robot frame (assuming X-axis is forward)
    forward_x = 1 - 2 * (quat_y**2 + quat_z**2)
    forward_y = 2 * (quat_x * quat_y + quat_w * quat_z)
    forward_z = 2 * (quat_x * quat_z - quat_w * quat_y)

    # Normalize and scale
    norm = np.sqrt(forward_x**2 + forward_y**2 + forward_z**2)
    return (
        length * forward_x / norm,
        length * forward_y / norm,
        length * forward_z / norm,
    )


def _plot_3d_trajectories_with_confidence(grouped_results: Dict, output_dir: Path):
    """
    Generates 3D trajectory plots with confidence intervals and saves as png.
    Creates both 3D plots and 2D projections (XY, XZ, YZ planes).
    Includes orientation visualization as arrows.
    """
    print("\n--- Generating 3D Trajectory Plots with Confidence Intervals (png) ---")

    color_palette = sns.color_palette("colorblind")
    method_color_map = {
        "DreamerV3": color_palette[0],
        "PPO": color_palette[1],
        "TD3": color_palette[2],
        "PPO (LSTM)": color_palette[3],
    }
    target_path_color = "black"
    fallback_colors = cycle(color_palette[4:] + color_palette[:4])

    for traj_name, methods in grouped_results.items():
        # 3D Plot
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection="3d")

        target_df_for_centering = None
        for method_name, paths in methods.items():
            for p in paths:
                try:
                    df = pd.read_csv(p.with_name(f"{p.stem}_processed.csv"))
                    if target_df_for_centering is None:
                        target_df_for_centering = df
                    break
                except FileNotFoundError:
                    continue
            if target_df_for_centering is not None:
                break

        if target_df_for_centering is None:
            print(f"Skipping 3D plot for '{traj_name}' due to missing data.")
            plt.close(fig)
            continue

        # Center the trajectories
        center_x = (
            target_df_for_centering["target_x"].min()
            + target_df_for_centering["target_x"].max()
        ) / 2.0
        center_y = (
            target_df_for_centering["target_y"].min()
            + target_df_for_centering["target_y"].max()
        ) / 2.0
        center_z = (
            target_df_for_centering["target_z"].min()
            + target_df_for_centering["target_z"].max()
        ) / 2.0

        target_df_plotted = False
        for i, (method_name, paths) in enumerate(methods.items()):
            all_x, all_y, all_z = [], [], []
            target_df = None

            for p in paths:
                try:
                    df = pd.read_csv(p.with_name(f"{p.stem}_processed.csv"))
                    all_x.append(df["robot_x"].values - center_x)
                    all_y.append(df["robot_y"].values - center_y)
                    all_z.append(df["robot_z"].values - center_z)
                    if target_df is None:
                        target_df = df
                except FileNotFoundError:
                    continue

            if not all_x:
                continue

            # Plot target path once
            if not target_df_plotted and target_df is not None:
                ax.plot(
                    target_df["target_x"] - center_x,
                    target_df["target_y"] - center_y,
                    target_df["target_z"] - center_z,
                    color=target_path_color,
                    linestyle="--",
                    linewidth=2,
                    label="Target Path",
                    alpha=0.8,
                )
                target_df_plotted = True

            # Calculate confidence intervals
            if len(all_x) > 1:
                min_len = min(len(traj) for traj in all_x)
                all_x_trimmed = np.array([traj[:min_len] for traj in all_x])
                all_y_trimmed = np.array([traj[:min_len] for traj in all_y])
                all_z_trimmed = np.array([traj[:min_len] for traj in all_z])

                mean_x = np.mean(all_x_trimmed, axis=0)
                mean_y = np.mean(all_y_trimmed, axis=0)
                mean_z = np.mean(all_z_trimmed, axis=0)
            else:
                mean_x, mean_y, mean_z = all_x[0], all_y[0], all_z[0]

            # Get color for this method
            color = method_color_map.get(method_name, next(fallback_colors))

            # Plot mean trajectory
            ax.plot(
                mean_x,
                mean_y,
                mean_z,
                color=color,
                linewidth=2.5,
                label=f"{method_name} (Mean)",
                alpha=0.9,
            )

            # Add orientation arrows along the trajectory (sample every N points for clarity)
            if target_df is not None:
                step = max(1, len(mean_x) // 10)  # Show ~10 orientation arrows
                for j in range(0, len(mean_x), step):
                    if j < len(target_df):
                        # Robot orientation arrow
                        dx_r, dy_r, dz_r = _quaternion_to_direction_vector(
                            target_df["robot_quat_w"].iloc[j],
                            target_df["robot_quat_x"].iloc[j],
                            target_df["robot_quat_y"].iloc[j],
                            target_df["robot_quat_z"].iloc[j],
                        )
                        ax.quiver(
                            mean_x[j],
                            mean_y[j],
                            mean_z[j],
                            dx_r,
                            dy_r,
                            dz_r,
                            color=color,
                            alpha=0.7,
                            arrow_length_ratio=0.1,
                            linewidth=1.5,
                        )

                        # Target orientation arrow (only for first method to avoid clutter)
                        if i == 0:
                            dx_t, dy_t, dz_t = _quaternion_to_direction_vector(
                                target_df["target_quat_w"].iloc[j],
                                target_df["target_quat_x"].iloc[j],
                                target_df["target_quat_y"].iloc[j],
                                target_df["target_quat_z"].iloc[j],
                            )
                            ax.quiver(
                                target_df["target_x"].iloc[j] - center_x,
                                target_df["target_y"].iloc[j] - center_y,
                                target_df["target_z"].iloc[j] - center_z,
                                dx_t,
                                dy_t,
                                dz_t,
                                color="red",
                                alpha=0.8,
                                arrow_length_ratio=0.1,
                                linewidth=2,
                            )

            # Plot individual runs with transparency
            for x_traj, y_traj, z_traj in zip(all_x, all_y, all_z):
                ax.plot(x_traj, y_traj, z_traj, color=color, alpha=0.2, linewidth=1)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio for all axes
        ax.legend()
        ax.set_title(f"3D Trajectory Comparison: {traj_name}")

        plot_path = output_dir / f"trajectory_3d_{traj_name}.png"
        fig.savefig(plot_path, format="png", dpi=300, bbox_inches="tight")
        print(f"Saved 3D trajectory plot: {plot_path}")
        plt.close(fig)

        # Create 2D projection plots
        projections = [
            ("XY", "robot_x", "robot_y", "target_x", "target_y", "X (m)", "Y (m)"),
            ("XZ", "robot_x", "robot_z", "target_x", "target_z", "X (m)", "Z (m)"),
            ("YZ", "robot_y", "robot_z", "target_y", "target_z", "Y (m)", "Z (m)"),
        ]

        for (
            proj_name,
            robot_dim1,
            robot_dim2,
            target_dim1,
            target_dim2,
            xlabel,
            ylabel,
        ) in projections:
            fig, ax = plt.subplots(figsize=(8, 8))

            target_df_plotted = False
            for method_name, paths in methods.items():
                all_dim1, all_dim2 = [], []
                target_df = None

                for p in paths:
                    try:
                        df = pd.read_csv(p.with_name(f"{p.stem}_processed.csv"))
                        center_dim1 = (
                            center_x
                            if "x" in robot_dim1
                            else (center_y if "y" in robot_dim1 else center_z)
                        )
                        center_dim2 = (
                            center_x
                            if "x" in robot_dim2
                            else (center_y if "y" in robot_dim2 else center_z)
                        )
                        all_dim1.append(df[robot_dim1].values - center_dim1)
                        all_dim2.append(df[robot_dim2].values - center_dim2)
                        if target_df is None:
                            target_df = df
                    except FileNotFoundError:
                        continue

                if not all_dim1:
                    continue

                # Plot target path once
                if not target_df_plotted and target_df is not None:
                    center_dim1 = (
                        center_x
                        if "x" in target_dim1
                        else (center_y if "y" in target_dim1 else center_z)
                    )
                    center_dim2 = (
                        center_x
                        if "x" in target_dim2
                        else (center_y if "y" in target_dim2 else center_z)
                    )
                    ax.plot(
                        target_df[target_dim1] - center_dim1,
                        target_df[target_dim2] - center_dim2,
                        color=target_path_color,
                        linestyle="--",
                        linewidth=2,
                        label="Target Path",
                        alpha=0.8,
                    )
                    target_df_plotted = True

                # Calculate mean trajectory
                if len(all_dim1) > 1:
                    min_len = min(len(traj) for traj in all_dim1)
                    all_dim1_trimmed = np.array([traj[:min_len] for traj in all_dim1])
                    all_dim2_trimmed = np.array([traj[:min_len] for traj in all_dim2])
                    mean_dim1 = np.mean(all_dim1_trimmed, axis=0)
                    mean_dim2 = np.mean(all_dim2_trimmed, axis=0)
                else:
                    mean_dim1, mean_dim2 = all_dim1[0], all_dim2[0]

                color = method_color_map.get(method_name, next(fallback_colors))
                ax.plot(
                    mean_dim1,
                    mean_dim2,
                    color=color,
                    linewidth=2.5,
                    label=f"{method_name} (Mean)",
                )

                # Add 2D orientation arrows for this projection
                if target_df is not None:
                    step = max(1, len(mean_dim1) // 8)  # Show ~8 orientation arrows
                    for j in range(0, len(mean_dim1), step):
                        if j < len(target_df):
                            # Get full 3D orientation vector
                            dx_r, dy_r, dz_r = _quaternion_to_direction_vector(
                                target_df["robot_quat_w"].iloc[j],
                                target_df["robot_quat_x"].iloc[j],
                                target_df["robot_quat_y"].iloc[j],
                                target_df["robot_quat_z"].iloc[j],
                                length=0.2,  # Smaller arrows for 2D plots
                            )

                            # Project to 2D plane
                            if proj_name == "XY":
                                arrow_u, arrow_v = dx_r, dy_r
                            elif proj_name == "XZ":
                                arrow_u, arrow_v = dx_r, dz_r
                            else:  # YZ
                                arrow_u, arrow_v = dy_r, dz_r

                            ax.arrow(
                                mean_dim1[j],
                                mean_dim2[j],
                                arrow_u,
                                arrow_v,
                                head_width=0.05,
                                head_length=0.05,
                                fc=color,
                                ec=color,
                                alpha=0.7,
                            )

            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_aspect("equal", adjustable="box")
            ax.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
            ax.set_title(f"{proj_name} Projection: {traj_name}")
            ax.grid(True, alpha=0.3)

            plot_path = output_dir / f"trajectory_{proj_name.lower()}_{traj_name}.png"
            fig.savefig(plot_path, format="png", dpi=300, bbox_inches="tight")
            print(f"Saved {proj_name} projection plot: {plot_path}")
            plt.close(fig)


def _generate_summary_plots_and_table(
    exp_definitions: List[Tuple[str, str]], output_dir: Path, caption: str, label: str
):
    """Generates bar charts (as png) and a LaTeX table from all experiment summary files."""
    summary_rows = []
    for method_name, glob_pattern in exp_definitions:
        for path in Path(".").glob(glob_pattern):
            summary_path = path.with_name(f"{path.stem}_summary.json")
            if summary_path.exists():
                with open(summary_path) as f:
                    summary = json.load(f)
                    summary_rows.append(
                        {
                            "Method": method_name,
                            "Trajectory": path.stem.split("_")[0],
                            "ATE (m)": summary.get("ate_m", 0),
                            "Orientation Error (rad)": summary.get("ate_rad", 0),
                            "Jerk (m/s³)": summary.get("jerk_avg_m_s3", 0),
                            "Angular Accel (rad/s²)": summary.get(
                                "angular_accel_avg_rad_s2", 0
                            ),
                            "Max Position Error (m)": summary.get(
                                "max_position_error_m", 0
                            ),
                            "Max Orientation Error (rad)": summary.get(
                                "max_orientation_error_rad", 0
                            ),
                            "Final Position Error (m)": summary.get(
                                "final_position_error_m", 0
                            ),
                            "Final Orientation Error (rad)": summary.get(
                                "final_orientation_error_rad", 0
                            ),
                        }
                    )

    if not summary_rows:
        print("Error: No summary data could be found. Cannot generate report.")
        return
    df_summary = pd.DataFrame(summary_rows)

    print("\n--- Generating 3D Performance Bar Charts (png) ---")
    metrics_to_plot = [
        "ATE (m)",
        "Orientation Error (rad)",
        "Jerk (m/s³)",
        "Angular Accel (rad/s²)",
        "Max Position Error (m)",
        "Max Orientation Error (rad)",
    ]

    for metric in metrics_to_plot:
        if metric not in df_summary.columns or df_summary[metric].sum() == 0:
            print(f"Skipping {metric} - no data available.")
            continue

        fig, ax = plt.subplots(figsize=(10, 6))
        sns.barplot(
            data=df_summary, x="Method", y=metric, ax=ax, palette="viridis", capsize=0.1
        )
        ax.set_title(f"3D Performance Comparison: {metric}")
        ax.tick_params(axis="x", rotation=30)
        fig.tight_layout()

        safe_metric_name = (
            metric.replace(" ", "_")
            .replace("(", "")
            .replace(")", "")
            .replace("/", "_per_")
        )
        plot_path = output_dir / f"comparison_3d_{safe_metric_name.lower()}.png"
        fig.savefig(plot_path, format="png", dpi=300, bbox_inches="tight")
        print(f"Saved comparison plot: {plot_path}")
        plt.close(fig)

    print("\n--- Generating 3D LaTeX Table ---")
    df_agg = df_summary.groupby("Method").agg([np.mean, np.std])

    # Find best performing methods
    ate_means = df_agg[("ATE (m)", "mean")]
    jerk_means = df_agg[("Jerk (m/s³)", "mean")]
    orient_means = df_agg[("Orientation Error (rad)", "mean")]

    min_ate_method = ate_means.idxmin()
    min_jerk_method = jerk_means.idxmin()
    min_orient_method = orient_means.idxmin()

    formatted_rows = {}
    for method, row in df_agg.iterrows():
        ate_str = f"{row[('ATE (m)', 'mean')]:.3f} $\\pm$ {row[('ATE (m)', 'std')]:.3f}"
        orient_str = f"{row[('Orientation Error (rad)', 'mean')]:.3f} $\\pm$ {row[('Orientation Error (rad)', 'std')]:.3f}"
        jerk_str = f"{row[('Jerk (m/s³)', 'mean')]:.3f} $\\pm$ {row[('Jerk (m/s³)', 'std')]:.3f}"

        # Bold the best results
        if method == min_ate_method:
            ate_str = f"\\textbf{{{ate_str}}}"
        if method == min_orient_method:
            orient_str = f"\\textbf{{{orient_str}}}"
        if method == min_jerk_method:
            jerk_str = f"\\textbf{{{jerk_str}}}"

        formatted_rows[method] = {
            "ATE (m)": ate_str,
            "Orientation Error (rad)": orient_str,
            "Jerk (m/s³)": jerk_str,
        }

    df_final = pd.DataFrame.from_dict(formatted_rows, orient="index")

    latex_str = df_final.to_latex(
        index=True,
        escape=False,
        caption=caption,
        label=label,
        column_format="lrrr",
        position="!ht",
    )
    latex_path = output_dir / f"{label}_3d.tex"
    with open(latex_path, "w") as f:
        f.write(latex_str)
    print(f"Saved LaTeX table: {latex_path}")
    print("\n--- LaTeX Source ---")
    print(latex_str)


def run_animation(args: argparse.Namespace):
    """Generates a high-quality MP4 animation of a single 3D run."""
    if isinstance(args.processed_file, (str, Path)):
        proc_paths = [args.processed_file]
    else:
        proc_paths = args.processed_file

    for proc_path in proc_paths:
        proc_path = Path(proc_path)
        if not proc_path.exists():
            print(f"Error: File {proc_path} does not exist.")
            continue

        print(f"--- Generating 3D Animation for {proc_path.name} ---")
        df = pd.read_csv(proc_path)

        # Center the trajectory
        center_x = (df["target_x"].min() + df["target_x"].max()) / 2.0
        center_y = (df["target_y"].min() + df["target_y"].max()) / 2.0
        center_z = (df["target_z"].min() + df["target_z"].max()) / 2.0

        for col in ["robot_x", "target_x"]:
            df[col] = df[col] - center_x
        for col in ["robot_y", "target_y"]:
            df[col] = df[col] - center_y
        for col in ["robot_z", "target_z"]:
            df[col] = df[col] - center_z

        # Create 3D animation
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection="3d")

        # Set axis limits
        all_pos = np.concatenate(
            [
                df[["target_x", "target_y", "target_z"]].values,
                df[["robot_x", "robot_y", "robot_z"]].values,
            ]
        )
        margin = 0.5
        ax.set_xlim(all_pos[:, 0].min() - margin, all_pos[:, 0].max() + margin)
        ax.set_ylim(all_pos[:, 1].min() - margin, all_pos[:, 1].max() + margin)
        ax.set_zlim(all_pos[:, 2].min() - margin, all_pos[:, 2].max() + margin)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio for all axes

        # Plot target path
        ax.plot(
            df["target_x"],
            df["target_y"],
            df["target_z"],
            "r--",
            linewidth=1.5,
            label="Target Path",
            alpha=0.7,
        )

        # Initialize animated elements
        (robot_path,) = ax.plot([], [], [], "b-", linewidth=2.5, label="Robot Path")

        # Only show position markers if not arrows-only mode
        if not args.arrows_only:
            robot_point = ax.scatter(
                [], [], [], color="blue", s=30, label="Robot", alpha=0.6
            )
            target_point = ax.scatter(
                [], [], [], color="red", s=30, label="Target", alpha=0.6
            )
        else:
            robot_point = None
            target_point = None

        # Initialize orientation arrows (will be updated in animation)
        robot_orientation = None
        target_orientation = None

        time_text = fig.text(
            0.02, 0.95, "", fontsize=12, bbox=dict(facecolor="white", alpha=0.8)
        )
        error_text = fig.text(
            0.02, 0.90, "", fontsize=10, bbox=dict(facecolor="white", alpha=0.8)
        )
        ax.legend(loc="upper right")

        def update(frame: int) -> Tuple:
            nonlocal robot_orientation, target_orientation

            # Clear previous orientation arrows
            if robot_orientation is not None:
                robot_orientation.remove()
            if target_orientation is not None:
                target_orientation.remove()

            # Update robot path
            robot_path.set_data(df["robot_x"][: frame + 1], df["robot_y"][: frame + 1])
            robot_path.set_3d_properties(df["robot_z"][: frame + 1])

            # Update current positions
            if frame < len(df):
                robot_x, robot_y, robot_z = (
                    df["robot_x"].iloc[frame],
                    df["robot_y"].iloc[frame],
                    df["robot_z"].iloc[frame],
                )
                target_x, target_y, target_z = (
                    df["target_x"].iloc[frame],
                    df["target_y"].iloc[frame],
                    df["target_z"].iloc[frame],
                )

                # Update position markers only if not in arrows-only mode
                if robot_point is not None and target_point is not None:
                    robot_point._offsets3d = ([robot_x], [robot_y], [robot_z])
                    target_point._offsets3d = ([target_x], [target_y], [target_z])

                # Add orientation arrows
                robot_dx, robot_dy, robot_dz = _quaternion_to_direction_vector(
                    df["robot_quat_w"].iloc[frame],
                    df["robot_quat_x"].iloc[frame],
                    df["robot_quat_y"].iloc[frame],
                    df["robot_quat_z"].iloc[frame],
                    length=0.5,  # Slightly longer arrows for better visibility
                )
                target_dx, target_dy, target_dz = _quaternion_to_direction_vector(
                    df["target_quat_w"].iloc[frame],
                    df["target_quat_x"].iloc[frame],
                    df["target_quat_y"].iloc[frame],
                    df["target_quat_z"].iloc[frame],
                    length=0.5,  # Slightly longer arrows for better visibility
                )

                if args.show_orientation:
                    robot_orientation = ax.quiver(
                        robot_x,
                        robot_y,
                        robot_z,
                        robot_dx,
                        robot_dy,
                        robot_dz,
                        color="blue",
                        alpha=0.9,
                        arrow_length_ratio=0.15,
                        linewidth=4,
                    )  # Thicker, more prominent arrows
                    target_orientation = ax.quiver(
                        target_x,
                        target_y,
                        target_z,
                        target_dx,
                        target_dy,
                        target_dz,
                        color="red",
                        alpha=0.9,
                        arrow_length_ratio=0.15,
                        linewidth=4,
                    )  # Thicker, more prominent arrows

                # Update text with time and error information
                time_text.set_text(f"Time: {df['time'].iloc[frame]:.2f}s")
                pos_error = df["euclidean_error"].iloc[frame]
                orient_error = (
                    df["orientation_error_rad"].iloc[frame] * 180 / np.pi
                )  # Convert to degrees
                error_text.set_text(
                    f"Pos Error: {pos_error:.3f}m | Orient Error: {orient_error:.1f}°"
                )

            # Return only non-None elements for blitting
            return_elements = [robot_path]
            if robot_point is not None:
                return_elements.append(robot_point)
            if target_point is not None:
                return_elements.append(target_point)
            return tuple(return_elements)

        frame_step = max(
            1, len(df) // 450
        )  # Limit to ~450 frames for reasonable file size
        ani = animation.FuncAnimation(
            fig, update, frames=range(0, len(df), frame_step), blit=False, interval=33
        )

        anim_path = (
            args.output_dir
            / f"{proc_path.stem.replace('_processed', '')}_animation_3d.mp4"
        )
        try:
            ani.save(anim_path, writer="ffmpeg", fps=30, bitrate=5000)
            print(f"Saved 3D animation: {anim_path}")
        except FileNotFoundError:
            print(
                "Error: ffmpeg not found. Please install ffmpeg to create animations."
            )
        plt.close(fig)

        # Create 2D projection animations for better orientation visualization
        if not args.no_2d_projections:
            projections = [
                ("XY", "robot_x", "robot_y", "target_x", "target_y", "X (m)", "Y (m)"),
                ("XZ", "robot_x", "robot_z", "target_x", "target_z", "X (m)", "Z (m)"),
                ("YZ", "robot_y", "robot_z", "target_y", "target_z", "Y (m)", "Z (m)"),
            ]

            for (
                proj_name,
                robot_dim1,
                robot_dim2,
                target_dim1,
                target_dim2,
                xlabel,
                ylabel,
            ) in projections:
                fig, ax = plt.subplots(figsize=(10, 8))

                # Set axis limits
                all_pos_1 = np.concatenate(
                    [df[target_dim1].values, df[robot_dim1].values]
                )
                all_pos_2 = np.concatenate(
                    [df[target_dim2].values, df[robot_dim2].values]
                )
                margin = 0.3
                ax.set_xlim(all_pos_1.min() - margin, all_pos_1.max() + margin)
                ax.set_ylim(all_pos_2.min() - margin, all_pos_2.max() + margin)
                ax.set_xlabel(xlabel)
                ax.set_ylabel(ylabel)
                ax.set_aspect("equal", adjustable="box")

                # Plot full target path
                ax.plot(
                    df[target_dim1],
                    df[target_dim2],
                    "r--",
                    linewidth=1.5,
                    label="Target Path",
                    alpha=0.7,
                )

                # Initialize animated elements
                (robot_path,) = ax.plot([], [], "b-", linewidth=2.5, label="Robot Path")

                # Only show markers if not in arrows-only mode
                if not args.arrows_only:
                    (robot_point,) = ax.plot(
                        [], [], "bo", markersize=4, label="Robot", alpha=0.7
                    )
                    (target_point,) = ax.plot(
                        [], [], "ro", markersize=4, label="Target", alpha=0.7
                    )
                else:
                    robot_point, target_point = None, None

                robot_arrow = ax.annotate(
                    "",
                    xy=(0, 0),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color="blue", lw=3, alpha=0.9),
                )
                target_arrow = ax.annotate(
                    "",
                    xy=(0, 0),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color="red", lw=3, alpha=0.9),
                )

                time_text = ax.text(
                    0.02,
                    0.98,
                    "",
                    transform=ax.transAxes,
                    fontsize=12,
                    va="top",
                    bbox=dict(facecolor="white", alpha=0.8),
                )
                error_text = ax.text(
                    0.02,
                    0.93,
                    "",
                    transform=ax.transAxes,
                    fontsize=10,
                    va="top",
                    bbox=dict(facecolor="white", alpha=0.8),
                )
                ax.legend(loc="upper right")
                ax.grid(True, alpha=0.3)

                def update_2d(frame: int):
                    if frame < len(df):
                        # Update paths and points
                        robot_path.set_data(
                            df[robot_dim1][: frame + 1], df[robot_dim2][: frame + 1]
                        )

                        # Update position markers only if not in arrows-only mode
                        if robot_point is not None and target_point is not None:
                            robot_point.set_data(
                                [df[robot_dim1].iloc[frame]],
                                [df[robot_dim2].iloc[frame]],
                            )
                            target_point.set_data(
                                [df[target_dim1].iloc[frame]],
                                [df[target_dim2].iloc[frame]],
                            )

                        # Update orientation arrows
                        robot_pos = (
                            df[robot_dim1].iloc[frame],
                            df[robot_dim2].iloc[frame],
                        )
                        target_pos = (
                            df[target_dim1].iloc[frame],
                            df[target_dim2].iloc[frame],
                        )

                        # Get 3D direction and project to 2D
                        robot_dx, robot_dy, robot_dz = _quaternion_to_direction_vector(
                            df["robot_quat_w"].iloc[frame],
                            df["robot_quat_x"].iloc[frame],
                            df["robot_quat_y"].iloc[frame],
                            df["robot_quat_z"].iloc[frame],
                            length=0.4,  # Slightly longer
                        )
                        target_dx, target_dy, target_dz = (
                            _quaternion_to_direction_vector(
                                df["target_quat_w"].iloc[frame],
                                df["target_quat_x"].iloc[frame],
                                df["target_quat_y"].iloc[frame],
                                df["target_quat_z"].iloc[frame],
                                length=0.4,  # Slightly longer
                            )
                        )

                        # Project to current 2D plane
                        if proj_name == "XY":
                            robot_dir = (robot_dx, robot_dy)
                            target_dir = (target_dx, target_dy)
                        elif proj_name == "XZ":
                            robot_dir = (robot_dx, robot_dz)
                            target_dir = (target_dx, target_dz)
                        else:  # YZ
                            robot_dir = (robot_dy, robot_dz)
                            target_dir = (target_dy, target_dz)

                        if args.show_orientation:
                            robot_arrow.set_position(robot_pos)
                            robot_arrow.xy = (
                                robot_pos[0] + robot_dir[0],
                                robot_pos[1] + robot_dir[1],
                            )
                            target_arrow.set_position(target_pos)
                            target_arrow.xy = (
                                target_pos[0] + target_dir[0],
                                target_pos[1] + target_dir[1],
                            )

                        # Update text
                        time_text.set_text(f"Time: {df['time'].iloc[frame]:.2f}s")
                        pos_error = df["euclidean_error"].iloc[frame]
                        orient_error = (
                            df["orientation_error_rad"].iloc[frame] * 180 / np.pi
                        )
                        error_text.set_text(
                            f"Pos Error: {pos_error:.3f}m | Orient Error: {orient_error:.1f}°"
                        )

                    # Return only non-None elements for blitting
                    return_elements = [robot_path]
                    if robot_point is not None:
                        return_elements.append(robot_point)
                    if target_point is not None:
                        return_elements.append(target_point)
                    return_elements.extend(
                        [robot_arrow, target_arrow, time_text, error_text]
                    )
                    return tuple(return_elements)

                frame_step = max(1, len(df) // 300)
                ani_2d = animation.FuncAnimation(
                    fig,
                    update_2d,
                    frames=range(0, len(df), frame_step),
                    blit=True,
                    interval=50,
                )

                anim_2d_path = (
                    args.output_dir
                    / f"{proc_path.stem.replace('_processed', '')}_animation_{proj_name.lower()}.mp4"
                )
                try:
                    ani_2d.save(anim_2d_path, writer="ffmpeg", fps=20, bitrate=3000)
                    print(f"Saved {proj_name} animation: {anim_2d_path}")
                except FileNotFoundError:
                    print(
                        "Error: ffmpeg not found. Please install ffmpeg to create animations."
                    )
                plt.close(fig)


def run_collection(args: argparse.Namespace):
    """Entry point for the 'collect' command."""
    if not ROS_AVAILABLE:
        print("Error: ROS 2 is not available. Cannot run collection.")
        return

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = TrajectoryCollectorNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()


def run_processing(args: argparse.Namespace):
    """Entry point for the 'process' command."""
    print("--- Starting Batch 3D Data Processing ---")
    for file_path in args.input_files:
        file_path = Path(file_path)
        if file_path.is_file() and "_processed" not in file_path.name:
            _process_file(file_path)
        else:
            print(f"Skipping {file_path} (not a valid raw file or already processed)")


def run_report(args: argparse.Namespace):
    """Entry point for the 'report' command."""
    args.output_dir.mkdir(parents=True, exist_ok=True)
    grouped_results = _load_and_group_experiments(args.exp)
    if not grouped_results:
        print("No data found. Exiting report generation.")
        return
    _plot_3d_trajectories_with_confidence(grouped_results, args.output_dir)
    _generate_summary_plots_and_table(
        args.exp, args.output_dir, args.caption, args.label
    )


def main():
    """Main function to parse arguments and execute the selected command."""
    parser = argparse.ArgumentParser(
        description="A comprehensive tool for 3D trajectory evaluation.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    subparsers = parser.add_subparsers(
        dest="command", required=True, help="Available commands"
    )

    p_collect = subparsers.add_parser(
        "collect",
        help="Run as a ROS 2 node to collect 3D trajectory data.",
        description="Launches a ROS 2 node to listen to TF2 transforms and record raw 3D trajectory data to a CSV file.",
    )
    p_collect.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="Path to the output CSV file for the raw data.",
    )
    p_collect.add_argument(
        "--robot-frame",
        type=str,
        default="robot",
        help="The TF frame of the robot.",
    )
    p_collect.add_argument(
        "--target-frame",
        type=str,
        default="target",
        help="The TF frame of the dynamic target.",
    )
    p_collect.add_argument(
        "--fixed-frame",
        type=str,
        default="world",
        help="The fixed TF frame (e.g., odom, map).",
    )
    p_collect.add_argument(
        "--rate", type=float, default=50.0, help="Data collection frequency (Hz)."
    )
    p_collect.set_defaults(func=run_collection)

    p_process = subparsers.add_parser(
        "process",
        help="Batch process raw CSV files to calculate 3D metrics.",
        description="Takes raw CSV files as input, calculates 3D performance metrics (error, jerk, etc.), and saves both a processed CSV and a summary JSON file for each input.",
    )
    p_process.add_argument(
        "input_files",
        type=Path,
        nargs="+",
        help="One or more raw trajectory CSV files to process.",
    )
    p_process.set_defaults(func=run_processing)

    p_report = subparsers.add_parser(
        "report",
        help="Generate all 3D plots (png) and tables for a publication.",
        description="Compares multiple methods by analyzing processed 3D data. Generates 3D trajectory plots with confidence intervals, 2D projections, performance bar charts with error bars, and a publication-ready LaTeX table.",
    )
    p_report.add_argument(
        "--exp",
        action="append",
        nargs=2,
        metavar=("METHOD_NAME", "RAW_CSV_GLOB"),
        required=True,
        help="Define a method and the glob pattern for its raw CSVs.\nCan be used multiple times for comparison.\nExample: --exp PPO 'data/*_ppo_*.csv' --exp DreamerV3 'data/*_dreamer_*.csv'",
    )
    p_report.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        default=Path("./paper_results_3d"),
        help="Directory to save all generated figures and tables.",
    )
    p_report.add_argument(
        "--caption",
        type=str,
        default="Comparison of Real-World 3D Performance Metrics.",
        help="Caption for the LaTeX table.",
    )
    p_report.add_argument(
        "--label",
        type=str,
        default="tab:results_3d",
        help="Label for the LaTeX table (e.g., 'tab:my_results_3d').",
    )
    p_report.set_defaults(func=run_report)

    p_anim = subparsers.add_parser(
        "animate",
        help="Create an MP4 animation for a single 3D run.",
        description="Generates a high-quality MP4 video of a single 3D trajectory run, showing the robot's path and orientation over time in 3D space.",
    )
    p_anim.add_argument(
        "processed_file",
        type=Path,
        nargs="+",
        help="Path to one or more '..._processed.csv' file to animate.",
    )
    p_anim.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        default=Path("./paper_results_3d"),
        help="Directory to save the animation.",
    )
    p_anim.add_argument(
        "--show-orientation",
        action="store_true",
        default=True,
        help="Show robot and target orientation as arrows in the animation.",
    )
    p_anim.add_argument(
        "--no-2d-projections",
        action="store_true",
        help="Skip generating 2D projection animations (XY, XZ, YZ).",
    )
    p_anim.add_argument(
        "--arrows-only",
        action="store_true",
        help="Show only orientation arrows without position markers for cleaner visualization.",
    )
    p_anim.set_defaults(func=run_animation)

    args, _ = parser.parse_known_args()
    if hasattr(args, "output_dir"):
        args.output_dir.mkdir(parents=True, exist_ok=True)
    args.func(args)


if __name__ == "__main__":
    main()
