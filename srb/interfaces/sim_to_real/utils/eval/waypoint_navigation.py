#!/usr/bin/env python3
import argparse
import json
import threading
from pathlib import Path
from typing import Any, Dict, List

try:
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
    import seaborn as sns
    from matplotlib.collections import LineCollection
except ImportError as e:
    print(
        f"Import Error: {e}. The 'analyze' command requires numpy, pandas, matplotlib, and seaborn."
    )
    print("Please install them: pip install numpy pandas matplotlib seaborn")
    exit(1)

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time

PLOT_STYLE = {
    "style": "seaborn-v0_8-whitegrid",
    "font.family": "serif",
    "font.serif": ["Times New Roman"],
    "axes.labelsize": 14,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.fontsize": 12,
    "figure.titlesize": 16,
}


class AnalysisEngine:
    """Handles all calculations and visualizations for a SINGLE experiment run."""

    def __init__(self, df: pd.DataFrame, output_dir: Path, experiment_name: str):
        self.df = df
        self.output_dir = output_dir
        self.experiment_name = experiment_name
        self.summary_metrics: Dict[str, Any] = {}

    def run_full_analysis(self):
        """Executes all analysis steps and saves all artifacts."""
        print(f"\n--- Starting Analysis for '{self.experiment_name}' ---")
        self._calculate_metrics()
        self._save_summary_metrics()
        self._generate_visuals()
        print(f"--- Analysis for '{self.experiment_name}' Complete ---")
        print(f"Results saved in: {self.output_dir}")

    def _calculate_metrics(self):
        """Calculates all key performance indicators from the raw data."""
        print("Calculating performance metrics...")
        df = self.df
        dt = df["time"].diff()
        dt.iloc[0] = dt.iloc[1] if len(dt) > 1 else 0  # Handle first NaN value
        dt[dt == 0] = 1e-6  # Prevent division by zero

        # 1. Tracking Errors
        df["error_x"] = df["target_x"] - df["robot_x"]
        df["error_y"] = df["target_y"] - df["robot_y"]
        df["euclidean_error"] = np.sqrt(df["error_x"] ** 2 + df["error_y"] ** 2)

        # 2. Velocities and Path Lengths
        df["robot_velocity"] = (
            np.sqrt(df["robot_x"].diff() ** 2 + df["robot_y"].diff() ** 2) / dt
        ).fillna(0)
        df["target_velocity"] = (
            np.sqrt(df["target_x"].diff() ** 2 + df["target_y"].diff() ** 2) / dt
        ).fillna(0)
        robot_path_length = (df["robot_velocity"] * dt).sum()
        target_path_length = (df["target_velocity"] * dt).sum()

        # 3. Smoothness (Acceleration and Jerk)
        df["robot_accel"] = (df["robot_velocity"].diff() / dt).fillna(0)
        df["robot_jerk"] = (df["robot_accel"].diff() / dt).fillna(0)

        self.summary_metrics = {
            "experiment_name": self.experiment_name,
            "duration_s": df["time"].iloc[-1] if not df.empty else 0,
            "tracking_error_m": {
                "mean": df["euclidean_error"].mean(),
                "std_dev": df["euclidean_error"].std(),
                "max": df["euclidean_error"].max(),
                "rms": np.sqrt(np.mean(df["euclidean_error"] ** 2)),
            },
            "path_length_m": {
                "robot": robot_path_length,
                "target": target_path_length,
                "efficiency_ratio": robot_path_length / target_path_length
                if target_path_length > 0
                else 0,
            },
            "smoothness": {
                "mean_abs_accel": df["robot_accel"].abs().mean(),
                "mean_abs_jerk": df["robot_jerk"].abs().mean(),
            },
        }

    def _save_summary_metrics(self):
        """Saves the calculated summary metrics to a JSON file."""
        json_path = self.output_dir / f"{self.experiment_name}_summary.json"
        with open(json_path, "w") as f:
            json.dump(self.summary_metrics, f, indent=4)
        print(f"Saved summary metrics to {json_path}")
        # Print summary for quick view
        print(json.dumps(self.summary_metrics, indent=4))

    def _generate_visuals(self):
        """Generates and saves all plots and animations."""
        with plt.style.context(PLOT_STYLE):
            self._plot_dashboard()
            self._generate_animation()

    def _plot_dashboard(self):
        """Creates a single, information-dense 2x2 dashboard figure."""
        print("Generating 2x2 summary dashboard plot...")
        fig, axs = plt.subplots(2, 2, figsize=(16, 14), constrained_layout=True)
        fig.suptitle(
            f"Trajectory Evaluation Dashboard: {self.experiment_name}", fontsize=20
        )
        df = self.df

        ax = axs[0, 0]
        points = df[["robot_x", "robot_y"]].to_numpy().reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        norm = plt.Normalize(df["robot_velocity"].min(), df["robot_velocity"].max())
        lc = LineCollection(segments, cmap="viridis", norm=norm)
        lc.set_array(df["robot_velocity"])
        line = ax.add_collection(lc)
        fig.colorbar(line, ax=ax, label="Robot Velocity (m/s)")

        ax.plot(df["target_x"], df["target_y"], "r--", label="Target Path", linewidth=2)
        ax.set_title("Trajectory Following (Color by Velocity)")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.legend()
        ax.grid(True)
        ax.set_aspect("equal", adjustable="box")

        ax = axs[0, 1]
        ax.plot(df["time"], df["euclidean_error"], color="b")
        ax.set_title("Euclidean Error over Time")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Error [m]")
        ax.grid(True)

        ax = axs[1, 0]
        ax.plot(df["time"], df["target_velocity"], "r--", label="Target Velocity")
        ax.plot(
            df["time"], df["robot_velocity"], "b-", label="Robot Velocity", alpha=0.8
        )
        ax.set_title("Velocity Profiles")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Velocity [m/s]")
        ax.legend()
        ax.grid(True)

        ax = axs[1, 1]
        sns.histplot(data=df, x="euclidean_error", kde=True, ax=ax, color="darkgreen")
        mean_err = df["euclidean_error"].mean()
        ax.axvline(
            mean_err, color="red", linestyle="--", label=f"Mean Error: {mean_err:.3f}m"
        )
        ax.set_title("Distribution of Tracking Errors")
        ax.set_xlabel("Euclidean Error [m]")
        ax.set_ylabel("Frequency")
        ax.legend()
        ax.grid(True)

        path = self.output_dir / f"{self.experiment_name}_dashboard.png"
        fig.savefig(path, dpi=300)
        plt.close(fig)

    def _generate_animation(self):
        """Generates a high-impact MP4 animation of the run."""
        print("Generating high-impact animation...")
        fig, ax = plt.subplots(figsize=(10, 10))
        all_x = np.concatenate([self.df["target_x"], self.df["robot_x"]])
        all_y = np.concatenate([self.df["target_y"], self.df["robot_y"]])
        ax.set_xlim(all_x.min() - 1, all_x.max() + 1)
        ax.set_ylim(all_y.min() - 1, all_y.max() + 1)

        ax.plot(
            self.df["target_x"],
            self.df["target_y"],
            "r--",
            linewidth=1.5,
            label="Full Target Path",
        )
        (robot_path,) = ax.plot([], [], "b-", linewidth=2, label="Robot Path")
        (robot_trail,) = ax.plot([], [], "b-", linewidth=2, alpha=0.3)  # Faded trail
        (target_marker,) = ax.plot([], [], "go", markersize=12, label="Target")
        robot_marker = ax.quiver(
            [], [], [], [], color="blue", scale=20, width=0.008, label="Robot Pose"
        )
        time_text = ax.text(
            0.02,
            0.95,
            "",
            transform=ax.transAxes,
            fontsize=12,
            bbox=dict(facecolor="white", alpha=0.7),
        )

        def update(frame):
            robot_path.set_data(
                self.df["robot_x"][: frame + 1], self.df["robot_y"][: frame + 1]
            )
            trail_start = max(0, frame - 30)
            robot_trail.set_data(
                self.df["robot_x"][trail_start : frame + 1],
                self.df["robot_y"][trail_start : frame + 1],
            )
            target_marker.set_data(
                [self.df["target_x"][frame]], [self.df["target_y"][frame]]
            )

            x, y, yaw = (
                self.df["robot_x"][frame],
                self.df["robot_y"][frame],
                self.df["robot_yaw"][frame],
            )
            robot_marker.set_offsets(np.c_[x, y])
            robot_marker.set_UVC(np.cos(yaw), np.sin(yaw))

            time_text.set_text(
                f"Time: {self.df['time'][frame]:.2f}s\nError: {self.df['euclidean_error'][frame]:.3f}m"
            )
            return robot_path, robot_trail, target_marker, robot_marker, time_text

        num_frames = len(self.df)
        frame_step = max(1, num_frames // 300)  # Aim for a ~10-second video at 30fps
        ani = animation.FuncAnimation(
            fig, update, frames=range(0, num_frames, frame_step), blit=True, interval=33
        )

        path = self.output_dir / f"{self.experiment_name}_animation.mp4"
        try:
            writer = animation.FFMpegWriter(
                fps=30, metadata=dict(artist="Gemini"), bitrate=2500
            )
            ani.save(path, writer=writer)
            print(f"Saved animation to {path}")
        except FileNotFoundError:
            print("\n--- FFmpeg NOT FOUND ---")
            print(
                "Animation not saved. Install ffmpeg to generate videos: sudo apt-get install ffmpeg\n"
            )
        plt.close(fig)


class TrajectoryCollectorNode(Node):
    """ROS 2 Node for high-fidelity recording of trajectory data."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("trajectory_collector")
        self.output_dir = args.output_dir
        self.experiment_name = args.experiment_name
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.fixed_frame = args.fixed_frame
        self.robot_frame = args.robot_frame
        self.target_frame = args.target_frame

        self.data_lock = threading.Lock()
        self.data: List[Dict[str, float]] = []
        self.start_time = None

        self.timer = self.create_timer(1.0 / args.rate, self.timer_callback)
        self.get_logger().info(
            f"Collector started for experiment '{self.experiment_name}'."
        )
        self.get_logger().info(
            f"Recording from TF frames '{self.robot_frame}' and '{self.target_frame}' relative to '{self.fixed_frame}'."
        )
        self.get_logger().info("Press Ctrl+C to stop collection and start analysis.")

    def timer_callback(self):
        try:
            now = Time()
            robot_tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.robot_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
            target_tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.target_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.02),
            )

            if self.start_time is None:
                self.start_time = self.get_clock().now()
                self.get_logger().info(
                    "First transform pair received. Recording started."
                )

            time_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            q = robot_tf.transform.rotation
            robot_yaw = np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2)
            )

            with self.data_lock:
                self.data.append(
                    {
                        "time": time_elapsed,
                        "robot_x": robot_tf.transform.translation.x,
                        "robot_y": robot_tf.transform.translation.y,
                        "robot_yaw": robot_yaw,
                        "target_x": target_tf.transform.translation.x,
                        "target_y": target_tf.transform.translation.y,
                    }
                )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=5.0)

    def run_post_collection_analysis(self):
        """Called on shutdown to save data and trigger the analysis engine."""
        self.get_logger().info("Shutdown signal received.")
        if not self.data:
            self.get_logger().warn("No data was collected. Exiting without analysis.")
            return

        df = pd.DataFrame(self.data)
        raw_csv_path = self.output_dir / f"{self.experiment_name}_raw_data.csv"
        df.to_csv(raw_csv_path, index=False)
        self.get_logger().info(f"Saved raw trajectory data to {raw_csv_path}")

        engine = AnalysisEngine(df, self.output_dir, self.experiment_name)
        engine.run_full_analysis()


class ComparisonAnalyzer:
    """Handles comparison of multiple completed experiment runs."""

    def __init__(self, experiment_dirs: List[Path], output_dir: Path):
        self.experiment_dirs = experiment_dirs
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def run(self):
        """Loads all data, generates comparison plots, and creates a LaTeX table."""
        print("\n--- Starting Comparison Analysis ---")
        print(f"Loading results from: {[str(d) for d in self.experiment_dirs]}")

        all_metrics = []
        for directory in self.experiment_dirs:
            exp_name = directory.name
            summary_path = directory / f"{exp_name}_summary.json"
            if summary_path.exists():
                with open(summary_path, "r") as f:
                    data = json.load(f)
                    flat_data = {
                        "Experiment": data["experiment_name"],
                        "RMSE (m)": data["tracking_error_m"]["rms"],
                        "Max Error (m)": data["tracking_error_m"]["max"],
                        "Path Efficiency": data["path_length_m"]["efficiency_ratio"],
                        "Mean Abs Jerk": data["smoothness"]["mean_abs_jerk"],
                    }
                    all_metrics.append(flat_data)
            else:
                print(f"Warning: No summary file found at {summary_path}")

        if not all_metrics:
            print("No valid experiment data found. Exiting.")
            return

        df_comp = pd.DataFrame(all_metrics).sort_values(by="RMSE (m)")

        with plt.style.context(PLOT_STYLE):
            self._plot_comparison(df_comp)

        self._generate_latex_table(df_comp)
        print(f"--- Comparison Complete. Results saved in: {self.output_dir} ---")

    def _plot_comparison(self, df_comp: pd.DataFrame):
        """Generates bar charts comparing key metrics."""
        metrics_to_plot = ["RMSE (m)", "Path Efficiency", "Mean Abs Jerk"]
        for metric in metrics_to_plot:
            fig, ax = plt.subplots(figsize=(10, 6))
            sns.barplot(
                data=df_comp, x="Experiment", y=metric, ax=ax, palette="viridis"
            )
            ax.set_title(f"Comparison of {metric}", fontsize=16)
            plt.xticks(rotation=45, ha="right")
            fig.tight_layout()
            plot_path = (
                self.output_dir / f"comparison_{metric.replace(' ', '_').lower()}.png"
            )
            fig.savefig(plot_path)
            print(f"Saved comparison plot to {plot_path}")
            plt.close(fig)

    def _generate_latex_table(self, df_comp: pd.DataFrame):
        """Generates a publication-ready LaTeX table."""
        latex_str = df_comp.to_latex(
            index=False,
            caption="Comparison of Key Performance Metrics Across Experiments.",
            label="tab:perf_comparison",
            float_format="%.3f",
            column_format="l" + "r" * (len(df_comp.columns) - 1),
        )
        latex_path = self.output_dir / "comparison_table.tex"
        with open(latex_path, "w") as f:
            f.write(latex_str)
        print(f"Saved LaTeX table to {latex_path}")
        print("\n--- LaTeX Table ---")
        print(latex_str)


def run_collection(args: argparse.Namespace):
    """Initializes ROS and runs the TrajectoryCollectorNode."""
    if "rclpy" not in globals() or not hasattr(rclpy, "init"):
        print("Error: ROS 2 is not sourced or installed. Cannot run 'collect'.")
        return

    rclpy.init()
    node = TrajectoryCollectorNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.run_post_collection_analysis()
        node.destroy_node()
        rclpy.shutdown()


def run_comparison(args: argparse.Namespace):
    """Runs the offline comparison analysis."""
    analyzer = ComparisonAnalyzer(
        experiment_dirs=[Path(p) for p in args.result_dirs],
        output_dir=Path(args.output),
    )
    analyzer.run()


def main():
    parser = argparse.ArgumentParser(
        description="A comprehensive, single-file tool for trajectory evaluation.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    p_collect = subparsers.add_parser(
        "collect",
        help="Run as a ROS 2 node to collect trajectory data.",
        description="""
Launches the ROS 2 node to listen to TF2 transforms and record data.
When stopped with Ctrl+C, it automatically saves the raw data and generates
a full analysis report (dashboard, animation, summary JSON) for this single run.
""",
    )
    p_collect.add_argument(
        "--experiment-name",
        type=str,
        required=True,
        help="Unique name for this experiment run.",
    )
    p_collect.add_argument(
        "--output-dir-base",
        type=str,
        default="/tmp/trajectory_eval",
        help="Base directory to save experiment results.",
    )
    p_collect.add_argument(
        "--robot-frame",
        type=str,
        default="leo_0/base_link",
        help="The TF frame of the robot.",
    )
    p_collect.add_argument(
        "--target-frame",
        type=str,
        default="target_waypoint",
        help="The TF frame of the dynamic target.",
    )
    p_collect.add_argument(
        "--fixed-frame",
        type=str,
        default="odom",
        help="The fixed TF frame (e.g., odom or map).",
    )
    p_collect.add_argument(
        "--rate", type=float, default=30.0, help="Data collection frequency (Hz)."
    )
    p_collect.set_defaults(func=run_collection)

    p_analyze = subparsers.add_parser(
        "analyze",
        help="Run offline analysis to compare multiple experiment results.",
        description="""
Compares multiple completed experiment runs. This command does not require ROS.
It reads the 'summary.json' file from each specified experiment directory,
then generates summary plots and a LaTeX table comparing all runs.
""",
    )
    p_analyze.add_argument(
        "result_dirs",
        type=str,
        nargs="+",
        help="List of individual experiment directories to compare.",
    )
    p_analyze.add_argument(
        "--output",
        "-o",
        type=str,
        default="./comparison_results",
        help="Directory to save comparison outputs.",
    )
    p_analyze.set_defaults(func=run_comparison)

    args = parser.parse_args()

    # For the 'collect' command, construct the specific output directory for the run
    if args.command == "collect":
        args.output_dir = Path(args.output_dir_base) / args.experiment_name

    # Execute the function associated with the chosen subcommand
    args.func(args)


if __name__ == "__main__":
    main()
