#!/usr/bin/env python3
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class TfToPathNode(Node):
    """Publish a frame's TF trajectory as ``nav_msgs/msg/Path`` for RViz."""

    def __init__(self):
        super().__init__("tf_to_path_node")

        # Parameters
        self.declare_parameter("source_frame", "world")
        self.declare_parameter("target_frame", "target")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_path_length", 1000)

        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.publish_rate = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        if not self.publish_rate > 0.0 or self.publish_rate == float("inf"):
            raise ValueError(
                f"publish_rate_hz must be finite and positive, got {self.publish_rate}"
            )
        self.max_path_length = max(
            1, self.get_parameter("max_path_length").get_parameter_value().integer_value
        )

        self.get_logger().info(
            f"Configuration: Tracking '{self.target_frame}' relative to '{self.source_frame}'"
        )

        # Listen for transforms.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Transient-local QoS gives new subscribers the latest path.
        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            Path, "target_path", qos_profile=latching_qos
        )

        # Store the trajectory history.
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.source_frame

        # Publish at the configured rate.
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("TF to Path node has started.")

    def timer_callback(self):
        """Look up the latest transform and append it to the published path."""
        try:
            # An empty time requests the latest available transform.
            t = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
                Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not get transform: {e}", throttle_duration_sec=5.0
            )
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.source_frame

        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z

        pose.pose.orientation.x = t.transform.rotation.x
        pose.pose.orientation.y = t.transform.rotation.y
        pose.pose.orientation.z = t.transform.rotation.z
        pose.pose.orientation.w = t.transform.rotation.w

        self._append_pose(pose)
        # Stamp and publish the updated path.
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path_msg)

    def _append_pose(self, pose: PoseStamped):
        """Append a pose while retaining only the newest configured history."""
        self.path_msg.poses.append(pose)  # type: ignore[attr-defined]
        excess = len(self.path_msg.poses) - self.max_path_length
        if excess > 0:
            del self.path_msg.poses[:excess]


def main(args=None):
    rclpy.init(args=args)
    node = TfToPathNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
