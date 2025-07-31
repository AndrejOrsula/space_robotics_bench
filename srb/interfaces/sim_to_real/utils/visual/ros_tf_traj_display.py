#!/usr/bin/env python3
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class TfToPathNode(Node):
    """
    A ROS 2 node that subscribes to the transform (TF) of a specific frame
    and republishes its trajectory as a nav_msgs/msg/Path. This path can be
    visualized in RViz to see the frame's movement history.
    """

    def __init__(self):
        super().__init__("tf_to_path_node")

        # --- Parameters ---
        # Declare and get parameters to make the node more configurable.
        self.declare_parameter("source_frame", "world")
        self.declare_parameter("target_frame", "target")
        self.declare_parameter("publish_rate_hz", 50.0)

        self.source_frame = (
            self.get_parameter("source_frame").get_parameter_value().string_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.publish_rate = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"Configuration: Tracking '{self.target_frame}' relative to '{self.source_frame}'"
        )

        # --- TF2 Listener ---
        # The buffer stores received transforms and the listener subscribes to the /tf topic.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Path Publisher ---
        # We use a Latching QoS to ensure that new subscribers to the path topic
        # receive the last published message, which contains the full trajectory.
        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            Path, "target_path", qos_profile=latching_qos
        )

        # --- Path Message ---
        # This will store the history of poses for the path.
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.source_frame

        # --- Timer ---
        # The timer triggers the main logic at the specified rate.
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("TF to Path node has started.")

    def timer_callback(self):
        """
        Main logic of the node, executed at a fixed rate by the timer.
        It looks up the transform, creates a PoseStamped message, appends it
        to the path, and publishes the updated path.
        """
        try:
            # Look up the latest transform between the source and target frames.
            # rclpy.time.Time() means get the latest available transform.
            t = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),  # Add a small timeout to avoid blocking
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not get transform: {e}", throttle_duration_sec=5.0
            )
            return

        # Create a new PoseStamped message from the transform.
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.source_frame

        # The transform's translation becomes the pose's position.
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z

        # Rotation is not preserved as requested, so we use an identity quaternion.
        pose.pose.orientation.w = t.transform.rotation.w
        pose.pose.orientation.x = t.transform.rotation.x
        pose.pose.orientation.y = t.transform.rotation.y
        pose.pose.orientation.z = t.transform.rotation.z

        # Append the new pose to the path's list of poses.
        self.path_msg.poses.append(pose)

        # Update the path's header timestamp and publish the message.
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfToPathNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
