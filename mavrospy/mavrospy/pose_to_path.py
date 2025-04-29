#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PoseToPathNode(Node):
    """
    Node to convert PoseStamped topics into Path for visualization.
    """
    def __init__(self):
        super().__init__('pose_to_path')

        # Get the topic name from parameters (default: /mavros/local_position/pose)
        self.declare_parameter('topic', '/mavros/local_position/pose')
        topic = self.get_parameter('topic').value

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "map"

        # Configure QoS profile for subscribing to px4 topics
        px4_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Configure QoS profile for publishing messages for rviz
        rviz_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to PoseStamped messages
        self.pose_sub = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            px4_qos_profile
        )

        # Publisher for the Path and Pose messages
        self.path_pub = self.create_publisher(Path, '/rviz/path', rviz_qos_profile)
        self.pose_pub = self.create_publisher(PoseStamped, '/rviz/pose', rviz_qos_profile)

        self.get_logger().info(f"Subscribed to topic: {topic}")
        self.get_logger().info("Publishing Pose messages on /rviz/pose and Path messages on /rviz/path")

    def pose_callback(self, msg: PoseStamped):
        """
        Callback function to process PoseStamped messages and update the Path.
        """
        # Update header timestamp
        self.path.header.stamp = self.get_clock().now().to_msg()

        # Append the new pose to the path
        self.path.poses.append(msg)

        # Publish the updated pose and path
        self.pose_pub.publish(msg)
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = PoseToPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down pose_to_path node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
