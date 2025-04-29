#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

class PoseToTFNode(Node):
    """
    ROS2 Node to convert PoseStamped messages to TransformStamped and publish them.
    """
    def __init__(self):
        super().__init__('pose_to_tf_node')

        # Publisher to /mavros/fake_gps/mocap/tf
        self.tf_pub = self.create_publisher(
            TransformStamped,
            '/mavros/fake_gps/mocap/tf',
            10
        )

        # Subscriber to /mavros/vision_pose/pose_cov
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/mavros/vision_pose/pose_cov',
            self.pose_callback,
            10
        )

        self.get_logger().info("pose_to_tf_node initialized and running")

    def pose_callback(self, pose_cov_msg):
        """
        Callback function for the mocap pose_cov subscriber.

        Converts the mocap pose to a TransformStamped message and publishes it.
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "fix"

        # Copy the position
        t.transform.translation.x = pose_cov_msg.pose.pose.position.x
        t.transform.translation.y = pose_cov_msg.pose.pose.position.y
        t.transform.translation.z = pose_cov_msg.pose.pose.position.z

        # Copy the orientation
        t.transform.rotation.x = pose_cov_msg.pose.pose.orientation.x
        t.transform.rotation.y = pose_cov_msg.pose.pose.orientation.y
        t.transform.rotation.z = pose_cov_msg.pose.pose.orientation.z
        t.transform.rotation.w = pose_cov_msg.pose.pose.orientation.w

        # Publish the transform to /mavros/fake_gps/mocap/tf
        self.tf_pub.publish(t)

def main():
    rclpy.init()
    node = PoseToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down pose_to_tf_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

