#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped, Quaternion

class GroundTruthTranfrom(Node):
    def __init__(self):
        super().__init__('ground_truth_tranfrom_node')

        # Communication setup ======================================================================
        # Create Subscriber
        self.create_subscription(Odometry, "/ground_truth/pose", self.ground_truth_callback, 10)

        # Create TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Ground Truth Transform Node Initialized")

    def ground_truth_callback(self, msg:Odometry):
        # Create TransformStamped message
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = "base_footprint"

        # Set translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Set rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTranfrom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
