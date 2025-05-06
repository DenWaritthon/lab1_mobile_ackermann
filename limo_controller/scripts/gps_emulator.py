#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random

class GPSEmulator(Node):

    def __init__(self):
        super().__init__('gps_emulator_node')

        # Communication setup ======================================================================
        self.odom_sub = self.create_subscription(Odometry,'/ground_truth/pose',self.odom_callback,10)

        self.gps_pub = self.create_publisher(Odometry,'/simulated_gps',10)

        # Variables ===============================================================================
        self.noise_stddev = 1.0

        self.get_logger().info("GPS Emulator Node started, publishing noisy GPS data")

    def odom_callback(self, msg: Odometry):
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id

        # Add Gaussian noise to position
        noisy_msg.pose.pose.position.x = msg.pose.pose.position.x + random.gauss(0, self.noise_stddev)
        noisy_msg.pose.pose.position.y = msg.pose.pose.position.y + random.gauss(0, self.noise_stddev)
        noisy_msg.pose.pose.position.z = msg.pose.pose.position.z + random.gauss(0, self.noise_stddev)

        # Copy orientation unchanged
        noisy_msg.pose.pose.orientation = msg.pose.pose.orientation

        # Optionally, you could add noise to velocity too
        noisy_msg.twist = msg.twist

        self.gps_pub.publish(noisy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
