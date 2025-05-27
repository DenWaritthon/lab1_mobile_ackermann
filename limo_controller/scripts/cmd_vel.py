#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--v_x', type=float, default=0.0, help="Robot linear velocity")
parser.add_argument('--w_z', type=float, default=0.0, help="Robot angular velocity")
parser.add_argument('--time', type=float, default=5.0, help="Time to run the node")
args_without_ros, ros_args = parser.parse_known_args()

class CmdVelNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')

        # Communication setup ======================================================================
        # Create Timer
        self.create_timer(0.01, self.timer_callback)

        # Create Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Variables ===============================================================================
        self.count = 0
        self.count_max = int(args_without_ros.time / 0.01)

    def timer_callback(self):
        if self.count < self.count_max and not (args_without_ros.v_x == 0.0 and args_without_ros.w_z == 0.0):
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = args_without_ros.v_x
            cmd_vel_msg.angular.z = args_without_ros.w_z
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.count += 1
        else:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.destroy_node()

def main(args=None):
    rclpy.init(args=ros_args)
    node = CmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()