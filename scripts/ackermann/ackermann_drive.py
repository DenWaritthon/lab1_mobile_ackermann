#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from ackermann_kinematics import AckermannKinematics

class AckermannDriveNode(Node):
    def __init__(self):
        super().__init__('ackermann_drive_node')

        # Communication setup ======================================================================
        # Create Timer
        self.create_timer(0.01, self.timer_callback)

        # Create Subscriber
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Create Publisher
        self.vel_control_pub = self.create_publisher(Float64MultiArray, "/velocity_controllers/commands", 10)
        self.pos_control_pub = self.create_publisher(Float64MultiArray, "/position_controllers/commands", 10)

        # Variables ===============================================================================
        self.steering_angle_L = 0.0
        self.steering_angle_R = 0.0
        self.w_Wr = 0.0
        self.w_WfL = 0.0
        self.w_WfR = 0.0

        self.kine = AckermannKinematics(r = 0.045, L = 0.2, B = 0.14)

    def cmd_vel_callback(self, msg):
        self.steering_angle_L, self.steering_angle_R, self.w_Wr, self.w_WfL, self.w_WfR = self.kine.get_wheel_speed([msg.linear.x, msg.angular.z])

    def timer_callback(self):
        vel_msg = Float64MultiArray()
        vel_msg.data = [self.w_WfL, self.w_WfR, self.w_Wr, self.w_Wr]
        self.vel_control_pub.publish(vel_msg)

        pos_msg = Float64MultiArray()
        pos_msg.data = [self.steering_angle_R, self.steering_angle_L]
        self.pos_control_pub.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()