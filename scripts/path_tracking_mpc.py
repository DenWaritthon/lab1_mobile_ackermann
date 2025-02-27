#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion


class PathTrackingMPC(Node):
    def __init__(self):
        super().__init__('path_tracking_mpc')
                # Load the path from the path.yaml file
        path_file = os.path.join(get_package_share_directory('lab1_mobile_ackermann'),'config','path.yaml')
        
        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        # Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variable
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.path_index = 0

        self.last_error = 0.0
        self.yaw_error = 0.0

        self.update_target()
        self.get_logger().info(f'Node PathTrackingPurePursuit Start!!!!!')
             
    def odom_callback(self, msg:Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
    
    def update_target(self):
        self.target_x = self.path[self.path_index]['x']
        self.target_y = self.path[self.path_index]['y']

    def control_loop(self):           
        if self.last_error < 1:
            if self.path_index+1 < len(self.path):
                self.path_index += 1
                self.update_target()
            else:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info('Path tracking completed lap')
                return
            
        # Calculate the error
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        error_yaw = math.atan2(error_y, error_x) - self.current_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
        
        distance_error = math.sqrt(error_x**2 + error_y**2)
        
        # MPC Controller

        # Limit the speed
        control_linear = np.clip(control_linear, -0.5, 0.5)
        control_angular = np.clip(control_angular, -1, 1)

        # Publish the control        
        twist_msg = Twist()
        twist_msg.linear.x = control_linear
        twist_msg.angular.z = control_angular
        
        self.cmd_vel_publisher.publish(twist_msg)

        self.get_logger().info(f'Target: {self.target_x}, {self.target_y}, {self.target_yaw}')
        self.get_logger().info(f'Error: {distance_error}, Yaw Error: {error_yaw}, Linear: {control_linear}, Angular: {control_angular}')
        

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()