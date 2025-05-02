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
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class PathTrackingPurePursuit(Node):
    def __init__(self):
        super().__init__('path_tracking_pure_pursuit')

        # Path setup ==============================================================================
        # Load the path from the path.yaml file
        path_file = os.path.join(get_package_share_directory('limo_controller'),'config','path.yaml')
        
        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        # Parameter setup ==========================================================================
        # Declare parameters with default values
        self.declare_parameter('use_ekf', False)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('kp_v', 1.5)
        self.declare_parameter('kp_omega', 3.0)

        # Get the value of a parameter
        self.use_ekf = self.get_parameter('use_ekf').value

        # Add a callback for parameter updates
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # Communication setup ======================================================================
        # Create Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Create Subscriber
        if self.use_ekf:
            self.odom_subscriber = self.create_subscription(Odometry, '/ekf_pose', self.odom_callback, 10)
        else:
            self.odom_subscriber = self.create_subscription(Odometry, '/ground_truth/pose', self.odom_callback, 10)

        # Create Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables ===============================================================================
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.path_index = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.kp_v = self.get_parameter('kp_v').value
        self.kp_omega = self.get_parameter('kp_omega').value

        self.update_target()

        self.get_logger().info('Path tracking Pure Pursuit initialized')

    def parameter_update_callback(self, params:list[Parameter]):
        for param in params:
            if param.name == 'kp_v':
                self.kp_v = param.value
                self.get_logger().info(f"Parameter 'kp_v' updated to: {self.kp_v}")
            elif param.name == 'kp_omega':
                self.kp_omega = param.value
                self.get_logger().info(f"Parameter 'kp_omega' updated to: {self.kp_omega}")
        return SetParametersResult(successful=True)
             
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

    def publish_cmd(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_publisher.publish(twist_msg)

    def control_loop(self):
        # Calculate the error in the x and y coordinates
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y

        # Calculate the distance error
        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Calculate the error in yaw
        error_yaw = math.atan2(error_y, error_x) - self.current_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        # Check if the path is completed           
        if distance_error < self.lookahead_distance: 
            if self.path_index+1 < len(self.path):
                self.path_index += 1
                self.update_target()
                return
            else:
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info('Path tracking Pure Pursuit Completed lap')
                exit()
        
        # P control
        control_linear = self.kp_v * distance_error
        control_angular = self.kp_omega * error_yaw
        
        # Publish Control Commands
        self.publish_cmd(control_linear, control_angular)

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()