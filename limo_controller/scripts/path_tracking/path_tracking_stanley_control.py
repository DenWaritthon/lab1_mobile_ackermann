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

class PathTrackingStanleyController(Node):
    def __init__(self):
        super().__init__('path_tracking_stanley_controller')

        # Path setup ==============================================================================
        # Load the path from the path.yaml file
        path_file = os.path.join(get_package_share_directory('limo_controller'),'config','path.yaml')
        
        with open(path_file, 'r') as file:
            self.path = yaml.safe_load(file)

        # Communication setup ======================================================================
        # Create Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Create Subscriber
        self.ground_truth_subscriber = self.create_subscription(Odometry, '/ground_truth/pose', self.ground_truth_callback, 10)

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

        self.velocity = 0.5
        self.k = 1.0

        self.update_target()

        self.get_logger().info('Path tracking Stanley Controller initialized')
             
    def ground_truth_callback(self, msg:Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
    
    def update_target(self):
        self.target_x = self.path[self.path_index]['x']
        self.target_y = self.path[self.path_index]['y']
        self.target_yaw = self.path[self.path_index]['yaw']

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
        distance_error = math.hypot(error_x, error_y)

        # Calculate the error in yaw
        error_yaw = math.atan2(error_y, error_x)

        # Check if the path is completed           
        if distance_error < 0.1: 
            if self.path_index+1 < len(self.path):
                self.path_index += 1
                self.update_target()
                return
            else:
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info('Path tracking Stanley Controller Completed lap')
                exit()
        
        # Calculate the heading error
        heading_error = error_yaw - self.current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Calculate the cross track error
        path_vector = [math.cos(error_yaw), math.sin(error_yaw)]
        vehicle_vector = [self.current_x - self.target_x, self.current_y - self.target_y]
        direction = np.sign(path_vector[0]*vehicle_vector[1] - path_vector[1]*vehicle_vector[0])

        cross_track_error = distance_error * direction

        # Calculate the steering angle using Stanley control law
        cross_track_steering = math.atan2(self.k * cross_track_error, self.velocity)
        steering_angle = heading_error + cross_track_steering

        # P control
        control_linear = self.velocity
        control_angular = (self.velocity / 0.2) * math.tan(steering_angle)
        
        # Publish Control Commands
        self.publish_cmd(control_linear, control_angular)

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingStanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()