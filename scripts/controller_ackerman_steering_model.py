#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class AckermanSteeringController(Node):

    def __init__(self):
        super().__init__('ackerman_steering_controller')

        # Subscriber
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)

        # Publisher
        self.velocity_pub= self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)

        # Variable
        self.steering_max = 0.523598767
        self.steering_min = -0.523598767
        self.wheelbase = 0.2
        self.track = 0.13
        self.wheel_radius = 0.045

        self.get_logger().info(f'Node Ackerman Steering Controller Start!!!!!')

    def cmd_vel_callback(self, msg:Twist):
        v = msg.linear.x
        omega = float(msg.angular.z)

        # Calculate wheel speed
        wheel_speed = v / self.wheel_radius


        # Calculate the steering angle
        delta = np.arctan2(omega * self.wheelbase, v)
        if delta >= 3.14:
            delta = 0.0

        # Limit the steering angle
        delta = float(np.clip(delta, self.steering_min, self.steering_max))

        # Calculate the steering angle for left and right wheels
        dalta_left = np.arctan2(self.wheelbase * np.tan(delta), self.wheelbase + (0.5 * self.track * np.tan(delta)))
        dalta_right = np.arctan2(self.wheelbase * np.tan(delta), self.wheelbase - (0.5 * self.track * np.tan(delta)))

        # Ackerman percentage to adjust the outside wheel angle.
        ackerman_percentage = 0.8
        if delta == 0:
            dalta_left = 0
            dalta_right = 0
        elif delta > 0: # Turning left
            dalta_right = dalta_right - (ackerman_percentage * (dalta_right - delta))
        else: # Turning right
            dalta_left = dalta_left - (ackerman_percentage * (dalta_right - delta))

        # Limit the steering angle for left and right wheels
        dalta_left = float(np.clip(dalta_left, self.steering_min, self.steering_max))
        dalta_right = float(np.clip(dalta_right, self.steering_min, self.steering_max))

        # Publish the velocity and steering angle
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [wheel_speed, wheel_speed, wheel_speed, wheel_speed]
        self.velocity_pub.publish(velocity_msg)

        steering_msg = Float64MultiArray()
        steering_msg.data = [dalta_left, dalta_right]
        self.steering_pub.publish(steering_msg)

        self.get_logger().info(f'v: {v}, wheel_speed: {wheel_speed}, omega: {omega}, delta: {delta}, delta_left: {dalta_left}, delta_right: {dalta_right}')

def main(args=None):
    rclpy.init(args=args)
    node = AckermanSteeringController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()