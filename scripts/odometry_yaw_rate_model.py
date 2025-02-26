import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
import numpy as np

#!/usr/bin/env python3
class OdometryYawRate(Node):

    def __init__(self):
        super().__init__('odometry_yaw_rate')

         # Subscriber
        self.subscription = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'yaw_rate/odom', 10)

        # Variable
        self.wheelbase = 0.2
        self.track = 0.13
        self.wheel_radius = 0.045
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_x = 0.0
        self.omega_z = 0.0

        self.last_time = self.get_clock().now()

        self.get_logger().info(f'Node OdometryYawRate Start!!!!!')


    def joint_state_callback(self, msg:JointState):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        

        steering_angle_left = msg.position[2]
        steering_angle_right = msg.position[0]
        wheel_speed_back_left = msg.velocity[1] * self.wheel_radius
        wheel_speed_back_right = msg.velocity[5] * self.wheel_radius

        # Calculate position
        x = self.x + self.v_x * dt * np.cos(self.yaw +(self.omega_z * dt / 2))
        y = self.y + self.v_x * dt * np.sin(self.yaw +(self.omega_z * dt / 2))

        # Calculate orientation
        yaw = self.yaw + self.omega_z * dt

        # Calculate linear velocity
        v_x = (wheel_speed_back_left + wheel_speed_back_right) / 2

        # Calculate angular velocity
        self.omega_z = (wheel_speed_back_right - wheel_speed_back_left) / self.track

        # Publish odometry message
        self.pub_odom(x, y, yaw, v_x, msg.header.stamp)

        # Update
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v_x = v_x
        self.last_time = current_time

    def pub_odom(self, x, y, yaw, v_x, stamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        odom_msg.pose.pose.orientation.x = orientation[0]
        odom_msg.pose.pose.orientation.y = orientation[1]
        odom_msg.pose.pose.orientation.z = orientation[2]
        odom_msg.pose.pose.orientation.w = orientation[3]
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.omega_z

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryYawRate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()