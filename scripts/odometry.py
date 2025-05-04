#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from kinematics import Kinematics
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from utils import pub_odom
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--model', type=str, default="yaw_rate", help="Forward Kinematics Model")
args_without_ros, ros_args = parser.parse_known_args()

class OdometryNode(Node):
    def __init__(self, args_cli):
        super().__init__('odometry_node')
        self.args_cli = args_cli

        if self.args_cli.model == "single_track":
            self.get_logger().info("Currently using [Single Track Model]")
        elif self.args_cli.model == "double_track":
            self.get_logger().info("Currently using [Double Track Model]")
        
        # Communication setup ======================================================================
        # Create Timer ---------------------------------------
        self.create_timer(0.01, self.timer_callback)

        # # Create Subscriber ----------------------------------
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        # # Create Publisher -----------------------------------
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Create Transform Broadcaster -----------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables ===============================================================================
        self.steering_angle = [0.0, 0.0]
        self.wheel_speed = [0.0, 0.0, 0.0, 0.0]
        self.t_last = self.get_clock().now()

        self.kine = Kinematics(r = 0.045, L = 0.2, B = 0.14)

    def joint_states_callback(self, msg):
        self.steering_angle = [msg.position[1], msg.position[0]]
        self.wheel_speed = msg.velocity[2:]

    def timer_callback(self):
        ackerman_steering_angle = self.kine.compute_ackermann_steering_angle(self.steering_angle)
        
        if self.args_cli.model == "single_track":
            self.twist = self.kine.forward_single_track(ackerman_steering_angle, self.wheel_speed[2:])
        elif self.args_cli.model == "double_track":
            self.twist = self.kine.forward_double_track(self.steering_angle, self.wheel_speed[2:], self.wheel_speed[:2])
        # elif args_cli.model == "yaw_rate":
        #     self.twist = self.kine.forward_yaw_rate(self.wheel_speed[2], self.wheel_speed[2:])

        t = self.get_clock().now()
        self.pos_global, self.ori_global = self.kine.get_pose(self.twist, (t.nanoseconds - self.t_last.nanoseconds)/(10**9))     # dt must be matched with timer
        self.t_last = t

        pub_odom(self.odom_pub, self.tf_broadcaster, t, self.twist, self.pos_global, self.ori_global)
    
def main(args=None):
    rclpy.init(args=ros_args)
    node = OdometryNode(args_without_ros)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()