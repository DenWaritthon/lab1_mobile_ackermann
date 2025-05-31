#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from kinematics import Kinematics
from tf2_ros import TransformBroadcaster
from utils import pub_odom
from pynput import keyboard
import threading 
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--model', type=str, default="yaw_rate", help="Forward Kinematics Model")
parser.add_argument('--pos_cov', type=float, default=0.0, help="Covariance for Position")
parser.add_argument('--twist_cov', type=float, default=0.0, help="Covariance for Twist")
args_without_ros, ros_args = parser.parse_known_args()

class OdometryNode(Node):
    def __init__(self, args_cli):
        super().__init__(f'odometry_{args_cli.model}_node')
        self.args_cli = args_cli

        if self.args_cli.model == "single_track":
            self.get_logger().info("Currently using [Single Track Model]")
        elif self.args_cli.model == "double_track":
            self.get_logger().info("Currently using [Double Track Model]")
        elif self.args_cli.model == "yaw_rate":
            self.get_logger().info("Currently using [Yaw Rate Model]")
        
        # Communication setup ======================================================================
        # Create Timer ---------------------------------------
        self.create_timer(0.01, self.timer_callback)

        # # Create Subscriber ----------------------------------
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        # # Create Publisher -----------------------------------
        self.odom_pub = self.create_publisher(Odometry, f"/odom/{self.args_cli.model}", 10)

        # Create Transform Broadcaster -----------------------
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster = None

        # Variables ===============================================================================
        self.steering_angle = [0.0, 0.0]
        self.wheel_speed = [0.0, 0.0, 0.0, 0.0]
        self.t_last = self.get_clock().now()
        self.w_Rz = 0.0
        self.ctrl_pressed = [False]

        self.kine = Kinematics(r = 0.045, L = 0.2, B = 0.14)

        # Start keyboard listener in a thread
        self.listener_thread = threading.Thread(target=self.start_keyboard_listener, daemon=True)
        self.listener_thread.start()

    def start_keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        try:
            if key.char == 'r' and self.ctrl_pressed[0]:
                self.get_logger().info("Ctrl+R pressed. Resetting pose.")
                self.kine.reset_pose()
        except AttributeError:
            if key in (keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
                self.ctrl_pressed[0] = True

    def on_release(self, key):
        if key in (keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
            self.ctrl_pressed[0] = False

    def joint_states_callback(self, msg):
        front_left_steering_joint_ind = msg.name.index("front_left_steering_joint")
        front_right_steering_joint_ind = msg.name.index("front_right_steering_joint")

        front_left_wheel_joint_ind = msg.name.index("front_left_wheel_joint")
        front_right_wheel_joint_ind = msg.name.index("front_right_wheel_joint")

        back_left_wheel_joint_ind = msg.name.index("back_left_wheel_joint")
        back_right_wheel_joint_ind = msg.name.index("back_right_wheel_joint")

        self.steering_angle = [msg.position[front_left_steering_joint_ind], msg.position[front_right_steering_joint_ind]]
        self.wheel_speed = [msg.velocity[front_left_wheel_joint_ind],
                            msg.velocity[front_right_wheel_joint_ind],
                            msg.velocity[back_left_wheel_joint_ind],
                            msg.velocity[back_right_wheel_joint_ind]]

    def imu_callback(self, msg):
        self.w_Rz = msg.angular_velocity.z
        # self.get_logger().info(f"IMU angular velocity: {self.w_Rz}")

    def timer_callback(self):
        t = self.get_clock().now()
        dt = (t.nanoseconds - self.t_last.nanoseconds) * 1e-9

        if self.args_cli.model == "single_track":
            angle = self.kine.compute_ackermann_steering_angle(self.steering_angle)
            self.twist = self.kine.forward_single_track(angle, self.wheel_speed[2:])
        elif self.args_cli.model == "double_track":
            self.twist = self.kine.forward_double_track(self.steering_angle, self.wheel_speed[2:], self.wheel_speed[:2])
        elif self.args_cli.model == "yaw_rate":
            self.twist = self.kine.forward_yaw_rate(self.w_Rz, self.wheel_speed[2:])

        self.pos_global, self.ori_global = self.kine.get_pose(self.twist, dt)
        self.t_last = t

        pub_odom(self.odom_pub, t, self.twist, self.pos_global, self.ori_global, self.args_cli.pos_cov, self.args_cli.twist_cov, tf_broadcaster=self.tf_broadcaster, ref_frame=f"odom_{self.args_cli.model}", logger=self.get_logger())


def main(args=None):
    rclpy.init(args=ros_args)
    node = OdometryNode(args_without_ros)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()