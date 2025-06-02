#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import yaml
import os
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
ref_path_dir = os.path.abspath(os.path.join(script_dir, "..", "..", "share", "limo_controller", "config", "path.yaml"))

class EKFPlotter(Node):
    def __init__(self):
        super().__init__('ekf_plotter')

        self.stop_tracking = False
        self.error_avg_window = 50

        # Transform offset
        self.gt_offset_position = None
        self.gt_offset_yaw = None

        # Subscriptions
        self.create_subscription(Odometry, "/ground_truth/pose", self.ground_truth_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.ekf_callback, 10)
        self.create_subscription(Odometry, "/simulated_gps", self.gps_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Load reference path
        with open(ref_path_dir, 'r') as f:
            self.ref_path = yaml.safe_load(f)
        self.ref_x = [p['x'] for p in self.ref_path]
        self.ref_y = [p['y'] for p in self.ref_path]
        self.ref_points = np.array(list(zip(self.ref_x, self.ref_y)))

        # Data storage
        self.gt_positions = []
        self.gt_orientations = []
        self.ekf_positions = []
        self.ekf_orientations = []
        self.ekf_raw_positions = []
        self.linear_velocities = []
        self.angular_velocities = []
        self.time_steps = []
        self.errors = []
        self.ekf_position_errors = []
        self.ekf_yaw_errors = []
        self.gps_points = []

        # Plot Figure 1
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_title("Reference vs Robot & EKF Path")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # Plot Figure 2 (3 subplots)
        self.fig_err, (self.ax_err, self.ax_pos, self.ax_ori) = plt.subplots(3, 1, figsize=(6, 8))
        self.fig_err.tight_layout(pad=3.0)

        plt.ion()
        plt.show()

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def transform_with_offset(self, pos):
        if self.gt_offset_position is None or self.gt_offset_yaw is None:
            return pos
        R = np.array([
            [np.cos(self.gt_offset_yaw), -np.sin(self.gt_offset_yaw)],
            [np.sin(self.gt_offset_yaw),  np.cos(self.gt_offset_yaw)],
        ])
        return R @ pos + self.gt_offset_position

    def cmd_vel_callback(self, msg):
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.stop_tracking = True
            self.get_logger().info("Received zero twist. Stopping tracking.")

        if not self.stop_tracking:
            self.linear_velocities.append(msg.linear.x)
            self.angular_velocities.append(msg.angular.z)
            self.time_steps.append(len(self.time_steps))

    def ground_truth_callback(self, msg):
        if self.stop_tracking:
            return

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        current_pos = np.array([pos.x, pos.y])
        current_yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        if self.gt_offset_position is None:
            self.gt_offset_position = current_pos
            self.gt_offset_yaw = current_yaw
            self.get_logger().info(f"Set GT offset position: {self.gt_offset_position}, yaw: {self.gt_offset_yaw:.3f}")

        self.gt_positions.append(tuple(current_pos))
        self.gt_orientations.append(current_yaw)

        dists = np.linalg.norm(self.ref_points - current_pos, axis=1)
        min_dist = np.min(dists)
        self.errors.append(min_dist)

        self.update_plot()

    def ekf_callback(self, msg):
        if self.stop_tracking:
            return

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        ekf_pos_raw = np.array([pos.x, pos.y])
        ekf_yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        self.ekf_raw_positions.append(tuple(ekf_pos_raw))
        ekf_pos_transformed = self.transform_with_offset(ekf_pos_raw)
        self.ekf_positions.append(tuple(ekf_pos_transformed))
        self.ekf_orientations.append(ekf_yaw)

        if len(self.gt_positions) > 0:
            gt_pos = np.array(self.gt_positions[-1])
            gt_yaw = self.gt_orientations[-1]
            pos_err = np.linalg.norm(ekf_pos_transformed - gt_pos)
            yaw_err = abs(ekf_yaw - gt_yaw)
            self.ekf_position_errors.append(pos_err)
            self.ekf_yaw_errors.append(yaw_err)

    def gps_callback(self, msg):
        pos = msg.pose.pose.position
        self.gps_points.append((pos.x, pos.y))

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Reference vs Robot & EKF Path")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        self.ax.plot(self.ref_x, self.ref_y, label='Reference Path', linestyle='--', color='blue')

        if self.gt_positions:
            gt_x = [p[0] for p in self.gt_positions]
            gt_y = [p[1] for p in self.gt_positions]
            self.ax.plot(gt_x, gt_y, label='GT Path', color='red')

        if self.ekf_positions:
            ekf_x = [p[0] for p in self.ekf_positions]
            ekf_y = [p[1] for p in self.ekf_positions]
            self.ax.plot(ekf_x, ekf_y, label='EKF Transformed', color='orange')

        # if self.ekf_raw_positions:
        #     raw_x = [p[0] for p in self.ekf_raw_positions]
        #     raw_y = [p[1] for p in self.ekf_raw_positions]
        #     self.ax.plot(raw_x, raw_y, label='EKF Raw', color='gray', linestyle=':')

        if self.gps_points:
            gps_x = [p[0] for p in self.gps_points]
            gps_y = [p[1] for p in self.gps_points]
            self.ax.scatter(gps_x, gps_y, label='Simulated GPS', color='magenta', alpha=0.2, s=10)

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # === Figure 2 ===
        self.ax_err.clear()
        self.ax_err.set_title(f"Tracking Error (Ref Path) w/ {self.error_avg_window}-Step Avg")
        self.ax_err.set_xlabel("Time Step")
        self.ax_err.set_ylabel("Error (m)")
        self.ax_err.grid(True)

        error_color = 'purple'
        self.ax_err.plot(self.errors, label='Tracking Error', color=error_color, alpha=0.2)

        if len(self.errors) >= self.error_avg_window:
            window = self.error_avg_window
            moving_avg = np.convolve(self.errors, np.ones(window) / window, mode='valid')
            self.ax_err.plot(range(window - 1, window - 1 + len(moving_avg)), moving_avg,
                             label=f'{window}-Step Avg', color=error_color, alpha=1.0)

        self.ax_err.legend()

        self.ax_pos.clear()
        self.ax_pos.set_title("EKF Position Error vs GT")
        self.ax_pos.set_xlabel("Time Step")
        self.ax_pos.set_ylabel("Pos Error (m)")
        self.ax_pos.grid(True)
        if self.ekf_position_errors:
            self.ax_pos.plot(self.ekf_position_errors, label='EKF Pos Error', color='green')
            self.ax_pos.legend()

        self.ax_ori.clear()
        self.ax_ori.set_title("EKF Yaw Error vs GT")
        self.ax_ori.set_xlabel("Time Step")
        self.ax_ori.set_ylabel("Yaw Error (rad)")
        self.ax_ori.grid(True)
        if self.ekf_yaw_errors:
            self.ax_ori.plot(self.ekf_yaw_errors, label='EKF Yaw Error', color='blue')
            self.ax_ori.legend()

        self.fig_err.canvas.draw()
        self.fig_err.canvas.flush_events()

        if len(self.errors) > 10:
            mean = np.mean(self.errors)
            var = np.var(self.errors)
            self.get_logger().info(f"Tracking Error Mean: {mean:.6f} | Variance: {var:.6f}")

def main():
    rclpy.init()
    node = EKFPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
