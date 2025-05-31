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

class PathPlotter(Node):
    def __init__(self):
        super().__init__('path_plotter')

        self.stop_tracking = False
        self.error_avg_window = 50  # <-- Adjustable moving average window


        # Subscribe to ground truth and velocity
        self.create_subscription(Odometry, "/ground_truth/pose", self.ground_truth_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Load reference path from YAML
        with open(ref_path_dir, 'r') as f:
            self.ref_path = yaml.safe_load(f)

        self.ref_x = [p['x'] for p in self.ref_path]
        self.ref_y = [p['y'] for p in self.ref_path]
        self.ref_points = np.array(list(zip(self.ref_x, self.ref_y)))

        # Ground truth and error storage
        self.gt_positions = []
        self.errors = []

        # Set up path figure
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_title("Reference vs Robot Path")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # Velocity storage
        self.linear_velocities = []
        self.angular_velocities = []
        self.time_steps = []

        # Error + velocity figure with 3 subplots
        self.fig_err, (self.ax_err, self.ax_lin, self.ax_ang) = plt.subplots(3, 1, figsize=(6, 8))
        self.fig_err.tight_layout(pad=3.0)

        # Setup subplot titles and labels
        self.ax_err.set_title("Tracking Error with Moving Average")
        self.ax_err.set_xlabel("Time Step")
        self.ax_err.set_ylabel("Error (m)")
        self.ax_err.grid(True)

        self.ax_lin.set_title("Linear Velocity Command")
        self.ax_lin.set_xlabel("Time Step")
        self.ax_lin.set_ylabel("Linear Velocity (m/s)")
        self.ax_lin.grid(True)

        self.ax_ang.set_title("Angular Velocity Command")
        self.ax_ang.set_xlabel("Time Step")
        self.ax_ang.set_ylabel("Angular Velocity (rad/s)")
        self.ax_ang.grid(True)

        plt.ion()
        plt.show()

    def cmd_vel_callback(self, msg):
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.stop_tracking = True
            self.get_logger().info("Received zero twist. Stopping tracking.")

        # Record velocities if still tracking
        if not self.stop_tracking:
            self.linear_velocities.append(msg.linear.x)
            self.angular_velocities.append(msg.angular.z)
            self.time_steps.append(len(self.time_steps))  # simple step counter


    def ground_truth_callback(self, msg):
        if self.stop_tracking:
            return

        pos = msg.pose.pose.position
        current_pos = np.array([pos.x, pos.y])
        self.gt_positions.append(tuple(current_pos))

        # Compute tracking error
        dists = np.linalg.norm(self.ref_points - current_pos, axis=1)
        min_dist = np.min(dists)
        self.errors.append(min_dist)

        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Reference vs Robot Path")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # Plot reference path
        self.ax.plot(self.ref_x, self.ref_y, label='Reference Path', linestyle='--', color='blue')

        # Plot robot path
        if self.gt_positions:
            gt_x = [p[0] for p in self.gt_positions]
            gt_y = [p[1] for p in self.gt_positions]
            self.ax.plot(gt_x, gt_y, label='Robot Path', color='red')

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Plot tracking error and moving average
        self.ax_err.clear()
        self.ax_err.set_title(f"Tracking Error with {self.error_avg_window}-Step Average")
        self.ax_err.set_xlabel("Time Step")
        self.ax_err.set_ylabel("Error (m)")
        self.ax_err.grid(True)

        error_color = 'purple'
        self.ax_err.plot(self.errors, label='Tracking Error', color=error_color, alpha=0.2)

        if len(self.errors) >= self.error_avg_window:
            window = self.error_avg_window
            moving_avg = np.convolve(self.errors, np.ones(window) / window, mode='valid')
            self.ax_err.plot(range(window - 1, window - 1 + len(moving_avg)), moving_avg,
                             label=f'{window}-Step Average', color=error_color, alpha=1.0)

        # Plot linear velocity
        self.ax_lin.clear()
        self.ax_lin.set_title("Linear Velocity Command")
        self.ax_lin.set_xlabel("Time Step")
        self.ax_lin.set_ylabel("Linear Velocity (m/s)")
        self.ax_lin.grid(True)
        if self.time_steps:
            self.ax_lin.plot(self.time_steps, self.linear_velocities, label='Linear Velocity', color='green')
            self.ax_lin.legend()

        # Plot angular velocity
        self.ax_ang.clear()
        self.ax_ang.set_title("Angular Velocity Command")
        self.ax_ang.set_xlabel("Time Step")
        self.ax_ang.set_ylabel("Angular Velocity (rad/s)")
        self.ax_ang.grid(True)
        if self.time_steps:
            self.ax_ang.plot(self.time_steps, self.angular_velocities, label='Angular Velocity', color='blue')
            self.ax_ang.legend()


        # Print MSE and Variance
        if len(self.errors) > 10:
            mean = np.mean(self.errors)
            var = np.var(self.errors)
            self.get_logger().info(f"Tracking Error Mean: {mean:.6f} | Variance: {var:.6f}")

def main():
    rclpy.init()
    node = PathPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
