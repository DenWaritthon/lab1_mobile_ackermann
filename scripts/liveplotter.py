#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
import numpy as np
from math import atan2, sqrt
import os

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')

        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
        self.create_subscription(Odometry, '/odom/single_track', self.odom_callback_factory('single_track'), 10)
        self.create_subscription(Odometry, '/odom/double_track', self.odom_callback_factory('double_track'), 10)
        self.create_subscription(Odometry, '/odom/yaw_rate', self.odom_callback_factory('yaw_rate'), 10)

        self.gt_position = None
        self.gt_orientation = None

        self.odom_data = {
            'single_track': {'positions': [], 'orientations': []},
            'double_track': {'positions': [], 'orientations': []},
            'yaw_rate': {'positions': [], 'orientations': []},
        }

        self.odom_data_point = {key: {'positions': None, 'orientations': None} for key in self.odom_data}
        self.yaw_last = {key: 0.0 for key in self.odom_data}
        self.wrap_count = {key: 0.0 for key in self.odom_data}
        self.gt_orientation_last = 0.0
        self.gt_wrap_count = 0.0

        self.gt_positions = []
        self.gt_orientations = []
        self.times = []
        self.start_time = self.get_clock().now().nanoseconds

        self.cmd_vel_node_is_running = False
        self.flag = True
        self.saved = False

        self.selected_models = ['single_track']

        # === PLOTTING SETUP ===
        self.fig, self.axes = plt.subplots(3, 2, figsize=(12, 10))
        self.fig.suptitle("Toggle Odometry vs Ground Truth (Live)")
        plt.subplots_adjust(left=0.25)

        rax = plt.axes([0.01, 0.4, 0.18, 0.2])
        self.check = CheckButtons(rax, ['single_track', 'double_track', 'yaw_rate'], [True, False, False])
        self.check.on_clicked(self.model_toggle)

        plt.ion()
        plt.show()

        self.timer = self.create_timer(0.1, self.update_plot)

    def model_toggle(self, label):
        if label in self.selected_models:
            self.selected_models.remove(label)
        else:
            self.selected_models.append(label)

    def odom_callback_factory(self, key):
        def callback(msg):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            x, y = pos.x, pos.y
            yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

            if yaw - self.yaw_last[key] < -3.0:
                self.wrap_count[key] += 1
            elif yaw - self.yaw_last[key] > 3.0:
                self.wrap_count[key] -= 1
            self.yaw_last[key] = yaw
            yaw += self.wrap_count[key] * 2 * np.pi

            self.odom_data_point[key]['positions'] = (x, y)
            self.odom_data_point[key]['orientations'] = yaw
        return callback

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index('limo')
            pos = msg.pose[idx].position
            ori = msg.pose[idx].orientation
            self.gt_position = (pos.x, pos.y)
            self.gt_orientation = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

            if self.gt_orientation - self.gt_orientation_last < -3.0:
                self.gt_wrap_count += 1
            elif self.gt_orientation - self.gt_orientation_last > 3.0:
                self.gt_wrap_count -= 1
            self.gt_orientation_last = self.gt_orientation
            self.gt_orientation += self.gt_wrap_count * 2 * np.pi
        except ValueError:
            self.get_logger().warn("Robot model not found in /gazebo/model_states")

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return atan2(siny_cosp, cosy_cosp)

    def update_plot(self):
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        node_names = [f"{ns.rstrip('/')}/{name}" if ns != '/' else f"/{name}"
                    for name, ns in node_names_and_namespaces]

        if '/cmd_vel_node' in node_names:
            self.cmd_vel_node_is_running = True
        if '/cmd_vel_node' not in node_names and self.cmd_vel_node_is_running:
            self.flag = False

        if self.gt_position is not None and self.gt_orientation is not None:
            if self.flag:
                now = self.get_clock().now().nanoseconds
                self.times.append((now - self.start_time) / 1e9)
                self.gt_positions.append(self.gt_position)
                self.gt_orientations.append(self.gt_orientation)

                for key in self.odom_data.keys():
                    self.odom_data[key]['positions'].append(self.odom_data_point[key]['positions'])
                    self.odom_data[key]['orientations'].append(self.odom_data_point[key]['orientations'])

            t = self.times
            gt_x = [p[0] for p in self.gt_positions]
            gt_y = [p[1] for p in self.gt_positions]
            gt_yaw = self.gt_orientations

            odom_x_all, odom_y_all, odom_yaw_all = {}, {}, {}
            err_pos_all, err_yaw_all = {}, {}

            for key in self.selected_models:
                data = self.odom_data[key]
                odom_x_all[key] = [p[0] for p in data['positions']]
                odom_y_all[key] = [p[1] for p in data['positions']]
                odom_yaw_all[key] = data['orientations']
                err_pos_all[key] = [sqrt((ox - gx) ** 2 + (oy - gy) ** 2)
                                    for (ox, oy), (gx, gy) in zip(data['positions'], self.gt_positions)]
                err_yaw_all[key] = [abs(o - g) for o, g in zip(data['orientations'], gt_yaw)]

            titles = [
                "X Position", "Y Position", "Yaw Orientation",
                "Position Error", "Yaw Error", ""
            ]
            plot_types = ['x', 'y', 'yaw', 'err_pos', 'err_yaw', None]

            for ax, title, plot_type in zip(self.axes.flat, titles, plot_types):
                ax.clear()
                if plot_type in ['x', 'y', 'yaw']:
                    ax.plot(t, gt_x if plot_type == 'x' else gt_y if plot_type == 'y' else gt_yaw, label='GT', linestyle='--')
                    for key in self.selected_models:
                        y_vals = odom_x_all[key] if plot_type == 'x' else odom_y_all[key] if plot_type == 'y' else odom_yaw_all[key]
                        ax.plot(t, y_vals, label=key)
                elif plot_type in ['err_pos', 'err_yaw']:
                    for key in self.selected_models:
                        y_vals = err_pos_all[key] if plot_type == 'err_pos' else err_yaw_all[key]
                        ax.plot(t, y_vals, label=key)

                ax.set_title(title)
                ax.legend()
                ax.grid(True)  # ✅ Add grid to every subplot

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()



def main():
    rclpy.init()
    node = LivePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
