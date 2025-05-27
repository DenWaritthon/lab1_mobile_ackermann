#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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

        self.create_subscription(Odometry, "/ground_truth/pose", self.ground_truth_callback, 10)
        self.create_subscription(Odometry, '/odom/single_track', self.odom_callback_factory('single_track'), 10)
        self.create_subscription(Odometry, '/odom/double_track', self.odom_callback_factory('double_track'), 10)
        self.create_subscription(Odometry, '/odom/yaw_rate', self.odom_callback_factory('yaw_rate'), 10)
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

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

        self.steering_angle_history = [[], []]  # [left, right]
        self.front_wheel_speed_history = [[], []]  # [left, right]
        self.rear_wheel_speed_history = [[], []]  # [left, right]


        self.cmd_vel_node_is_running = False
        self.flag = True
        self.saved = False

        self.start_flag = True
        self.gt_position_start = (None, None)
        self.gt_orientation_start = None

        self.selected_models = ['single_track']

        self.odom_flag = False

        self.steering_angle = [0.0, 0.0]
        self.wheel_speed = [0.0, 0.0, 0.0, 0.0]

        # === PLOTTING SETUP ===
        self.fig, self.axes = plt.subplots(3, 3, figsize=(15, 10))  # Updated
        self.fig.suptitle("Toggle Odometry vs Ground Truth (Live)")
        plt.subplots_adjust(left=0.25)

        rax = plt.axes([0.01, 0.4, 0.18, 0.2])
        self.check = CheckButtons(rax, ['single_track', 'double_track', 'yaw_rate'], [True, False, False])
        self.check.on_clicked(self.model_toggle)

        plt.ion()
        plt.show()

        # self.timer = self.create_timer(0.1, self.update_plot)

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
            self.odom_flag = True
        return callback

    def ground_truth_callback(self, msg):
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
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

        self.update_plot()

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

        if self.gt_position is not None and self.gt_orientation is not None and self.odom_flag:
            if self.flag:
                if self.start_flag:
                    self.gt_position_start = self.gt_position
                    self.gt_orientation_start = self.gt_orientation
                    self.start_flag = False
                    

                now = self.get_clock().now().nanoseconds
                self.times.append((now - self.start_time) / 1e9)
                self.gt_positions.append((self.gt_position[1] - self.gt_position_start[1], -self.gt_position[0] + self.gt_position_start[0]))
                self.gt_orientations.append(self.gt_orientation - self.gt_orientation_start)

                self.steering_angle_history[0].append(self.steering_angle[0])
                self.steering_angle_history[1].append(self.steering_angle[1])

                self.front_wheel_speed_history[0].append(self.wheel_speed[0])
                self.front_wheel_speed_history[1].append(self.wheel_speed[1])
                self.rear_wheel_speed_history[0].append(self.wheel_speed[2])
                self.rear_wheel_speed_history[1].append(self.wheel_speed[3])


                for key in self.odom_data.keys():
                    self.odom_data[key]['positions'].append(self.odom_data_point[key]['positions'])
                    self.odom_data[key]['orientations'].append(self.odom_data_point[key]['orientations'])

            # Plotting
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
                "Position Error", "Yaw Error", "X-Y Trajectory",
                "Steering Angle (rad)", "Front Wheel Velocities", "Rear Wheel Velocities"
            ]
            plot_types = [
                'x', 'y', 'yaw',
                'err_pos', 'err_yaw', 'xy',
                'steering', 'front_vel', 'rear_vel'
            ]

            color_map = {
                    'GT': 'gray',
                    'single_track': 'green',
                    'double_track': 'blue',
                    'yaw_rate': 'red'
                }

            for ax, title, plot_type in zip(self.axes.flat, titles, plot_types):
                ax.clear()

                if plot_type in ['x', 'y', 'yaw']:
                    y_gt = gt_x if plot_type == 'x' else gt_y if plot_type == 'y' else gt_yaw
                    ax.plot(t, y_gt, label='GT', linestyle='--', color=color_map['GT'])
                    for key in self.selected_models:
                        y_vals = odom_x_all[key] if plot_type == 'x' else odom_y_all[key] if plot_type == 'y' else odom_yaw_all[key]
                        ax.plot(t, y_vals, label=key, color=color_map.get(key, None))
                elif plot_type in ['err_pos', 'err_yaw']:
                    for key in self.selected_models:
                        y_vals = err_pos_all[key] if plot_type == 'err_pos' else err_yaw_all[key]
                        ax.plot(t, y_vals, label=key, color=color_map.get(key, None))
                elif plot_type == 'steering':
                    ax.plot(t, self.steering_angle_history[0], label='Left Steering', color='purple')
                    ax.plot(t, self.steering_angle_history[1], label='Right Steering', color='magenta')
                elif plot_type == 'front_vel':
                    ax.plot(t, self.front_wheel_speed_history[0], label='Front Left', color='orange')
                    ax.plot(t, self.front_wheel_speed_history[1], label='Front Right', color='brown')
                elif plot_type == 'rear_vel':
                    ax.plot(t, self.rear_wheel_speed_history[0], label='Rear Left', color='cyan')
                    ax.plot(t, self.rear_wheel_speed_history[1], label='Rear Right', color='navy')
                elif plot_type == 'xy':
                    ax.plot(gt_x, gt_y, label='GT', linestyle='--', color=color_map['GT'])
                    for key in self.selected_models:
                        odom_x = odom_x_all[key]
                        odom_y = odom_y_all[key]
                        ax.plot(odom_x, odom_y, label=key, color=color_map.get(key, None))
                    
                    ax.set_aspect('equal', 'box') 
                    ax.set_ylim(-2.2, 2.2)
                    ax.set_xlim(-3.5, 3.5)

                ax.set_title(title)
                ax.legend()
                ax.grid(True)


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