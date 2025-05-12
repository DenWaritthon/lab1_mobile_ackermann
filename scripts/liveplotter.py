#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg' if you have Qt
import matplotlib.pyplot as plt
from math import atan2, sqrt

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')

        # Create subscribers
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
        self.create_subscription(Odometry, '/odom/single_track', self.odom_callback_factory('single_track'), 10)
        self.create_subscription(Odometry, '/odom/double_track', self.odom_callback_factory('double_track'), 10)
        self.create_subscription(Odometry, '/odom/yaw_rate', self.odom_callback_factory('yaw_rate'), 10)

        # Data storage
        self.gt_position = None
        self.gt_orientation = None  # Yaw

        self.odom_data = {
            'single_track': {'positions': [], 'orientations': []},
            'double_track': {'positions': [], 'orientations': []},
            'yaw_rate': {'positions': [], 'orientations': []},
        }

        self.odom_data_point = {
            'single_track': {'positions': None, 'orientations': None},
            'double_track': {'positions': None, 'orientations': None},
            'yaw_rate': {'positions': None, 'orientations': None},
        }

        self.gt_positions = []
        self.gt_orientations = []
        self.times = []

        self.start_time = self.get_clock().now().nanoseconds

        # Plotting setup
        self.fig, self.axes = plt.subplots(2, 2, figsize=(10, 8))
        plt.ion()
        plt.show()

        # Update plot every 0.1 sec
        self.timer = self.create_timer(0.1, self.update_plot)

    def odom_callback_factory(self, key):
        def callback(msg):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            x, y = pos.x, pos.y
            yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

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

        except ValueError:
            self.get_logger().warn("Robot model not found in /gazebo/model_states")

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return atan2(siny_cosp, cosy_cosp)

    def update_plot(self):
        if self.gt_position is not None and self.gt_orientation is not None:
            # self.get_logger().info(f'{self.odom_data}') -> the self.odom_data is working
            # Record time and GT
            now = self.get_clock().now().nanoseconds
            self.times.append((now - self.start_time) / 1e9)
            self.gt_positions.append(self.gt_position)
            self.gt_orientations.append(self.gt_orientation)

            if not self.gt_positions or not self.times:
                return

            t = self.times
            gt_x = [p[0] for p in self.gt_positions]
            gt_yaw = self.gt_orientations

            ax1, ax2, ax3, ax4 = self.axes.flat
            ax1.clear()
            ax2.clear()
            ax3.clear()
            ax4.clear()

            # === Position ===
            ax1.plot(t, gt_x, label='Ground Truth')
            for key, data in self.odom_data.items():
                self.odom_data[key]['positions'].append(self.odom_data_point[key]['positions'])
                odom_x = [p[0] for p in data['positions']]
                ax1.plot(t, odom_x, label=key)
            ax1.set_title("X Position")
            ax1.legend()

            # === Orientation ===
            ax2.plot(t, gt_yaw, label='Ground Truth')
            for key, data in self.odom_data.items():
                self.odom_data[key]['orientations'].append(self.odom_data_point[key]['orientations'])
                ax2.plot(t, data['orientations'], label=key)
            ax2.set_title("Yaw Orientation")
            ax2.legend()

            # === Position Error ===
            for key, data in self.odom_data.items():
                err = [sqrt((ox - gx)**2 + (oy - gy)**2)
                        for (ox, oy), (gx, gy) in zip(data['positions'], self.gt_positions)]
                ax3.plot(t, err, label=key)
            ax3.set_title("Position Error")
            ax3.legend()

            # === Orientation Error ===
            for key, data in self.odom_data.items():
                err = [abs(o - g) for o, g in zip(data['orientations'], gt_yaw)]
                ax4.plot(t, err, label=key)
            ax4.set_title("Orientation Error")
            ax4.legend()

            plt.tight_layout()
            plt.pause(0.01)

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