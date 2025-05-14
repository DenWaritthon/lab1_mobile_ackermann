#!/usr/bin/python3

import numpy as np

class Kinematics():
    def __init__(self, r: float, L: float, B: float, kinematic_slip_angle: float = 0):
        """
        Initialize the kinematic parameters.

        :param r: Wheel radius.
        :param L: Distance between front and rear axles (wheelbase).
        :param B: Track width (distance between left and right wheels).
        :param kinematic_slip_angle: Slip angle to be subtracted from steering (default is 0).
        """
        self.r = r
        self.L = L
        self.B = B
        self.kinematic_slip_angle = kinematic_slip_angle

        self.pos_global = [0.0, 0.0]
        self.ori_global = 0

    def compute_ackermann_steering_angle(self, steering_angle: list):
        """
        Compute the Ackermann steering angles for left and right wheels.

        :param steering_angle: [steering_angle_L, steering_angle_R] - steering angles relative to the x-axis.
        :return: [steering_angle_L, steering_angle_R] - adjusted steering angles.
        """
        return np.arctan(2*np.tan(steering_angle[0])*np.tan(steering_angle[1])/(np.tan(steering_angle[0]) + np.tan(steering_angle[1])))

    def inverse(self, twist: list):
        """
        Placeholder for inverse kinematics to be implemented in child class.

        :param twist: [v_Rx, w_Rz] where v_Rx is the linear velocity in the x-axis,
                      and w_Rz is the angular velocity around the z-axis.
        """
        pass

    def forward_double_track(self, steering_angle: list, w_Wr: list, w_Wf: list):
        """
        Forward kinematics for a double-track model (with two front steering wheels).

        :param steering_angle: [steering_angle_L, steering_angle_R] - steering angles relative to the x-axis.
        :param w_Wr: [w_WrL, w_WrR] - Rear wheel angular velocity.
        :param w_Wf: [w_WfL, w_WfR] - front left and right wheel angular velocities.
        :return: [v_Rx, w_Rz] - Linear and angular velocity in the robot frame.
        """
        v_Rx = self.r * np.sum(w_Wr) / 2.0
        v_Wf = np.array(w_Wf) * self.r

        N1 = v_Wf[0] * np.cos(steering_angle[1] - self.kinematic_slip_angle)
        N2 = v_Wf[1] * np.cos(steering_angle[0] - self.kinematic_slip_angle)

        D1 = self.L * np.sin(steering_angle[0]) * np.cos(steering_angle[1] - self.kinematic_slip_angle)
        D2 = (self.B / 2) * np.cos(steering_angle[0]) * np.cos(steering_angle[1] - self.kinematic_slip_angle)
        D3 = self.L * np.sin(steering_angle[1]) * np.cos(steering_angle[0] - self.kinematic_slip_angle)
        D4 = (self.B / 2) * np.cos(steering_angle[1]) * np.cos(steering_angle[0] - self.kinematic_slip_angle)

        w_Rz = (N1 - N2) / (D1 + D2 + D3 + D4)

        return [v_Rx, w_Rz]

    def forward_single_track(self, steering_angle: float, w_Wr: list):
        """
        Forward kinematics for a single-track vehicle model.

        :param steering_angle: Steering angle relative to the x-axis.
        :param w_Wr: [w_WrL, w_WrR] - Rear wheel angular velocity.
        :return: [v_Rx, w_Rz] - Linear and angular velocity in the robot frame.
        """
        v_Rx = self.r * np.sum(w_Wr) / 2.0
        w_Rz = (v_Rx / self.L) * np.tan(steering_angle)

        return [v_Rx, w_Rz]

    def forward_yaw_rate(self, w_Rz: float, w_Wr: list):
        """
        Compute forward velocity from known yaw rate and wheel speed.

        :param w_Rz: Angular velocity around the z-axis (yaw rate).
        :param w_Wr: [w_WrL, w_WrR] - Rear wheel angular velocity.
        :return: [v_Rx, w_Rz] - Linear and angular velocity in the robot frame.
        """
        v_Rx = self.r * np.sum(w_Wr) / 2.0

        return [v_Rx, w_Rz]

    def get_pose(self, twist: list, dt: float):
        """
        Update and return global pose using velocities and timestep.

        :param twist: [v_Rx, w_Rz] - linear and angular velocity in robot frame.
        :param dt: Time step for integration.
        :return: Tuple (pos_global, ori_global)
                 - pos_global: [x, y] position in the global frame.
                 - ori_global: Orientation (theta) in radians.
        """
        v_Rx = twist[0]
        w_Rz = twist[1]

        dx = v_Rx * dt * np.cos(self.kinematic_slip_angle + self.ori_global + (w_Rz * dt) / 2)
        dy = v_Rx * dt * np.sin(self.kinematic_slip_angle + self.ori_global + (w_Rz * dt) / 2)
        dtheta = w_Rz * dt

        if (not(np.isnan(val) for val in [dx, dy, dtheta])):
            self.pos_global[0] += dx
            self.pos_global[1] += dy
            self.ori_global += dtheta

        return self.pos_global, self.ori_global

    def reset_pose(self):
        """
        Reset the global pose (position and orientation) to the origin.

        :return: None
        """
        self.pos_global = [0.0, 0.0]
        self.ori_global = 0



