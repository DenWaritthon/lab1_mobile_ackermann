#!/usr/bin/python3

import numpy as np
from kinematics import Kinematics

class AckermannKinematics(Kinematics):
    def __init__(self, r: float, L: float, B: float):
        super().__init__(r, L, B)

    def inverse(self, twist: list):
        '''- twist: [v_Rx, w_Rz] when v_Rx is a robot linear velocity in x axis 
                and w_Rz is a robot angular velocity in z axis'''
        
        # Robot Frame Space
        v_Rx = twist[0] # Robot linear velocity in x axis
        w_Rz = twist[1] # Robot angular velocity in z axis

        # Configuration Space
        if v_Rx == 0: v_Rx = 1e-5
        
        steering_angle = np.arctan(w_Rz*self.L/v_Rx)

        # Limit the Steering Angle
        if steering_angle > np.pi/6:
            steering_angle = np.pi/6
        elif steering_angle < -np.pi/6:
            steering_angle = -np.pi/6

        w_Wr = v_Rx/self.r                                                        # Rear wheel angular velocity

        # Left & Right Wheels Steering Angle
        tan_ack = np.tan(steering_angle)
        steering_angle_L = np.arctan(self.L*tan_ack/(self.L + 0.5*self.B*tan_ack))
        steering_angle_R = np.arctan(self.L*tan_ack/(self.L - 0.5*self.B*tan_ack))

        # Front Wheel Angular Velocity
        w_WfL = w_Wr
        w_WfR = w_Wr
        if steering_angle_L != 0:
            R = self.L/np.tan(steering_angle_L)
            w_WfL = w_Wr/np.abs(w_Wr)*np.abs(w_Rz*np.linalg.norm([self.L, R])/self.r)
            w_WfR = w_Wr/np.abs(w_Wr)*np.abs(w_Rz*np.linalg.norm([self.L, R-self.B])/self.r)

        return [steering_angle_L, steering_angle_R], w_Wr, [w_WfL, w_WfR]