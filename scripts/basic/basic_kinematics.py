#!/usr/bin/python3

import numpy as np
from kinematics import Kinematics

class BasicKinematics(Kinematics):
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

        w_Wr = v_Rx/self.r                                                      # Rear wheel angular velocity
        w_Wf = w_Wr/np.abs(w_Wr)*np.linalg.norm([self.L*w_Rz, v_Rx])/self.r     # Front wheel angular velocity (depend with rear wheel angular velocity)

        return [steering_angle, steering_angle], w_Wr, [w_Wf, w_Wf]

    def forward(self, steering_angle: float, w_Wr: float):
        '''- steering_angle: steering angle reference to robot x-axis
           - w_Wr:  rear wheel angular velocity'''
        
        # Robot Frame Space
        v_Rx = w_Wr * self.r                        # Robot linear velocity in x axis
        w_Rz = v_Rx * np.tan(steering_angle)/self.L # Robot angular velocity in z axis

        return v_Rx, w_Rz